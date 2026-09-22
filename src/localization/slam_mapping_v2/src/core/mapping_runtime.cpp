/**
 * @file mapping_runtime.cpp
 * @brief SLAM建图运行时核心实现
 *
 * 本文件实现了MappingRuntime类，负责：
 * - 里程计/激光雷达/GNSS数据输入与同步
 * - 关键帧管理与全局地图构建
 * - 后端回环检测与位姿图优化（支持GNSS约束）
 * - 多线程架构：主线程处理前端，后台线程处理后端优化
 * - 地图保存与状态管理
 */

#include "slam_mapping_v2/core/mapping_runtime.hpp"
#include "syslog/logger.h"

#include "slam_mapping_v2/core/loop_detector_kdtree_gicp.hpp"
#include "slam_mapping_v2/core/loop_detector_factory.hpp"
#include "slam_mapping_v2/core/visual_loop_detector.hpp"
#include "slam_mapping_v2/core/orb_feature_provider.hpp"
#include <cv_bridge/cv_bridge.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

namespace slam_mapping_v2 {

namespace {

/// 每行号独立的节流器（替代 ROS_INFO_THROTTLE）
template<int Line>
struct ThrottleByLine {
  static bool shouldLog(double rate) {
    static std::chrono::steady_clock::time_point last_log_time;
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration<double>(now - last_log_time).count();
    if (elapsed >= rate) {
      last_log_time = now;
      return true;
    }
    return false;
  }
};

#define SHOULD_LOG(rate) ThrottleByLine<__LINE__>::shouldLog(rate)

/**
 * @brief 从FrameInput提取位姿信息生成Pose对象
 * @param f 输入帧数据（包含时间戳、位置、姿态等）
 * @return Pose 提取后的位姿对象
 */
Pose poseFromFrame(const FrameInput& f) {
  Pose p;
  p.timestamp = f.timestamp;       // 时间戳
  p.position = f.position;         // 位置向量
  p.orientation = f.orientation;   // 姿态四元数
  p.frame_id = f.frame_id;         // 帧ID
  return p;
}

}  // namespace

/**
 * @brief 析构函数，确保后台线程正确停止
 *
 * 调用stopThreads()等待后台线程结束，防止资源泄漏
 */
MappingRuntime::~MappingRuntime() { stopThreads(); }

/**
 * @brief 初始化运行时模块
 * @param cfg 建图配置参数
 * @return bool 初始化是否成功
 *
 * 初始化流程：
 * 1. 保存配置参数
 * 2. 配置关键帧管理器、全局地图、GNSS模块、优化器、地图保存器
 * 3. 创建回环检测器（基于KD树+GICP）
 * 4. 设置运行状态为READY
 */
bool MappingRuntime::initialize(const MappingConfig& cfg) {
  cfg_ = cfg;
  // 配置各子模块
  kf_manager_.configure(cfg_.keyframe);          // 关键帧管理器配置
  global_map_.configure(cfg_.map);               // 全局地图配置
  gnss_.configure(cfg_.loop_closure.gnss);       // GNSS约束配置
  optimizer_.configure(cfg_.loop_closure);       // 位姿图优化器配置
  map_saver_.configure(cfg_.map_saver);          // 地图保存器配置

  // 使用工厂模式创建检测器（配置驱动）
  detector_ = LoopDetectorFactory::create(
      cfg_.loop_closure.detector_type, cfg_.loop_closure);

  // 如果是视觉检测器，注入特征缓冲区
  if (auto* vd = dynamic_cast<VisualLoopDetector*>(detector_.get())) {
    feat_buffer_ = std::make_shared<VisualFeatureBuffer>();
    vd->setFeatureBuffer(feat_buffer_);
    feat_buffer_->setBufferSize(cfg_.loop_closure.visual.buffer_size);
  }

  // 初始化运行状态
  std::lock_guard<std::mutex> lock(status_mutex_);
  status_ = RuntimeStatus{};
  status_.state = RuntimeState::READY;    // 状态：就绪
  status_.message = "ready";
  status_.loop_supported = true;          // 支持回环检测
  initialized_ = true;
  return true;
}

/**
 * @brief 启动建图任务
 * @param req 启动请求参数（任务ID、名称、输出目录等）
 * @param now_sec 当前时间戳（秒）
 * @return bool 启动是否成功
 *
 * 启动前检查：
 * - 必须已初始化
 * - 可选择重置之前的建图数据
 * 启动后状态变为RUNNING，开始接收并处理传感器数据
 */
bool MappingRuntime::start(const StartRequest& req, double now_sec) {
  if (!initialized_) return false;
  // 如果请求重置，则清空所有历史数据
  if (req.reset_before_start) reset();
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    status_.task_id = req.task_id;
    status_.task_name = req.task_name;
    status_.output_dir = req.output_dir;
    status_.loop_enabled = req.enable_loop_closure;  // 是否启用回环检测
    status_.loop_supported = true;
    status_.state = RuntimeState::RUNNING;           // 状态：运行中
    status_.message = "running";
    status_.running = true;
    status_.saving = false;
    status_.output_valid = false;
    status_.start_time = now_sec;
  }
  loop_enabled_ = req.enable_loop_closure;
  // ROS_INFO("[slam_mapping_v2] start RUNNING task=%s output=%s loop=%d",
  //          req.task_name.c_str(), req.output_dir.c_str(), req.enable_loop_closure ? 1 : 0);
  SYS_LOG(syslog::LogSeverity::INFO, true)
          << "start RUNNING task=" << req.task_name << " output=" << req.output_dir << " loop=" << (req.enable_loop_closure ? 1 : 0);
  return true;
}

/**
 * @brief 停止建图任务
 * @param req 停止请求参数（是否保存地图等）
 * @return bool 停止是否成功
 *
 * 停止流程：
 * 1. 检查当前状态是否为RUNNING或READY
 * 2. 设置状态为STOPPING
 * 3. 如果需要保存地图且配置允许，执行保存操作
 * 4. 最终状态变为FINISHED
 */
bool MappingRuntime::stop(const StopRequest& req) {
  const auto st = statusSnapshot();
  // 只有运行中或就绪状态才能停止
  if (st.state != RuntimeState::RUNNING && st.state != RuntimeState::READY) return false;
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    status_.state = RuntimeState::STOPPING;   // 状态：停止中
    status_.message = "stopping";
    status_.running = false;
  }
  // 根据请求和配置决定是否保存地图
  if (req.save_map && cfg_.map_saver.save_on_stop) {
    SaveRequest save_req;
    save_req.output_dir = st.output_dir;
    save_req.map_name = cfg_.map_saver.map_filename;
    save_req.resolution = cfg_.map_saver.default_resolution;
    save_req.save_keyframes = req.save_keyframes;
    save_req.save_keyframe_poses = req.save_keyframe_poses;
    save_req.save_trajectory = req.save_trajectory;
    save_req.save_loop_results = cfg_.map_saver.save_loop_results;
    return save(save_req);
  }
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    status_.state = RuntimeState::FINISHED;   // 状态：已完成
    status_.message = "finished";
  }
  return true;
}

/**
 * @brief 保存建图结果到磁盘
 * @param req 保存请求参数（输出目录、地图名称、分辨率等）
 * @return bool 保存是否成功
 *
 * 保存内容：
 * - 点云地图（PCD格式）
 * - 关键帧点云
 * - 关键帧位姿
 * - 轨迹文件
 * - 回环检测结果
 * - GNSS对齐参数（如启用GNSS）
 */
bool MappingRuntime::save(const SaveRequest& req) {
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    status_.state = RuntimeState::SAVING;     // 状态：保存中
    status_.saving = true;
    status_.message = "saving";
  }
  // 拷贝关键帧数据（读锁保护）
  std::vector<Keyframe> kfs;
  double lat = 0.0, lon = 0.0, alt = 0.0;
  {
    std::shared_lock<std::shared_mutex> kf_lock(kf_mutex_);
    kfs = keyframes_;
    // 如果启用GNSS，获取ENU坐标系原点
    if (cfg_.loop_closure.gnss.enable) {
      gnss_.getEnuOrigin(&lat, &lon, &alt);
    }
  }
  // 获取回环数据
  LoopData ld = loopData();
  SaveResult result;
  // 调用地图保存器保存所有数据
  const bool ok = map_saver_.saveAll(req, kfs, ld.loop_pairs, ld.current_submap,
                                     ld.history_submap, &result);
  // 如果启用GNSS且对齐有效，保存GNSS对齐文件
  if (cfg_.loop_closure.gnss.enable && ld.map_enu_valid) {
    map_saver_.saveGnssAlignment(req.output_dir + "/gnss_alignment.txt", ld.map_enu_rot,
                                 ld.map_enu_trans, lat, lon, alt);
  }
  // 更新状态为完成或失败
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    status_.saving = false;
    status_.output_valid = ok;
    status_.map_path = result.map_path;
    status_.keyframe_dir = result.keyframe_dir;
    status_.keyframe_pose_path = result.keyframe_pose_path;
    status_.trajectory_path = result.trajectory_path;
    status_.loop_result_path = result.loop_result_path;
    status_.state = ok ? RuntimeState::FINISHED : RuntimeState::FAILED;
    status_.message = ok ? "saved:" + result.map_path : "save failed";
  }
  return ok;
}

/**
 * @brief 重置所有运行时数据
 *
 * 清空内容：
 * - 输入队列（里程计、激光雷达、GNSS）
 * - 待处理关键帧队列
 * - 已有关键帧列表
 * - 回环数据
 * - 最新帧数据
 * - 全局地图
 * - 各子模块状态
 * 重置后状态变为READY
 */
void MappingRuntime::reset() {
  // 清空输入队列
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    odom_queue_.clear();
    scan_queue_.clear();
    gnss_queue_.clear();
    gnss_buffer_.clear();
  }
  // 清空待处理关键帧
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    pending_.clear();
  }
  // 清空已有关键帧（写锁）
  {
    std::unique_lock<std::shared_mutex> kf_lock(kf_mutex_);
    keyframes_.clear();
  }
  // 清空回环数据
  {
    std::lock_guard<std::mutex> lock(loop_mutex_);
    loop_data_ = LoopData{};
  }
  // 清空最新帧
  {
    std::lock_guard<std::mutex> lock(latest_mutex_);
    latest_frame_ = FrameInput{};
  }
  // 重置成员变量
  last_keyframe_ = Keyframe{};
  frame_count_ = 0;
  loop_enabled_ = false;

  // 重置全局地图
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    global_map_.reset();
  }
  // 重置各子模块
  gnss_.reset();
  optimizer_.reset();
  detector_->reset();
  map_saver_.configure(cfg_.map_saver);

  // 重置状态为READY
  std::lock_guard<std::mutex> lock(status_mutex_);
  status_ = RuntimeStatus{};
  status_.state = RuntimeState::READY;
  status_.message = "ready";
  status_.loop_supported = true;
}

/**
 * @brief 输入里程计数据
 * @param odom 里程计帧数据
 *
 * 线程安全地将里程计数据加入队列，队列满时自动丢弃最旧数据
 */
void MappingRuntime::feedOdom(const FrameInput& odom) {
  std::lock_guard<std::mutex> lock(input_mutex_);
  // 队列超过最大长度时，丢弃最旧数据
  if (odom_queue_.size() >= cfg_.threading.max_input_queue) odom_queue_.pop_front();
  odom_queue_.push_back(odom);
}

/**
 * @brief 输入激光雷达扫描数据
 * @param scan 激光雷达帧数据（包含点云）
 *
 * 线程安全地将扫描数据加入队列，队列满时自动丢弃最旧数据
 */
void MappingRuntime::feedScan(const FrameInput& scan) {
  std::lock_guard<std::mutex> lock(input_mutex_);
  // 队列超过最大长度时，丢弃最旧数据
  if (scan_queue_.size() >= cfg_.threading.max_input_queue) scan_queue_.pop_front();
  scan_queue_.push_back(scan);
}

/**
 * @brief 输入GNSS观测数据
 * @param gnss GNSS观测数据
 *
 * 线程安全地将GNSS数据加入队列，队列长度限制为5000
 */
void MappingRuntime::feedGnss(const GnssObservation& gnss) {
  std::lock_guard<std::mutex> lock(input_mutex_);
  gnss_queue_.push_back(gnss);
  // 限制GNSS队列长度，防止内存无限增长
  while (gnss_queue_.size() > 5000) gnss_queue_.pop_front();
}

/**
 * @brief 从队列中弹出时间同步的里程计和扫描帧
 * @param odom 输出里程计帧
 * @param scan 输出扫描帧
 * @return bool 是否成功获取同步帧对
 *
 * 时间同步策略：
 * - 比较队列头部里程计和扫描的时间戳
 * - 如果时间差在容差范围内，返回该帧对
 * - 如果时间差超出容差，丢弃时间较早的一帧
 */
bool MappingRuntime::popFrame(FrameInput* odom, FrameInput* scan) {
  std::lock_guard<std::mutex> lock(input_mutex_);
  const double tol = cfg_.threading.time_sync_tolerance_sec;  // 时间同步容差
  while (!odom_queue_.empty() && !scan_queue_.empty()) {
    const auto& o = odom_queue_.front();
    const auto& s = scan_queue_.front();
    const double dt = std::abs(o.timestamp - s.timestamp);
    // 时间差在容差内，返回同步帧对
    if (dt <= tol) {
      *odom = o;
      *scan = s;
      odom_queue_.pop_front();
      scan_queue_.pop_front();
      return true;
    }
    // 丢弃时间较早的帧
    if (o.timestamp < s.timestamp) {
      odom_queue_.pop_front();
    } else {
      scan_queue_.pop_front();
    }
  }
  return false;
}

/**
 * @brief 主线程单次处理循环
 * @return bool 是否成功处理一帧
 *
 * 处理流程：
 * 1. 检查初始化和运行状态
 * 2. 将GNSS数据从输入队列转移到处理缓冲区
 * 3. 时间同步获取里程计和扫描帧对
 * 4. 合并位姿和点云为完整帧
 * 5. 更新最新帧和状态
 * 6. 插入全局地图
 * 7. 关键帧判定，若满足条件则加入待处理队列并通知后端线程
 */
bool MappingRuntime::processOnce() {
  // 检查是否已初始化且处于运行状态
  if (!initialized_ || statusSnapshot().state != RuntimeState::RUNNING) {
    // ROS_WARN_THROTTLE(5.0, "[slam_mapping_v2] processOnce skip: state=%s (call start or set runtime/auto_start)",
    //                   toString(statusSnapshot().state).c_str());
    if (SHOULD_LOG(5.0)) {
      SYS_LOG(syslog::LogSeverity::WARNING, true) << "processOnce skip: state=" << toString(statusSnapshot().state) << " (call start or set runtime/auto_start)";
    }
    return false;
  }
  // 检查输入队列状态
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    if (odom_queue_.empty() || scan_queue_.empty()) {
      // ROS_WARN_THROTTLE(5.0, "[slam_mapping_v2] waiting input: odom_q=%zu scan_q=%zu",
      //                   odom_queue_.size(), scan_queue_.size());
      if (SHOULD_LOG(5.0)) {
        SYS_LOG(syslog::LogSeverity::WARNING, true) << "waiting input: odom_q=" << odom_queue_.size() << " scan_q=" << scan_queue_.size();
      }
    }
  }

  // 将GNSS数据从输入队列转移到处理缓冲区
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    while (!gnss_queue_.empty()) {
      gnss_buffer_.push_back(gnss_queue_.front());
      gnss_queue_.pop_front();
    }
  }
  // 限制GNSS缓冲区长度
  while (gnss_buffer_.size() > 5000) gnss_buffer_.pop_front();

  // 获取时间同步的帧对
  FrameInput odom, scan;
  if (!popFrame(&odom, &scan)) return false;
  // ROS_INFO_THROTTLE(2.0, "[slam_mapping_v2] consumed frame ts=%.3f odom_valid=%d pts=%zu",
  //                   odom.timestamp, odom.valid ? 1 : 0, scan.cloud ? scan.cloud->size() : 0);
  if (SHOULD_LOG(2.0)) {
    SYS_LOG(syslog::LogSeverity::INFO, false) << "consumed frame ts=" << odom.timestamp << " odom_valid=" << (odom.valid ? 1 : 0) << " pts=" << (scan.cloud ? scan.cloud->size() : 0);
  }
  
  // 验证数据有效性
  if (!odom.valid || !scan.cloud || scan.cloud->empty()) return false;

  // 合并odom位姿与scan点云为完整帧（关键帧判定需要点云与位姿）
  FrameInput frame = scan;
  frame.position = odom.position;
  frame.orientation = odom.orientation;
  frame.valid = true;
  // 更新最新帧（线程安全）
  {
    std::lock_guard<std::mutex> lock(latest_mutex_);
    latest_frame_ = frame;
  }
  // 更新状态时间信息
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    status_.last_frame_time = odom.timestamp;
    status_.duration_sec = odom.timestamp - status_.start_time;
  }
  ++frame_count_;

  // 关键帧判定（主线程私有last_keyframe_，无锁）
  if (kf_manager_.shouldAdd(last_keyframe_, frame, frame_count_)) {
    Keyframe kf;
    kf.id = 0;  // 临时 ID，避免 shouldAdd() 误判为首帧（实际 ID 在后端线程分配）
    kf.timestamp = frame.timestamp;
    kf.pose_odom = poseFromFrame(frame);   // 里程计位姿
    kf.pose_map = kf.pose_odom;            // 初始地图位姿与里程计一致
    kf.cloud = frame.cloud;                // 点云数据
    kf.point_count = frame.cloud->size();  // 点云数量
    kf.source_frame = frame_count_;        // 来源帧序号
    // 如果启用GNSS，关联最近的GNSS观测到关键帧
    if (cfg_.loop_closure.gnss.enable) {
      associateGnssToKeyframe(&kf);
    }

    // 保存最新图像到关键帧（用于离线 marker map 构建）
    {
      std::lock_guard<std::mutex> img_lock(image_mutex_);
      // ROS_INFO_THROTTLE(2.0, "[Keyframe] kf_ts=%.3f image_queue_size=%zu",
      //                   kf.timestamp, image_queue_.size());
      if (SHOULD_LOG(2.0)) {
        SYS_LOG(syslog::LogSeverity::INFO, false) << "kf_ts=" << kf.timestamp << " image_queue_size=" << image_queue_.size();
      }
      if (!image_queue_.empty()) {
        // 找到时间戳最接近的图像
        auto best_it = image_queue_.begin();
        double best_dt = std::abs(best_it->second - kf.timestamp);
        for (auto it = image_queue_.begin(); it != image_queue_.end(); ++it) {
          double dt = std::abs(it->second - kf.timestamp);
          if (dt < best_dt) {
            best_dt = dt;
            best_it = it;
          }
        }
        // ROS_INFO_THROTTLE(2.0, "[Keyframe] best_dt=%.3f img_ts=%.3f kf_ts=%.3f",
        //                   best_dt, best_it->second, kf.timestamp);
        if (SHOULD_LOG(2.0)) {
          SYS_LOG(syslog::LogSeverity::INFO, false) << "best_dt=" << best_dt << " img_ts=" << best_it->second << " kf_ts=" << kf.timestamp;
        }
        if (best_dt < 0.05) {  // 50ms 容差
          kf.has_visual = true;
          kf.image_timestamp = best_it->second;
          kf.image = best_it->first.clone();
          // ROS_INFO_THROTTLE(2.0, "[Keyframe] saved image for kf %d, img_size=%dx%d",
          //                   kf.id, kf.image.cols, kf.image.rows);
        } else {
          // ROS_WARN_THROTTLE(2.0, "[Keyframe] no matching image: best_dt=%.3f > 0.05", best_dt);
          if (SHOULD_LOG(2.0)) {
            SYS_LOG(syslog::LogSeverity::WARNING, true) << "no matching image: best_dt=" << best_dt << " > 0.05";
          }
        }
      } else {
        // ROS_WARN_THROTTLE(2.0, "[Keyframe] image_queue is empty");
        if (SHOULD_LOG(2.0)) {
          SYS_LOG(syslog::LogSeverity::WARNING, true) << "image_queue is empty";
        }
      }
    }

    last_keyframe_ = kf;
    // 将关键帧加入待处理队列，通知后端线程
    {
      std::lock_guard<std::mutex> lock(pending_mutex_);
      pending_.push_back(std::move(kf));
    }
    pending_cv_.notify_one();
  }
  return true;
}

/**
 * @brief 将GNSS观测关联到关键帧
 * @param kf 待关联的关键帧指针
 *
 * 关联策略：
 * - 在GNSS缓冲区中查找时间戳最接近关键帧的观测
 * - 时间差必须在max_age_sec范围内
 * - 找到后从缓冲区移除该观测，避免重复关联
 * - 清理过期的GNSS数据
 */
void MappingRuntime::associateGnssToKeyframe(Keyframe* kf) {
  const double max_age = cfg_.loop_closure.gnss.max_age_sec;  // 最大时间差
  auto best = gnss_buffer_.end();
  double best_dt = max_age;
  // 遍历缓冲区查找最佳匹配
  for (auto it = gnss_buffer_.begin(); it != gnss_buffer_.end(); ++it) {
    const double dt = std::abs(it->timestamp - kf->timestamp);
    if (dt <= best_dt) {
      best_dt = dt;
      best = it;
    }
  }
  // 如果找到匹配的GNSS观测，关联到关键帧
  if (best != gnss_buffer_.end()) {
    kf->gnss = *best;
    gnss_buffer_.erase(best);
  }
  // 清理过期的GNSS数据（时间戳早于关键帧max_age_sec以上）
  while (!gnss_buffer_.empty() && gnss_buffer_.front().timestamp < kf->timestamp - max_age) {
    gnss_buffer_.pop_front();
  }
}

/**
 * @brief 后端处理线程主循环
 *
 * 后端线程职责：
 * 1. 等待待处理关键帧（条件变量超时等待）
 * 2. 将关键帧加入全局关键帧列表
 * 3. 添加里程计因子到位姿图
 * 4. 如果启用GNSS，添加GNSS约束因子
 * 5. 如果启用回环检测，执行回环检测并添加回环因子
 * 6. 执行位姿图优化
 * 7. 更新回环数据和状态
 *
 * 线程安全：使用kf_mutex_保护关键帧列表
 */
void MappingRuntime::backendLoop() {
  while (!stop_threads_) {
    std::vector<Keyframe> batch;
    {
      // 等待待处理关键帧或停止信号
      std::unique_lock<std::mutex> lock(pending_mutex_);
      pending_cv_.wait_for(
          lock,
          std::chrono::milliseconds(
              static_cast<int>(cfg_.threading.backend_interval_sec * 1000.0)),
          [this]() { return stop_threads_ || !pending_.empty(); });
      if (stop_threads_) break;  // 收到停止信号，退出循环
      batch.swap(pending_);      // 取出所有待处理关键帧
    }
    if (batch.empty()) continue;
    // 检查运行状态
    if (statusSnapshot().state != RuntimeState::RUNNING) continue;

    LoopResult r;
    double latest_score = -1.0;
    const Keyframe* latest_kf = nullptr;
    {
      // 写锁保护关键帧列表
      std::unique_lock<std::shared_mutex> kf_lock(kf_mutex_);
      // 将批量关键帧加入全局列表并分配ID
      const int appended_start = static_cast<int>(keyframes_.size());
      for (auto& kf : batch) {
        kf.id = static_cast<int>(keyframes_.size());
        keyframes_.push_back(std::move(kf));
      }
      const int latest_index = static_cast<int>(keyframes_.size()) - 1;

      bool did_optimize = false;
      bool loop_detected = false;

      // 如果启用回环检测或GNSS，构建并优化位姿图
      if (loop_enabled_ || cfg_.loop_closure.gnss.enable) {
        // 添加里程计因子（连续关键帧之间的约束）
        optimizer_.addOdomFactors(keyframes_);
        // 如果启用GNSS，添加GNSS约束因子
        const std::size_t gnss_before = gnss_.factorCount();
        if (cfg_.loop_closure.gnss.enable) {
          gnss_.update(keyframes_, &optimizer_.graph(), latest_index);
        }
        // 回环检测
        LoopCandidate candidate;
        if (loop_enabled_) {
          candidate = detector_->detect(keyframes_); // 视觉的话先detect，在detect中提取视觉特征
          if (candidate.valid) {
            loop_detected = true;
            // 标记回环关键帧
            keyframes_[candidate.current_idx].loop_closed = true;
            keyframes_[candidate.history_idx].loop_closed = true;
            keyframes_[candidate.current_idx].loop_match_count += 1;
            keyframes_[candidate.history_idx].loop_match_count += 1;
            latest_score = candidate.score;
            // 添加回环因子到位姿图
            optimizer_.addLoopFactor(candidate);
          }
        }
        // 执行位姿图优化
        optimizer_.optimize(&keyframes_);
        // 判断是否真正发生了优化：检测到回环 OR 新增了GNSS因子
        did_optimize = loop_detected || (gnss_.factorCount() > gnss_before);
      }

      // 根据是否优化选择重建方式：优化后全量重建，否则增量追加
      {
        std::lock_guard<std::mutex> map_lock(map_mutex_);
        if (did_optimize) {
          // 优化后全量重建（所有关键帧使用优化后的位姿重新变换）
          global_map_.rebuild(keyframes_);
        } else {
          // 无优化：只增量追加本轮新增的关键帧
          for (int i = appended_start; i <= latest_index; ++i) {
            global_map_.append(keyframes_[i]);
          }
        }
      }
      
      // 记录最新关键帧
      if (!keyframes_.empty()) latest_kf = &keyframes_.back();
      // ROS_INFO_THROTTLE(1.0, "[slam_mapping_v2] backend merged: kfs=%zu loops=%zu gnss_factors=%zu",
      //                   keyframes_.size(), detector_->loopPairs().size(), gnss_.factorCount());
      if (SHOULD_LOG(2.0)) {  
        SYS_LOG(syslog::LogSeverity::INFO, true) << "backend merged: kfs=" << keyframes_.size() << " loops=" << detector_->loopPairs().size() << " gnss_factors=" << gnss_.factorCount();
      }
      // 判断是否有新的回环更新
      r.updated = !keyframes_.empty() && loop_enabled_ &&
                  !detector_->loopPairs().empty() &&
                  detector_->loopPairs().back().second == latest_index;
      r.loop_pairs = detector_->loopPairs();
      r.loop_count = static_cast<int>(r.loop_pairs.size());
      r.latest_score = latest_score;
      r.gnss_factor_count = gnss_.factorCount();
      r.gnss_rejected_count = gnss_.rejectedCount();
      r.gnss_aligned = gnss_.aligned();
      // 更新回环数据
      updateLoopData(latest_kf, r);
    }
    // 更新状态信息
    updateStatusFromLoop(r);
  }
}

/**
 * @brief 更新回环检测相关数据
 * @param latest_kf 最新关键帧指针（可能为nullptr）
 * @param r 回环检测结果
 *
 * 更新内容：
 * - 回环匹配对
 * - 当前和历史子地图
 * - GNSS因子位置（接受和拒绝的）
 * - 地图到ENU坐标系的变换
 * - 地图到里程计的变换（用于地图对齐）
 */
void MappingRuntime::updateLoopData(const Keyframe* latest_kf, const LoopResult& r) {
  LoopData d;
  d.loop_pairs = r.loop_pairs;
  d.current_submap = detector_->latestCurrentSubmap();
  d.history_submap = detector_->latestHistorySubmap();
  d.gnss_factor_positions = gnss_.acceptedPositions();
  d.gnss_rejected_positions = gnss_.rejectedPositions();
  d.gnss_accepted_factors = gnss_.acceptedFactors();
  d.gnss_rejected_factors = gnss_.rejectedFactors();
  d.map_enu_valid = gnss_.getMapEnuTransform(&d.map_enu_rot, &d.map_enu_trans);
  // 计算地图坐标系到里程计坐标系的变换
  if (latest_kf) {
    d.t_map_odom = poseToMatrix(latest_kf->pose_map) * poseToMatrix(latest_kf->pose_odom).inverse();
    d.map_odom_valid = d.t_map_odom.allFinite();
  }
  std::lock_guard<std::mutex> lock(loop_mutex_);
  loop_data_ = std::move(d);
}

/**
 * @brief 根据回环结果更新运行时状态
 * @param r 回环检测结果
 *
 * 更新状态信息包括：
 * - 回环计数
 * - GNSS因子统计
 * - 最新回环匹配对和分数
 * - 状态消息（检测到回环时显示详细信息）
 */
void MappingRuntime::updateStatusFromLoop(const LoopResult& r) {
  std::lock_guard<std::mutex> lock(status_mutex_);
  status_.loop_count = r.loop_count;
  status_.gnss_factor_count = r.gnss_factor_count;
  status_.gnss_rejected_count = r.gnss_rejected_count;
  status_.gnss_aligned = r.gnss_aligned;
  status_.loop_updated_last_frame = r.updated;
  status_.last_loop_score = r.latest_score;
  // 记录最新回环匹配对
  if (!r.loop_pairs.empty()) {
    status_.last_loop_from = r.loop_pairs.back().first;
    status_.last_loop_to = r.loop_pairs.back().second;
  }
  // 更新状态消息
  if (r.updated && status_.last_loop_from >= 0 && status_.last_loop_to >= 0) {
    status_.message = "loop_detected:" + std::to_string(status_.last_loop_from) + "->" +
                      std::to_string(status_.last_loop_to) +
                      " score=" + std::to_string(status_.last_loop_score);
  } else if (status_.state == RuntimeState::RUNNING) {
    status_.message = "running";
  }
}

/**
 * @brief 启动后台处理线程
 *
 * 如果后台线程未启动，创建新线程运行backendLoop()
 */
void MappingRuntime::startThreads() {
  if (!backend_thread_.joinable()) {
    stop_threads_ = false;
    backend_thread_ = std::thread(&MappingRuntime::backendLoop, this);
  }

  // 条件启动视觉特征线程（仅 visual 或 hybrid 模式）
  if ((cfg_.loop_closure.detector_type == "visual" ||
       cfg_.loop_closure.detector_type == "hybrid") &&
      !visual_feat_thread_.joinable()) {
    feat_provider_ = std::make_unique<OrbFeatureProvider>();
    feat_provider_->configure(cfg_.loop_closure.visual);
    visual_feat_thread_ = std::thread(&MappingRuntime::visualFeatLoop, this);
    // ROS_INFO("[slam_mapping_v2] Visual feature thread started (detector_type=%s)",
    //          cfg_.loop_closure.detector_type.c_str());
    SYS_LOG(syslog::LogSeverity::INFO, true) << "Visual feature thread started (detector_type=" << cfg_.loop_closure.detector_type << ")";
  }
}

/**
 * @brief 停止后台处理线程
 *
 * 设置停止标志，通知条件变量，等待线程结束
 */
void MappingRuntime::stopThreads() {
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    stop_threads_ = true;
  }
  pending_cv_.notify_all();  // 唤醒等待的线程
  image_cv_.notify_all();    // 唤醒视觉线程

  if (backend_thread_.joinable()) backend_thread_.join();
  if (visual_feat_thread_.joinable()) visual_feat_thread_.join();
}

/**
 * @brief 关闭运行时（安全退出）
 *
 * 如果正在运行，先停止并保存地图，然后停止后台线程
 */
void MappingRuntime::shutdown() {
  const auto st = statusSnapshot();
  if (st.state == RuntimeState::RUNNING) {
    StopRequest req;
    req.task_id = st.task_id;
    req.save_map = true;
    req.save_keyframes = cfg_.map_saver.save_keyframes;
    req.save_keyframe_poses = cfg_.map_saver.save_keyframe_poses;
    req.save_trajectory = cfg_.map_saver.save_trajectory;
    stop(req);
  }
  stopThreads();
}

/**
 * @brief 获取运行时状态的快照（线程安全）
 * @return RuntimeStatus 当前状态副本
 */
RuntimeStatus MappingRuntime::statusSnapshot() const {
  std::lock_guard<std::mutex> lock(status_mutex_);
  return status_;
}

/**
 * @brief 获取完整的运行时状态（包含关键帧计数等）
 * @return RuntimeStatus 完整状态信息
 */
RuntimeStatus MappingRuntime::status() const {
  auto st = statusSnapshot();
  {
    std::shared_lock<std::shared_mutex> kf_lock(kf_mutex_);
    st.keyframe_count = keyframes_.size();
  }
  st.frame_count = frame_count_;
  if (st.start_time > 0.0) st.duration_sec = st.last_frame_time - st.start_time;
  return st;
}

/**
 * @brief 获取所有关键帧的副本
 * @return std::vector<Keyframe> 关键帧列表副本
 */
std::vector<Keyframe> MappingRuntime::keyframes() const {
  std::shared_lock<std::shared_mutex> kf_lock(kf_mutex_);
  return keyframes_;
}

/**
 * @brief 获取回环数据副本
 * @return LoopData 回环检测相关数据
 */
LoopData MappingRuntime::loopData() const {
  std::lock_guard<std::mutex> lock(loop_mutex_);
  return loop_data_;
}

/**
 * @brief 获取最新处理的帧
 * @return FrameInput 最新帧数据副本
 */
FrameInput MappingRuntime::latestFrame() const {
  std::lock_guard<std::mutex> lock(latest_mutex_);
  return latest_frame_;
}

/**
 * @brief 获取地图到里程计坐标系的变换矩阵
 * @param t_map_odom 输出4x4齐次变换矩阵
 * @return bool 变换是否有效
 */
bool MappingRuntime::mapOdomTransform(Eigen::Matrix4d* t_map_odom) const {
  std::lock_guard<std::mutex> lock(loop_mutex_);
  if (!loop_data_.map_odom_valid) return false;
  *t_map_odom = loop_data_.t_map_odom;
  return true;
}

/**
 * @brief 获取全局地图点云（map 系，已降采样）
 * @return PointCloudPtr 地图点云指针，可直接发布
 */
PointCloudPtr MappingRuntime::mapCloud() const {
  std::lock_guard<std::mutex> lock(map_mutex_);
  return global_map_.cloud();
}

/**
 * @brief 输入图像帧（供视觉特征提取线程消费）
 * @param image 灰度/彩色图像
 * @param timestamp 图像时间戳
 */
void MappingRuntime::feedImage(const cv::Mat& image, double timestamp) {
  std::lock_guard<std::mutex> lock(image_mutex_);
  if (image_queue_.size() >= cfg_.threading.max_input_queue) {
    image_queue_.pop_front();
  }
  image_queue_.push_back({image.clone(), timestamp});
  image_cv_.notify_one();
}

/**
 * @brief 视觉特征提取线程主循环
 *
 * 流程：
 * 1. 等待图像到达
 * 2. 通过时间戳匹配关键帧 ID
 * 3. 提取 ORB 特征
 * 4. 写入特征缓冲区
 */
void MappingRuntime::visualFeatLoop() {
  // ROS_INFO("[slam_mapping_v2] Visual feature thread started");
  SYS_LOG(syslog::LogSeverity::INFO, true) << "Visual feature thread started";

  while (!stop_threads_) {
    std::deque<std::pair<cv::Mat, double>> batch;
    {
      std::unique_lock<std::mutex> lock(image_mutex_);
      image_cv_.wait_for(lock, std::chrono::milliseconds(100),
                         [this]() { return stop_threads_ || !image_queue_.empty(); });
      if (stop_threads_) break;
      batch.swap(image_queue_);
    }

    // ROS_INFO_THROTTLE(2.0, "[VisualFeat] batch_size=%zu image_queue_size after swap=%zu",
    //                   batch.size(), image_queue_.size());

    if (batch.empty()) continue;

    for (auto& [image, ts] : batch) {
      if (stop_threads_) break;

      // 通过时间戳查找关键帧 ID
      // 策略：找时间戳 <= 图像时间戳的最新关键帧（即图像拍摄时"当前"的关键帧）
      int kf_id = -1;
      double kf_ts = -1;
      {
        std::shared_lock<std::shared_mutex> kf_lock(kf_mutex_);
        for (int i = static_cast<int>(keyframes_.size()) - 1; i >= 0; --i) {
          if (keyframes_[i].timestamp <= ts + 0.5) {  // 允许 50ms 的未来容差
            kf_id = keyframes_[i].id;
            kf_ts = keyframes_[i].timestamp;
            break;
          }
        }
      }

      if (kf_id >= 0) {
        // 检查 buffer 中是否已有该关键帧的特征
        VisualFrame existing;
        if (feat_buffer_->get(kf_id, &existing) && existing.features_ready) {
          continue;  // 已有特征，跳过提取！
        }

        auto frame = feat_provider_->extract(image, ts, kf_id);
        if (frame.features_ready) {
          feat_buffer_->insert(frame);
          // ROS_INFO_THROTTLE(2.0, "[VisualFeat] Extracted kf %d: img_ts=%.3f kf_ts=%.3f dt=%.3fs keypoints=%zu buf_size=%zu",
          //                   kf_id, ts, kf_ts, ts - kf_ts, frame.keypoints.size(), feat_buffer_->size());
          if (SHOULD_LOG(2.0)) {
            SYS_LOG(syslog::LogSeverity::INFO, true) << "Extracted kf " << kf_id << ": img_ts=" << ts << " kf_ts=" << kf_ts << " dt=" << ts - kf_ts << " keypoints=" << frame.keypoints.size() << " buf_size=" << feat_buffer_->size();
          }
        }
      } else {
        double latest_kf_ts = -1;
        {
          std::shared_lock<std::shared_mutex> kf_lock(kf_mutex_);
          if (!keyframes_.empty()) latest_kf_ts = keyframes_.back().timestamp;
        }
        // ROS_INFO_THROTTLE(2.0, "[VisualFeat] No match: img_ts=%.3f latest_kf_ts=%.3f kf_count=%zu",
        //                   ts, latest_kf_ts, keyframes_.size());
        if (SHOULD_LOG(2.0)) {
          SYS_LOG(syslog::LogSeverity::INFO, true) << "No match: img_ts=" << ts << " latest_kf_ts=" << latest_kf_ts << " kf_count=" << keyframes_.size();
        }
      }
    }
  }

  // ROS_INFO("[slam_mapping_v2] Visual feature thread stopped");
  SYS_LOG(syslog::LogSeverity::INFO, true) << "Visual feature thread stopped";
}

}  // namespace slam_mapping_v2
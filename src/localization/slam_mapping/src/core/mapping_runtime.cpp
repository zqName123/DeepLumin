/**
 * @file mapping_runtime.cpp
 * @brief SLAM Mapping 运行时核心模块
 *
 * 该模块是 slam_mapping 包的核心运行时管理器，负责：
 * - 生命周期管理：初始化、启动、停止、重置
 * - 数据流管理：接收 IMU、LiDAR、轮速计、GNSS 数据并转发给 SLAM 后端
 * - IMU 预热：系统启动时收集足够 IMU 样本以保证初始化质量
 * - 帧处理：从 SLAM 后端获取里程计和点云，管理关键帧和回环检测
 * - 地图保存：保存关键帧、轨迹、回环结果等
 *
 * 架构设计：
 * - MappingRuntime 作为高层协调器，不直接实现 SLAM 算法
 * - 通过 ISlamOdometry 接口与具体的 SLAM 后端（如 slam_odometry）交互
 * - 关键帧管理、回环检测、地图保存由独立模块负责
 *
 * 状态机：
 * IDLE → READY → RUNNING → STOPPING → SAVING → FINISHED/FAILED
 *
 * @see slam_mapping::MappingRuntime
 * @see localization::ISlamOdometry
 */

// 模块头文件
#include "slam_mapping/core/mapping_runtime.hpp"

// 工厂注册
#include <localization/factory/factory_registry.hpp>

// 标准库
#include <algorithm>  // 算法工具
#include <sstream>    // 字符串流

namespace slam_mapping {

namespace {

/**
 * @brief 将里程计结果转换为位姿结构
 *
 * @param odom 输入的里程计结果
 * @return localization::Pose 转换后的位姿
 *
 * 用于将 SLAM 后端的里程计输出转换为通用的位姿格式，
 * 便于关键帧管理和地图保存。
 */
localization::Pose poseFromOdom(const localization::OdomResult& odom) {
  localization::Pose pose;
  pose.timestamp = odom.timestamp;
  pose.position = odom.position;
  pose.orientation = odom.orientation;
  pose.covariance = odom.covariance;
  pose.frame_id = odom.frame_id;
  return pose;
}

}  // namespace

/**
 * @brief 将运行时状态枚举转换为字符串
 *
 * @param state 运行时状态枚举
 * @return std::string 状态字符串表示
 *
 * 用于日志输出和状态显示。
 */
std::string toString(RuntimeState state) {
  switch (state) {
    case RuntimeState::IDLE:     return "IDLE";     // 空闲
    case RuntimeState::READY:    return "READY";    // 就绪
    case RuntimeState::RUNNING:  return "RUNNING";  // 运行中
    case RuntimeState::STOPPING: return "STOPPING"; // 停止中
    case RuntimeState::SAVING:   return "SAVING";   // 保存中
    case RuntimeState::FINISHED: return "FINISHED"; // 已完成
    case RuntimeState::FAILED:   return "FAILED";   // 失败
  }
  return "UNKNOWN";
}

/**
 * @brief 初始化 MappingRuntime
 *
 * @param cfg SLAM Mapping 配置
 * @return true 初始化成功
 * @return false 初始化失败
 *
 * 初始化流程：
 * 1. 保存配置
 * 2. 设置 IMU 预热状态（若禁用预热则直接就绪）
 * 3. 注册默认插件（工厂模式）
 * 4. 创建 SLAM 后端（通过工厂根据配置类型创建）
 * 5. 初始化 SLAM 后端
 * 6. 配置关键帧管理器
 * 7. 配置地图保存器
 * 8. 配置回环检测后端
 * 9. 设置状态为 READY
 */
bool MappingRuntime::initialize(const SlamMappingConfig& cfg) {
  config_ = cfg;

  // IMU 预热：若禁用预热，则直接标记为就绪
  imu_warmup_ready_ = !config_.startup.enable_imu_warmup;

  // 注册默认插件（工厂模式）
  localization::registerDefaultPlugins();

  // 创建 SLAM 后端（根据配置类型，如 "faster_lio"）
  slam_ = localization::SlamFactory::instance().create(config_.slam_backend.type);
  if (!slam_) {
    status_.state = RuntimeState::FAILED;
    status_.message = "failed to create slam backend";
    return false;
  }

  // 初始化 SLAM 后端
  if (!slam_->initialize(config_.slam_backend)) {
    status_.state = RuntimeState::FAILED;
    status_.message = "failed to initialize slam backend";
    return false;
  }

  // 配置子模块
  keyframe_manager_.configure(config_.keyframe);
  map_saver_.configure(config_.map_saver);
  loop_backend_.configure(config_.loop_closure);

  // 设置状态
  status_.state = RuntimeState::READY;
  status_.message = "ready";
  status_.loop_supported = loop_backend_.supported();
  status_.warming_up = config_.startup.enable_imu_warmup && !imu_warmup_ready_;

  return true;
}

/**
 * @brief 启动 MappingRuntime
 *
 * @param req 启动请求（包含任务 ID、名称、输出目录等）
 * @param now_sec 当前时间戳（秒）
 * @return true 启动成功
 * @return false 启动失败
 *
 * 启动流程：
 * 1. 检查 SLAM 后端是否已创建
 * 2. 若请求重置，则调用 reset()
 * 3. 设置任务信息
 * 4. 设置回环检测状态
 * 5. 设置运行状态
 * 6. 记录启动时间
 * 7. 更新预热状态消息
 */
bool MappingRuntime::start(const RuntimeStartRequest& req, double now_sec) {
  if (!slam_) return false;

  // 若请求重置，先重置系统
  if (req.reset_before_start) reset();

  // 设置任务信息
  status_.task_id = req.task_id;
  status_.task_name = req.task_name;
  status_.output_dir = req.output_dir;

  // 设置回环检测状态
  status_.loop_enabled = req.enable_loop_closure;
  status_.loop_supported = loop_backend_.supported();

  // 设置运行状态
  status_.state = RuntimeState::RUNNING;
  status_.message = "running";
  status_.running = true;
  status_.saving = false;
  status_.output_valid = false;
  status_.start_time = now_sec;

  // 更新预热状态消息
  updateWarmupStatusMessage();

  return true;
}

/**
 * @brief 停止 MappingRuntime
 *
 * @param req 停止请求（包含是否保存地图等选项）
 * @return true 停止成功
 * @return false 停止失败
 *
 * 停止流程：
 * 1. 检查当前状态是否为 RUNNING 或 READY
 * 2. 设置状态为 STOPPING
 * 3. 设置运行标志为 false
 * 4. 若请求保存地图且配置允许，调用 save()
 * 5. 若不保存，直接设置为 FINISHED
 */
bool MappingRuntime::stop(const RuntimeStopRequest& req) {
  if (status_.state != RuntimeState::RUNNING && status_.state != RuntimeState::READY) return false;

  status_.state = RuntimeState::STOPPING;
  status_.message = "stopping";
  status_.running = false;

  // 若请求保存地图且配置允许
  if (req.save_map && config_.map_saver.save_on_stop) {
    SaveRequest save_req;
    save_req.output_dir = status_.output_dir;
    save_req.map_name = config_.map_saver.map_filename;
    save_req.resolution = config_.map_saver.default_resolution;
    save_req.save_keyframes = req.save_keyframes;
    save_req.save_keyframe_poses = req.save_keyframe_poses;
    save_req.save_trajectory = req.save_trajectory;
    save_req.save_loop_results = config_.map_saver.save_loop_results;
    return save(save_req);
  }

  // 不保存地图，直接完成
  status_.state = RuntimeState::FINISHED;
  status_.message = "finished";
  return true;
}

/**
 * @brief 保存地图和相关数据
 *
 * @param req 保存请求（包含输出目录、分辨率等）
 * @return true 保存成功
 * @return false 保存失败
 *
 * 保存内容：
 * - 关键帧点云
 * - 关键帧位姿
 * - 轨迹（path）
 * - 回环检测结果
 * - 子地图（当前和历史）
 */
bool MappingRuntime::save(const SaveRequest& req) {
  status_.state = RuntimeState::SAVING;
  status_.saving = true;
  status_.message = "saving";

  SaveResult result;

  // 调用 MapSaver 保存所有数据
  const bool ok = map_saver_.saveAll(req,
                                   keyframe_manager_.keyframes(),
                                   loop_pairs_,
                                   latest_loop_current_submap_.cloud,
                                   latest_loop_history_submap_.cloud,
                                   &result);

  // GNSS 启用时，导出 ENU->map 对齐参数，供离线叠加/坐标转换验证
  if (config_.loop_closure.gnss.enable) {
    Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
    Eigen::Vector3d trans = Eigen::Vector3d::Zero();
    double lat = 0.0, lon = 0.0, alt = 0.0;
    if (loop_backend_.gnssMapEnuTransform(&rot, &trans) &&
        loop_backend_.gnssEnuOrigin(&lat, &lon, &alt)) {
      map_saver_.saveGnssAlignment(status_.output_dir + "/gnss_alignment.txt",
                                   rot, trans, lat, lon, alt);
    }
  }

  // 更新状态
  status_.saving = false;
  status_.output_valid = ok;
  status_.map_path = result.map_path;
  status_.keyframe_dir = result.keyframe_dir;
  status_.keyframe_pose_path = result.keyframe_pose_path;
  status_.trajectory_path = result.trajectory_path;
  status_.loop_result_path = result.loop_result_path;
  status_.state = ok ? RuntimeState::FINISHED : RuntimeState::FAILED;
  status_.message = result.message;
  status_.running = false;
  status_.warming_up = false;

  return ok;
}

/**
 * @brief 重置 MappingRuntime 状态
 *
 * 重置流程：
 * 1. 重置 SLAM 后端
 * 2. 重置关键帧管理器
 * 3. 重置回环检测后端
 * 4. 清空当前数据
 * 5. 清空回环数据
 * 6. 重置 IMU 预热状态
 * 7. 重置运行时状态
 */
void MappingRuntime::reset() {
  if (slam_) slam_->reset();
  keyframe_manager_.reset();
  loop_backend_.reset();

  // 清空当前数据
  current_odom_ = localization::OdomResult();
  current_scan_ = localization::TimestampedPointCloud();
  local_map_ = localization::TimestampedPointCloud();

  // 清空回环数据
  latest_loop_current_submap_ = localization::TimestampedPointCloud();
  latest_loop_history_submap_ = localization::TimestampedPointCloud();
  loop_pairs_.clear();

  // 清空 GNSS 缓冲
  gnss_buffer_.clear();

  // 重置 IMU 预热状态
  first_imu_stamp_ = 0.0;
  last_imu_stamp_ = 0.0;
  imu_warmup_samples_ = 0;
  dropped_lidar_before_warmup_ = 0;
  imu_warmup_ready_ = !config_.startup.enable_imu_warmup;

  // 重置运行时状态
  status_ = MappingRuntimeStatus();
  status_.state = RuntimeState::READY;
  status_.message = "ready";
  status_.loop_supported = loop_backend_.supported();
  status_.warming_up = config_.startup.enable_imu_warmup && !imu_warmup_ready_;
}

/**
 * @brief 更新 IMU 预热状态
 *
 * @param stamp 当前 IMU 时间戳
 *
 * 预热逻辑：
 * 1. 记录首个和最后一个 IMU 时间戳
 * 2. 累积 IMU 样本数
 * 3. 检查是否满足预热条件：
 *    - 持续时间 >= 配置的预热时长
 *    - 样本数 >= 配置的最小样本数
 * 4. 若满足条件，标记预热完成
 */
void MappingRuntime::updateImuWarmup(double stamp) {
  if (first_imu_stamp_ <= 0.0) {
    first_imu_stamp_ = stamp;
  }
  last_imu_stamp_ = stamp;
  ++imu_warmup_samples_;

  // 若禁用预热或已就绪，直接返回
  if (!config_.startup.enable_imu_warmup || imu_warmup_ready_) {
    return;
  }

  // 检查预热条件
  const double duration = last_imu_stamp_ - first_imu_stamp_;
  if (duration >= config_.startup.imu_warmup_duration &&
      imu_warmup_samples_ >= static_cast<std::uint64_t>(std::max(1, config_.startup.min_imu_samples))) {
    imu_warmup_ready_ = true;
  }
}

/**
 * @brief 检查是否可以向 SLAM 后端输入 LiDAR 数据
 *
 * @return true 可以输入（禁用预热或预热已完成）
 * @return false 不可输入（预热中）
 */
bool MappingRuntime::canFeedLidar() const {
  return !config_.startup.enable_imu_warmup || imu_warmup_ready_;
}

/**
 * @brief 检查 IMU 预热是否完成
 *
 * @return true 预热完成或禁用预热
 * @return false 预热中
 */
bool MappingRuntime::warmupReady() const {
  return !config_.startup.enable_imu_warmup || imu_warmup_ready_;
}

/**
 * @brief 更新预热状态消息
 *
 * 更新状态信息：
 * - warming_up: 是否正在预热
 * - imu_warmup_samples: IMU 样本数
 * - dropped_lidar_before_warmup: 预热前丢弃的 LiDAR 帧数
 * - imu_warmup_duration_sec: 预热持续时间
 *
 * 若正在预热，生成状态消息字符串。
 */
void MappingRuntime::updateWarmupStatusMessage() {
  status_.warming_up = status_.running && !warmupReady();
  status_.imu_warmup_samples = imu_warmup_samples_;
  status_.dropped_lidar_before_warmup = dropped_lidar_before_warmup_;
  status_.imu_warmup_duration_sec =
      first_imu_stamp_ > 0.0 ? std::max(0.0, last_imu_stamp_ - first_imu_stamp_) : 0.0;

  if (status_.warming_up) {
    std::ostringstream oss;
    oss << "warming_up imu_samples=" << status_.imu_warmup_samples
        << " duration=" << status_.imu_warmup_duration_sec
        << " dropped_lidar=" << status_.dropped_lidar_before_warmup;
    status_.message = oss.str();
  }
}

/**
 * @brief 输入 IMU 数据
 *
 * @param imu IMU 数据
 *
 * 处理流程：
 * 1. 检查 SLAM 后端是否已创建
 * 2. 将 IMU 数据转发给 SLAM 后端
 * 3. 更新 IMU 预热状态
 * 4. 更新预热状态消息
 */
void MappingRuntime::feedImu(const localization::ImuData& imu) {
  if (!slam_) return;
  slam_->feedImu(imu);
  updateImuWarmup(imu.timestamp);
  updateWarmupStatusMessage();
}

/**
 * @brief 输入 LiDAR 点云数据
 *
 * @param cloud 时间戳点云数据
 *
 * 处理流程：
 * 1. 检查 SLAM 后端是否已创建
 * 2. 检查是否允许输入 LiDAR（预热完成）
 * 3. 若预热中，丢弃该帧并记录
 * 4. 若预热完成，将点云转发给 SLAM 后端
 */
void MappingRuntime::feedLidar(const localization::TimestampedPointCloud& cloud) {
  if (!slam_) return;

  // 若预热中，丢弃 LiDAR 数据
  if (!canFeedLidar()) {
    ++dropped_lidar_before_warmup_;
    updateWarmupStatusMessage();
    return;
  }

  slam_->feedLidar(cloud);
}

/**
 * @brief 输入轮速计数据
 * @param can 轮速计数据
 */
void MappingRuntime::feedCanVelocity(const localization::CanVelocityData& can) {
  if (slam_) slam_->feedCanVelocity(can);
}

/**
 * @brief 输入 GNSS 姿态数据
 * @param gnss GNSS 姿态数据
 */
void MappingRuntime::feedGnssAttitude(const localization::GnssAttitudeData& gnss) {
  if (slam_) slam_->feedGnssAttitude(gnss);
}

/**
 * @brief 输入 GNSS 位置观测（后端 GNSS 因子数据源）
 *
 * 仅做缓冲，不在此处做坐标转换或可靠性判断；
 * 与关键帧的时间戳对齐在 handleFrame 中完成。
 */
void MappingRuntime::feedGnss(const GnssObservation& gnss) {
  gnss_buffer_.push_back(gnss);
  // 缓冲上限保护：GNSS 频率通常 <= 100Hz，5000 条足够覆盖长时间无关键帧场景
  constexpr std::size_t kMaxGnssBufferSize = 5000;
  while (gnss_buffer_.size() > kMaxGnssBufferSize) {
    gnss_buffer_.pop_front();
  }
}

/**
 * @brief 将 GNSS 缓冲中与最新关键帧时间最接近的观测挂载到关键帧
 *
 * 对齐规则（与 LIO-SAM 一致）：
 * 1. 在缓冲中找 |t_gnss - t_kf| 最小的观测；
 * 2. 仅在时间差 <= max_age_sec 时接受，否则该关键帧不带 GNSS；
 * 3. 被消费的观测弹出（每个观测最多被一个关键帧使用）；
 * 4. 清理早于当前关键帧窗口的过期观测。
 */
void MappingRuntime::associateGnssToLatestKeyframe() {
  MappingKeyframe* kf = keyframe_manager_.latestMutable();
  if (!kf) return;

  const double max_age = config_.loop_closure.gnss.max_age_sec;
  auto best = gnss_buffer_.end();
  double best_dt = max_age;
  for (auto it = gnss_buffer_.begin(); it != gnss_buffer_.end(); ++it) {
    const double dt = std::abs(it->timestamp - kf->timestamp);
    if (dt <= best_dt) {
      best_dt = dt;
      best = it;
    }
  }

  if (best != gnss_buffer_.end()) {
    kf->gnss = *best;
    gnss_buffer_.erase(best);
  }

  // 弹出不会再被未来关键帧匹配到的过期观测
  while (!gnss_buffer_.empty() &&
         gnss_buffer_.front().timestamp < kf->timestamp - max_age) {
    gnss_buffer_.pop_front();
  }
}

/**
 * @brief 单次处理流程
 *
 * @return true 处理成功
 * @return false 处理失败或跳过
 *
 * 处理流程：
 * 1. 检查 SLAM 后端和运行状态
 * 2. 检查 IMU 预热是否完成
 * 3. 调用 SLAM 后端处理一帧
 * 4. 获取里程计、当前扫描、局部地图
 * 5. 检查数据有效性
 * 6. 调用 handleFrame 处理关键帧和回环检测
 */
bool MappingRuntime::processOnce() {
  if (!slam_ || status_.state != RuntimeState::RUNNING) return false;

  // 检查预热状态
  if (!warmupReady()) {
    updateWarmupStatusMessage();
    return false;
  }

  // 调用 SLAM 后端处理
  if (!slam_->processOnce()) return false;

  // 获取处理结果
  current_odom_ = slam_->getOdometry();
  current_scan_ = slam_->getCurrentScan();
  local_map_ = slam_->getLocalMap();

  // 检查数据有效性
  if (!current_odom_.valid || !current_scan_.cloud || current_scan_.cloud->empty()) return false;

  // 处理帧（关键帧管理和回环检测）
  return handleFrame(current_odom_, current_scan_, local_map_);
}

/**
 * @brief 处理单帧数据（关键帧管理和回环检测）
 *
 * @param odom 当前帧里程计结果
 * @param scan 当前帧点云
 * @param local_map 局部地图（未使用）
 * @return true 处理成功
 * @return false 处理失败
 *
 * 处理流程：
 * 1. 尝试添加关键帧
 * 2. 若添加成功且回环检测启用：
 *    - 调用回环检测后端更新
 *    - 获取回环对
 *    - 获取当前和历史子地图
 *    - 更新回环状态
 * 3. 更新帧状态信息
 * 4. 更新状态消息
 */
bool MappingRuntime::handleFrame(const localization::OdomResult& odom,
                                 const localization::TimestampedPointCloud& scan,
                                 const localization::TimestampedPointCloud&) {
  // 尝试添加关键帧
  const bool keyframe_added = keyframe_manager_.add(odom, scan);

  // 启用 GNSS 因子时，将时间对齐的 GNSS 观测挂载到新关键帧
  if (keyframe_added && config_.loop_closure.gnss.enable) {
    associateGnssToLatestKeyframe();
  }

  LoopClosureResult loop_result;

  // 若添加关键帧成功且回环检测或 GNSS 优化启用
  if (keyframe_added && (status_.loop_enabled || config_.loop_closure.gnss.enable)) {
    // 调用回环检测后端
    loop_result = loop_backend_.update(&keyframe_manager_.mutableKeyframes());
    loop_pairs_ = loop_result.loop_pairs;
    status_.loop_count = loop_result.loop_count;
    status_.gnss_factor_count = loop_result.gnss_factor_count;
    status_.gnss_rejected_count = loop_result.gnss_rejected_count;
    status_.gnss_aligned = loop_result.gnss_aligned;

    // 获取当前子地图
    if (auto current_submap = loop_backend_.latestCurrentSubmap()) {
      latest_loop_current_submap_.timestamp = odom.timestamp;
      latest_loop_current_submap_.frame_id = config_.frames.map;
      latest_loop_current_submap_.cloud = current_submap;
    }

    // 获取历史子地图
    if (auto history_submap = loop_backend_.latestHistorySubmap()) {
      latest_loop_history_submap_.timestamp = odom.timestamp;
      latest_loop_history_submap_.frame_id = config_.frames.map;
      latest_loop_history_submap_.cloud = history_submap;
    }

    // 更新回环对信息
    const auto latest_pair = loop_backend_.latestLoopPair();
    if (latest_pair.first >= 0 && latest_pair.second >= 0) {
      // 从回环后端获取最新回环对
      status_.last_loop_from = latest_pair.first;
      status_.last_loop_to = latest_pair.second;
    } else if (!loop_result.loop_pairs.empty()) {
      // 从回环结果获取最后一对
      status_.last_loop_from = loop_result.loop_pairs.back().first;
      status_.last_loop_to = loop_result.loop_pairs.back().second;
    }
  }

  // 更新回环状态
  status_.loop_updated_last_frame = loop_result.updated;
  status_.last_loop_score = loop_result.latest_score;

  // 更新帧状态
  updateStatusOnFrame(odom, scan);

  // 更新关键帧计数
  status_.keyframe_count = keyframe_manager_.keyframes().size();

  // 更新状态消息
  if (loop_result.updated && status_.last_loop_from >= 0 && status_.last_loop_to >= 0) {
    // 检测到回环
    status_.message = "loop_detected:" + std::to_string(status_.last_loop_from) + "->" +
                      std::to_string(status_.last_loop_to) + " score=" + std::to_string(status_.last_loop_score);
  } else if (keyframe_added && status_.loop_enabled && loop_result.latest_score >= 0.0) {
    // 回环被拒绝
    status_.message = "loop_rejected:score=" + std::to_string(loop_result.latest_score);
  } else if (status_.state == RuntimeState::RUNNING) {
    // 正常运行
    status_.message = "running";
  }

  return true;
}

/**
 * @brief 更新帧状态信息
 *
 * @param odom 当前帧里程计结果
 * @param scan 当前帧点云
 *
 * 更新内容：
 * - 帧计数
 * - 最后一帧时间
 * - 运行时长
 * - 累积点数
 * - 预热状态消息
 */
void MappingRuntime::updateStatusOnFrame(const localization::OdomResult& odom,
                                         const localization::TimestampedPointCloud& scan) {
  status_.frame_count += 1;
  status_.last_frame_time = odom.timestamp;
  status_.duration_sec = status_.start_time > 0.0 ? (odom.timestamp - status_.start_time) : 0.0;
  status_.accumulated_point_count += scan.cloud ? scan.cloud->size() : 0;
  updateWarmupStatusMessage();
}

// ==================== 查询接口 ====================

/**
 * @brief 获取当前运行时状态
 */
MappingRuntimeStatus MappingRuntime::status() const { return status_; }

/**
 * @brief 获取所有关键帧
 */
std::vector<MappingKeyframe> MappingRuntime::keyframes() const { return keyframe_manager_.keyframes(); }

/**
 * @brief 获取回环对列表
 */
std::vector<std::pair<int, int>> MappingRuntime::loopPairs() const { return loop_pairs_; }

/**
 * @brief 获取已添加 GNSS 因子的位置（map 系），供可视化
 */
std::vector<Eigen::Vector3d> MappingRuntime::gnssFactorPositions() const {
  return loop_backend_.gnssFactorPositions();
}

/**
 * @brief 获取被门控拒绝的 GNSS 位置（map 系），供可视化
 */
std::vector<Eigen::Vector3d> MappingRuntime::gnssRejectedPositions() const {
  return loop_backend_.gnssRejectedPositions();
}

std::vector<std::pair<int, Eigen::Vector3d>> MappingRuntime::gnssAcceptedFactors() const {
  return loop_backend_.gnssAcceptedFactors();
}

std::vector<std::pair<int, Eigen::Vector3d>> MappingRuntime::gnssRejectedFactors() const {
  return loop_backend_.gnssRejectedFactors();
}

bool MappingRuntime::gnssMapEnuTransform(Eigen::Matrix3d* rot,
                                         Eigen::Vector3d* trans) const {
  return loop_backend_.gnssMapEnuTransform(rot, trans);
}

bool MappingRuntime::gnssEnuOrigin(double* lat_deg, double* lon_deg,
                                   double* alt) const {
  return loop_backend_.gnssEnuOrigin(lat_deg, lon_deg, alt);
}

/**
 * @brief 获取当前里程计结果
 */
localization::OdomResult MappingRuntime::currentOdom() const { return current_odom_; }

/**
 * @brief 获取当前扫描点云
 */
localization::TimestampedPointCloud MappingRuntime::currentScan() const { return current_scan_; }

/**
 * @brief 获取局部地图
 */
localization::TimestampedPointCloud MappingRuntime::localMap() const { return local_map_; }

/**
 * @brief 获取最新回环当前子地图
 */
localization::TimestampedPointCloud MappingRuntime::latestLoopCurrentSubmap() const { return latest_loop_current_submap_; }

/**
 * @brief 获取最新回环历史子地图
 */
localization::TimestampedPointCloud MappingRuntime::latestLoopHistorySubmap() const { return latest_loop_history_submap_; }

}  // namespace slam_mapping

# ORB 视觉回环检测详细设计文档

> **版本**: v1.0
> **模块**: slam_mapping_v2 / VisualLoopClosure
> **设计原则**: 解耦接入、独立线程、接口兼容现有 ILoopDetector

---

## 目录

1. [背景与目标](#1-背景与目标)
2. [现有架构分析](#2-现有架构分析)
3. [整体架构设计](#3-整体架构设计)
4. [模块详细设计](#4-模块详细设计)
5. [接口设计](#5-接口设计)
6. [执行流程设计](#6-执行流程设计)
7. [线程模型设计](#7-线程模型设计)
8. [配置设计](#8-配置设计)
9. [解耦接入设计](#9-解耦接入设计)
10. [文件清单](#10-文件清单)
11. [集成步骤](#11-集成步骤)
12. [附录](#12-附录)

---

## 1. 背景与目标

### 1.1 现状

当前 slam_mapping_v2 的回环检测采用 **KDTree + GICP** 方案（`KdtreeGicpLoopDetector`）：
- 基于历史关键帧的位姿空间邻近搜索
- 子图 GICP 精配准验证
- **优点**：无需额外传感器，纯激光方案
- **缺点**：依赖几何特征，在重复几何结构（长走廊、对称结构）中易误检

### 1.2 目标

引入 **基于视觉 ORB 特征的回环检测**，作为现有激光回环的补充/替代：
- 利用视觉特征的高区分性，解决重复几何结构的误检问题
- **独立线程**提取 ORB 特征，不阻塞主流程
- 复用现有 `ILoopDetector` 接口，实现**零侵入式接入**
- 可通过配置开关在激光回环/视觉回环/混合回环之间切换

### 1.3 参考项目

- ORB-SLAM3 的 DBoW2 词袋检索 + Sim3 几何验证
- 关键源文件：`ORBextractor.cc`, `KeyFrameDatabase.cc`, `ORBmatcher.cc`, `LoopClosing.cc`

---

## 2. 现有架构分析

### 2.1 关键接口（已存在，无需修改）

```cpp
// i_loop_detector.hpp — 可插拔回环检测器接口
class ILoopDetector {
 public:
  virtual ~ILoopDetector() = default;
  virtual void configure(const LoopClosureConfig& cfg) = 0;
  virtual void reset() = 0;
  virtual LoopCandidate detect(const std::vector<Keyframe>& kfs) = 0;
  virtual const std::vector<std::pair<int, int>>& loopPairs() const = 0;
  virtual PointCloudPtr latestCurrentSubmap() const = 0;
  virtual PointCloudPtr latestHistorySubmap() const = 0;
};
```

### 2.2 现有数据类型

```cpp
struct Keyframe {
  int id = -1;
  double timestamp = 0.0;
  Pose pose_odom;
  Pose pose_map;
  PointCloudPtr cloud;          // 关键帧点云（odom系）
  bool loop_closed = false;
  int loop_match_count = 0;
  GnssObservation gnss;
};

struct LoopCandidate {
  bool valid = false;
  int history_idx = -1;
  int current_idx = -1;
  double score = 0.0;
  Mat3d diff_rot = Mat3d::Identity();
  Vec3d diff_pos = Vec3d::Zero();
};
```

### 2.3 现有调用链

```
MappingRuntime::backendLoop()
    ├── optimizer_.addOdomFactors(keyframes_)
    ├── gnss_.update(...)
    ├── detector_->detect(keyframes_)        ← 回环检测入口（ILoopDetector接口）
    ├── optimizer_.addLoopFactor(candidate)   ← 添加回环因子
    └── optimizer_.optimize(&keyframes_)      ← 位姿图优化（GTSAM ISAM2）
```

### 2.4 需要扩展的地方

| 问题 | 现状 | 解决方案 |
|------|------|----------|
| Keyframe 无图像 | 只有 `PointCloudPtr cloud` | 扩展可选字段，图像由 Buffer 管理 |
| LoopClosureConfig 无视觉参数 | 只有激光参数 | 新增 `VisualLoopConfig` 子结构 |
| 检测器硬编码为激光 | `make_unique<KdtreeGicpLoopDetector>()` | 改用工厂模式 |
| 无图像输入接口 | 只有 `feedOdom/feedScan/feedGnss` | 新增 `feedImage` |

---

## 3. 整体架构设计

### 3.1 架构总览

```
┌─────────────────────────────────────────────────────────────────────┐
│                           MappingRuntime                            │
│                                                                     │
│  主线程                   后端线程                   视觉特征线程    │
│  ┌──────────┐           ┌──────────────┐          ┌──────────────┐ │
│  │feedOdom()│           │backendLoop() │          │visualFeatLoop│ │
│  │feedScan()│─pending──▶│              │          │              │ │
│  │feedImage*│           │ ┌──────────┐ │          │ ORBextractor │ │
│  └──────────┘           │ │ILoopDet. │ │          │ DBoW2 trans  │ │
│                         │ └────┬─────┘ │          │              │ │
│                         │      │       │          └──────┬───────┘ │
│                         │      ▼       │                 │         │
│                         │ ┌──────────┐ │          ┌──────▼───────┐ │
│                         │ │VisualLoop│◀───────────│  VisualFeat  │ │
│                         │ │ Detector │ │          │    Buffer    │ │
│                         │ └────┬─────┘ │          └──────────────┘ │
│                         │      │       │                           │
│                         │      ▼       │                           │
│                         │ ┌──────────┐ │                           │
│                         │ │PoseGraph │ │                           │
│                         │ │Optimizer │ │                           │
│                         │ └──────────┘ │                           │
│                         └──────────────┘                           │
└─────────────────────────────────────────────────────────────────────┘

* = 新增接口
```

### 3.2 核心设计决策

| 设计决策 | 说明 |
|----------|------|
| **独立视觉特征线程** | ORB 特征提取计算量大（~10ms/帧），独立线程避免阻塞后端 |
| **实现 ILoopDetector** | 零侵入接入，后端循环无需修改核心逻辑 |
| **VisualFeatureBuffer** | 线程间通过缓冲区解耦，`shared_mutex` 读写分离 |
| **工厂模式创建检测器** | 配置驱动，运行时可切换 激光/视觉/混合 |
| **扩展 Keyframe** | 添加可选视觉字段，不破坏现有接口 |

---

## 4. 模块详细设计

### 4.1 视觉数据类型 (`types_visual.hpp`)

```cpp
#pragma once
#include "slam_mapping_v2/common/types.hpp"
#include <opencv2/core.hpp>
#include <vector>

namespace slam_mapping_v2 {

/// 视觉帧：包含图像和提取的 ORB 特征
struct VisualFrame {
  double timestamp = 0.0;
  cv::Mat image;                           // 原始灰度图像
  std::vector<cv::KeyPoint> keypoints;     // ORB 关键点
  cv::Mat descriptors;                     // ORB 描述子 (N x 32, CV_8U)
  int keyframe_id = -1;                    // 关联的关键帧 ID（通过时间戳匹配）
  bool features_ready = false;             // 特征是否已提取完成
};

/// BoW 检索结果
struct BowQueryResult {
  int keyframe_id = -1;
  double bow_score = 0.0;
  int shared_words = 0;
};

/// BoW 匹配中间结果
struct BoWMatchResult {
  std::vector<std::pair<int, int>> matches;  // (query_kp_idx, cand_kp_idx)
  int num_matches = 0;
};

}  // namespace slam_mapping_v2
```

### 4.2 视觉特征提供者接口 (`i_visual_feature_provider.hpp`)

```cpp
#pragma once
#include "slam_mapping_v2/common/types_visual.hpp"

namespace slam_mapping_v2 {

/// 视觉特征提供者接口（可插拔：ORB、SuperPoint 等）
class IVisualFeatureProvider {
 public:
  virtual ~IVisualFeatureProvider() = default;
  virtual void configure(const VisualLoopConfig& cfg) = 0;
  virtual VisualFrame extract(const cv::Mat& image,
                              double timestamp, int keyframe_id) = 0;
  virtual std::vector<VisualFrame> extractBatch(
      const std::vector<std::pair<cv::Mat, double>>& images,
      const std::vector<int>& keyframe_ids) = 0;
};

}  // namespace slam_mapping_v2
```

### 4.3 视觉特征缓冲区 (`visual_feature_buffer.hpp`)

```cpp
#pragma once
#include "slam_mapping_v2/common/types_visual.hpp"
#include <map>
#include <mutex>
#include <shared_mutex>
#include <condition_variable>

namespace slam_mapping_v2 {

/**
 * @brief 线程安全的视觉特征缓冲区
 *
 * 生产者：视觉特征线程（insert → feat_buffer_）
 * 消费者：后端线程（get → VisualLoopDetector::detect）
 */
class VisualFeatureBuffer {
 public:
  void insert(const VisualFrame& frame);
  void insertBatch(const std::vector<VisualFrame>& frames);
  bool get(int keyframe_id, VisualFrame* out) const;
  std::vector<VisualFrame> getBatch(const std::vector<int>& ids) const;
  std::vector<VisualFrame> getRecent(int n) const;
  std::vector<int> availableIds() const;
  bool waitForNew(int timeout_ms = 100);
  void clear();
  std::size_t size() const;

 private:
  mutable std::shared_mutex mutex_;
  std::map<int, VisualFrame> frames_;  // keyframe_id → VisualFrame
  std::condition_variable_any cv_;
  int latest_id_ = -1;
};

}  // namespace slam_mapping_v2
```

### 4.4 视觉回环检测器 (`visual_loop_detector.hpp`)

```cpp
#pragma once
#include "slam_mapping_v2/core/i_loop_detector.hpp"
#include "slam_mapping_v2/core/visual_feature_buffer.hpp"
#include <DBoW2/DBoW2.h>
#include <DBoW2/FORB.h>

namespace slam_mapping_v2 {

/**
 * @brief 基于视觉 ORB 特征的回环检测器
 *
 * 完整实现 ILoopDetector 接口，与现有后端循环完全兼容。
 * 检测流程：BoW检索 → 特征匹配 → 几何验证 → 连续性确认 → 返回 LoopCandidate
 */
class VisualLoopDetector : public ILoopDetector {
 public:
  VisualLoopDetector() = default;
  ~VisualLoopDetector() override = default;

  // ===== ILoopDetector 接口（完整实现） =====
  void configure(const LoopClosureConfig& cfg) override;
  void reset() override;
  LoopCandidate detect(const std::vector<Keyframe>& kfs) override;
  const std::vector<std::pair<int, int>>& loopPairs() const override { return loop_history_; }
  PointCloudPtr latestCurrentSubmap() const override { return nullptr; }
  PointCloudPtr latestHistorySubmap() const override { return nullptr; }

  // ===== 视觉特有接口 =====
  void setFeatureBuffer(std::shared_ptr<VisualFeatureBuffer> buffer);
  const BoWMatchResult& latestMatchResult() const { return latest_match_; }

 private:
  bool shouldCheck(const std::vector<Keyframe>& kfs, int current_idx) const;
  void computeBowVector(const VisualFrame& frame,
                        DBoW2::BowVector* bow_vec,
                        DBoW2::FeatureVector* feat_vec);
  std::vector<BowQueryResult> retrieveCandidates(
      const DBoW2::BowVector& query_bow,
      const std::vector<Keyframe>& kfs,
      int current_idx, double min_score);
  BoWMatchResult matchFeatures(const VisualFrame& query,
                               const VisualFrame& candidate,
                               const DBoW2::FeatureVector& query_fvec,
                               const DBoW2::FeatureVector& cand_fvec);
  bool geometricVerification(const VisualFrame& query,
                             const VisualFrame& candidate,
                             const BoWMatchResult& matches,
                             Mat3d* out_rot, Vec3d* out_pos, int* out_inliers);
  bool continuityCheck(int current_idx, int history_idx);

  LoopClosureConfig cfg_;
  VisualLoopConfig vcfg_;
  std::shared_ptr<VisualFeatureBuffer> feature_buffer_;
  std::unique_ptr<DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>> voc_;
  KeyFrameDatabase* kf_database_ = nullptr;

  std::vector<std::pair<int, int>> loop_history_;
  int last_check_index_ = -1;
  int pending_history_idx_ = -1;
  int continuity_count_ = 0;
  BoWMatchResult latest_match_;
};

}  // namespace slam_mapping_v2
```

### 4.5 回环检测器工厂 (`loop_detector_factory.hpp`)

```cpp
#pragma once
#include "slam_mapping_v2/core/i_loop_detector.hpp"
#include "slam_mapping_v2/common/config.hpp"
#include <memory>

namespace slam_mapping_v2 {

enum class LoopDetectorType {
  LASER_KDTREE_GICP,   // 现有激光方案
  VISUAL_ORB_BOW,      // 视觉 ORB + DBoW2
  HYBRID,              // 混合（激光 + 视觉）
};

class LoopDetectorFactory {
 public:
  static std::unique_ptr<ILoopDetector> create(
      LoopDetectorType type, const LoopClosureConfig& cfg);
  static std::unique_ptr<ILoopDetector> create(
      const std::string& type_str, const LoopClosureConfig& cfg);
};

}  // namespace slam_mapping_v2
```

---

## 5. 接口设计

### 5.1 新增输入接口

```cpp
// mapping_runtime.hpp 中新增
class MappingRuntime {
 public:
  // 现有接口（feedOdom/feedScan/feedGnss）不变
  void feedOdom(const FrameInput& odom);
  void feedScan(const FrameInput& scan);
  void feedGnss(const GnssObservation& gnss);

  /// 新增：喂入图像帧（供视觉特征提取线程消费）
  void feedImage(const cv::Mat& image, double timestamp);

 private:
  // 现有成员不变 ...

  // ===== 新增 =====
  void visualFeatLoop();
  std::thread visual_feat_thread_;

  std::deque<std::pair<cv::Mat, double>> image_queue_;
  std::mutex image_mutex_;
  std::condition_variable image_cv_;

  std::shared_ptr<VisualFeatureBuffer> feat_buffer_;
  std::unique_ptr<IVisualFeatureProvider> feat_provider_;
};
```

### 5.2 ILoopDetector 接口兼容性

视觉回环检测器完整实现现有接口，**接口零修改**：

| 接口方法 | 视觉回环实现 |
|----------|-------------|
| `configure(cfg)` | 加载 DBoW2 词库 + 配置 ORB 参数 |
| `reset()` | 清空倒排索引 + 特征缓冲区 + 历史 |
| `detect(kfs)` | BoW检索 → 匹配 → 几何验证 → 返回 `LoopCandidate` |
| `loopPairs()` | 返回历史回环对列表 |
| `latestCurrentSubmap()` | 返回 `nullptr`（可扩展为当前图像） |
| `latestHistorySubmap()` | 返回 `nullptr`（可扩展为匹配帧图像） |

### 5.3 扩展配置接口

```cpp
// config.hpp 中新增
struct VisualLoopConfig {
  // 词库
  std::string vocabulary_path = "";    // ORBvoc.bin 路径

  // ORB 特征
  int nfeatures = 2000;
  float scale_factor = 1.2f;
  int nlevels = 8;
  int ini_th_fast = 20;
  int min_th_fast = 7;

  // 检索
  double min_bow_score = 0.03;
  int min_shared_words = 20;
  int max_candidates = 3;

  // 匹配
  int min_bow_matches = 50;
  double nn_ratio = 0.6;
  int ransac_min_inliers = 15;
  double ransac_probability = 0.99;
  int ransac_max_iterations = 300;

  // 连续性验证
  int min_continuity = 3;
  int max_continuity_miss = 2;

  // 线程
  int feature_extract_rate_hz = 10;
  int buffer_size = 100;
};

// LoopClosureConfig 中新增字段
struct LoopClosureConfig {
  // 现有字段全部保留 ...
  std::string detector_type = "laser";     // "laser" | "visual" | "hybrid"
  VisualLoopConfig visual;
};
```

---

## 6. 执行流程设计

### 6.1 整体数据流

```
传感器输入                 特征提取                回环检测              优化输出
──────────────           ──────────             ──────────            ──────────

[相机图像] ──┐           ┌─────────────┐       ┌─────────────┐
             ├─feedImage │ 视觉特征线程 │       │  后端线程    │
[激光雷达] ─┤           │ ORBextractor│       │ VisualLoop  │
[里程计]   ──┘           │ DBoW2 trans │       │ Detector    │
                        └──────┬──────┘       │ ┌─────────┐│    ┌───────────┐
                               │               │ │BoW检索   ││    │PoseGraph  │
                               ▼               │ │特征匹配  ││    │Optimizer  │
                        ┌─────────────┐       │ │几何验证  │├───▶│  (ISAM2)  │
                        │ VisualFeat  │──────▶│ │连续性验证││    └─────┬─────┘
                        │   Buffer    │       │ └─────────┘│          │
                        └─────────────┘       └─────────────┘          ▼
                                                                    [优化后位姿]
```

### 6.2 视觉特征提取流程（独立线程）

```
visualFeatLoop() 主循环：
│
├─ 1. 等待图像到达
│     image_cv_.wait_for(lock, 100ms, [&]{ return !image_queue_.empty() || stop_threads_; })
│
├─ 2. 取出队列中的图像
│     batch.swap(image_queue_)
│
├─ 3. 通过时间戳匹配关键帧 ID
│     kf_id = findClosestKeyframeId(ts)   // 在 keyframes_ 中二分查找
│
├─ 4. 提取 ORB 特征
│     frame = feat_provider_->extract(image, ts, kf_id)
│
├─ 5. 计算 BoW 向量
│     computeBowVector(frame, &bow_vec, &feat_vec)
│
├─ 6. 写入特征缓冲区
│     feat_buffer_->insert(frame)
│
└─ 7. 更新倒排索引
      kf_database_->add(frame.keyframe_id, bow_vec)
```

### 6.3 回环检测流程（后端线程）

```
detect(kfs) 调用流程：
│
├─ 步骤1：前置检查
│   └── shouldCheck(kfs, current_idx)
│       ├── 间隔检查：current_idx - last_check_index_ >= loop_kf_gap
│       ├── 关键帧数检查：kfs.size() >= min_index_gap
│       └── 特征就绪检查：feature_buffer_->get(current_id, &frame)
│
├─ 步骤2：计算 BoW 向量
│   └── computeBowVector(frame, &bow_vec, &feat_vec)
│       └── voc_->transform(frame.descriptors, bow_vec, feat_vec, 4)
│
├─ 步骤3：候选帧检索
│   └── retrieveCandidates(bow_vec, kfs, current_idx, min_bow_score)
│       ├── 倒排索引查找：遍历 bow_vec 中每个 word_id → inverted_file
│       ├── 过滤共视帧：排除最近的 min_index_gap 个帧
│       ├── 统计共享单词数 → 过滤 < min_shared_words 的
│       ├── 计算精确 BoW 分数（L1_NORM）
│       └── 返回 Top-N 候选
│
├─ 步骤4：对每个候选帧验证
│   ├── 4a. BoW 特征匹配
│   │   └── matchFeatures(query, candidate, feat_vec)
│   │       ├── 双指针遍历 FeatureVector（同节点内比较）
│   │       ├── Hamming 距离 + Lowe's ratio test (0.6)
│   │       └── 旋转一致性过滤（30-bin 直方图，保留 top-3）
│   │
│   ├── 4b. 几何验证
│   │   └── geometricVerification(query, candidate, matches, ...)
│   │       ├── 匹配数 < ransac_min_inliers → 失败
│   │       ├── cv::findFundamentalMat + RANSAC
│   │       ├── 计算内点数 → < ransac_min_inliers → 失败
│   │       └── 输出 diff_rot, diff_pos（从 F 恢复 R,t）
│   │
│   └── 4c. 连续性验证
│       └── continuityCheck(current_idx, history_idx)
│           ├── 如果 history_idx == pending_history_idx_ → continuity_count_++
│           ├── 否则 → 重置为1，更新 pending_history_idx_
│           ├── continuity_count_ >= min_continuity → 确认回环
│           └── 连续 max_continuity_miss 帧未找到 → 重置
│
└─ 步骤5：返回 LoopCandidate
    ├── valid = continuity_count_ >= min_continuity
    ├── history_idx, current_idx
    ├── score = bow_score
    └── diff_rot, diff_pos
```

### 6.4 MappingRuntime 修改点

```cpp
// initialize() 修改：使用工厂创建检测器
bool MappingRuntime::initialize(const MappingConfig& cfg) {
  cfg_ = cfg;
  kf_manager_.configure(cfg_.keyframe);
  global_map_.configure(cfg_.map);
  gnss_.configure(cfg_.loop_closure.gnss);
  optimizer_.configure(cfg_.loop_closure);
  map_saver_.configure(cfg_.map_saver);

  // ===== 修改前 =====
  // detector_ = std::make_unique<KdtreeGicpLoopDetector>();
  // detector_->configure(cfg_.loop_closure);

  // ===== 修改后 =====
  detector_ = LoopDetectorFactory::create(
      cfg_.loop_closure.detector_type, cfg_.loop_closure);

  // 如果是视觉检测器，注入特征缓冲区
  if (auto* vd = dynamic_cast<VisualLoopDetector*>(detector_.get())) {
    feat_buffer_ = std::make_shared<VisualFeatureBuffer>();
    vd->setFeatureBuffer(feat_buffer_);
  }

  // ... 状态初始化不变 ...
}

// startThreads() 修改：条件启动视觉线程
void MappingRuntime::startThreads() {
  // 后端线程（不变）
  if (!backend_thread_.joinable()) {
    stop_threads_ = false;
    backend_thread_ = std::thread(&MappingRuntime::backendLoop, this);
  }

  // ===== 新增：条件启动视觉特征线程 =====
  if ((cfg_.loop_closure.detector_type == "visual" ||
       cfg_.loop_closure.detector_type == "hybrid") &&
      !visual_feat_thread_.joinable()) {
    feat_provider_ = std::make_unique<OrbFeatureProvider>();
    feat_provider_->configure(cfg_.loop_closure.visual);
    visual_feat_thread_ = std::thread(&MappingRuntime::visualFeatLoop, this);
  }
}

// stopThreads() 修改：同时停止视觉线程
void MappingRuntime::stopThreads() {
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    stop_threads_ = true;
  }
  pending_cv_.notify_all();
  image_cv_.notify_all();               // 新增：唤醒视觉线程
  if (backend_thread_.joinable()) backend_thread_.join();
  if (visual_feat_thread_.joinable()) visual_feat_thread_.join();  // 新增
}
```

---

## 7. 线程模型设计

### 7.1 线程职责与锁分配

| 线程 | 职责 | 涉及的锁 |
|------|------|----------|
| **主线程** | 接收 odom/scan/image 输入，关键帧判定，全局地图插入 | `input_mutex_`, `pending_mutex_`, `map_mutex_` |
| **视觉特征线程** | 从 image_queue_ 取图，提取 ORB 特征，计算 BoW，写入 Buffer | `image_mutex_`(读), `feat_buffer_`(写) |
| **后端线程** | 合并关键帧，回环检测，GNSS优化，位姿图优化 | `kf_mutex_`(写), `feat_buffer_`(读), `status_mutex_` |

### 7.2 线程间通信

```
主线程 ────────▶ [image_queue_] ────────▶ 视觉特征线程
                  (image_mutex_ + image_cv_)

视觉特征线程 ──▶ [VisualFeatureBuffer] ──▶ 后端线程
                   (shared_mutex: 写锁/读锁分离)

主线程 ────────▶ [pending_queue_] ────────▶ 后端线程
                  (pending_mutex_ + pending_cv_)
```

### 7.3 关键帧 ID 关联策略

**推荐方案：时间戳关联**

```cpp
void MappingRuntime::visualFeatLoop() {
  while (!stop_threads_) {
    std::vector<std::pair<cv::Mat, double>> batch;
    {
      std::unique_lock<std::mutex> lock(image_mutex_);
      image_cv_.wait_for(lock, std::chrono::milliseconds(100),
                         [this]() { return stop_threads_ || !image_queue_.empty(); });
      if (stop_threads_) break;
      batch.swap(image_queue_);
    }
    if (batch.empty()) continue;

    for (auto& [image, ts] : batch) {
      // 通过时间戳查找最近的关键帧 ID
      int kf_id = -1;
      {
        std::shared_lock<std::shared_mutex> kf_lock(kf_mutex_);
        for (int i = static_cast<int>(keyframes_.size()) - 1; i >= 0; --i) {
          if (std::abs(keyframes_[i].timestamp - ts) < 0.05) {  // 50ms 容差
            kf_id = i;
            break;
          }
        }
      }

      if (kf_id >= 0) {
        auto frame = feat_provider_->extract(image, ts, kf_id);
        feat_buffer_->insert(frame);
      }
    }
  }
}
```

---

## 8. 配置设计

### 8.1 YAML 配置文件扩展

```yaml
# slam_mapping_v2/config/mapping.yaml

loop_closure:
  enable: true

  # ===== 新增：检测器类型选择 =====
  detector_type: "visual"     # "laser" | "visual" | "hybrid"

  # ===== 现有激光配置（全部保留） =====
  min_index_gap: 20
  time_thresh_sec: 20.0
  loop_kf_gap: 30
  min_id_interval: 20
  closest_id_th: 50
  max_range: 20.0
  submap_search_num: 25
  submap_resolution: 0.3
  icp_fitness_thresh: 0.3
  loop_noise_scale: 1.0
  loop_rot_noise: 0.05
  loop_pos_noise: 0.5
  loop_score_floor: 0.05
  odom_rot_noise: 1e-6
  odom_pos_noise: 1e-4

  # ===== 新增：视觉配置 =====
  visual:
    vocabulary_path: "/home/hhy/2004ros/ORB_SLAM3/Vocabulary/ORBvoc.bin"

    # ORB 特征
    nfeatures: 2000
    scale_factor: 1.2
    nlevels: 8
    ini_th_fast: 20
    min_th_fast: 7

    # 检索
    min_bow_score: 0.03
    min_shared_words: 20
    max_candidates: 3

    # 匹配
    min_bow_matches: 50
    nn_ratio: 0.6
    ransac_min_inliers: 15
    ransac_probability: 0.99
    ransac_max_iterations: 300

    # 连续性验证
    min_continuity: 3
    max_continuity_miss: 2

    # 线程
    feature_extract_rate_hz: 10
    buffer_size: 100

  # ===== GNSS 配置（不变） =====
  gnss:
    enable: true
```

---

## 9. 解耦接入设计

### 9.1 侵入性分析

| 修改文件 | 修改内容 | 侵入程度 |
|----------|----------|----------|
| `config.hpp` | 新增 `VisualLoopConfig` 结构体 + `LoopClosureConfig` 新字段 | **低**（仅新增，不改现有） |
| `types.hpp` | `Keyframe` 新增 3 个可选字段 | **低**（新增字段有默认值） |
| `mapping_runtime.hpp` | 新增 `feedImage()`、视觉线程、缓冲区 | **中**（新增成员和方法） |
| `mapping_runtime.cpp` | `initialize()` 改用工厂，新增 `visualFeatLoop()` | **中**（核心文件需修改） |
| `config_loader.cpp` | 新增视觉配置加载 | **低** |
| `i_loop_detector.hpp` | **不修改** | **无** |
| `pose_graph_optimizer.hpp` | **不修改** | **无** |
| `keyframe_manager.hpp` | **不修改** | **无** |

### 9.2 解耦策略

**策略1：配置驱动（工厂模式）**

```cpp
// initialize() 中仅改一行
// 改前：detector_ = std::make_unique<KdtreeGicpLoopDetector>();
// 改后：
detector_ = LoopDetectorFactory::create(cfg_.loop_closure.detector_type, cfg_.loop_closure);
```

**策略2：条件启动视觉线程**

```cpp
// 仅当 detector_type == "visual" 或 "hybrid" 时启动视觉线程
// 使用 "laser" 时完全不影响现有流程
```

**策略3：动态类型检查注入缓冲区**

```cpp
if (auto* vd = dynamic_cast<VisualLoopDetector*>(detector_.get())) {
  vd->setFeatureBuffer(feat_buffer_);
}
```

### 9.3 Keyframe 扩展（向前兼容）

```cpp
struct Keyframe {
  // ... 现有字段全部保留 ...

  // 新增（可选，不影响现有代码）
  bool has_visual = false;           // 是否包含视觉数据
  double image_timestamp = 0.0;      // 关联的图像时间戳
  int visual_feature_count = 0;      // ORB 特征点数量
};
```

---

## 10. 文件清单

### 10.1 新增文件

| 文件路径 | 行数估算 | 说明 |
|----------|----------|------|
| `include/.../common/types_visual.hpp` | ~80 | 视觉数据类型 |
| `include/.../core/i_visual_feature_provider.hpp` | ~40 | 特征提供者接口 |
| `include/.../core/orb_feature_provider.hpp` | ~50 | ORB 提取器头文件 |
| `include/.../core/visual_feature_buffer.hpp` | ~80 | 特征缓冲区头文件 |
| `include/.../core/visual_loop_detector.hpp` | ~120 | 视觉回环检测器头文件 |
| `include/.../core/loop_detector_factory.hpp` | ~40 | 工厂头文件 |
| `src/core/orb_feature_provider.cpp` | ~100 | ORB 提取器实现 |
| `src/core/visual_feature_buffer.cpp` | ~120 | 缓冲区实现 |
| `src/core/visual_loop_detector.cpp` | ~400 | 视觉回环检测器实现 |
| `src/core/loop_detector_factory.cpp` | ~50 | 工厂实现 |
| `thirdparty/DBoW2/` | ~2000 | 从 ORB-SLAM3 移植 |
| `thirdparty/ORB_SLAM3_extract/` | ~1500 | 精简的 ORB 提取器 |

### 10.2 修改文件

| 文件路径 | 修改内容 |
|----------|----------|
| `include/.../common/config.hpp` | 新增 `VisualLoopConfig` + `LoopClosureConfig` 新字段 |
| `include/.../common/types.hpp` | `Keyframe` 新增 3 个可选字段 |
| `include/.../core/mapping_runtime.hpp` | 新增 `feedImage()`, 视觉线程成员, 缓冲区 |
| `src/core/mapping_runtime.cpp` | `initialize()` 工厂化, `startThreads()` 条件启动, 新增 `visualFeatLoop()` |
| `src/ros/config_loader.cpp` | 新增视觉配置加载 |
| `CMakeLists.txt` | 新增 DBoW2、ORB 提取器编译 |

---

## 11. 集成步骤

### Phase 1：移植第三方库（~2天）

```
1. 从 ORB_SLAM3/Thirdparty/DBoW2 复制到 slam_mapping_v2/thirdparty/DBoW2
2. 精简 ORBextractor 到 thirdparty/ORB_SLAM3_extract
3. 修改 CMakeLists.txt，编译第三方库
4. 单元测试：验证 ORB 提取和 BoW 转换
```

### Phase 2：实现视觉特征模块（~2天）

```
1. 实现 types_visual.hpp
2. 实现 VisualFeatureBuffer（单元测试线程安全性）
3. 实现 OrbFeatureProvider（单元测试特征提取）
```

### Phase 3：实现视觉回环检测器（~3天）

```
1. 实现 VisualLoopDetector::detect() 完整流程
2. 实现 BoW 检索 + 特征匹配 + 几何验证
3. 实现连续性验证
4. 离线数据测试：验证回环检测准确率
```

### Phase 4：集成到 MappingRuntime（~2天）

```
1. 扩展 config.hpp 和 types.hpp
2. 修改 mapping_runtime.cpp（工厂化 + 视觉线程）
3. 端到端测试：激光建图 + 视觉回环
4. 性能测试：确认视觉线程不影响主流程
```

### Phase 5：混合回环（可选，~1天）

```
1. 实现 HybridLoopDetector
2. 配置权重策略
3. 对比测试：单一方案 vs 混合方案
```

---

## 12. 附录

### 12.1 ORB-SLAM3 → DeepLumin 类映射

| ORB-SLAM3 类 | DeepLumin 对应 | 作用 |
|-------------|----------------|------|
| `ORBextractor` | `OrbFeatureProvider` | ORB 特征提取 |
| `TemplatedVocabulary` | `VisualLoopDetector` 内部 | 词库管理 |
| `KeyFrameDatabase` | `VisualLoopDetector` 内部 | 倒排索引 |
| `ORBmatcher::SearchByBoW` | `matchFeatures()` | BoW 加速匹配 |
| `Sim3Solver` | `geometricVerification()` | 几何验证 |
| `LoopClosing` | `VisualLoopDetector::detect()` | 完整检测流程 |

### 12.2 性能估算

| 操作 | 耗时 | 线程 |
|------|------|------|
| ORB 特征提取 (2000点) | ~10ms | 视觉线程 |
| BoW 向量计算 | ~1ms | 视觉线程 |
| 倒排索引检索 | ~2ms | 后端线程 |
| BoW 特征匹配 | ~5ms | 后端线程 |
| 几何验证 (RANSAC) | ~10ms | 后端线程 |
| **总计** | **~20ms/帧** | 后端线程内 |

### 12.3 内存估算

| 组件 | 内存占用 |
|------|----------|
| ORB 词库 (100k 单词) | ~30MB |
| 特征缓冲区 (100帧) | ~50MB |
| 倒排索引 | ~10MB |
| **总计** | **~90MB** |

### 12.4 设计决策记录

| 决策 | 选项 | 选择 | 理由 |
|------|------|------|------|
| 特征提取线程 | 同步/独立 | **独立** | 不阻塞后端 |
| 图像存储位置 | KF内/Buffer | **Buffer** | 避免 KF 过大 |
| 帧关联方式 | ID预分配/时间戳 | **时间戳** | KF 判定是动态的 |
| 检测器切换 | 硬编码/工厂 | **工厂** | 配置驱动 |
| 词库格式 | txt/bin | **bin** | 加载快 100 倍 |
| 回环验证 | 单帧/连续性 | **连续性** | 降低误检率 |

---

## 13. 设计细节补充

### 13.1 DBoW2 词库来源与管理

**词库来源**：直接复用 ORB-SLAM3 预训练词库

```bash
# 词库文件位置
/home/hhy/2004ros/ORB_SLAM3/Vocabulary/ORBvoc.bin

# 复制到项目 thirdparty 目录（推荐）
cp /home/hhy/2004ros/ORB_SLAM3/Vocabulary/ORBvoc.bin \
   slam_mapping_v2/thirdparty/DBoW2/vocabulary/ORBvoc.bin
```

**配置方式**：

```yaml
# config/mapping.yaml
loop_closure:
  visual:
    # 方式1：绝对路径
    vocabulary_path: "/home/hhy/2004ros/ORB_SLAM3/Vocabulary/ORBvoc.bin"
    # 方式2：相对路径（相对于 launch 文件所在目录）
    # vocabulary_path: "$(find slam_mapping_v2)/thirdparty/DBoW2/vocabulary/ORBvoc.bin"
```

**词库加载异常处理**：

```cpp
// visual_loop_detector.cpp
void VisualLoopDetector::configure(const LoopClosureConfig& cfg) {
  vcfg_ = cfg.visual;

  // 词库加载失败处理
  if (vcfg_.vocabulary_path.empty()) {
    ROS_ERROR("[VisualLoopDetector] vocabulary_path is empty!");
    throw std::runtime_error("VisualLoopDetector: vocabulary_path not configured");
  }

  voc_ = std::make_unique<DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>>();

  try {
    voc_->loadFromBinaryFile(vcfg_.vocabulary_path);
    ROS_INFO("[VisualLoopDetector] Loaded vocabulary from %s (%d words, %d levels)",
             vcfg_.vocabulary_path.c_str(), voc_->size(), voc_->getDepthLevels());
  } catch (const std::exception& e) {
    ROS_ERROR("[VisualLoopDetector] Failed to load vocabulary: %s", e.what());
    throw;
  }

  kf_database_ = new DBoW2::KeyFrameDatabase(*voc_);
}
```

### 13.2 相机内参配置

**问题说明**：几何验证从 Fundamental 矩阵恢复 R,t 需要相机内参 K：

```
Essential Matrix: E = K^T * F * K
从 E 分解得到 R, t
```

**配置扩展**（在 `VisualLoopConfig` 中新增）：

```cpp
// config.hpp
struct VisualLoopConfig {
  // ... 现有字段 ...

  // 相机内参（几何验证必需）
  struct CameraIntrinsic {
    double fx = 500.0;    // 焦距 x
    double fy = 500.0;    // 焦距 y
    double cx = 320.0;    // 主点 x
    double cy = 240.0;    // 主点 y
  } camera;
};
```

**YAML 配置**：

```yaml
loop_closure:
  visual:
    # ... 现有配置 ...
    camera:
      fx: 500.0
      fy: 500.0
      cx: 320.0
      cy: 240.0
```

**几何验证实现修正**：

```cpp
bool VisualLoopDetector::geometricVerification(
    const VisualFrame& query, const VisualFrame& candidate,
    const BoWMatchResult& matches,
    Mat3d* out_rot, Vec3d* out_pos, int* out_inliers) {

  if (matches.num_matches < vcfg_.ransac_min_inliers) return false;

  // 提取匹配点对
  std::vector<cv::Point2f> pts1, pts2;
  for (const auto& [qi, ci] : matches.matches) {
    pts1.push_back(query.keypoints[qi].pt);
    pts2.push_back(candidate.keypoints[ci].pt);
  }

  // RANSAC 求 Fundamental 矩阵
  cv::Mat mask;
  cv::Mat F = cv::findFundamentalMat(pts1, pts2, cv::FM_RANSAC,
                                      3.0, vcfg_.ransac_probability, mask);

  if (F.empty()) return false;

  // 统计内点数
  int inliers = cv::countNonZero(mask);
  if (inliers < vcfg_.ransac_min_inliers) return false;

  // 从 Fundamental 恢复 R, t（需要相机内参）
  cv::Mat K = (cv::Mat_<double>(3, 3) <<
    vcfg_.camera.fx, 0, vcfg_.camera.cx,
    0, vcfg_.camera.fy, vcfg_.camera.cy,
    0, 0, 1);

  cv::Mat E = K.t() * F * K;  // Essential 矩阵

  cv::Mat R, t;
  cv::recoverPose(E, pts1, pts2, K, R, t, mask);

  // 输出结果
  *out_rot = Eigen::Map<Eigen::Matrix3d>(R.ptr<double>()).transpose();
  *out_pos = Eigen::Map<Eigen::Vector3d>(t.ptr<double>());
  *out_inliers = inliers;

  return true;
}
```

### 13.3 图像输入话题配置

**新增配置字段**：

```cpp
// config.hpp
struct InputConfig {
  std::string odom_topic = "/localization/slam_odom";
  std::string scan_topic = "/localization/cloud_registered";
  std::string gnss_topic = "/gnss_chc_data";
  std::string image_topic = "/camera/image_raw";  // 新增：相机图像话题
};
```

**YAML 配置**：

```yaml
input:
  odom_topic: "/localization/slam_odom"
  scan_topic: "/localization/cloud_registered"
  gnss_topic: "/gnss_chc_data"
  image_topic: "/camera/image_raw"  # 新增
```

**ROS 节点订阅**（在 node 文件中）：

```cpp
// slam_mapping_v2_node.cpp
class SlamMappingV2Node {
 public:
  SlamMappingV2Node(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    // ... 现有订阅 ...

    // 新增：图像订阅（仅视觉/混合模式）
    if (cfg_.loop_closure.detector_type == "visual" ||
        cfg_.loop_closure.detector_type == "hybrid") {
      image_sub_ = nh.subscribe(cfg_.input.image_topic, 10,
                                &SlamMappingV2Node::imageCallback, this);
      ROS_INFO("[SlamMappingV2] Subscribing to image topic: %s",
               cfg_.input.image_topic.c_str());
    }
  }

 private:
  void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    cv::Mat image;
    try {
      image = cv_bridge::toCvShare(msg, "mono8")->image;
    } catch (const cv_bridge::Exception& e) {
      ROS_ERROR("[SlamMappingV2] cv_bridge exception: %s", e.what());
      return;
    }
    runtime_.feedImage(image, msg->header.stamp.toSec());
  }

  ros::Subscriber image_sub_;
};
```

### 13.4 VisualFeatureBuffer 淘汰机制

**问题**：`buffer_size = 100` 未定义淘汰策略，长期运行会内存溢出。

**实现方案**：基于 keyframe_id 的 FIFO 淘汰

```cpp
// visual_feature_buffer.cpp
void VisualFeatureBuffer::insert(const VisualFrame& frame) {
  std::unique_lock<std::shared_mutex> lock(mutex_);

  // 插入新帧
  frames_[frame.keyframe_id] = frame;
  latest_id_ = std::max(latest_id_, frame.keyframe_id);

  // FIFO 淘汰：当超过 buffer_size 时，移除最小 keyframe_id 的帧
  while (frames_.size() > static_cast<std::size_t>(buffer_size_)) {
    auto it = frames_.begin();  // map 按 key 升序，begin() 是最小的
    ROS_DEBUG("[VisualFeatureBuffer] Evicting frame id=%d (buffer full)", it->first);
    frames_.erase(it);
  }

  cv_.notify_all();
}

void VisualFeatureBuffer::setBufferSize(std::size_t size) {
  std::unique_lock<std::shared_mutex> lock(mutex_);
  buffer_size_ = size;

  // 立即淘汰多余的帧
  while (frames_.size() > buffer_size_) {
    frames_.erase(frames_.begin());
  }
}
```

**配置字段**：

```cpp
// visual_feature_buffer.hpp
class VisualFeatureBuffer {
 public:
  // ... 现有接口 ...
  void setBufferSize(std::size_t size);

 private:
  mutable std::shared_mutex mutex_;
  std::map<int, VisualFrame> frames_;
  std::condition_variable_any cv_;
  int latest_id_ = -1;
  std::size_t buffer_size_ = 100;  // 默认值，可配置
};
```

### 13.5 时间戳关联策略优化

**问题**：文档中使用线性遍历（O(n)）匹配关键帧 ID，效率低。

**优化方案**：使用 `std::map` 建立时间戳索引

```cpp
// mapping_runtime.hpp 新增成员
class MappingRuntime {
 private:
  // ... 现有成员 ...

  // 时间戳→关键帧ID索引（用于视觉线程快速查找）
  std::map<double, int> timestamp_to_kf_id_;
  mutable std::shared_mutex ts_index_mutex_;
};

// mapping_runtime.cpp - 在添加关键帧时更新索引
void MappingRuntime::backendLoop() {
  while (!stop_threads_) {
    // ... 现有代码 ...

    {
      std::unique_lock<std::shared_mutex> kf_lock(kf_mutex_);
      for (auto& kf : batch) {
        kf.id = static_cast<int>(keyframes_.size());
        keyframes_.push_back(std::move(kf));

        // 更新时间戳索引
        {
          std::unique_lock<std::shared_mutex> ts_lock(ts_index_mutex_);
          timestamp_to_kf_id_[keyframes_.back().timestamp] = keyframes_.back().id;
        }
      }
      // ... 其余代码 ...
    }
  }
}

// 视觉线程使用二分查找
void MappingRuntime::visualFeatLoop() {
  while (!stop_threads_) {
    // ... 取出 batch ...

    for (auto& [image, ts] : batch) {
      int kf_id = -1;
      {
        std::shared_lock<std::shared_mutex> ts_lock(ts_index_mutex_);
        // 二分查找最近的时间戳
        auto it = timestamp_to_kf_id_.lower_bound(ts - 0.05);
        if (it != timestamp_to_kf_id_.end() && std::abs(it->first - ts) < 0.05) {
          kf_id = it->second;
        }
      }

      if (kf_id >= 0) {
        auto frame = feat_provider_->extract(image, ts, kf_id);
        feat_buffer_->insert(frame);
      }
    }
  }
}
```

### 13.6 HybridLoopDetector 详细设计

**设计策略**：串行验证 + 置信度加权

```cpp
// loop_detector_hybrid.hpp
#pragma once
#include "slam_mapping_v2/core/i_loop_detector.hpp"
#include "slam_mapping_v2/core/loop_detector_kdtree_gicp.hpp"
#include "slam_mapping_v2/core/visual_loop_detector.hpp"

namespace slam_mapping_v2 {

class HybridLoopDetector : public ILoopDetector {
 public:
  void configure(const LoopClosureConfig& cfg) override {
    cfg_ = cfg;
    laser_detector_ = std::make_unique<KdtreeGicpLoopDetector>();
    laser_detector_->configure(cfg);

    if (cfg.detector_type == "hybrid") {
      visual_detector_ = std::make_unique<VisualLoopDetector>();
      visual_detector_->configure(cfg);
    }
  }

  void setFeatureBuffer(std::shared_ptr<VisualFeatureBuffer> buffer) {
    if (visual_detector_) {
      visual_detector_->setFeatureBuffer(buffer);
    }
  }

  LoopCandidate detect(const std::vector<Keyframe>& kfs) override {
    // 策略：先激光候选，再视觉验证
    // 1. 激光检测
    LoopCandidate laser_candidate = laser_detector_->detect(kfs);

    if (!visual_detector_) {
      return laser_candidate;  // 纯激光模式
    }

    // 2. 视觉检测
    LoopCandidate visual_candidate = visual_detector_->detect(kfs);

    // 3. 融合策略
    return fuseCandidates(laser_candidate, visual_candidate);
  }

 private:
  LoopCandidate fuseCandidates(const LoopCandidate& laser,
                                const LoopCandidate& visual) {
    // 情况1：两者都检测到 → 取置信度高的
    if (laser.valid && visual.valid) {
      // 如果指向同一对回环，合并置信度
      if (laser.history_idx == visual.history_idx &&
          laser.current_idx == visual.current_idx) {
        LoopCandidate merged = laser;
        merged.score = (laser.score + visual.score) / 2.0;  // 平均分
        ROS_INFO("[HybridLoopDetector] Both agree: %d->%d (score=%.3f)",
                 merged.current_idx, merged.history_idx, merged.score);
        return merged;
      }
      // 指向不同回环，取分数高的
      return (laser.score >= visual.score) ? laser : visual;
    }

    // 情况2：仅激光检测到 → 使用激光结果
    if (laser.valid) {
      ROS_INFO("[HybridLoopDetector] Laser only: %d->%d (score=%.3f)",
               laser.current_idx, laser.history_idx, laser.score);
      return laser;
    }

    // 情况3：仅视觉检测到 → 使用视觉结果
    if (visual.valid) {
      ROS_INFO("[HybridLoopDetector] Visual only: %d->%d (score=%.3f)",
               visual.current_idx, visual.history_idx, visual.score);
      return visual;
    }

    // 情况4：都未检测到
    return LoopCandidate{};
  }

  LoopClosureConfig cfg_;
  std::unique_ptr<KdtreeGicpLoopDetector> laser_detector_;
  std::unique_ptr<VisualLoopDetector> visual_detector_;
};

}  // namespace slam_mapping_v2
```

**工厂模式扩展**：

```cpp
// loop_detector_factory.cpp
std::unique_ptr<ILoopDetector> LoopDetectorFactory::create(
    const std::string& type_str, const LoopClosureConfig& cfg) {

  if (type_str == "laser") {
    auto detector = std::make_unique<KdtreeGicpLoopDetector>();
    detector->configure(cfg);
    return detector;
  }

  if (type_str == "visual") {
    auto detector = std::make_unique<VisualLoopDetector>();
    detector->configure(cfg);
    return detector;
  }

  if (type_str == "hybrid") {
    auto detector = std::make_unique<HybridLoopDetector>();
    detector->configure(cfg);
    return detector;
  }

  ROS_ERROR("[LoopDetectorFactory] Unknown detector type: %s", type_str.c_str());
  return nullptr;
}
```

### 13.7 异常处理策略汇总

| 异常场景 | 处理方式 | 日志级别 |
|----------|----------|----------|
| 词库文件不存在 | 抛出异常，节点启动失败 | ERROR |
| 词库格式错误 | 抛出异常，节点启动失败 | ERROR |
| ORB 特征提取失败 | 跳过当前帧，返回空 VisualFrame | WARN |
| 图像为空/格式错误 | 跳过当前帧，打印警告 | WARN |
| cv_bridge 转换失败 | 跳过当前帧，打印错误 | ERROR |
| DBoW2 检索无候选 | 返回 LoopCandidate{valid=false} | DEBUG |
| RANSAC 匹配失败 | 返回 LoopCandidate{valid=false} | DEBUG |
| 特征缓冲区满 | FIFO 淘汰最旧帧 | DEBUG |
| 相机内参未配置 | 使用默认值，打印警告 | WARN |

---

## 14. 独立单元测试方案

> **目标**：在不接入 slam_mapping_v2 主流程的情况下，独立验证视觉回环检测的正确性。

### 14.1 测试架构设计

```
test_visual_loop/
├── CMakeLists.txt                    # 测试构建配置
├── test_orb_extractor.cpp            # 测试1：ORB 特征提取
├── test_visual_feature_buffer.cpp    # 测试2：特征缓冲区
├── test_visual_loop_detector.cpp     # 测试3：视觉回环检测器
├── test_loop_closure_pipeline.cpp    # 测试4：完整回环检测流程
├── config/
│   └── test_config.yaml              # 测试配置
├── data/
│   ├── test_images/                  # 测试图像序列
│   │   ├── 000000.png
│   │   ├── 000001.png
│   │   └── ...
│   ├── test_poses.txt                # 真值位姿（TUM 格式）
│   └── ORBvoc.bin                    # 测试用词库（可链接到主词库）
└── scripts/
    └── run_all_tests.sh              # 一键测试脚本
```

### 14.2 测试配置文件

```yaml
# test_config.yaml
visual_loop_test:
  # 测试数据路径
  data_dir: "/home/hhy/2004ros/DeepLumin/src/localization/slam_mapping_v2/test_visual_loop/data"
  image_dir: "${data_dir}/test_images"
  pose_file: "${data_dir}/test_poses.txt"
  vocabulary_path: "/home/hhy/2004ros/ORB_SLAM3/Vocabulary/ORBvoc.bin"

  # ORB 参数
  orb:
    nfeatures: 2000
    scale_factor: 1.2
    nlevels: 8
    ini_th_fast: 20
    min_th_fast: 7

  # 回环检测参数
  loop_detection:
    min_bow_score: 0.03
    min_shared_words: 20
    max_candidates: 3
    min_bow_matches: 50
    nn_ratio: 0.6
    ransac_min_inliers: 15
    min_continuity: 2  # 测试时降低连续性要求

  # 相机内参（测试用）
  camera:
    fx: 517.3
    fy: 516.5
    cx: 318.6
    cy: 255.3

  # 测试场景配置
  test_scenarios:
    # 场景1：相同位置不同角度（应检测到回环）
    - name: "loop_at_same_place"
      query_indices: [100, 101, 102, 103, 104]
      candidate_indices: [0, 1, 2, 3, 4]
      expected_loop: true

    # 场景2：不同位置（不应检测到回环）
    - name: "no_loop_different_places"
      query_indices: [100, 101, 102]
      candidate_indices: [500, 501, 502]
      expected_loop: false
```

### 14.3 测试1：ORB 特征提取器

```cpp
// test_orb_extractor.cpp
/**
 * @file test_orb_extractor.cpp
 * @brief 测试 ORB 特征提取器的正确性
 *
 * 测试内容：
 * 1. 特征点数量是否符合预期
 * 2. 描述子维度是否正确（N x 32）
 * 3. 特征点分布是否均匀
 * 4. 处理时间是否在可接受范围内
 */

#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <chrono>

#include "slam_mapping_v2/core/orb_feature_provider.hpp"
#include "slam_mapping_v2/common/config.hpp"

using namespace slam_mapping_v2;

class OrbExtractorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // 配置 ORB 参数
    VisualLoopConfig cfg;
    cfg.nfeatures = 2000;
    cfg.scale_factor = 1.2f;
    cfg.nlevels = 8;
    cfg.ini_th_fast = 20;
    cfg.min_th_fast = 7;

    provider_ = std::make_unique<OrbFeatureProvider>();
    provider_->configure(cfg);

    // 加载测试图像
    test_image_ = cv::imread("/home/hhy/2004ros/DeepLumin/src/localization/"
                             "slam_mapping_v2/test_visual_loop/data/test_images/000000.png",
                             cv::IMREAD_GRAYSCALE);
    ASSERT_FALSE(test_image_.empty()) << "Failed to load test image";
  }

  std::unique_ptr<OrbFeatureProvider> provider_;
  cv::Mat test_image_;
};

// 测试1：特征提取基本功能
TEST_F(OrbExtractorTest, ExtractFeatures) {
  auto frame = provider_->extract(test_image_, 0.0, 0);

  // 检查时间戳
  EXPECT_DOUBLE_EQ(frame.timestamp, 0.0);
  EXPECT_EQ(frame.keyframe_id, 0);
  EXPECT_TRUE(frame.features_ready);

  // 检查特征点数量
  EXPECT_GT(frame.keypoints.size(), 0);
  EXPECT_LE(frame.keypoints.size(), 2000);

  // 检查描述子维度
  EXPECT_EQ(frame.descriptors.rows, static_cast<int>(frame.keypoints.size()));
  EXPECT_EQ(frame.descriptors.cols, 32);  // ORB 描述子 32 字节
  EXPECT_EQ(frame.descriptors.type(), CV_8U);

  std::cout << "[OrbExtractorTest] Extracted " << frame.keypoints.size()
            << " features" << std::endl;
}

// 测试2：特征点分布均匀性
TEST_F(OrbExtractorTest, FeatureDistribution) {
  auto frame = provider_->extract(test_image_, 0.0, 0);

  // 将图像分为 4x4 网格，检查每个网格是否有特征点
  const int grid_rows = 4;
  const int grid_cols = 4;
  int cell_width = test_image_.cols / grid_cols;
  int cell_height = test_image_.rows / grid_rows;

  std::vector<std::vector<int>> grid(grid_rows, std::vector<int>(grid_cols, 0));

  for (const auto& kp : frame.keypoints) {
    int col = std::min(static_cast<int>(kp.pt.x) / cell_width, grid_cols - 1);
    int row = std::min(static_cast<int>(kp.pt.y) / cell_height, grid_rows - 1);
    grid[row][col]++;
  }

  // 检查每个网格至少有 10 个特征点（排除纹理较少的区域）
  int empty_cells = 0;
  for (int r = 0; r < grid_rows; ++r) {
    for (int c = 0; c < grid_cols; ++c) {
      if (grid[r][c] < 10) {
        empty_cells++;
        std::cout << "[OrbExtractorTest] Warning: grid[" << r << "][" << c
                  << "] has only " << grid[r][c] << " features" << std::endl;
      }
    }
  }

  // 允许最多 2 个网格特征较少（角落/天空等）
  EXPECT_LE(empty_cells, 2);
}

// 测试3：描述子唯一性
TEST_F(OrbExtractorTest, DescriptorUniqueness) {
  auto frame = provider_->extract(test_image_, 0.0, 0);

  // 检查描述子不全相同（极端情况）
  if (frame.descriptors.rows > 1) {
    cv::Mat first_desc = frame.descriptors.row(0);
    bool all_same = true;

    for (int i = 1; i < frame.descriptors.rows; ++i) {
      if (cv::countNonZero(first_desc != frame.descriptors.row(i)) > 0) {
        all_same = false;
        break;
      }
    }

    EXPECT_FALSE(all_same) << "All descriptors are identical!";
  }
}

// 测试4：提取性能
TEST_F(OrbExtractorTest, ExtractionPerformance) {
  const int num_iterations = 100;
  auto start = std::chrono::high_resolution_clock::now();

  for (int i = 0; i < num_iterations; ++i) {
    auto frame = provider_->extract(test_image_, 0.0, i);
  }

  auto end = std::chrono::high_resolution_clock::now();
  double elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();
  double avg_ms = elapsed_ms / num_iterations;

  std::cout << "[OrbExtractorTest] Average extraction time: " << avg_ms
            << " ms/frame" << std::endl;

  // 单帧提取应小于 20ms
  EXPECT_LT(avg_ms, 20.0);
}

// 测试5：批量提取
TEST_F(OrbExtractorTest, BatchExtraction) {
  std::vector<std::pair<cv::Mat, double>> images;
  std::vector<int> keyframe_ids;

  for (int i = 0; i < 5; ++i) {
    images.push_back({test_image_, static_cast<double>(i) * 0.1});
    keyframe_ids.push_back(i);
  }

  auto frames = provider_->extractBatch(images, keyframe_ids);

  EXPECT_EQ(frames.size(), 5);
  for (size_t i = 0; i < frames.size(); ++i) {
    EXPECT_EQ(frames[i].keyframe_id, keyframe_ids[i]);
    EXPECT_TRUE(frames[i].features_ready);
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
```

### 14.4 测试2：视觉特征缓冲区

```cpp
// test_visual_feature_buffer.cpp
/**
 * @file test_visual_feature_buffer.cpp
 * @brief 测试 VisualFeatureBuffer 的线程安全性和淘汰机制
 */

#include <gtest/gtest.h>
#include <thread>
#include <vector>
#include <atomic>

#include "slam_mapping_v2/core/visual_feature_buffer.hpp"

using namespace slam_mapping_v2;

class VisualFeatureBufferTest : public ::testing::Test {
 protected:
  void SetUp() override {
    buffer_ = std::make_unique<VisualFeatureBuffer>();
  }

  // 辅助函数：创建测试帧
  VisualFrame createFrame(int keyframe_id, double timestamp = 0.0) {
    VisualFrame frame;
    frame.keyframe_id = keyframe_id;
    frame.timestamp = timestamp;
    frame.features_ready = true;
    frame.keypoints.resize(100);  // 模拟 100 个特征点
    return frame;
  }

  std::unique_ptr<VisualFeatureBuffer> buffer_;
};

// 测试1：基本插入和获取
TEST_F(VisualFeatureBufferTest, InsertAndGet) {
  auto frame = createFrame(42, 1.0);
  buffer_->insert(frame);

  VisualFrame out;
  EXPECT_TRUE(buffer_->get(42, &out));
  EXPECT_EQ(out.keyframe_id, 42);
  EXPECT_DOUBLE_EQ(out.timestamp, 1.0);
}

// 测试2：获取不存在的帧
TEST_F(VisualFeatureBufferTest, GetNonExistent) {
  VisualFrame out;
  EXPECT_FALSE(buffer_->get(999, &out));
}

// 测试3：批量插入和获取
TEST_F(VisualFeatureBufferTest, BatchInsertAndGet) {
  std::vector<VisualFrame> frames;
  for (int i = 0; i < 10; ++i) {
    frames.push_back(createFrame(i, static_cast<double>(i) * 0.1));
  }

  buffer_->insertBatch(frames);
  EXPECT_EQ(buffer_->size(), 10);

  auto retrieved = buffer_->getBatch({0, 1, 2, 3, 4});
  EXPECT_EQ(retrieved.size(), 5);
}

// 测试4：FIFO 淘汰机制
TEST_F(VisualFeatureBufferTest, FifoEviction) {
  buffer_->setBufferSize(5);  // 设置缓冲区大小为 5

  // 插入 10 帧
  for (int i = 0; i < 10; ++i) {
    buffer_->insert(createFrame(i, static_cast<double>(i)));
  }

  // 应该只剩下最新的 5 帧（id 5-9）
  EXPECT_EQ(buffer_->size(), 5);

  // 验证旧帧被淘汰
  VisualFrame out;
  EXPECT_FALSE(buffer_->get(0, &out));  // 已淘汰
  EXPECT_FALSE(buffer_->get(4, &out));  // 已淘汰
  EXPECT_TRUE(buffer_->get(5, &out));   // 保留
  EXPECT_TRUE(buffer_->get(9, &out));   // 保留
}

// 测试5：获取最近 N 帧
TEST_F(VisualFeatureBufferTest, GetRecent) {
  for (int i = 0; i < 10; ++i) {
    buffer_->insert(createFrame(i, static_cast<double>(i)));
  }

  auto recent = buffer_->getRecent(3);
  EXPECT_EQ(recent.size(), 3);

  // 应该是 id 7, 8, 9
  std::vector<int> expected_ids = {7, 8, 9};
  for (size_t i = 0; i < recent.size(); ++i) {
    EXPECT_EQ(recent[i].keyframe_id, expected_ids[i]);
  }
}

// 测试6：多线程并发读写
TEST_F(VisualFeatureBufferTest, ConcurrentReadWrite) {
  const int num_writers = 4;
  const int num_readers = 4;
  const int frames_per_writer = 100;

  std::atomic<int> write_count{0};
  std::atomic<int> read_count{0};

  // 写线程
  auto writer = [&](int start_id) {
    for (int i = 0; i < frames_per_writer; ++i) {
      int id = start_id + i;
      buffer_->insert(createFrame(id, static_cast<double>(id) * 0.01));
      write_count++;
    }
  };

  // 读线程
  auto reader = [&]() {
    for (int i = 0; i < 200; ++i) {
      VisualFrame out;
      buffer_->get(i, &out);  // 可能存在也可能不存在
      read_count++;
    }
  };

  // 启动线程
  std::vector<std::thread> threads;
  for (int i = 0; i < num_writers; ++i) {
    threads.emplace_back(writer, i * frames_per_writer);
  }
  for (int i = 0; i < num_readers; ++i) {
    threads.emplace_back(reader);
  }

  // 等待所有线程完成
  for (auto& t : threads) {
    t.join();
  }

  // 验证没有崩溃，且写入成功
  EXPECT_EQ(write_count.load(), num_writers * frames_per_writer);
  EXPECT_GT(read_count.load(), 0);

  std::cout << "[VisualFeatureBufferTest] Concurrent test: "
            << write_count.load() << " writes, "
            << read_count.load() << " reads" << std::endl;
}

// 测试7：等待新帧
TEST_F(VisualFeatureBufferTest, WaitForNew) {
  std::atomic<bool> insert_done{false};

  // 在另一个线程延迟插入
  std::thread inserter([&]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    buffer_->insert(createFrame(42));
    insert_done = true;
  });

  // 等待新帧（超时 200ms）
  EXPECT_TRUE(buffer_->waitForNew(200));
  EXPECT_TRUE(insert_done.load());

  inserter.join();
}

// 测试8：清空缓冲区
TEST_F(VisualFeatureBufferTest, Clear) {
  for (int i = 0; i < 10; ++i) {
    buffer_->insert(createFrame(i));
  }
  EXPECT_EQ(buffer_->size(), 10);

  buffer_->clear();
  EXPECT_EQ(buffer_->size(), 0);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
```

### 14.5 测试3：视觉回环检测器

```cpp
// test_visual_loop_detector.cpp
/**
 * @file test_visual_loop_detector.cpp
 * @brief 测试 VisualLoopDetector 的回环检测正确性
 *
 * 测试策略：
 * 1. 使用真实图像序列测试
 * 2. 验证同一位置不同帧能检测到回环
 * 3. 验证不同位置不会误检
 * 4. 验证连续性确认机制
 */

#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <fstream>

#include "slam_mapping_v2/core/visual_loop_detector.hpp"
#include "slam_mapping_v2/core/orb_feature_provider.hpp"
#include "slam_mapping_v2/core/visual_feature_buffer.hpp"

using namespace slam_mapping_v2;

class VisualLoopDetectorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // 加载测试配置
    cfg_.enable = true;
    cfg_.detector_type = "visual";
    cfg_.visual.vocabulary_path =
        "/home/hhy/2004ros/ORB_SLAM3/Vocabulary/ORBvoc.bin";
    cfg_.visual.nfeatures = 2000;
    cfg_.visual.min_bow_score = 0.03;
    cfg_.visual.min_continuity = 2;  // 测试时降低连续性要求
    cfg_.visual.camera.fx = 517.3;
    cfg_.visual.camera.fy = 516.5;
    cfg_.visual.camera.cx = 318.6;
    cfg_.visual.camera.cy = 255.3;

    // 创建检测器和特征缓冲区
    detector_ = std::make_unique<VisualLoopDetector>();
    detector_->configure(cfg_);

    buffer_ = std::make_shared<VisualFeatureBuffer>();
    detector_->setFeatureBuffer(buffer_);

    // 创建特征提取器
    provider_ = std::make_unique<OrbFeatureProvider>();
    provider_->configure(cfg_.visual);

    // 加载测试图像
    loadTestData();
  }

  void loadTestData() {
    std::string image_dir =
        "/home/hhy/2004ros/DeepLumin/src/localization/"
        "slam_mapping_v2/test_visual_loop/data/test_images";

    // 加载前 200 帧图像
    for (int i = 0; i < 200; ++i) {
      std::string path = image_dir + "/" +
                         std::to_string(i) + ".png";
      cv::Mat img = cv::imread(path, cv::IMREAD_GRAYSCALE);
      if (img.empty()) {
        // 尝试 6 位数格式
        std::ostringstream oss;
        oss << image_dir << "/" << std::setw(6) << std::setfill('0') << i << ".png";
        img = cv::imread(oss.str(), cv::IMREAD_GRAYSCALE);
      }
      if (!img.empty()) {
        images_.push_back(img);
      }
    }

    std::cout << "[VisualLoopDetectorTest] Loaded " << images_.size()
              << " test images" << std::endl;
  }

  // 辅助函数：提取并缓存特征
  void extractAndCacheFeatures(int start_idx, int end_idx) {
    for (int i = start_idx; i < end_idx && i < static_cast<int>(images_.size()); ++i) {
      auto frame = provider_->extract(images_[i], static_cast<double>(i) * 0.1, i);
      buffer_->insert(frame);
    }
  }

  // 辅助函数：构建关键帧列表（模拟）
  std::vector<Keyframe> buildKeyframes(int count) {
    std::vector<Keyframe> kfs;
    for (int i = 0; i < count; ++i) {
      Keyframe kf;
      kf.id = i;
      kf.timestamp = static_cast<double>(i) * 0.1;
      kf.pose_odom.position = Eigen::Vector3d(i * 0.5, 0, 0);  // 简单轨迹
      kfs.push_back(kf);
    }
    return kfs;
  }

  LoopClosureConfig cfg_;
  std::unique_ptr<VisualLoopDetector> detector_;
  std::shared_ptr<VisualFeatureBuffer> buffer_;
  std::unique_ptr<OrbFeatureProvider> provider_;
  std::vector<cv::Mat> images_;
};

// 测试1：检测器初始化
TEST_F(VisualLoopDetectorTest, Initialization) {
  EXPECT_NE(detector_, nullptr);

  // 重置不应崩溃
  detector_->reset();

  // loopPairs 应为空
  EXPECT_TRUE(detector_->loopPairs().empty());
}

// 测试2：无特征时的检测
TEST_F(VisualLoopDetectorTest, DetectWithoutFeatures) {
  auto kfs = buildKeyframes(10);
  auto result = detector_->detect(kfs);

  // 没有特征时应返回无效结果
  EXPECT_FALSE(result.valid);
}

// 测试3：单帧特征检测
TEST_F(VisualLoopDetectorTest, DetectSingleFrame) {
  if (images_.size() < 1) {
    GTEST_SKIP() << "No test images available";
  }

  // 提取第一帧特征
  extractAndCacheFeatures(0, 1);

  auto kfs = buildKeyframes(1);
  auto result = detector_->detect(kfs);

  // 单帧无法检测回环
  EXPECT_FALSE(result.valid);
}

// 测试4：相同位置回环检测（模拟）
TEST_F(VisualLoopDetectorTest, DetectLoopAtSamePlace) {
  if (images_.size() < 100) {
    GTEST_SKIP() << "Need at least 100 test images";
  }

  // 提取前 50 帧特征
  extractAndCacheFeatures(0, 50);

  // 模拟关键帧（假设前 5 帧和最后 5 帧在同一位置）
  auto kfs = buildKeyframes(50);

  // 执行多次检测以满足连续性要求
  int loop_detected_count = 0;
  for (int i = 45; i < 50; ++i) {
    auto result = detector_->detect(kfs);
    if (result.valid) {
      loop_detected_count++;
      std::cout << "[VisualLoopDetectorTest] Loop detected: "
                << result.current_idx << " -> " << result.history_idx
                << " (score=" << result.score << ")" << std::endl;
    }
  }

  // 注意：这个测试可能需要根据实际数据调整期望值
  std::cout << "[VisualLoopDetectorTest] Loop detected " << loop_detected_count
            << " times" << std::endl;
}

// 测试5：不同位置不应检测到回环
TEST_F(VisualLoopDetectorTest, NoLoopAtDifferentPlaces) {
  if (images_.size() < 200) {
    GTEST_SKIP() << "Need at least 200 test images";
  }

  // 提取特征
  extractAndCacheFeatures(0, 200);

  // 模拟关键帧（前 100 帧和后 100 帧在不同位置）
  auto kfs = buildKeyframes(200);

  // 检测最后一帧与前 100 帧的回环
  detector_->reset();
  for (int i = 150; i < 200; ++i) {
    auto result = detector_->detect(kfs);
    if (result.valid && result.history_idx < 100) {
      // 不应该检测到跨区域的回环
      ADD_FAILURE() << "False positive loop detected: "
                    << result.current_idx << " -> " << result.history_idx;
    }
  }
}

// 测试6：连续性验证
TEST_F(VisualLoopDetectorTest, ContinuityCheck) {
  if (images_.size() < 50) {
    GTEST_SKIP() << "Need at least 50 test images";
  }

  extractAndCacheFeatures(0, 50);
  auto kfs = buildKeyframes(50);

  // 连续检测多帧，验证连续性机制
  int consecutive_loops = 0;
  int last_history_idx = -1;

  for (int i = 40; i < 50; ++i) {
    auto result = detector_->detect(kfs);
    if (result.valid) {
      if (result.history_idx == last_history_idx) {
        consecutive_loops++;
      } else {
        consecutive_loops = 1;
        last_history_idx = result.history_idx;
      }
    } else {
      consecutive_loops = 0;
      last_history_idx = -1;
    }
  }

  std::cout << "[VisualLoopDetectorTest] Max consecutive loops: "
            << consecutive_loops << std::endl;
}

// 测试7：检测性能
TEST_F(VisualLoopDetectorTest, DetectionPerformance) {
  if (images_.size() < 100) {
    GTEST_SKIP() << "Need at least 100 test images";
  }

  extractAndCacheFeatures(0, 100);
  auto kfs = buildKeyframes(100);

  auto start = std::chrono::high_resolution_clock::now();

  const int num_iterations = 50;
  for (int i = 0; i < num_iterations; ++i) {
    detector_->detect(kfs);
  }

  auto end = std::chrono::high_resolution_clock::now();
  double elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();
  double avg_ms = elapsed_ms / num_iterations;

  std::cout << "[VisualLoopDetectorTest] Average detection time: "
            << avg_ms << " ms" << std::endl;

  // 单次检测应小于 50ms
  EXPECT_LT(avg_ms, 50.0);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
```

### 14.6 测试4：完整回环检测流程（端到端）

```cpp
// test_loop_closure_pipeline.cpp
/**
 * @file test_loop_closure_pipeline.cpp
 * @brief 端到端测试完整视觉回环检测流程
 *
 * 测试流程：
 * 1. 加载图像序列和真值位姿
 * 2. 模拟关键帧生成
 * 3. 执行视觉回环检测
 * 4. 与真值比较计算准确率
 * 5. 输出评估报告
 */

#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iomanip>

#include "slam_mapping_v2/core/visual_loop_detector.hpp"
#include "slam_mapping_v2/core/orb_feature_provider.hpp"
#include "slam_mapping_v2/core/visual_feature_buffer.hpp"

using namespace slam_mapping_v2;

struct GroundTruth {
  int frame_id;
  double timestamp;
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
};

class LoopClosurePipelineTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // 配置
    cfg_.enable = true;
    cfg_.detector_type = "visual";
    cfg_.visual.vocabulary_path =
        "/home/hhy/2004ros/ORB_SLAM3/Vocabulary/ORBvoc.bin";
    cfg_.visual.nfeatures = 2000;
    cfg_.visual.min_bow_score = 0.03;
    cfg_.visual.min_shared_words = 20;
    cfg_.visual.min_bow_matches = 50;
    cfg_.visual.ransac_min_inliers = 15;
    cfg_.visual.min_continuity = 2;
    cfg_.visual.camera.fx = 517.3;
    cfg_.visual.camera.fy = 516.5;
    cfg_.visual.camera.cx = 318.6;
    cfg_.visual.camera.cy = 255.3;

    // 创建组件
    detector_ = std::make_unique<VisualLoopDetector>();
    detector_->configure(cfg_);

    buffer_ = std::make_shared<VisualFeatureBuffer>();
    detector_->setFeatureBuffer(buffer_);

    provider_ = std::make_unique<OrbFeatureProvider>();
    provider_->configure(cfg_.visual);

    // 加载数据
    loadData();
  }

  void loadData() {
    std::string data_dir =
        "/home/hhy/2004ros/DeepLumin/src/localization/"
        "slam_mapping_v2/test_visual_loop/data";

    // 加载真值位姿
    loadGroundTruth(data_dir + "/test_poses.txt");

    // 加载图像
    loadImages(data_dir + "/test_images");
  }

  void loadGroundTruth(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
      std::cerr << "[PipelineTest] Warning: Cannot open ground truth file: "
                << path << std::endl;
      return;
    }

    // TUM 格式: timestamp tx ty tz qx qy qz qw
    double ts, tx, ty, tz, qx, qy, qz, qw;
    while (file >> ts >> tx >> ty >> tz >> qx >> qy >> qz >> qw) {
      GroundTruth gt;
      gt.frame_id = ground_truth_.size();
      gt.timestamp = ts;
      gt.position = Eigen::Vector3d(tx, ty, tz);
      gt.orientation = Eigen::Quaterniond(qw, qx, qy, qz);
      ground_truth_.push_back(gt);
    }

    std::cout << "[PipelineTest] Loaded " << ground_truth_.size()
              << " ground truth poses" << std::endl;
  }

  void loadImages(const std::string& dir) {
    for (int i = 0; i < 1000; ++i) {
      std::ostringstream oss;
      oss << dir << "/" << std::setw(6) << std::setfill('0') << i << ".png";
      cv::Mat img = cv::imread(oss.str(), cv::IMREAD_GRAYSCALE);
      if (!img.empty()) {
        images_.push_back(img);
      } else {
        break;
      }
    }

    std::cout << "[PipelineTest] Loaded " << images_.size()
              << " images" << std::endl;
  }

  // 根据真值判断两帧是否在同一位置
  bool isSameLocation(int id1, int id2, double distance_thresh = 5.0) {
    if (id1 >= ground_truth_.size() || id2 >= ground_truth_.size()) {
      return false;
    }

    double dist = (ground_truth_[id1].position - ground_truth_[id2].position).norm();
    return dist < distance_thresh;
  }

  // 计算评估指标
  struct EvaluationMetrics {
    int true_positives = 0;
    int false_positives = 0;
    int false_negatives = 0;
    int total_ground_truth_loops = 0;
    double precision = 0.0;
    double recall = 0.0;
    double f1_score = 0.0;
  };

  EvaluationMetrics evaluate(const std::vector<std::pair<int, int>>& detected_loops,
                              double distance_thresh = 5.0) {
    EvaluationMetrics metrics;

    // 计算真值中的回环对数
    for (size_t i = 0; i < ground_truth_.size(); ++i) {
      for (size_t j = i + 50; j < ground_truth_.size(); ++j) {  // 至少间隔 50 帧
        if (isSameLocation(i, j, distance_thresh)) {
          metrics.total_ground_truth_loops++;
        }
      }
    }

    // 评估检测结果
    for (const auto& [current, history] : detected_loops) {
      if (isSameLocation(current, history, distance_thresh)) {
        metrics.true_positives++;
      } else {
        metrics.false_positives++;
      }
    }

    metrics.false_negatives = metrics.total_ground_truth_loops - metrics.true_positives;

    // 计算精确率和召回率
    if (metrics.true_positives + metrics.false_positives > 0) {
      metrics.precision = static_cast<double>(metrics.true_positives) /
                          (metrics.true_positives + metrics.false_positives);
    }

    if (metrics.true_positives + metrics.false_negatives > 0) {
      metrics.recall = static_cast<double>(metrics.true_positives) /
                       (metrics.true_positives + metrics.false_negatives);
    }

    if (metrics.precision + metrics.recall > 0) {
      metrics.f1_score = 2.0 * metrics.precision * metrics.recall /
                         (metrics.precision + metrics.recall);
    }

    return metrics;
  }

  LoopClosureConfig cfg_;
  std::unique_ptr<VisualLoopDetector> detector_;
  std::shared_ptr<VisualFeatureBuffer> buffer_;
  std::unique_ptr<OrbFeatureProvider> provider_;
  std::vector<cv::Mat> images_;
  std::vector<GroundTruth> ground_truth_;
};

// 测试1：完整流程测试
TEST_F(LoopClosurePipelineTest, FullPipeline) {
  if (images_.size() < 100 || ground_truth_.empty()) {
    GTEST_SKIP() << "Insufficient test data";
  }

  const int num_frames = std::min(static_cast<int>(images_.size()), 500);

  // Step 1: 提取所有帧的特征
  std::cout << "[PipelineTest] Step 1: Extracting features..." << std::endl;
  for (int i = 0; i < num_frames; ++i) {
    auto frame = provider_->extract(images_[i], ground_truth_[i].timestamp, i);
    buffer_->insert(frame);
  }

  // Step 2: 构建关键帧列表
  std::cout << "[PipelineTest] Step 2: Building keyframes..." << std::endl;
  std::vector<Keyframe> keyframes;
  for (int i = 0; i < num_frames; ++i) {
    Keyframe kf;
    kf.id = i;
    kf.timestamp = ground_truth_[i].timestamp;
    kf.pose_odom.position = ground_truth_[i].position;
    kf.pose_odom.orientation = ground_truth_[i].orientation;
    keyframes.push_back(kf);
  }

  // Step 3: 执行回环检测
  std::cout << "[PipelineTest] Step 3: Running loop detection..." << std::endl;
  std::vector<std::pair<int, int>> detected_loops;

  for (int i = 50; i < num_frames; ++i) {
    auto result = detector_->detect(keyframes);
    if (result.valid) {
      detected_loops.push_back({result.current_idx, result.history_idx});
      std::cout << "[PipelineTest] Loop detected: " << result.current_idx
                << " -> " << result.history_idx
                << " (score=" << result.score << ")" << std::endl;
    }
  }

  // Step 4: 评估结果
  std::cout << "[PipelineTest] Step 4: Evaluating results..." << std::endl;
  auto metrics = evaluate(detected_loops);

  std::cout << "\n========================================" << std::endl;
  std::cout << "       Loop Detection Evaluation        " << std::endl;
  std::cout << "========================================" << std::endl;
  std::cout << "Total frames processed:     " << num_frames << std::endl;
  std::cout << "Detected loops:             " << detected_loops.size() << std::endl;
  std::cout << "Ground truth loops:         " << metrics.total_ground_truth_loops << std::endl;
  std::cout << "True positives:             " << metrics.true_positives << std::endl;
  std::cout << "False positives:            " << metrics.false_positives << std::endl;
  std::cout << "False negatives:            " << metrics.false_negatives << std::endl;
  std::cout << "Precision:                  " << std::fixed << std::setprecision(3)
            << metrics.precision << std::endl;
  std::cout << "Recall:                     " << std::fixed << std::setprecision(3)
            << metrics.recall << std::endl;
  std::cout << "F1 Score:                   " << std::fixed << std::setprecision(3)
            << metrics.f1_score << std::endl;
  std::cout << "========================================\n" << std::endl;

  // 基本断言
  EXPECT_GT(detected_loops.size(), 0) << "No loops detected at all";
  EXPECT_GT(metrics.precision, 0.5) << "Precision too low";
  EXPECT_GT(metrics.recall, 0.3) << "Recall too low";
}

// 测试2：不同参数敏感性分析
TEST_F(LoopClosurePipelineTest, ParameterSensitivity) {
  if (images_.size() < 100) {
    GTEST_SKIP() << "Insufficient test data";
  }

  const int num_frames = 200;

  // 提取特征（只做一次）
  for (int i = 0; i < num_frames; ++i) {
    auto frame = provider_->extract(images_[i], static_cast<double>(i) * 0.1, i);
    buffer_->insert(frame);
  }

  std::vector<Keyframe> keyframes;
  for (int i = 0; i < num_frames; ++i) {
    Keyframe kf;
    kf.id = i;
    kf.timestamp = static_cast<double>(i) * 0.1;
    keyframes.push_back(kf);
  }

  // 测试不同 min_bow_score
  std::vector<double> bow_scores = {0.01, 0.02, 0.03, 0.05, 0.1};

  std::cout << "\n[PipelineTest] Parameter Sensitivity Analysis:" << std::endl;
  std::cout << "min_bow_score | Loops Detected" << std::endl;
  std::cout << "-----------------------------" << std::endl;

  for (double score : bow_scores) {
    // 重新配置检测器
    LoopClosureConfig test_cfg = cfg_;
    test_cfg.visual.min_bow_score = score;
    detector_->configure(test_cfg);
    detector_->reset();

    int loop_count = 0;
    for (int i = 50; i < num_frames; ++i) {
      auto result = detector_->detect(keyframes);
      if (result.valid) loop_count++;
    }

    std::cout << std::setw(13) << score << " | " << loop_count << std::endl;
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
```

### 14.7 CMakeLists.txt 配置

```cmake
# test_visual_loop/CMakeLists.txt
cmake_minimum_required(VERSION 3.0.2)
project(test_visual_loop)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  deeplumin_msgs
)

find_package(OpenCV 4.2 REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(PCL 1.10 REQUIRED)

# Google Test
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# slam_mapping_v2 头文件
set(SLAM_MAPPING_V2_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)
include_directories(
  ${SLAM_MAPPING_V2_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

# DBoW2 库（假设已编译）
link_directories(${SLAM_MAPPING_V2_DIR}/thirdparty/DBoW2/lib)

# 测试源文件
set(TEST_SOURCES
  src/core/orb_feature_provider.cpp
  src/core/visual_feature_buffer.cpp
  src/core/visual_loop_detector.cpp
)

# 测试1：ORB 特征提取器
catkin_add_gtest(test_orb_extractor
  test_orb_extractor.cpp
  ${TEST_SOURCES}
)
target_link_libraries(test_orb_extractor
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  DBoW2
  gtest_main
)

# 测试2：视觉特征缓冲区
catkin_add_gtest(test_visual_feature_buffer
  test_visual_feature_buffer.cpp
  ${TEST_SOURCES}
)
target_link_libraries(test_visual_feature_buffer
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  gtest_main
)

# 测试3：视觉回环检测器
catkin_add_gtest(test_visual_loop_detector
  test_visual_loop_detector.cpp
  ${TEST_SOURCES}
)
target_link_libraries(test_visual_loop_detector
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  DBoW2
  gtest_main
)

# 测试4：完整流程
catkin_add_gtest(test_loop_closure_pipeline
  test_loop_closure_pipeline.cpp
  ${TEST_SOURCES}
)
target_link_libraries(test_loop_closure_pipeline
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  DBoW2
  gtest_main
)
```

### 14.8 测试数据准备

**准备测试图像序列**：

```bash
#!/bin/bash
# scripts/prepare_test_data.sh

# 从 rosbag 提取图像
BAG_FILE="/path/to/your/test.bag"
IMAGE_TOPIC="/camera/image_raw"
OUTPUT_DIR="data/test_images"

mkdir -p $OUTPUT_DIR

# 使用 rosbag 提取图像
rosrun image_view extract_images image:=$IMAGE_TOPIC _sec_per_frame:=0.1 &
rosbag play $BAG_FILE --clock
sleep 5
kill %1

# 重命名为连续编号
i=0
for img in $(ls *.jpg | sort); do
  mv "$img" $(printf "%06d.png" $i)
  i=$((i+1))
done

mv *.png $OUTPUT_DIR/

echo "Extracted $i images to $OUTPUT_DIR"
```

**准备真值位姿**：

```bash
# 从 SLAM 结果或 GT 系统导出 TUM 格式位姿
# 格式: timestamp tx ty tz qx qy qz qw
```

### 14.9 一键测试脚本

```bash
#!/bin/bash
# scripts/run_all_tests.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

echo "=========================================="
echo "  Visual Loop Closure Unit Tests"
echo "=========================================="

# 编译测试
echo "[1/4] Building tests..."
cd /home/hhy/2004ros/DeepLumin
catkin_make --pkg test_visual_loop

# source 环境
source devel/setup.bash

# 运行测试
echo "[2/4] Running ORB Extractor tests..."
rosrun test_visual_loop test_orb_extractor --gtest_output=xml:test_results_orb.xml

echo "[3/4] Running Visual Feature Buffer tests..."
rosrun test_visual_loop test_visual_feature_buffer --gtest_output=xml:test_results_buffer.xml

echo "[4/4] Running Visual Loop Detector tests..."
rosrun test_visual_loop test_visual_loop_detector --gtest_output=xml:test_results_detector.xml

echo "[Bonus] Running Pipeline tests..."
rosrun test_visual_loop test_loop_closure_pipeline --gtest_output=xml:test_results_pipeline.xml

echo ""
echo "=========================================="
echo "  All tests completed!"
echo "=========================================="
echo ""
echo "Test results saved to:"
echo "  - test_results_orb.xml"
echo "  - test_results_buffer.xml"
echo "  - test_results_detector.xml"
echo "  - test_results_pipeline.xml"
```

### 14.10 测试验证清单

| 测试项 | 验证内容 | 通过标准 |
|--------|----------|----------|
| ORB 特征提取 | 特征数量、描述子维度、分布均匀性 | 2000±10% 特征点，32 字节描述子 |
| 特征缓冲区 | 线程安全、FIFO 淘汰、并发读写 | 无死锁、无崩溃、淘汰正确 |
| 回环检测器 | 正确检测、无误检、连续性验证 | TP > 80%, FP < 20% |
| 完整流程 | 端到端性能、准确率 | F1 > 0.6, 延迟 < 50ms |
| 性能测试 | 单帧处理时间 | ORB < 20ms, 检测 < 50ms |

---

**文档补充完成**

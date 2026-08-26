/**
 * @file scan_context_manager.cpp
 * @brief Scan Context 描述子管理器
 *
 * Scan Context 原理：
 *   将一帧 LiDAR 点云投影到水平面极坐标网格 (ring x sector)，
 *   每个格子记录该区域内点的最大高度 z，形成 num_rings x num_sectors 矩阵。
 *
 *   默认 20 rings x 60 sectors，max_radius=80m 时：
 *     每个 ring 宽约 4m，每个 sector 约 6°
 *
 * 检索流程（两阶段）：
 *   Stage-1：ring_key L2 距离预过滤 → 保留 ring_key_candidates 条
 *   Stage-2：sector_key 循环对齐估计 yaw → 完整余弦距离 → Top-K
 */

#include <relocalization/descriptor/scan_context_descriptor.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>

namespace relocalization {

namespace {
constexpr const char* kMagic = "SCR_SCAN_CONTEXT_DB_V1";
}

ScanContextDescriptor::ScanContextDescriptor(const ScanContextConfig& config) : config_(config) {}

/**
 * @brief 从点云生成 Scan Context 描述子矩阵
 *
 * 对每个点：
 *   r = hypot(x, y)           -> 映射到 ring（径向分区）
 *   theta = atan2(y, x)       -> 映射到 sector（角向分区）
 *   descriptor(ring, sector) = max(z)  取该格子内最大高度
 *
 * 空格子（无点落入）最终置 0。
 */
Eigen::MatrixXd ScanContextDescriptor::makeDescriptor(const CloudConstPtr& cloud) const {
  // 用 -1000 哨兵值标记空格子，便于后续区分
  Eigen::MatrixXd desc = Eigen::MatrixXd::Constant(config_.num_rings, config_.num_sectors, -1000.0);
  if (!cloud || cloud->empty()) {
    return Eigen::MatrixXd::Zero(config_.num_rings, config_.num_sectors);
  }

  for (const auto& pt : cloud->points) {
    if (!pcl::isFinite(pt)) {
      continue;
    }
    const double r = std::hypot(static_cast<double>(pt.x), static_cast<double>(pt.y));
    // 忽略 min_range 以内（车体/近场噪声）和 max_radius 以外的点
    if (r < config_.min_range || r > config_.max_radius) {
      continue;
    }

    // ring 索引：ceil 保证 r=max_radius 时落在最后一个 ring
    int ring = static_cast<int>(std::ceil((r / config_.max_radius) * config_.num_rings)) - 1;
    ring = std::max(0, std::min(config_.num_rings - 1, ring));

    // sector 索引：将角度归一化到 [0, 2π) 再均匀划分
    double theta = std::atan2(static_cast<double>(pt.y), static_cast<double>(pt.x));
    if (theta < 0.0) {
      theta += 2.0 * M_PI;
    }
    int sector = static_cast<int>(std::floor((theta / (2.0 * M_PI)) * config_.num_sectors));
    sector = std::max(0, std::min(config_.num_sectors - 1, sector));

    // 同一格子内保留最大高度（对垂直结构敏感：墙、树、建筑等）
    desc(ring, sector) = std::max(desc(ring, sector), static_cast<double>(pt.z));
  }

  // 空格子归零
  for (int r = 0; r < desc.rows(); ++r) {
    for (int c = 0; c < desc.cols(); ++c) {
      if (desc(r, c) < -999.0) {
        desc(r, c) = 0.0;
      }
    }
  }
  return desc;
}

/// ring_key：每个 ring 行均值，用于快速预过滤（当前 query 未使用）
Eigen::VectorXd ScanContextDescriptor::makeRingKey(const Eigen::MatrixXd& descriptor) const {
  Eigen::VectorXd key(descriptor.rows());
  for (int r = 0; r < descriptor.rows(); ++r) {
    key(r) = descriptor.row(r).mean();
  }
  return key;
}

/// sector_key：每个 sector 列均值，用于 yaw 循环对齐搜索
Eigen::VectorXd ScanContextDescriptor::makeSectorKey(const Eigen::MatrixXd& descriptor) const {
  Eigen::VectorXd key(descriptor.cols());
  for (int c = 0; c < descriptor.cols(); ++c) {
    key(c) = descriptor.col(c).mean();
  }
  return key;
}

void ScanContextDescriptor::clear() {
  entries_.clear();
}

/// 添加一条关键帧描述子记录（建库时调用）
void ScanContextDescriptor::add(int keyframe_id, double timestamp, const Pose& pose, const Eigen::MatrixXd& descriptor) {
  ScanContextEntry entry;
  entry.keyframe_id = keyframe_id;
  entry.timestamp = timestamp;
  entry.pose = pose;
  entry.descriptor = descriptor;
  entry.ring_key = makeRingKey(descriptor);
  entry.sector_key = makeSectorKey(descriptor);
  entries_.push_back(std::move(entry));
}

/**
 * @brief 两阶段检索：ring_key 预过滤 + 完整 SC 距离
 *
 * Stage-1（ring_key，旋转不变）：
 *   ring_key = 每个 ring 行均值向量，反映各距离环带的平均高度。
 *   车辆朝向改变时 sector 列循环平移，但 ring_key 不变——因此 ring_key L2 距离
 *   可以快速（O(N×R) 而非 O(N×S×(S+R))）筛除大量不相关地点。
 *   保留最近的 ring_key_candidates 条记录进入 Stage-2。
 *
 * Stage-2（完整 SC）：
 *   对 Stage-1 候选做 sector_key 循环对齐（估计 yaw_diff），再计算完整余弦距离。
 *   按 score 升序返回 Top-K。
 *
 * @note ring_key_candidates=0 时退化为原始线性遍历（兼容旧行为）。
 */
std::vector<DescriptorCandidate> ScanContextDescriptor::query(const Eigen::MatrixXd& descriptor,
                                                             int top_k) const {
  std::vector<DescriptorCandidate> out;
  if (top_k <= 0 || entries_.empty()) {
    return out;
  }

  const Eigen::VectorXd query_ring_key    = makeRingKey(descriptor);
  const Eigen::VectorXd query_sector_key  = makeSectorKey(descriptor);
  const int n_entries = static_cast<int>(entries_.size());

  // ── Stage-1：ring_key L2 预过滤 ──────────────────────────────────────────
  // ring_key_candidates=0 或候选数不足时，对全量记录做 SC（线性遍历）
  const int pre_n = (config_.ring_key_candidates > 0)
                        ? std::min(config_.ring_key_candidates, n_entries)
                        : n_entries;

  // 计算 ring_key L2 距离并找出最近的 pre_n 条
  struct RingDist { double d; int idx; };
  std::vector<RingDist> ring_dists;
  ring_dists.reserve(n_entries);
  for (int i = 0; i < n_entries; ++i) {
    ring_dists.push_back({(query_ring_key - entries_[i].ring_key).norm(), i});
  }
  std::partial_sort(ring_dists.begin(), ring_dists.begin() + pre_n, ring_dists.end(),
                    [](const RingDist& a, const RingDist& b) { return a.d < b.d; });
  ring_dists.resize(pre_n);

  // ── Stage-2：对预过滤候选做完整 SC 距离 ─────────────────────────────────
  out.reserve(pre_n);
  for (const auto& rd : ring_dists) {
    const auto& entry = entries_[rd.idx];
    const int shift = fastSectorAlignment(query_sector_key, entry.sector_key);
    const double score = descriptorDistance(descriptor, entry.descriptor, shift);

    DescriptorCandidate candidate;
    candidate.keyframe_id = entry.keyframe_id;
    candidate.score       = score;
    candidate.ring_dist   = rd.d;
    candidate.sector_shift = shift;
    // yaw_diff：query 相对 candidate 的航向差，用于 GICP 初值修正
    candidate.yaw_diff = -static_cast<double>(shift) * 2.0 * M_PI
                         / static_cast<double>(config_.num_sectors);
    candidate.pose = entry.pose;
    out.push_back(candidate);
  }

  // 按 SC score 升序，返回 Top-K
  const int k = std::min(top_k, static_cast<int>(out.size()));
  std::partial_sort(out.begin(), out.begin() + k, out.end(),
                    [](const DescriptorCandidate& a, const DescriptorCandidate& b) {
                      return a.score < b.score;
                    });
  out.resize(k);
  return out;
}

/// 将描述子数据库序列化为二进制文件（含 magic、配置、所有条目）
bool ScanContextDescriptor::save(const std::string& path) const {
  std::ofstream out(path, std::ios::binary);
  if (!out.is_open()) {
    return false;
  }

  const std::string magic(kMagic);
  const std::size_t magic_size = magic.size();
  out.write(reinterpret_cast<const char*>(&magic_size), sizeof(magic_size));
  out.write(magic.data(), static_cast<std::streamsize>(magic.size()));
  out.write(reinterpret_cast<const char*>(&config_.num_rings), sizeof(config_.num_rings));
  out.write(reinterpret_cast<const char*>(&config_.num_sectors), sizeof(config_.num_sectors));
  out.write(reinterpret_cast<const char*>(&config_.max_radius), sizeof(config_.max_radius));
  out.write(reinterpret_cast<const char*>(&config_.min_range), sizeof(config_.min_range));

  const std::size_t count = entries_.size();
  out.write(reinterpret_cast<const char*>(&count), sizeof(count));
  for (const auto& entry : entries_) {
    out.write(reinterpret_cast<const char*>(&entry.keyframe_id), sizeof(entry.keyframe_id));
    out.write(reinterpret_cast<const char*>(&entry.timestamp), sizeof(entry.timestamp));
    out.write(reinterpret_cast<const char*>(entry.pose.t.data()), sizeof(double) * 3);
    const double q[4] = {entry.pose.q.x(), entry.pose.q.y(), entry.pose.q.z(), entry.pose.q.w()};
    out.write(reinterpret_cast<const char*>(q), sizeof(double) * 4);
    for (int r = 0; r < config_.num_rings; ++r) {
      for (int c = 0; c < config_.num_sectors; ++c) {
        const double value = entry.descriptor(r, c);
        out.write(reinterpret_cast<const char*>(&value), sizeof(value));
      }
    }
  }
  return static_cast<bool>(out);
}

/// 从二进制文件加载描述子数据库，并重建 ring_key / sector_key
bool ScanContextDescriptor::load(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in.is_open()) {
    return false;
  }

  std::size_t magic_size = 0;
  in.read(reinterpret_cast<char*>(&magic_size), sizeof(magic_size));
  std::string magic(magic_size, '\0');
  in.read(&magic[0], static_cast<std::streamsize>(magic_size));
  if (magic != kMagic) {
    return false;
  }

  in.read(reinterpret_cast<char*>(&config_.num_rings), sizeof(config_.num_rings));
  in.read(reinterpret_cast<char*>(&config_.num_sectors), sizeof(config_.num_sectors));
  in.read(reinterpret_cast<char*>(&config_.max_radius), sizeof(config_.max_radius));
  in.read(reinterpret_cast<char*>(&config_.min_range), sizeof(config_.min_range));

  std::size_t count = 0;
  in.read(reinterpret_cast<char*>(&count), sizeof(count));
  entries_.clear();
  entries_.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    ScanContextEntry entry;
    in.read(reinterpret_cast<char*>(&entry.keyframe_id), sizeof(entry.keyframe_id));
    in.read(reinterpret_cast<char*>(&entry.timestamp), sizeof(entry.timestamp));
    entry.pose.id = entry.keyframe_id;
    entry.pose.timestamp = entry.timestamp;
    in.read(reinterpret_cast<char*>(entry.pose.t.data()), sizeof(double) * 3);
    double q[4] = {0.0, 0.0, 0.0, 1.0};
    in.read(reinterpret_cast<char*>(q), sizeof(double) * 4);
    entry.pose.q = Eigen::Quaterniond(q[3], q[0], q[1], q[2]).normalized();
    entry.descriptor = Eigen::MatrixXd::Zero(config_.num_rings, config_.num_sectors);
    for (int r = 0; r < config_.num_rings; ++r) {
      for (int c = 0; c < config_.num_sectors; ++c) {
        in.read(reinterpret_cast<char*>(&entry.descriptor(r, c)), sizeof(double));
      }
    }
    entry.ring_key = makeRingKey(entry.descriptor);
    entry.sector_key = makeSectorKey(entry.descriptor);
    entries_.push_back(std::move(entry));
  }
  return static_cast<bool>(in);
}

/**
 * @brief 在已知 yaw shift 下计算 query 与 candidate 的描述子距离
 *
 * 对每个 sector 列，计算 query 列与 candidate 循环平移后列的余弦相似度，
 * score = 1 - 平均余弦相似度。score 越小表示结构越相似。
 *
 * 优化：加入重叠率惩罚，解决部分观测（单帧扫描）导致的假阳性问题。
 * 当 query 只有部分 sector 有数据时，如果 candidate 在 query 缺失的 sector
 * 有大量数据，会显著增加 score，避免错误匹配。
 */
double ScanContextDescriptor::descriptorDistance(const Eigen::MatrixXd& query,
                                              const Eigen::MatrixXd& candidate,
                                              int shift) const {
  double sum_cos = 0.0;
  int valid_cols = 0;
  int query_nonzero = 0;
  int cand_nonzero = 0;

  for (int c = 0; c < config_.num_sectors; ++c) {
    const int shifted = (c + shift + config_.num_sectors) % config_.num_sectors;
    const Eigen::VectorXd q_col = query.col(c);
    const Eigen::VectorXd c_col = candidate.col(shifted);
    const double q_norm = q_col.norm();
    const double c_norm = c_col.norm();

    if (q_norm >= 1e-6) ++query_nonzero;
    if (c_norm >= 1e-6) ++cand_nonzero;

    if (q_norm < 1e-6 || c_norm < 1e-6) {
      continue;
    }
    sum_cos += q_col.dot(c_col) / (q_norm * c_norm);
    ++valid_cols;
  }

  if (valid_cols == 0) {
    return std::numeric_limits<double>::max();
  }

  const double base_score = 1.0 - sum_cos / static_cast<double>(valid_cols);

  const int overlap_base = std::max(1, std::min(query_nonzero, cand_nonzero));
  const double overlap_ratio = static_cast<double>(valid_cols) / overlap_base;

  const double overlap_penalty = (1.0 - overlap_ratio) * 0.5;

  return base_score + overlap_penalty;
}

/**
 * @brief 用 sector_key 循环平移搜索最佳 yaw 对齐 shift
 *
 * 车辆在同一地点朝向不同时，Scan Context 的 sector 列会发生循环平移。
 * 遍历 shift=0..num_sectors-1，找使 ||query_key - shift(candidate_key)||² 最小的 shift。
 */
int ScanContextDescriptor::fastSectorAlignment(const Eigen::VectorXd& query_sector_key,
                                            const Eigen::VectorXd& candidate_sector_key) const {
  int best_shift = 0;
  double best_norm = std::numeric_limits<double>::max();
  for (int shift = 0; shift < config_.num_sectors; ++shift) {
    double norm = 0.0;
    for (int c = 0; c < config_.num_sectors; ++c) {
      const int shifted = (c + shift) % config_.num_sectors;
      const double diff = query_sector_key(c) - candidate_sector_key(shifted);
      norm += diff * diff;
    }
    if (norm < best_norm) {
      best_norm = norm;
      best_shift = shift;
    }
  }
  return best_shift;
}

std::string ScanContextDescriptor::summary() const {
  return "ScanContext entries=" + std::to_string(entries_.size()) +
         " rings=" + std::to_string(config_.num_rings) +
         " sectors=" + std::to_string(config_.num_sectors) +
         " max_radius=" + std::to_string(config_.max_radius) +
         " min_range=" + std::to_string(config_.min_range) +
         " ring_key_candidates=" + std::to_string(config_.ring_key_candidates);
}

}  // namespace relocalization

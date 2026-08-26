/**
 * @file csc_descriptor.cpp
 * @brief CSC 描述子构建与两阶段检索（从 relocalization_framework 移植）
 *
 * CSC（Cylinder Sector Context）描述子是一种基于圆柱坐标系分割的
 * 点云全局描述子。它将点云沿径向划分为扇区（sector），沿高度划分为
 * 高度层（bin），在每个单元格中提取多通道统计特征，形成紧凑的矩阵表示。
 *
 * 检索采用两阶段策略：
 *  1. 粗筛：利用 RingKey（高度-通道均值向量）快速筛选候选关键帧
 *  2. 精排：对候选帧进行扇区旋转对齐，计算最佳相似度得分并排序
 */

#include <relocalization/descriptor/csc_descriptor.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>

namespace relocalization {

namespace {

// 二进制数据库文件的魔数标识，用于版本校验
constexpr const char* kMagic = "SCR_CSC_DB_V2";

// 描述子支持的最大通道数（max_range, density, intensity, max_z）
constexpr int kMaxChannels = 4;

// 圆周率常量
constexpr double kPi = 3.14159265358979323846;

// ── balanced 相似度模式的距离带划分与权重 ──────────────────────────────
// 在 balanced 模式下，根据点云到中心的距离对匹配误差施加不同权重，
// 以平衡近处细节与远处结构在相似度计算中的贡献。
constexpr double kRangeNearM = 30.0;   // 近距阈值（米）
constexpr double kRangeFarM = 60.0;    // 远距阈值（米）
constexpr double kWeightNear = 0.6;    // 近距权重（降低近处噪声影响）
constexpr double kWeightMid = 1.0;     // 中距权重（标准权重）
constexpr double kWeightFar = 1.3;     // 远距权重（增强远处结构区分度）

/**
 * @brief 将输入值限制在 [0, 1] 区间内
 * @param v 输入值
 * @return  限制后的值
 */
double clamp01(double v) {
  return std::max(0.0, std::min(1.0, v));
}

/**
 * @brief 判断给定通道是否为强度通道
 * @param ch         通道索引
 * @param n_channels 总通道数
 * @return true 表示该通道是强度通道（当通道数≥3时，索引2为强度）
 *
 * 强度通道不参与 RingKey 构建，因为强度信息对旋转不敏感，
 * 且容易受环境光照影响，不适合用于粗筛阶段的高度-位置特征。
 */
bool isIntensityChannel(int ch, int n_channels) {
  return n_channels >= 3 && ch == 2;
}

/**
 * @brief 计算 RingKey 的维度数
 * @param n_bins      高度层数
 * @param n_channels  通道数
 * @return RingKey 向量的长度
 *
 * RingKey 排除强度通道，因此几何通道数为：
 *   n_geo = n_channels        (n_channels < 3)
 *   n_geo = n_channels - 1    (n_channels >= 3)
 */
int zKeyDims(int n_bins, int n_channels) {
  const int n_geo = (n_channels >= 3) ? (n_channels - 1) : n_channels;
  return n_bins * std::max(1, n_geo);
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// 构造函数
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief 使用给定配置构造 CSC 描述子管理器
 * @param config CSC 配置参数，包含扇区数、高度层数、通道数、相似度模式等
 */
CscDescriptor::CscDescriptor(const CscConfig& config) : config_(config) {}

// ─────────────────────────────────────────────────────────────────────────────
// RingKey / SectorKey 构建
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief 从描述子矩阵构建 RingKey（高度-通道均值向量）
 * @param descriptor 输入的描述子矩阵，形状为 (n_bins * n_channels, n_sectors)
 * @return RingKey 向量，形状为 (n_bins * n_geo,)
 *
 * RingKey 是描述子在扇区维度上的均值，反映了每个高度层、每个几何通道
 * 的平均特征。它不包含强度通道，因此对旋转具有不变性，适合作为粗筛特征。
 */
Eigen::VectorXd CscDescriptor::makeRingKey(const Eigen::MatrixXd& descriptor) const {
  const int n_bins = std::max(1, config_.num_height_bins);
  const int n_channels = std::max(1, std::min(config_.num_channels, kMaxChannels));
  Eigen::VectorXd key(zKeyDims(n_bins, n_channels));
  int out = 0;
  for (int bin = 0; bin < n_bins; ++bin) {
    for (int ch = 0; ch < n_channels; ++ch) {
      if (isIntensityChannel(ch, n_channels)) {
        continue;  // 跳过强度通道
      }
      // 对当前高度层和通道的所有扇区取均值
      key(out++) = descriptor.row(bin * n_channels + ch).mean();
    }
  }
  return key;
}

/**
 * @brief 从描述子矩阵构建 SectorKey（扇区均值向量）
 * @param descriptor 输入的描述子矩阵
 * @return SectorKey 向量，形状为 (n_sectors,)
 *
 * SectorKey 是描述子在高度-通道维度上的均值，反映每个扇区的综合特征。
 * 当前实现中未直接用于检索，但可用于快速可视化或辅助分析。
 */
Eigen::VectorXd CscDescriptor::makeSectorKey(const Eigen::MatrixXd& descriptor) const {
  Eigen::VectorXd key(descriptor.cols());
  for (int c = 0; c < descriptor.cols(); ++c) {
    key(c) = descriptor.col(c).mean();
  }
  return key;
}

// ─────────────────────────────────────────────────────────────────────────────
// 描述子构建：点云 → CSC 矩阵
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief 从输入点云构建 CSC 描述子矩阵
 * @param cloud 输入点云（pcl::PointCloud<PointType>）
 * @return CSC 描述子矩阵，形状为 (n_bins * n_channels, n_sectors)
 *
 * 构建流程：
 *  1. 将每个有效点投影到圆柱坐标系 (r, θ, z)
 *  2. 根据角度 θ 划分扇区（sector），根据高度 z 划分高度层（bin）
 *  3. 在每个 (bin, sector) 单元格中统计：
 *     - max_r:   该单元格中点的最大径向距离
 *     - count:   该单元格中点的数量（密度）
 *     - sum_intensity: 强度值之和
 *     - max_z:   该单元格中点的最大高度
 *  4. 对每个单元格的统计量进行归一化，形成多通道特征
 *  5. 组装成矩阵，每行对应一个 (bin, channel)，每列对应一个 sector
 */
Eigen::MatrixXd CscDescriptor::makeDescriptor(const CloudConstPtr& cloud) const {
  const int n_bins = std::max(1, config_.num_height_bins);
  const int n_sectors = std::max(1, config_.num_sectors);
  const int n_channels = std::max(1, std::min(config_.num_channels, kMaxChannels));
  const int rows = n_bins * n_channels;

  // 初始化全零描述子矩阵
  Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(rows, n_sectors);
  if (!cloud || cloud->empty()) {
    return matrix;  // 空点云返回零矩阵
  }

  /**
   * @struct Cell
   * @brief 单个 (bin, sector) 单元格的统计信息
   */
  struct Cell {
    double max_r = 0.0;          // 最大径向距离
    int count = 0;               // 点数量
    double sum_intensity = 0.0;  // 强度值累加
    double max_z = -1e9;         // 最大高度（初始化为极小值）
  };

  // 扁平化存储所有单元格：索引 = bin * n_sectors + sector
  std::vector<Cell> cells(static_cast<std::size_t>(n_bins) * n_sectors);
  const double h_range = config_.max_height - config_.min_height;

  // ── 第一遍遍历：统计每个单元格的原始数据 ──────────────────────────────
  for (const auto& pt : cloud->points) {
    if (!pcl::isFinite(pt)) {
      continue;  // 跳过无效点（NaN/Inf）
    }

    // 计算径向距离 r = sqrt(x^2 + y^2)
    const double r = std::hypot(static_cast<double>(pt.x), static_cast<double>(pt.y));

    // 过滤超出有效范围的点
    if (r < config_.min_range || r > config_.max_radius) {
      continue;
    }
    // 过滤地面点（高度低于地面阈值）
    if (static_cast<double>(pt.z) < config_.ground_threshold) {
      continue;
    }

    // 计算方位角 θ ∈ [0, 2π)
    double theta = std::atan2(static_cast<double>(pt.y), static_cast<double>(pt.x));
    if (theta < 0.0) {
      theta += 2.0 * kPi;
    }

    // 将 θ 映射到扇区索引 [0, n_sectors-1]
    int sector = static_cast<int>(std::floor((theta / (2.0 * kPi)) * n_sectors));
    sector = std::max(0, std::min(n_sectors - 1, sector));

    // 将高度 z 映射到高度层索引 [0, n_bins-1]
    int bin = 0;
    if (h_range > 1e-6) {
      bin = static_cast<int>(std::floor(
          (static_cast<double>(pt.z) - config_.min_height) / h_range * n_bins));
    }
    bin = std::max(0, std::min(n_bins - 1, bin));

    // 更新对应单元格的统计信息
    Cell& cell = cells[static_cast<std::size_t>(bin) * n_sectors + sector];
    cell.max_r = std::max(cell.max_r, r);
    ++cell.count;
    cell.sum_intensity += static_cast<double>(pt.intensity);
    cell.max_z = std::max(cell.max_z, static_cast<double>(pt.z));
  }

  // ── 计算全局归一化因子 ────────────────────────────────────────────────
  double max_count = 1.0;          // 所有单元格中的最大点数（用于密度归一化）
  double max_mean_intensity = 0.0; // 所有单元格中的最大平均强度
  for (const auto& cell : cells) {
    max_count = std::max(max_count, static_cast<double>(cell.count));
    if (cell.count > 0) {
      max_mean_intensity =
          std::max(max_mean_intensity, cell.sum_intensity / static_cast<double>(cell.count));
    }
  }
  if (max_mean_intensity < 1e-9) {
    max_mean_intensity = 0.0;  // 避免除零
  }

  // ── 第二遍遍历：生成归一化特征并填充矩阵 ──────────────────────────────
  for (int bin = 0; bin < n_bins; ++bin) {
    for (int sector = 0; sector < n_sectors; ++sector) {
      const Cell& cell = cells[static_cast<std::size_t>(bin) * n_sectors + sector];
      if (cell.count <= 0) {
        continue;  // 空单元格保持为零
      }

      // 通道 0: 最大径向距离归一化
      const double max_range_norm = clamp01(cell.max_r / config_.max_radius);

      // 通道 1: 密度归一化（使用 log1p 压缩动态范围）
      const double density_norm =
          std::log1p(static_cast<double>(cell.count)) / std::log1p(max_count);

      // 通道 2: 平均强度归一化
      const double mean_intensity =
          cell.sum_intensity / static_cast<double>(cell.count);
      const double intensity_norm =
          (max_mean_intensity > 0.0) ? clamp01(mean_intensity / max_mean_intensity) : 0.0;

      // 通道 3: 最大高度归一化
      const double z_norm = clamp01((cell.max_z - config_.min_height) / h_range);

      // 按通道顺序组装特征值
      const double values[kMaxChannels] = {max_range_norm, density_norm, intensity_norm,
                                           z_norm};
      for (int ch = 0; ch < n_channels; ++ch) {
        matrix(bin * n_channels + ch, sector) = values[ch];
      }
    }
  }
  return matrix;
}

// ─────────────────────────────────────────────────────────────────────────────
// 描述子相似度计算：旋转对齐与最佳匹配
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief 计算查询描述子与候选描述子之间的最佳相似度得分
 * @param query      查询描述子（当前帧）
 * @param candidate  候选描述子（历史关键帧）
 * @param best_shift 输出参数，返回最佳扇区偏移量（用于估计 yaw 角差异）
 * @return 最佳相似度得分（越小表示越相似）
 *
 * 算法流程：
 *  1. 对候选描述子进行扇区循环移位（模拟不同 yaw 角下的对齐）
 *  2. 对每个移位位置计算相似度得分
 *  3. 返回最小得分及对应的移位量
 *
 * 支持两种相似度模式：
 *  - "balanced": 基于余弦距离的加权相似度，按距离带施加不同权重
 *  - 其他:      基于 L1 距离的加权相似度
 */
double CscDescriptor::bestDescriptorScore(const Eigen::MatrixXd& query,
                                          const Eigen::MatrixXd& candidate,
                                          int* best_shift) const {
  const int n_bins = std::max(1, config_.num_height_bins);
  const int n_sectors = std::max(1, config_.num_sectors);
  const int n_channels = std::max(1, std::min(config_.num_channels, kMaxChannels));

  // ── 构建通道权重向量 ──────────────────────────────────────────────────
  // 默认使用均匀权重，若配置中提供了自定义权重则进行归一化
  std::vector<double> weights(static_cast<std::size_t>(n_channels), 1.0 / n_channels);
  if (config_.channel_weights.size() == static_cast<std::size_t>(n_channels)) {
    double sum = 0.0;
    for (double w : config_.channel_weights) {
      sum += w;
    }
    if (sum > 0.0) {
      for (int ch = 0; ch < n_channels; ++ch) {
        weights[static_cast<std::size_t>(ch)] =
            config_.channel_weights[static_cast<std::size_t>(ch)] / sum;
      }
    }
  }

  const bool balanced_mode = (config_.similarity == "balanced");
  double best_score = std::numeric_limits<double>::max();
  int best_shift_local = 0;

  // ── 遍历所有可能的扇区移位（模拟旋转对齐）─────────────────────────────
  for (int shift = 0; shift < n_sectors; ++shift) {
    double score = 0.0;

    if (balanced_mode) {
      // ── Balanced 模式：基于余弦距离的加权相似度 ──────────────────────
      // 该模式考虑了不同距离带的结构重要性差异：
      //   - 近处（<30m）：权重较低，降低噪声和动态物体影响
      //   - 中处（30-60m）：标准权重
      //   - 远处（>60m）：权重较高，增强远处结构的区分能力
      double wsum = 0.0;
      for (int bin = 0; bin < n_bins; ++bin) {
        const int row0 = bin * n_channels;  // 当前高度层的起始行
        for (int c = 0; c < n_sectors; ++c) {
          // 对候选描述子进行循环移位
          const int shifted = (c + shift) % n_sectors;

          // 计算加权后的向量模长和点积
          double q2 = 0.0, c2 = 0.0, dot = 0.0;
          for (int ch = 0; ch < n_channels; ++ch) {
            const double w = weights[static_cast<std::size_t>(ch)];
            const double qv = query(row0 + ch, c) * w;
            const double cv = candidate(row0 + ch, shifted) * w;
            q2 += qv * qv;
            c2 += cv * cv;
            dot += qv * cv;
          }

          // 判断查询和候选在该位置是否都有有效点
          const bool occ_q = q2 > 1e-12;
          const bool occ_c = c2 > 1e-12;
          if (!occ_q && !occ_c) {
            continue;  // 双方都为空，跳过
          }

          // 计算余弦距离 d = 1 - cos(θ)
          double d = 0.0;
          if (occ_q && occ_c) {
            d = 1.0 - dot / (std::sqrt(q2) * std::sqrt(c2));
          } else {
            d = 1.0;  // 一方为空，距离设为最大
          }

          // 根据距离带选择权重
          const double r_norm = query(row0, c);       // 通道0是归一化距离
          const double r = r_norm * config_.max_radius; // 反归一化到实际距离
          double w_r = kWeightMid;
          if (r < kRangeNearM) {
            w_r = kWeightNear;
          } else if (r > kRangeFarM) {
            w_r = kWeightFar;
          }

          score += w_r * d;
          wsum += w_r;
        }
      }
      // 加权平均
      score = (wsum > 0.0) ? score / wsum : std::numeric_limits<double>::max();
    } else {
      // ── 标准模式：基于 L1 距离的加权相似度 ────────────────────────────
      for (int ch = 0; ch < n_channels; ++ch) {
        double dist = 0.0;
        for (int bin = 0; bin < n_bins; ++bin) {
          const int row = bin * n_channels + ch;
          for (int c = 0; c < n_sectors; ++c) {
            const int shifted = (c + shift) % n_sectors;
            dist += std::abs(query(row, c) - candidate(row, shifted));
          }
        }
        score += weights[static_cast<std::size_t>(ch)] * (dist / (n_bins * n_sectors));
      }
    }

    // 更新最佳得分和移位
    if (score < best_score) {
      best_score = score;
      best_shift_local = shift;
    }
  }

  if (best_shift) {
    *best_shift = best_shift_local;
  }
  return best_score;
}

// ─────────────────────────────────────────────────────────────────────────────
// 两阶段检索：RingKey 粗筛 + 描述子精排
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief 在数据库中检索与查询描述子最相似的 top_k 个候选关键帧
 * @param descriptor 查询描述子矩阵（当前帧）
 * @param top_k      返回的候选数量上限
 * @return 候选关键帧列表，按相似度得分升序排列
 *
 * 检索流程：
 *  1. 从查询描述子构建 RingKey
 *  2. 计算 RingKey 与数据库中所有条目的 L2 距离
 *  3. 使用 partial_sort 筛选出距离最近的 pre_n 个候选
 *  4. 对每个候选调用 bestDescriptorScore 进行精排（含旋转对齐）
 *  5. 根据精排得分选取 top_k 个最佳候选
 */
std::vector<DescriptorCandidate> CscDescriptor::query(const Eigen::MatrixXd& descriptor,
                                                      int top_k) const {
  std::vector<DescriptorCandidate> out;
  if (top_k <= 0 || entries_.empty()) {
    return out;  // 无效请求或空数据库
  }

  // 校验描述子维度
  const int rows = config_.num_height_bins * config_.num_channels;
  const int cols = config_.num_sectors;
  if (descriptor.rows() != rows || descriptor.cols() != cols) {
    std::cerr << "[CscDescriptor] query descriptor shape mismatch: got " << descriptor.rows()
              << "x" << descriptor.cols() << " expected " << rows << "x" << cols << std::endl;
    return out;
  }

  // ── 阶段 1: RingKey 粗筛 ─────────────────────────────────────────────
  const Eigen::VectorXd query_ring_key = makeRingKey(descriptor);
  const int n_entries = static_cast<int>(entries_.size());

  // 确定粗筛候选数量：若配置为0则使用全部条目
  const int pre_n = (config_.ring_key_candidates > 0)
                        ? std::min(config_.ring_key_candidates, n_entries)
                        : n_entries;

  /**
   * @struct RingDist
   * @brief RingKey 距离及对应的数据库索引
   */
  struct RingDist {
    double d;   // L2 距离
    int idx;    // 数据库条目索引
  };

  // 计算查询 RingKey 与所有条目的距离
  std::vector<RingDist> ring_dists;
  ring_dists.reserve(static_cast<std::size_t>(n_entries));
  for (int i = 0; i < n_entries; ++i) {
    ring_dists.push_back(
        {(query_ring_key - entries_[static_cast<std::size_t>(i)].ring_key).norm(), i});
  }

  // 使用 partial_sort 高效获取最近的 pre_n 个候选（O(n log k)）
  std::partial_sort(ring_dists.begin(), ring_dists.begin() + pre_n, ring_dists.end(),
                    [](const RingDist& a, const RingDist& b) { return a.d < b.d; });
  ring_dists.resize(static_cast<std::size_t>(pre_n));

  // ── 阶段 2: 描述子精排 ───────────────────────────────────────────────
  out.reserve(static_cast<std::size_t>(pre_n));
  for (const auto& rd : ring_dists) {
    const auto& entry = entries_[static_cast<std::size_t>(rd.idx)];

    // 计算最佳相似度得分及扇区偏移（用于估计 yaw 差异）
    int shift = 0;
    const double score = bestDescriptorScore(descriptor, entry.descriptor, &shift);

    // 构建候选结果
    DescriptorCandidate candidate;
    candidate.keyframe_id = entry.keyframe_id;
    candidate.score = score;
    candidate.ring_dist = rd.d;  // RingKey 粗筛距离（用于调试分析）
    candidate.sector_shift = shift;
    // 根据扇区偏移计算 yaw 角差异：
    //   每个扇区对应角度 = 2π / n_sectors
    //   yaw_diff = -shift * Δθ（负号表示候选需要反向旋转以对齐查询）
    candidate.yaw_diff = -static_cast<double>(shift) * 2.0 * kPi / config_.num_sectors;
    candidate.pose = entry.pose;
    out.push_back(candidate);
  }

  // 按精排得分升序选取 top_k
  const int k = std::min(top_k, static_cast<int>(out.size()));
  std::partial_sort(out.begin(), out.begin() + k, out.end(),
                    [](const DescriptorCandidate& a, const DescriptorCandidate& b) {
                      return a.score < b.score;
                    });
  out.resize(static_cast<std::size_t>(k));

  std::cout << "[CSC-Query] entries=" << n_entries << " pre_n=" << pre_n
            << " top_k=" << out.size() << " zkey_dims=" << query_ring_key.size() << std::endl;
  return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// 数据库持久化：加载
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief 从二进制文件加载 CSC 数据库
 * @param path 数据库文件路径
 * @return 加载成功返回 true，失败返回 false
 *
 * 文件格式（二进制）：
 *  [magic_size] [magic_string]    // 魔数标识与版本校验
 *  [num_sectors] [num_height_bins] [num_channels]
 *  [min_height] [max_height] [max_radius] [min_range] [ground_threshold]
 *  [n_weights] [weight_0] ... [weight_n-1]  // 通道权重
 *  [count]                                  // 条目数量
 *  对每个条目：
 *    [keyframe_id] [timestamp]
 *    [tx] [ty] [tz]                         // 平移向量
 *    [qx] [qy] [qz] [qw]                    // 四元数（xyzw顺序）
 *    [descriptor_matrix] (rows × cols 个 double)
 *
 * 加载后会自动重新计算每个条目的 RingKey 和 SectorKey。
 */
bool CscDescriptor::load(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in.is_open()) {
    std::cerr << "[CscDescriptor] failed to open: " << path << std::endl;
    return false;
  }

  // ── 读取并校验魔数 ────────────────────────────────────────────────────
  std::size_t magic_size = 0;
  in.read(reinterpret_cast<char*>(&magic_size), sizeof(magic_size));
  if (!in || magic_size == 0 || magic_size > 256) {
    std::cerr << "[CscDescriptor] invalid magic size" << std::endl;
    return false;
  }
  std::string magic(magic_size, '\0');
  in.read(&magic[0], static_cast<std::streamsize>(magic_size));
  if (magic != kMagic) {
    std::cerr << "[CscDescriptor] magic mismatch: " << magic << std::endl;
    return false;
  }

  // ── 读取配置参数 ──────────────────────────────────────────────────────
  // 注意：ring_key_candidates 不参与文件存储，保留当前配置值
  const int ring_key_candidates = config_.ring_key_candidates;
  in.read(reinterpret_cast<char*>(&config_.num_sectors), sizeof(config_.num_sectors));
  in.read(reinterpret_cast<char*>(&config_.num_height_bins), sizeof(config_.num_height_bins));
  in.read(reinterpret_cast<char*>(&config_.num_channels), sizeof(config_.num_channels));
  in.read(reinterpret_cast<char*>(&config_.min_height), sizeof(config_.min_height));
  in.read(reinterpret_cast<char*>(&config_.max_height), sizeof(config_.max_height));
  in.read(reinterpret_cast<char*>(&config_.max_radius), sizeof(config_.max_radius));
  in.read(reinterpret_cast<char*>(&config_.min_range), sizeof(config_.min_range));
  in.read(reinterpret_cast<char*>(&config_.ground_threshold),
          sizeof(config_.ground_threshold));

  // 读取通道权重
  std::size_t n_weights = 0;
  in.read(reinterpret_cast<char*>(&n_weights), sizeof(n_weights));
  config_.channel_weights.clear();
  if (n_weights > 0 && n_weights <= 16) {
    config_.channel_weights.resize(n_weights);
    for (std::size_t i = 0; i < n_weights; ++i) {
      in.read(reinterpret_cast<char*>(&config_.channel_weights[i]),
              sizeof(config_.channel_weights[i]));
    }
  }
  config_.ring_key_candidates = ring_key_candidates;  // 恢复保留值

  // ── 读取数据库条目 ────────────────────────────────────────────────────
  const int rows = config_.num_height_bins * config_.num_channels;
  const int cols = config_.num_sectors;
  std::size_t count = 0;
  in.read(reinterpret_cast<char*>(&count), sizeof(count));

  entries_.clear();
  entries_.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    CscEntry entry;

    // 读取关键帧 ID 和时间戳
    in.read(reinterpret_cast<char*>(&entry.keyframe_id), sizeof(entry.keyframe_id));
    in.read(reinterpret_cast<char*>(&entry.timestamp), sizeof(entry.timestamp));

    // 读取位姿（平移 + 四元数）
    entry.pose.id = entry.keyframe_id;
    entry.pose.timestamp = entry.timestamp;
    in.read(reinterpret_cast<char*>(entry.pose.t.data()), sizeof(double) * 3);
    double q[4] = {0.0, 0.0, 0.0, 1.0};
    in.read(reinterpret_cast<char*>(q), sizeof(double) * 4);
    // 注意：文件存储为 xyzw 顺序，Eigen::Quaterniond 构造为 (w, x, y, z)
    entry.pose.q = Eigen::Quaterniond(q[3], q[0], q[1], q[2]).normalized();

    // 读取描述子矩阵
    entry.descriptor = Eigen::MatrixXd::Zero(rows, cols);
    for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
        in.read(reinterpret_cast<char*>(&entry.descriptor(r, c)), sizeof(double));
      }
    }

    // 重新计算派生特征（RingKey 和 SectorKey）
    entry.ring_key = makeRingKey(entry.descriptor);
    entry.sector_key = makeSectorKey(entry.descriptor);
    entries_.push_back(std::move(entry));
  }

  // ── 校验文件完整性 ────────────────────────────────────────────────────
  if (!static_cast<bool>(in)) {
    entries_.clear();
    std::cerr << "[CscDescriptor] truncated or corrupt file: " << path << std::endl;
    return false;
  }

  std::cout << "[CscDescriptor] loaded entries=" << entries_.size()
            << " sectors=" << config_.num_sectors
            << " height_bins=" << config_.num_height_bins
            << " channels=" << config_.num_channels
            << " height=[" << config_.min_height << "," << config_.max_height << "]"
            << " max_radius=" << config_.max_radius << " min_range=" << config_.min_range
            << " ground=" << config_.ground_threshold
            << " ring_key_candidates=" << config_.ring_key_candidates << " path=" << path
            << std::endl;
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// 摘要信息
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief 返回 CSC 描述子管理器的当前状态摘要
 * @return 格式化的状态字符串，包含条目数、配置参数等
 */
std::string CscDescriptor::summary() const {
  std::ostringstream ss;
  ss << "csc entries=" << entries_.size() << " sectors=" << config_.num_sectors
     << " height_bins=" << config_.num_height_bins << " channels=" << config_.num_channels
     << " similarity=" << config_.similarity;
  return ss.str();
}

}  // namespace relocalization
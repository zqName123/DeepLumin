/**
 * @file csc_descriptor.cpp
 * @brief CSC 描述子构建与两阶段检索（从 relocalization_framework 移植）
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

constexpr const char* kMagic = "SCR_CSC_DB_V2";
constexpr int kMaxChannels = 4;
constexpr double kPi = 3.14159265358979323846;
// balanced 相似度的距离带划分与权重（与 relocalization_framework 一致）
constexpr double kRangeNearM = 30.0;
constexpr double kRangeFarM = 60.0;
constexpr double kWeightNear = 0.6;
constexpr double kWeightMid = 1.0;
constexpr double kWeightFar = 1.3;

double clamp01(double v) {
  return std::max(0.0, std::min(1.0, v));
}

bool isIntensityChannel(int ch, int n_channels) {
  return n_channels >= 3 && ch == 2;
}

int zKeyDims(int n_bins, int n_channels) {
  const int n_geo = (n_channels >= 3) ? (n_channels - 1) : n_channels;
  return n_bins * std::max(1, n_geo);
}

}  // namespace

CscDescriptor::CscDescriptor(const CscConfig& config) : config_(config) {}

Eigen::VectorXd CscDescriptor::makeRingKey(const Eigen::MatrixXd& descriptor) const {
  const int n_bins = std::max(1, config_.num_height_bins);
  const int n_channels = std::max(1, std::min(config_.num_channels, kMaxChannels));
  Eigen::VectorXd key(zKeyDims(n_bins, n_channels));
  int out = 0;
  for (int bin = 0; bin < n_bins; ++bin) {
    for (int ch = 0; ch < n_channels; ++ch) {
      if (isIntensityChannel(ch, n_channels)) {
        continue;
      }
      key(out++) = descriptor.row(bin * n_channels + ch).mean();
    }
  }
  return key;
}

Eigen::VectorXd CscDescriptor::makeSectorKey(const Eigen::MatrixXd& descriptor) const {
  Eigen::VectorXd key(descriptor.cols());
  for (int c = 0; c < descriptor.cols(); ++c) {
    key(c) = descriptor.col(c).mean();
  }
  return key;
}

Eigen::MatrixXd CscDescriptor::makeDescriptor(const CloudConstPtr& cloud) const {
  const int n_bins = std::max(1, config_.num_height_bins);
  const int n_sectors = std::max(1, config_.num_sectors);
  const int n_channels = std::max(1, std::min(config_.num_channels, kMaxChannels));
  const int rows = n_bins * n_channels;

  Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(rows, n_sectors);
  if (!cloud || cloud->empty()) {
    return matrix;
  }

  struct Cell {
    double max_r = 0.0;
    int count = 0;
    double sum_intensity = 0.0;
    double max_z = -1e9;
  };
  std::vector<Cell> cells(static_cast<std::size_t>(n_bins) * n_sectors);
  const double h_range = config_.max_height - config_.min_height;

  for (const auto& pt : cloud->points) {
    if (!pcl::isFinite(pt)) {
      continue;
    }
    const double r = std::hypot(static_cast<double>(pt.x), static_cast<double>(pt.y));
    if (r < config_.min_range || r > config_.max_radius) {
      continue;
    }
    if (static_cast<double>(pt.z) < config_.ground_threshold) {
      continue;
    }

    double theta = std::atan2(static_cast<double>(pt.y), static_cast<double>(pt.x));
    if (theta < 0.0) {
      theta += 2.0 * kPi;
    }
    int sector = static_cast<int>(std::floor((theta / (2.0 * kPi)) * n_sectors));
    sector = std::max(0, std::min(n_sectors - 1, sector));

    int bin = 0;
    if (h_range > 1e-6) {
      bin = static_cast<int>(std::floor(
          (static_cast<double>(pt.z) - config_.min_height) / h_range * n_bins));
    }
    bin = std::max(0, std::min(n_bins - 1, bin));

    Cell& cell = cells[static_cast<std::size_t>(bin) * n_sectors + sector];
    cell.max_r = std::max(cell.max_r, r);
    ++cell.count;
    cell.sum_intensity += static_cast<double>(pt.intensity);
    cell.max_z = std::max(cell.max_z, static_cast<double>(pt.z));
  }

  double max_count = 1.0;
  double max_mean_intensity = 0.0;
  for (const auto& cell : cells) {
    max_count = std::max(max_count, static_cast<double>(cell.count));
    if (cell.count > 0) {
      max_mean_intensity =
          std::max(max_mean_intensity, cell.sum_intensity / static_cast<double>(cell.count));
    }
  }
  if (max_mean_intensity < 1e-9) {
    max_mean_intensity = 0.0;
  }

  for (int bin = 0; bin < n_bins; ++bin) {
    for (int sector = 0; sector < n_sectors; ++sector) {
      const Cell& cell = cells[static_cast<std::size_t>(bin) * n_sectors + sector];
      if (cell.count <= 0) {
        continue;
      }
      const double max_range_norm = clamp01(cell.max_r / config_.max_radius);
      const double density_norm =
          std::log1p(static_cast<double>(cell.count)) / std::log1p(max_count);
      const double mean_intensity =
          cell.sum_intensity / static_cast<double>(cell.count);
      const double intensity_norm =
          (max_mean_intensity > 0.0) ? clamp01(mean_intensity / max_mean_intensity) : 0.0;
      const double z_norm = clamp01((cell.max_z - config_.min_height) / h_range);
      const double values[kMaxChannels] = {max_range_norm, density_norm, intensity_norm,
                                           z_norm};
      for (int ch = 0; ch < n_channels; ++ch) {
        matrix(bin * n_channels + ch, sector) = values[ch];
      }
    }
  }
  return matrix;
}

double CscDescriptor::bestDescriptorScore(const Eigen::MatrixXd& query,
                                          const Eigen::MatrixXd& candidate,
                                          int* best_shift) const {
  const int n_bins = std::max(1, config_.num_height_bins);
  const int n_sectors = std::max(1, config_.num_sectors);
  const int n_channels = std::max(1, std::min(config_.num_channels, kMaxChannels));

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
  for (int shift = 0; shift < n_sectors; ++shift) {
    double score = 0.0;
    if (balanced_mode) {
      double wsum = 0.0;
      for (int bin = 0; bin < n_bins; ++bin) {
        const int row0 = bin * n_channels;
        for (int c = 0; c < n_sectors; ++c) {
          const int shifted = (c + shift) % n_sectors;
          double q2 = 0.0, c2 = 0.0, dot = 0.0;
          for (int ch = 0; ch < n_channels; ++ch) {
            const double w = weights[static_cast<std::size_t>(ch)];
            const double qv = query(row0 + ch, c) * w;
            const double cv = candidate(row0 + ch, shifted) * w;
            q2 += qv * qv;
            c2 += cv * cv;
            dot += qv * cv;
          }
          const bool occ_q = q2 > 1e-12;
          const bool occ_c = c2 > 1e-12;
          if (!occ_q && !occ_c) {
            continue;
          }
          double d = 0.0;
          if (occ_q && occ_c) {
            d = 1.0 - dot / (std::sqrt(q2) * std::sqrt(c2));
          } else {
            d = 1.0;
          }
          const double r_norm = query(row0, c);
          const double r = r_norm * config_.max_radius;
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
      score = (wsum > 0.0) ? score / wsum : std::numeric_limits<double>::max();
    } else {
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

std::vector<DescriptorCandidate> CscDescriptor::query(const Eigen::MatrixXd& descriptor,
                                                      int top_k) const {
  std::vector<DescriptorCandidate> out;
  if (top_k <= 0 || entries_.empty()) {
    return out;
  }
  const int rows = config_.num_height_bins * config_.num_channels;
  const int cols = config_.num_sectors;
  if (descriptor.rows() != rows || descriptor.cols() != cols) {
    std::cerr << "[CscDescriptor] query descriptor shape mismatch: got " << descriptor.rows()
              << "x" << descriptor.cols() << " expected " << rows << "x" << cols << std::endl;
    return out;
  }

  const Eigen::VectorXd query_ring_key = makeRingKey(descriptor);
  const int n_entries = static_cast<int>(entries_.size());
  const int pre_n = (config_.ring_key_candidates > 0)
                        ? std::min(config_.ring_key_candidates, n_entries)
                        : n_entries;

  struct RingDist {
    double d;
    int idx;
  };
  std::vector<RingDist> ring_dists;
  ring_dists.reserve(static_cast<std::size_t>(n_entries));
  for (int i = 0; i < n_entries; ++i) {
    ring_dists.push_back(
        {(query_ring_key - entries_[static_cast<std::size_t>(i)].ring_key).norm(), i});
  }
  std::partial_sort(ring_dists.begin(), ring_dists.begin() + pre_n, ring_dists.end(),
                    [](const RingDist& a, const RingDist& b) { return a.d < b.d; });
  ring_dists.resize(static_cast<std::size_t>(pre_n));

  out.reserve(static_cast<std::size_t>(pre_n));
  for (const auto& rd : ring_dists) {
    const auto& entry = entries_[static_cast<std::size_t>(rd.idx)];
    int shift = 0;
    const double score = bestDescriptorScore(descriptor, entry.descriptor, &shift);

    DescriptorCandidate candidate;
    candidate.keyframe_id = entry.keyframe_id;
    candidate.score = score;
    candidate.ring_dist = rd.d;
    candidate.sector_shift = shift;
    candidate.yaw_diff = -static_cast<double>(shift) * 2.0 * kPi / config_.num_sectors;
    candidate.pose = entry.pose;
    out.push_back(candidate);
  }

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

bool CscDescriptor::load(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in.is_open()) {
    std::cerr << "[CscDescriptor] failed to open: " << path << std::endl;
    return false;
  }

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
  config_.ring_key_candidates = ring_key_candidates;

  const int rows = config_.num_height_bins * config_.num_channels;
  const int cols = config_.num_sectors;
  std::size_t count = 0;
  in.read(reinterpret_cast<char*>(&count), sizeof(count));
  entries_.clear();
  entries_.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    CscEntry entry;
    in.read(reinterpret_cast<char*>(&entry.keyframe_id), sizeof(entry.keyframe_id));
    in.read(reinterpret_cast<char*>(&entry.timestamp), sizeof(entry.timestamp));
    entry.pose.id = entry.keyframe_id;
    entry.pose.timestamp = entry.timestamp;
    in.read(reinterpret_cast<char*>(entry.pose.t.data()), sizeof(double) * 3);
    double q[4] = {0.0, 0.0, 0.0, 1.0};
    in.read(reinterpret_cast<char*>(q), sizeof(double) * 4);
    entry.pose.q = Eigen::Quaterniond(q[3], q[0], q[1], q[2]).normalized();
    entry.descriptor = Eigen::MatrixXd::Zero(rows, cols);
    for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
        in.read(reinterpret_cast<char*>(&entry.descriptor(r, c)), sizeof(double));
      }
    }
    entry.ring_key = makeRingKey(entry.descriptor);
    entry.sector_key = makeSectorKey(entry.descriptor);
    entries_.push_back(std::move(entry));
  }

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

std::string CscDescriptor::summary() const {
  std::ostringstream ss;
  ss << "csc entries=" << entries_.size() << " sectors=" << config_.num_sectors
     << " height_bins=" << config_.num_height_bins << " channels=" << config_.num_channels
     << " similarity=" << config_.similarity;
  return ss.str();
}

}  // namespace relocalization

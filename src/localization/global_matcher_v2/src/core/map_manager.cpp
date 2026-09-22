#include "global_matcher_v2/core/map_manager.hpp"

#include <algorithm>
#include <cstdint>
#include <chrono>
#include <cmath>
#include <limits>
#include <fstream>
#include <sstream>
#include <unordered_map>

#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>


namespace global_matcher_v2 {

namespace {

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;
using CloudPtr = CloudT::Ptr;

CloudPtr downsample(const CloudT::ConstPtr& input, double leaf) {
  CloudPtr output(new CloudT());
  if (!input || input->empty()) {
    return output;
  }
  if (leaf <= 1e-4) {
    *output = *input;
    return output;
  }
  // 质心体素降采样（PCL VoxelGrid 语义）：不受 int32 体素数量限制，
  // 适用于坐标范围很大的整图（如 2600x3500m 地图 + 0.4m 体素）
  const double inv_leaf = 1.0 / leaf;
  // 质心体素降采样：与 PCL VoxelGrid 语义一致（体素内取质心），
  // 哈希实现，避免 PCL int32 体素数量限制（适用大范围整图）
  struct VoxelAccum {
    std::size_t out_index;
    double sx, sy, sz;
    std::size_t count;
  };
  std::unordered_map<std::uint64_t, VoxelAccum> voxel_map;
  voxel_map.reserve(input->size() / 4);
  output->reserve(input->size() / 8);
  for (const auto& p : input->points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;
    }
    const std::int64_t ix = static_cast<std::int64_t>(std::floor(p.x * inv_leaf));
    const std::int64_t iy = static_cast<std::int64_t>(std::floor(p.y * inv_leaf));
    const std::int64_t iz = static_cast<std::int64_t>(std::floor(p.z * inv_leaf));
    const std::uint64_t key = (static_cast<std::uint64_t>(ix) & 0x1FFFFF) |
                              ((static_cast<std::uint64_t>(iy) & 0x1FFFFF) << 21) |
                              ((static_cast<std::uint64_t>(iz) & 0x1FFFFF) << 42);
    auto it = voxel_map.find(key);
    if (it == voxel_map.end()) {
      const std::size_t idx = output->size();
      output->push_back(p);
      voxel_map.emplace(key, VoxelAccum{idx, p.x, p.y, p.z, 1u});
    } else {
      it->second.sx += p.x;
      it->second.sy += p.y;
      it->second.sz += p.z;
      ++it->second.count;
    }
  }
  for (const auto& kv : voxel_map) {
    const VoxelAccum& acc = kv.second;
    pcl::PointXYZ& pt = output->points[acc.out_index];
    pt.x = static_cast<float>(acc.sx / static_cast<double>(acc.count));
    pt.y = static_cast<float>(acc.sy / static_cast<double>(acc.count));
    pt.z = static_cast<float>(acc.sz / static_cast<double>(acc.count));
  }
  return output;
}

CloudPtr removeInvalid(const CloudT::ConstPtr& input) {
  CloudPtr output(new CloudT());
  if (!input) {
    return output;
  }
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*input, *output, indices);
  return output;
}

std::pair<int, int> posToGrid(double x, double y, double chunk_size) {
  return {static_cast<int>(std::floor(x / chunk_size + 0.5)),
          static_cast<int>(std::floor(y / chunk_size + 0.5))};
}

}  // namespace

MapManager::~MapManager() {
  {
    std::unique_lock<std::mutex> lock(mutex_);
    stop_ = true;
    cv_.notify_all();
  }
  if (loader_thread_.joinable()) {
    loader_thread_.join();
  }
}

bool MapManager::configure(const MapManagerConfig& config) {
  cfg_ = config;
  return true;
}

bool MapManager::parseManifest() {
  manifest_.clear();
  id_to_info_.clear();

  const std::string chunks_path = cfg_.map_dir + "/" + cfg_.chunks_file;
  std::ifstream fin(chunks_path);
  if (fin.good()) {
    std::string line;
    while (std::getline(fin, line)) {
      if (line.empty() || line[0] == '#') {
        continue;
      }
      std::istringstream ss(line);
      std::string tag;
      ss >> tag;
      if (tag == "chunk_size") {
        ss >> cfg_.chunk_size;
      } else if (tag == "chunk") {
        ChunkInfo info;
        ss >> info.id >> info.gx >> info.gy >> info.min_pt.x() >> info.min_pt.y() >>
            info.min_pt.z() >> info.max_pt.x() >> info.max_pt.y() >> info.max_pt.z() >>
            info.point_count >> info.path;
        if (info.id >= 0 && !info.path.empty()) {
          manifest_.push_back(info);
          id_to_info_[info.id] = info;
        }
      }
    }
    if (!manifest_.empty()) {
      return true;
    }
  }

  // 兼容旧 index.txt（lightning-lm 格式）：首行 "0 0 0" 为占位，跳过
  const std::string index_path = cfg_.map_dir + "/" + cfg_.index_file;
  std::ifstream fin_index(index_path);
  if (!fin_index.good()) {
    return false;
  }
  std::string line;
  bool first = true;
  while (std::getline(fin_index, line)) {
    if (first) {
      first = false;
      continue;
    }
    std::istringstream ss(line);
    ChunkInfo info;
    ss >> info.id >> info.gx >> info.gy >> info.path;
    if (info.id < 0 || info.path.empty()) {
      continue;
    }
    // 旧格式没有边界：按网格约定推导
    info.min_pt << (info.gx - 0.5) * cfg_.chunk_size, (info.gy - 0.5) * cfg_.chunk_size, -1e6;
    info.max_pt << (info.gx + 0.5) * cfg_.chunk_size, (info.gy + 0.5) * cfg_.chunk_size, 1e6;
    manifest_.push_back(info);
    id_to_info_[info.id] = info;
  }
  return !manifest_.empty();
}

bool MapManager::reloadSingleMap(const std::string& pcd_path) {
  if (cfg_.mode != "single") {
    return false;
  }
  ChunkInfo info;
  info.id = 0;
  info.gx = 0;
  info.gy = 0;
  info.path = pcd_path;
  LoadedChunk chunk;
  if (!loadChunk(info, &chunk)) {
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    loaded_[info.id] = chunk;
    active_ids_ = {info.id};
    manifest_.push_back(info);
    id_to_info_[info.id] = info;
    rebuildActiveStructuresLocked();
    map_changed_ = true;
    ++revision_;
  }
  ready_ = true;
  return true;
}

bool MapManager::loadInitial() {
  if (cfg_.mode == "chunked") {
    if (cfg_.map_dir.empty() || !parseManifest()) {
      return false;
    }
    ready_ = true;
    return true;
  }

  // single：整图作为一个分片
  std::string pcd_path = cfg_.pcd_path;
  if (pcd_path.empty() && !cfg_.map_dir.empty()) {
    pcd_path = cfg_.map_dir + "/global.pcd";
  }
  if (pcd_path.empty()) {
    return false;
  }

  ChunkInfo info;
  info.id = 0;
  info.gx = 0;
  info.gy = 0;
  info.path = pcd_path;
  LoadedChunk chunk;
  if (!loadChunk(info, &chunk)) {
    return false;
  }
  {
    std::unique_lock<std::mutex> lock(mutex_);
    loaded_[info.id] = chunk;
    active_ids_ = {info.id};
    manifest_.push_back(info);
    id_to_info_[info.id] = info;
    rebuildActiveStructuresLocked();
    map_changed_ = true;
  }
  ready_ = true;
  return true;
}

bool MapManager::loadChunk(const ChunkInfo& info, LoadedChunk* out) {
  const std::string full_path =
      info.path[0] == '/' ? info.path : cfg_.map_dir + "/" + info.path;
  CloudPtr raw(new CloudT());
  if (pcl::io::loadPCDFile<PointT>(full_path, *raw) != 0 || raw->empty()) {
    return false;
  }
  raw = removeInvalid(raw);
  if (!raw || raw->empty()) {
    return false;
  }

  out->info = info;
  out->pcl = downsample(raw, cfg_.load_voxel_leaf);
  if (!out->pcl || out->pcl->empty()) {
    return false;
  }

  // 同点构建 small_gicp 点云，并一次性估计协方差（之后每帧复用）
  out->sg.reset(new small_gicp::PointCloud());
  out->sg->resize(out->pcl->size());
  for (size_t i = 0; i < out->pcl->size(); ++i) {
    const auto& p = out->pcl->points[i];
    out->sg->point(i) << p.x, p.y, p.z, 1.0;
  }
  small_gicp::estimate_covariances_omp(*out->sg, std::max(3, cfg_.cov_num_neighbors),
                                       std::max(1, cfg_.cov_num_threads));
  return true;
}

void MapManager::startLoaderLocked() {
  if (loader_started_) {
    return;
  }
  loader_started_ = true;
  loader_thread_ = std::thread(&MapManager::loaderThreadLoop, this);
}

void MapManager::loaderThreadLoop() {
  while (true) {
    int id = -1;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(lock, [this] { return stop_ || !load_queue_.empty(); });
      if (stop_) {
        return;
      }
      id = load_queue_.front();
      load_queue_.pop_front();
      queued_ids_.erase(id);
    }

    ChunkInfo info;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      auto it = id_to_info_.find(id);
      if (it == id_to_info_.end()) {
        continue;
      }
      info = it->second;
    }

    LoadedChunk chunk;
    if (!loadChunk(info, &chunk)) {
      // ROS_WARN_THROTTLE(2.0, "[map_manager] failed to load chunk id=%d path=%s", id,
      //                   info.path.c_str());
      SYS_LOG(syslog::LogSeverity::ERROR, true)
                    << "failed to load chunk id=" << id << " path=" << info.path;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        failed_ids_.insert(id);
      }
      continue;
    }

    {
      std::unique_lock<std::mutex> lock(mutex_);
      loaded_[id] = chunk;
      cv_.notify_all();
    }
  }
}

bool MapManager::ensureActive(const Eigen::Vector3d& position) {
  if (!ready_) {
    return false;
  }

  const auto [gx, gy] = posToGrid(position.x(), position.y(), cfg_.chunk_size);
  std::set<int> needed;
  if (cfg_.mode == "single") {
    // 单图：整图恒为所需分片，不按网格距离判定
    if (!manifest_.empty()) {
      needed.insert(manifest_.front().id);
    }
  } else {
    for (const auto& info : manifest_) {
      if (std::abs(info.gx - gx) <= cfg_.load_radius_chunks &&
          std::abs(info.gy - gy) <= cfg_.load_radius_chunks) {
        needed.insert(info.id);
      }
    }
  }
  if (needed.empty()) {
    return false;
  }

  std::unique_lock<std::mutex> lock(mutex_);
  startLoaderLocked();
  ++tick_;

  // 入队缺失分片
  for (int id : needed) {
    if (loaded_.count(id) == 0 && queued_ids_.count(id) == 0 && failed_ids_.count(id) == 0) {
      load_queue_.push_back(id);
      queued_ids_.insert(id);
    }
  }
  for (const auto& kv : loaded_) {
    last_used_[kv.first] = needed.count(kv.first) ? tick_ : last_used_[kv.first];
  }

  // 当前分片必须加载完成（同步等待，避免匹配空地图）
  const auto [cur_gx, cur_gy] = posToGrid(position.x(), position.y(), cfg_.chunk_size);
  int cur_id = -1;
  for (const auto& info : manifest_) {
    if (info.gx == cur_gx && info.gy == cur_gy) {
      cur_id = info.id;
      break;
    }
  }
  if (cur_id >= 0 && loaded_.count(cur_id) == 0) {
    cv_.wait_for(lock, std::chrono::seconds(5), [this, cur_id] { return loaded_.count(cur_id) > 0; });
  }

  // 活动集合 = 已加载的所需分片；变化时重建 target 结构
  std::set<int> new_active;
  for (int id : needed) {
    if (loaded_.count(id) > 0) {
      new_active.insert(id);
    }
  }
  if (new_active.empty()) {
    return false;
  }
  if (new_active != active_ids_) {
    active_ids_ = new_active;
    rebuildActiveStructuresLocked();
    ++revision_;
    map_changed_ = true;
  }
  unloadUnusedLocked(needed);
  return true;
}

void MapManager::unloadUnusedLocked(const std::set<int>& needed) {
  if (static_cast<int>(loaded_.size()) <= cfg_.max_loaded_chunks) {
    return;
  }
  std::vector<int> candidates;
  for (const auto& kv : loaded_) {
    if (needed.count(kv.first) == 0) {
      candidates.push_back(kv.first);
    }
  }
  std::sort(candidates.begin(), candidates.end(),
            [this](int a, int b) { return last_used_[a] < last_used_[b]; });
  for (int id : candidates) {
    if (static_cast<int>(loaded_.size()) <= cfg_.max_loaded_chunks) {
      break;
    }
    loaded_.erase(id);
    last_used_.erase(id);
    queued_ids_.erase(id);
  }
}

void MapManager::rebuildActiveStructuresLocked() {
  size_t total = 0;
  for (int id : active_ids_) {
    auto it = loaded_.find(id);
    if (it != loaded_.end()) {
      total += it->second.sg->size();
    }
  }
  if (total == 0) {
    active_sg_.reset();
    active_pcl_.reset();
    active_kdtree_.reset();
    active_voxel_coarse_.reset();
    active_voxel_fine_.reset();
    return;
  }

  auto sg = std::make_shared<small_gicp::PointCloud>();
  CloudPtr pcl(new CloudT());
  sg->resize(total);
  pcl->resize(total);

  size_t offset = 0;
  for (int id : active_ids_) {
    auto it = loaded_.find(id);
    if (it == loaded_.end()) {
      continue;
    }
    const auto& chunk_sg = it->second.sg;
    const auto& chunk_pcl = it->second.pcl;
    const size_t n = chunk_sg->size();
    for (size_t i = 0; i < n; ++i) {
      sg->point(offset + i) = chunk_sg->point(i);
      sg->cov(offset + i) = chunk_sg->cov(i);
      pcl->points[offset + i] = chunk_pcl->points[i];
    }
    offset += n;
  }

  active_sg_ = sg;
  active_pcl_ = pcl;
  active_kdtree_ = std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(
      sg, small_gicp::KdTreeBuilderOMP(std::max(1, cfg_.cov_num_threads)));

  if (cfg_.prebuild_voxel_maps) {
    active_voxel_coarse_ = std::make_shared<small_gicp::GaussianVoxelMap>(
        cfg_.vgicp_voxel_resolution * cfg_.vgicp_coarse_scale);
    active_voxel_coarse_->set_search_offsets(27);
    active_voxel_coarse_->insert(*sg);

    active_voxel_fine_ = std::make_shared<small_gicp::GaussianVoxelMap>(
        cfg_.vgicp_voxel_resolution * cfg_.vgicp_fine_scale);
    active_voxel_fine_->set_search_offsets(27);
    active_voxel_fine_->insert(*sg);
  } else {
    active_voxel_coarse_.reset();
    active_voxel_fine_.reset();
  }
}

size_t MapManager::activeChunkCount() const {
  std::unique_lock<std::mutex> lock(mutex_);
  return active_ids_.size();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MapManager::cropSubmap(
    const Eigen::Matrix4f& map_base, const CropBoxConfig& crop) const {
  CloudPtr cropped(new CloudT());
  std::unique_lock<std::mutex> lock(mutex_);
  pcl::CropBox<PointT> box;
  box.setMin(Eigen::Vector4f(static_cast<float>(-crop.backward),
                             static_cast<float>(-crop.right),
                             static_cast<float>(-crop.down), 1.0f));
  box.setMax(Eigen::Vector4f(static_cast<float>(crop.forward),
                             static_cast<float>(crop.left),
                             static_cast<float>(crop.up), 1.0f));
  box.setTransform(Eigen::Affine3f(map_base.inverse()));
  for (int id : active_ids_) {
    auto it = loaded_.find(id);
    if (it == loaded_.end()) {
      continue;
    }
    CloudPtr part(new CloudT());
    box.setInputCloud(it->second.pcl);
    box.filter(*part);
    *cropped += *part;
  }
  return cropped;
}

bool MapManager::estimateGroundZ(double x, double y, double radius, double* z_out) const {
  if (radius <= 0.0 || !z_out) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_pcl_ || active_pcl_->empty()) {
    return false;
  }
  const double r2 = radius * radius;
  bool found = false;
  double z_min = std::numeric_limits<double>::max();
  double z_max = std::numeric_limits<double>::lowest();
  for (const auto& p : active_pcl_->points) {
    const double dx = p.x - x;
    const double dy = p.y - y;
    if (dx * dx + dy * dy <= r2) {
      found = true;
      z_min = std::min(z_min, static_cast<double>(p.z));
      z_max = std::max(z_max, static_cast<double>(p.z));
    }
  }
  if (!found) {
    return false;
  }
  *z_out = 0.5 * (z_min + z_max);
  return true;
}
}  // namespace global_matcher_v2

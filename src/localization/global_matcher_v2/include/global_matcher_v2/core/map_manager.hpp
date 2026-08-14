#pragma once

/**
 * @file map_manager.hpp
 * @brief 地图管理器：单图全量加载 / 多图分片实时加载
 *
 * 两种模式：
 * - single : 启动时一次性加载整张 PCD（pcd_path 或 map_dir/global.pcd）
 * - chunked: 解析分片清单（chunks.txt，兼容旧 index.txt），按车辆位置
 *            实时加载当前位置周围 (2*load_radius+1)^2 个分片，超出后 LRU 卸载
 *
 * 性能设计：
 * - 分片加载在后台线程进行，不阻塞匹配；
 * - 每个分片加载时一次性完成体素降采样 + 协方差估计；
 * - 活动分片集合变化时才重建合并的 target 结构（带协方差点云 / PCL 点云 /
 *   KDTree / VGICP coarse+fine 体素地图），匹配帧内零 target 预处理；
 * - 单图模式等价于只有一个永不卸载的分片。
 */

#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <small_gicp/ann/gaussian_voxelmap.hpp>
#include <small_gicp/ann/kdtree.hpp>
#include <small_gicp/points/point_cloud.hpp>

namespace global_matcher_v2 {

/// 分片元数据（manifest 中的一行）
struct ChunkInfo {
  int id = -1;
  int gx = 0;
  int gy = 0;
  std::string path;  ///< 相对 map_dir 的路径
  Eigen::Vector3d min_pt = Eigen::Vector3d::Constant(1e9);
  Eigen::Vector3d max_pt = Eigen::Vector3d::Constant(-1e9);
  size_t point_count = 0;
};

struct MapManagerConfig {
  std::string mode = "single";   ///< single | chunked
  std::string pcd_path;          ///< single 模式：整图 PCD 路径
  std::string map_dir;           ///< chunked 模式：分片目录（含 chunks.txt / index.txt / N.pcd）
  std::string chunks_file = "chunks.txt";
  std::string index_file = "index.txt";
  double chunk_size = 100.0;     ///< 分片边长（与脚本 pos_to_chunk_grid 一致）
  int load_radius_chunks = 1;    ///< 当前分片周围加载半径（分片数）
  double load_voxel_leaf = 0.4;  ///< 分片加载时体素降采样（0 = 不降采样）
  int max_loaded_chunks = 64;    ///< 内存中最多保留的分片数（LRU 淘汰）
  bool async_load = true;        ///< 邻接分片异步加载（当前分片总是同步等待）
  int cov_num_neighbors = 20;    ///< 协方差估计近邻数（加载时一次性计算）
  int cov_num_threads = 4;
  // VGICP 预构建体素地图（活动集合重建时构建，粗/细各一张）
  bool prebuild_voxel_maps = true;
  double vgicp_voxel_resolution = 0.5;
  double vgicp_coarse_scale = 10.0;
  double vgicp_fine_scale = 1.0;
};

/// 可视化裁剪盒（车辆坐标系，单位米）
struct CropBoxConfig {
  double forward = 70.0;
  double backward = 7.0;
  double left = 35.0;
  double right = 35.0;
  double down = 35.0;
  double up = 35.0;
};

/**
 * @brief 地图管理器
 */
class MapManager {
 public:
  MapManager() = default;
  ~MapManager();

  MapManager(const MapManager&) = delete;
  MapManager& operator=(const MapManager&) = delete;

  bool configure(const MapManagerConfig& config);

  /// 解析清单并加载初始地图（chunked 模式只加载清单，地图由 ensureActive 加载）
  bool loadInitial();

  /// single 模式：替换整图（供 load_map_command 使用）
  bool reloadSingleMap(const std::string& pcd_path);

  bool mapReady() const { return ready_; }

  /// 按车辆位置确保所需分片已加载；返回活动集合是否发生变化
  bool ensureActive(const Eigen::Vector3d& position);

  size_t activeChunkCount() const;
  int mapRevision() const { return revision_; }
  bool mapChanged() const { return map_changed_; }
  void consumeMapChanged() { map_changed_ = false; }

  /// 活动地图的 target 结构（map 坐标系，已按 load_voxel_leaf 降采样）
  small_gicp::PointCloud::ConstPtr targetCloud() const { return active_sg_; }
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr targetPcl() const { return active_pcl_; }
  std::shared_ptr<const small_gicp::KdTree<small_gicp::PointCloud>> targetKdTree() const {
    return active_kdtree_;
  }
  small_gicp::GaussianVoxelMap::ConstPtr targetVoxelMapCoarse() const {
    return active_voxel_coarse_;
  }
  small_gicp::GaussianVoxelMap::ConstPtr targetVoxelMapFine() const {
    return active_voxel_fine_;
  }

  /// 从活动地图裁剪可视化子图（map 坐标系）
  pcl::PointCloud<pcl::PointXYZ>::Ptr cropSubmap(const Eigen::Matrix4f& map_base,
                                                 const CropBoxConfig& crop) const;

  const std::vector<ChunkInfo>& manifest() const { return manifest_; }

 private:
  struct LoadedChunk {
    ChunkInfo info;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl;  ///< 降采样后的 PCL 点云（map 系）
    small_gicp::PointCloud::Ptr sg;           ///< 同点 + 预计算协方差
  };

  bool parseManifest();
  bool loadChunk(const ChunkInfo& info, LoadedChunk* out);
  void rebuildActiveStructuresLocked();
  void unloadUnusedLocked(const std::set<int>& needed);
  void loaderThreadLoop();
  void startLoaderLocked();

  MapManagerConfig cfg_;
  std::vector<ChunkInfo> manifest_;
  std::unordered_map<int, ChunkInfo> id_to_info_;

  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<int> load_queue_;
  std::set<int> queued_ids_;
  std::set<int> failed_ids_;
  bool loader_started_ = false;
  bool stop_ = false;
  std::thread loader_thread_;

  std::unordered_map<int, LoadedChunk> loaded_;
  std::set<int> active_ids_;
  uint64_t tick_ = 0;
  std::unordered_map<int, uint64_t> last_used_;

  // 活动集合的重建产物（在 mutex_ 保护下更新；匹配线程串行读取）
  small_gicp::PointCloud::Ptr active_sg_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr active_pcl_;
  std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>> active_kdtree_;
  small_gicp::GaussianVoxelMap::Ptr active_voxel_coarse_;
  small_gicp::GaussianVoxelMap::Ptr active_voxel_fine_;
  int revision_ = 0;
  bool map_changed_ = false;
  bool ready_ = false;
};

}  // namespace global_matcher_v2

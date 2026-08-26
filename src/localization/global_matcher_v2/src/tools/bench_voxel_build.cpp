/**
 * @file bench_voxel_build.cpp
 * @brief VGICP 整图体素地图预构建耗时基准
 *
 * 用法：
 *   bench_voxel_build <pcd_path> [threads] [num_neighbors] [fine_res] [coarse_res]
 *
 * 测量项（与 MapManager 单图模式加载路径一致）：
 *   1. PCD 读取 + 去 NaN
 *   2. 转 small_gicp 点云
 *   3. 协方差估计 estimate_covariances_omp（加载时一次性）
 *   4. KDTree 构建（配准/内点统计用）
 *   5. 粗 / 细 GaussianVoxelMap 插入（VGICP target 预构建）
 */

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <vector>

#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <small_gicp/ann/gaussian_voxelmap.hpp>
#include <small_gicp/ann/kdtree.hpp>
#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>

namespace {

using Clock = std::chrono::steady_clock;

double elapsedMs(const Clock::time_point& start, const Clock::time_point& end) {
  return std::chrono::duration<double, std::milli>(end - start).count();
}

void report(const char* name, const Clock::time_point& start, const Clock::time_point& end) {
  std::printf("%-36s %10.1f ms\n", name, elapsedMs(start, end));
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::printf("usage: %s <pcd_path> [threads] [num_neighbors] [fine_res] [coarse_res]\n",
                argv[0]);
    return 1;
  }
  const std::string pcd_path = argv[1];
  const int threads = argc > 2 ? std::atoi(argv[2]) : 4;
  const int num_neighbors = argc > 3 ? std::atoi(argv[3]) : 20;
  const double fine_res = argc > 4 ? std::atof(argv[4]) : 0.5;
  const double coarse_res = argc > 5 ? std::atof(argv[5]) : 5.0;

  std::printf("threads=%d num_neighbors=%d fine_res=%.2f coarse_res=%.2f\n", threads,
              num_neighbors, fine_res, coarse_res);

  // 1. 读取 + 去 NaN
  auto t0 = Clock::now();
  pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>());
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *raw) != 0 || raw->empty()) {
    std::printf("failed to load %s\n", pcd_path.c_str());
    return 1;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*raw, *cloud, indices);
  auto t1 = Clock::now();
  report("1. load pcd + remove NaN", t0, t1);

  // 2. 转 small_gicp 点云
  auto pc = std::make_shared<small_gicp::PointCloud>();
  pc->resize(cloud->size());
  for (size_t i = 0; i < cloud->size(); ++i) {
    const auto& p = cloud->points[i];
    pc->point(i) << p.x, p.y, p.z, 1.0;
  }
  auto t2 = Clock::now();
  report("2. to small_gicp point cloud", t1, t2);

  // 3. 协方差估计（加载时一次性，供 gicp/vgicp target 使用）
  small_gicp::estimate_covariances_omp(*pc, num_neighbors, threads);
  auto t3 = Clock::now();
  report("3. covariance estimation (omp)", t2, t3);

  // 4. KDTree 构建（配准 / 内点统计）
  small_gicp::KdTree<small_gicp::PointCloud> kdtree(pc,
                                                   small_gicp::KdTreeBuilderOMP(threads));
  auto t4 = Clock::now();
  report("4. kdtree build (omp)", t3, t4);

  // 5. 体素地图插入
  auto coarse = std::make_shared<small_gicp::GaussianVoxelMap>(coarse_res);
  coarse->set_search_offsets(27);
  coarse->insert(*pc);
  auto t5 = Clock::now();
  report("5. coarse voxel map insert", t4, t5);

  auto fine = std::make_shared<small_gicp::GaussianVoxelMap>(fine_res);
  fine->set_search_offsets(27);
  fine->insert(*pc);
  auto t6 = Clock::now();
  report("6. fine voxel map insert", t5, t6);

  std::printf("------------------------------------------------------------\n");
  std::printf("points=%zu coarse_voxels=%zu fine_voxels=%zu\n", pc->size(), coarse->size(),
              fine->size());
  std::printf("prebuild total (cov+kdtree+voxels) : %10.1f ms\n", elapsedMs(t2, t6));
  return 0;
}

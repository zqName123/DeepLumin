#include "lio_odometry/core/types.h"

#include <pcl/impl/pcl_base.hpp>
#include <pcl/filters/impl/voxel_grid.hpp>

template class pcl::PCLBase<slam_odometry::PointXYZIT>;
template class pcl::VoxelGrid<slam_odometry::PointXYZIT>;
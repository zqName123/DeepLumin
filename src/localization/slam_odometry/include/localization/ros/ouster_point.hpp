#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cstdint>

namespace localization_ros {

// 与 ysw_loc faster-lio pointcloud_preprocess.h 中 ouster_ros::Point 一致
namespace ouster_ros {

struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    std::uint32_t t;
    std::uint16_t reflectivity;
    std::uint8_t ring;
    std::uint16_t ambient;
    std::uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace ouster_ros

}  // namespace localization_ros

POINT_CLOUD_REGISTER_POINT_STRUCT(
    localization_ros::ouster_ros::Point,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint32_t, t, t)(
        std::uint16_t, reflectivity, reflectivity)(std::uint8_t, ring, ring)(std::uint16_t, ambient,
                                                                            ambient)(std::uint32_t, range, range))

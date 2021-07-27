#ifndef OPTICALFLOW_SLAM_ALGORITHM_BASE_COMPONENT_TOOL_H_
#define OPTICALFLOW_SLAM_ALGORITHM_BASE_COMPONENT_TOOL_H_

#include "algorithm/common_include.h"

#include "algorithm/base_component/include/feature2d.h"
#include  "algorithm/base_component/include/mappoint3d.h"
#include "algorithm/base_component/include/camera_config.h"
#include "algorithm/opticalflow_slam/include/macro_define.h"


namespace OpticalFlow_SLAM_algorithm_opticalflow_slam {


Vec2 CoordinateTransformWorldToImage(const std::shared_ptr<Mappoint3d>& mappoint3d, const SE3& current_pose, 
                                                                       const Mat33& camera_intrinsics);

Vec3 CoordinateTransformWorldToCamera(const Vec3& world_coordinate_position, const SE3& current_pose);

Vec3 CoordinateTransformImageToCamera(const Vec2& image_coordinate_position, const double_t& depth,
                                                                                          const Mat33& camera_intrinsics);

Vec3 CoordinateTransformImageToNormalizedPlane(const Vec2& image_coordinate_position, 
                                                                                                              const Mat33& camera_intrinsics);

bool TriangulateNormalizedPoint(const Vec3& point1, const Vec3& point2, const SE3& pose1, const SE3& pose2, Vec3& position3d);

} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam

#endif //OPTICALFLOW_SLAM_ALGORITHM_BASE_COMPONENT_TOOL_H_
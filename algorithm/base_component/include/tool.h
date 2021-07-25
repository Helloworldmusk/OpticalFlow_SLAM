#ifndef OPTICALFLOW_SLAM_ALGORITHM_BASE_COMPONENT_TOOL_H_
#define OPTICALFLOW_SLAM_ALGORITHM_BASE_COMPONENT_TOOL_H_

#include "algorithm/common_include.h"

#include "algorithm/base_component/include/feature2d.h"
#include  "algorithm/base_component/include/mappoint3d.h"
#include "algorithm/base_component/include/camera_config.h"


namespace OpticalFlow_SLAM_algorithm_opticalflow_slam {


Vec2 ProjectWorldtoRightCamera(const std::shared_ptr<Mappoint3d> mappoint3d, const SE3 current_pose, 
                                                                       const std::shared_ptr<CameraConfig> camera_config);

Vec3 CoordinateTransformWorldToCamera(const Vec3 world_coordinate_position, const SE3 current_pose);

} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam

#endif //OPTICALFLOW_SLAM_ALGORITHM_BASE_COMPONENT_TOOL_H_
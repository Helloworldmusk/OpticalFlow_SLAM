#include "algorithm/base_component/include/tool.h"
namespace OpticalFlow_SLAM_algorithm_opticalflow_slam {


/**
 * @brief 
 * @author snowden
 * @return coordinate in current image coordinate;
 * @date 2021-07-25
 * @version 1.0
 */ 
Vec2 ProjectWorldtoRightCamera(const std::shared_ptr<Mappoint3d> mappoint3d,  const SE3 current_pose, 
                                                                       const std::shared_ptr<CameraConfig> camera_config)
{
        Vec3 position = CoordinateTransformWorldToCamera(mappoint3d->position3d_, current_pose);
        Vec2 position_2d ;
        position_2d(0) = camera_config->fx_right * position(0) / position(2) + camera_config->cx_right;
        position_2d(1) = camera_config->fy_right * position(1) / position(2) + camera_config->cy_right;
        return position_2d;
}


/**
 * @brief 
 * @author snowden
 * @return coordinate in current camera coordinate system
 * @date 2021-07-25
 * @version 1.0
 */  
Vec3 CoordinateTransformWorldToCamera(const Vec3 world_coordinate_position, const SE3 current_pose)
{
        Vec3 new_position = current_pose * world_coordinate_position;
}


} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam
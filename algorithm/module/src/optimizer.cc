#include "algorithm/module/include/optimizer.h"

namespace OpticalFlow_SLAM_algorithm_opticalflow_slam {

/**
 * @brief 
 * @author snowden
 * @date 2021-07-18
 * @version 1.0
 */
Optimizer::Optimizer( std::weak_ptr<Map> map, const std::shared_ptr<SystemConfig>  sp_slam_config, 
                                             const std::shared_ptr<CameraConfig> sp_camera_config)
        : wp_map_(map), sp_slam_config_(sp_slam_config), sp_camera_config_(sp_camera_config)
{
        
}

} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam
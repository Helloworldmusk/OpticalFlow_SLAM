#include "algorithm/module/include/viewer.h"

namespace OpticalFlow_SLAM_algorithm_opticalflow_slam {

/**
 * @brief 
 * @author snowden
 * @date 2021-07-18
 * @version 1.0
 */
Viewer::Viewer(std::weak_ptr<Map> wp_map, std::weak_ptr<Tracker> wp_tracker, std::weak_ptr<Optimizer> wp_optimizer)
        :wp_map_(wp_map), wp_tracker_(wp_tracker), wp_optimizer_(wp_optimizer)
{

}

} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam
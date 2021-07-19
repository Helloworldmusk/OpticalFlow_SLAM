#include "algorithm/module/include/tracker.h"

namespace OpticalFlow_SLAM_algorithm_opticalflow_slam {

/**
 * @brief 
 * @author snowden
 * @date 2021-07-18
 * @version 1.0
 */
Tracker::Tracker( std::weak_ptr<Map> map, const std::shared_ptr<SystemConfig>  sp_slam_config, 
                                             const std::shared_ptr<CameraConfig> sp_camera_config)
        : wp_map_(map), sp_slam_config_(sp_slam_config), sp_camera_config_(sp_camera_config)
{
        is_running_.store(true);
        front_end_thread_ = std::thread(std::bind(&Tracker::front_end_loop,this));
}


/**
 * @brief  
 * @author snowden
 * @date 2021-07-19
 * @version 1.0
 */
void Tracker::front_end_loop()
{
        while(is_running_)
        {
                std::this_thread::sleep_for(std::chrono::milliseconds(3000));
                SHOW_FUNCTION_INFO
        }
}


/**
 * @brief  join front_end_thread;
 * @author snowden
 * @date 2021-07-19
 * @version 1.0
 */
void Tracker::stop()
{
        front_end_thread_.join();
}

} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam
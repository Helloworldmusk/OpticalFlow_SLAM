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
        is_running_.store(true);
        back_end_thread_ = std::thread(std::bind(&Optimizer::back_end_loop,this));
}

/**
 * @brief 
 * @author snowden
 * @date 2021-07-19
 * @version 1.0
 */
void Optimizer::back_end_loop()
{
        while (is_running_.load())
        {
                // wp_map_->is_front_end_updated.wait(this->wp_map_->data_lock_);
                SHOW_FUNCTION_INFO
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        
}


/**
 * @brief 
 * @author snowden
 * @date 2021-07-19
 * @version 1.0
 */
void Optimizer::stop()
{
        back_end_thread_.join();
}

} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam
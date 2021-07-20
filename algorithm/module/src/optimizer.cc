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
                // std::this_thread::sleep_for(std::chrono::milliseconds(800));
                SHOW_FUNCTION_INFO
                //后端等待前端发送地图更新消息；
                wp_map_.lock()->condition_var_is_map_updated_.wait(wp_map_.lock()->data_lock_);
                DLOG_INFO << " optimizer received map update " << std::endl;
                // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                DLOG_INFO << " optimizer notify all " << std::endl;
                wp_map_.lock()->condition_var_is_map_updated_.notify_all();
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
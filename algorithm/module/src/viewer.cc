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
        is_running_.store(true);
        viewer_thread_ = std::thread(std::bind(&Viewer::viewer_loop,this));
}


/**
 * @brief  update viewer
 * @author snowden
 * @date 2021-07-18
 * @version 1.0
 */
void Viewer::viewer_loop()
{
        while (is_running_.load())
        {
                SHOW_FUNCTION_INFO
                // std::this_thread::sleep_for(std::chrono::milliseconds(100));
                //等待地图更新的通知；
                wp_map_.lock()->condition_var_is_map_updated_.wait(wp_map_.lock()->data_lock_);
                DLOG_INFO << " viewer received map update " << std::endl;
        }
        
}


/**
 * @brief  join view thread;
 * @author snowden
 * @date 2021-07-18
 * @version 1.0
 */
void Viewer::stop()
{
        viewer_thread_.join();
}

} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam
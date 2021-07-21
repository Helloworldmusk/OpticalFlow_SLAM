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
 * @brief  join view thread;
 * @author snowden
 * @date 2021-07-18
 * @version 1.0
 */
void Viewer::stop()
{
        //TODO(snowden) : directly detach thread , if this thread will be terminated ? 
        viewer_thread_.detach(); 
}


/**
 * @brief  viewer thread;
 * @author snowden
 * @date 2021-07-18
 * @version 1.0
 */
void Viewer::viewer_loop()
{
        while (is_running_.load())
        {
                DLOG_INFO << "viewer is running " << std::endl;
                // SHOW_FUNCTION_INFO
                // std::this_thread::sleep_for(std::chrono::milliseconds(100));
                if(wait_update_map_notify())
                {
                        update_viewer();
                }
        }
        DLOG_INFO << " viewer loop is finished " << std::endl;
}


/**
 * @brief  
 * @author snowden
 * @date 2021-07-21
 * @version 1.0
 *  @note wait notify from front_end thread and back_end thread;
 */
bool Viewer::wait_update_map_notify()
{
        //等待地图更新的通知；
        DLOG_INFO << " viewer wait update map notify " << std::endl;
        wp_map_.lock()->condition_var_is_map_updated_.wait(wp_map_.lock()->data_lock_);
        DLOG_INFO << " viewer wait update map notify " << std::endl;
        if(!is_running_.load())
        {
                return false;
        }
        DLOG_INFO << " viewer received map update " << std::endl;
        return true;
}


/**
 * @brief  
 * @author snowden
 * @date 2021-07-21
 * @version 1.0
 */
bool Viewer::update_viewer()
{
        DLOG_INFO << " updated viewer " << std::endl;
}

} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam
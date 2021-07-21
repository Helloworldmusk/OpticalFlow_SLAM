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
 * @date 2021-07-21
 * @version 1.0
 */
void Optimizer::stop()
{
        //TODO(snowden) : directly detach thread , if this thread will be terminated ? 
        back_end_thread_.detach();
}


/**
 * @brief set back end status in some constraint;
 * @return true if set successfully, otherwise return false;
 * @author snowden
 * @date 2021-07-19
 * @version 1.0
 */
bool Optimizer::set_back_end_status(const BackEndStatus &new_status)
{
        bool is_allowed_change { false };
        switch (new_status)
        {
                case BackEndStatus::READY:
                {
                        if (BackEndStatus::UNKNOW == enum_back_end_status_)
                        {
                                is_allowed_change = true;
                        }
                        break;
                }
                case BackEndStatus::INIT:
                {
                        if (BackEndStatus::READY == enum_back_end_status_ || BackEndStatus::RESET == enum_back_end_status_)
                        {
                                is_allowed_change = true;
                        }
                        break;
                }
                case BackEndStatus::IDLE:
                {
                        if (BackEndStatus::INIT == enum_back_end_status_ || BackEndStatus::FINISHED == enum_back_end_status_)
                        {
                                is_allowed_change = true;
                        }
                        break;
                }
                case BackEndStatus::OPTIMIZING:
                {
                        if (BackEndStatus::IDLE == enum_back_end_status_)
                        {
                                is_allowed_change = true;
                        }
                        break;
                } 
                case BackEndStatus::FINISHED:
                {
                        if (BackEndStatus::OPTIMIZING == enum_back_end_status_)
                        {
                                is_allowed_change = true;
                        }
                        break;
                }
                case BackEndStatus::RESET:
                {
                        if (BackEndStatus::OPTIMIZING == enum_back_end_status_ )
                        {
                                is_allowed_change = true;
                        }
                        break;
                }
                default:
                {
                        LOG_ERROR << " set_back_end_status: give a unkonw new status : " << static_cast<int64_t>(new_status) << std::endl;
                        break;
                }
        }
        if(is_allowed_change)
        {
                enum_back_end_status_ = new_status;
        }
        else
        {
                LOG_ERROR << " set_back_end_status error : from " << static_cast<int64_t>(enum_back_end_status_) \
                << " to " << static_cast<int64_t>(new_status) << " is not allowed " << std::endl;
        }
        return is_allowed_change;
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
                DLOG_INFO << " back_end_loop is running " << std::endl;
                switch (get_back_end_status())
                {
                        case BackEndStatus::READY:
                        {
                                DLOG_INFO << " BackEndStatus::READY " << std::endl;
                                set_back_end_status(BackEndStatus::INIT);
                                break;
                        }
                        case BackEndStatus::INIT:
                        {
                                DLOG_INFO << " BackEndStatus::INIT " << std::endl;
                                if (init_back_end())
                                {
                                        set_back_end_status(BackEndStatus::IDLE);
                                }
                                else
                                {
                                        set_back_end_status(BackEndStatus::RESET);
                                }
                                break;
                        }
                        case BackEndStatus::IDLE:
                        {
                                DLOG_INFO << " BackEndStatus::IDLE " << std::endl;
                                if(wait_update_map_notify())
                                {
                                        set_back_end_status(BackEndStatus::OPTIMIZING);
                                }
                                break;
                        }
                        case BackEndStatus::OPTIMIZING:
                        {
                                DLOG_INFO << " BackEndStatus::OPTIMIZING " << std::endl;
                                BackEndStatus status = optimize();
                                set_back_end_status(status);
                                break;
                        }
                        case BackEndStatus::FINISHED:
                        {
                                DLOG_INFO << " BackEndStatus::FINISHED " << std::endl;
                                notify_all_updated_map();
                                set_back_end_status(BackEndStatus::IDLE);
                                break;
                        }
                        case BackEndStatus::RESET:
                        {
                                DLOG_INFO << " BackEndStatus::RESET " << std::endl;
                                set_back_end_status(BackEndStatus::INIT);
                                break;
                        }
                        case BackEndStatus::UNKNOW:
                        {
                                DLOG_INFO << " BackEndStatus::UNKNOW " << std::endl;
                                set_back_end_status(BackEndStatus::READY);
                                break;
                        }
                        default:
                        {
                                LOG_ERROR << " get_back_end_status :  error status : " << static_cast<int64_t>(get_back_end_status()) << std::endl;
                                break;
                        }
                } //switch (get_back_end_status())
        } //while (is_running_.load())
        DLOG_INFO << " back end is terminated " << std::endl;
} //void Optimizer::back_end_loop()


/**
 * @brief 
 * @author snowden
 * @date 2021-07-21
 * @version 1.0
 */
bool Optimizer::init_back_end()
{
        //TODO(snowden) init;
        return true;
}


/**
 * @brief 
 * @return true if received notify;
 * @author snowden
 * @date 2021-07-21
 * @version 1.0
 * @note wait front_end thread notify map have updated ;
 */
//TODO(snowden) : need to change the name of this function , no just wait update_map, but also end optimizer;
bool Optimizer::wait_update_map_notify()
{
        DLOG_INFO << "optimizer wait update map notify " << std::endl;
        wp_map_.lock()->condition_var_is_map_updated_.wait(wp_map_.lock()->data_lock_);
        if(!is_running_.load())
        {
                return false;
        }
        DLOG_INFO << " optimizer received map update " << std::endl;
        return true;
}


/**
 * @brief 
 * @author snowden
 * @date 2021-07-21
 * @version 1.0
 * @note while optimized map, notify all map have updated, mainly for viewer thread;
 */
void Optimizer::notify_all_updated_map()
{
        wp_map_.lock()->condition_var_is_map_updated_.notify_all();
        DLOG_INFO << " optimizer void Optimizer::notify_all_updated_map() " << std::endl;
}


/**
 * @brief 
 * @author snowden
 * @date 2021-07-21
 * @version 1.0
 */
Optimizer::BackEndStatus Optimizer::optimize()
{
        static int64_t counter = 0;
        counter++;
        if (counter < 3) { return BackEndStatus::RESET; } 
        if(counter < 5) { return BackEndStatus::INIT; }
       return BackEndStatus::FINISHED; 
}





} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam
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
        set_front_end_status(FrontEndStatus::READY);
        front_end_thread_ = std::thread(std::bind(&Tracker::front_end_loop,this));
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


/**
 * @brief  change front end status in some constraint;
 * @return true if change successfully;
 * @author snowden
 * @date 2021-07-20
 * @version 1.0
 */
bool Tracker::set_front_end_status(FrontEndStatus new_status)
{
        bool is_allowed_change { false };
        switch (new_status)
        {
                case FrontEndStatus::READY:
                {
                        if (enum_front_end_status_ == FrontEndStatus::UNKONW )
                        {
                                is_allowed_change = true;
                        }
                        break;
                }
                case FrontEndStatus::INITING:
                {
                        if (enum_front_end_status_ == FrontEndStatus::READY || enum_front_end_status_ == FrontEndStatus::RESET)
                        {
                                is_allowed_change = true;
                        }
                        break;
                }
                case FrontEndStatus::TRACKING:
                {
                        if (enum_front_end_status_ == FrontEndStatus::INITING || enum_front_end_status_ == FrontEndStatus::NEED_INSERT_KEYFRAM)
                        {
                                is_allowed_change = true;
                        }
                        break;
                }
                case FrontEndStatus::NEED_INSERT_KEYFRAM:
                {
                        if (enum_front_end_status_ == FrontEndStatus::TRACKING)
                        {
                                is_allowed_change = true;
                        }
                        break;
                }
                case FrontEndStatus::LOST:
                {
                        if (enum_front_end_status_ == FrontEndStatus::TRACKING)
                        {
                                is_allowed_change = true;
                        }
                        break;
                }
                case FrontEndStatus::RESET:
                {
                        if (enum_front_end_status_ == FrontEndStatus::LOST)
                        {
                                is_allowed_change = true;
                        }
                        break;
                }
                case FrontEndStatus::FINISHED:
                {
                        //TODO(snowden) : need to decide which status can change to FINISHED, may be all could;
                        if (enum_front_end_status_ == FrontEndStatus::TRACKING || enum_front_end_status_ == FrontEndStatus::NEED_INSERT_KEYFRAM )
                        {
                                is_allowed_change = true;
                        }
                        break;
                } 
                default:
                {
                        LOG_FATAL <<  "set_front_end_status fatal: unknow status : " << static_cast<int64_t>(new_status) << std::endl; 
                        break;
                }      
         }

        if (is_allowed_change)
        {
                enum_front_end_status_ = new_status;
        }
        else
        {
                LOG_WARNING << " set_front_end_status from  " <<  static_cast<int64_t>(enum_front_end_status_)  << " to " << static_cast<int64_t>(new_status)  << " is not allowed " << std::endl;
        }

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
                switch (get_front_end_status() )
                {
                        case FrontEndStatus::READY:
                        {
                                DLOG_INFO << "FrontEndStatus::READY " << std::endl;
                                set_front_end_status(FrontEndStatus::INITING);
                        }
                        case FrontEndStatus::INITING:
                        {
                                DLOG_INFO << "FrontEndStatus::INITING " << std::endl;
                                if (!init_front_end())
                                {
                                        LOG_ERROR << "init_front_end failed , now ,rest front end " << std::endl;
                                        set_front_end_status(FrontEndStatus::RESET);
                                }
                                else
                                {
                                        set_front_end_status(FrontEndStatus::TRACKING);
                                }
                                break;
                        }
                        case FrontEndStatus::TRACKING:
                        {
                                DLOG_INFO << "FrontEndStatus::TRACKING " << std::endl;
                                auto start_time =  std::chrono::steady_clock::now();
                                FrontEndStatus status =  tracking();
                                auto end_time =  std::chrono::steady_clock::now();                                
                                if (FrontEndStatus::TRACKING == status)
                                {
                                        wp_map_.lock()->condition_var_is_map_updated_.notify_all();
                                        DLOG_INFO << " tracker work corrently , and notify all " << std::endl;

                                        auto time_used = std::chrono::duration_cast<std::chrono::duration<double_t>>(end_time - start_time);
                                        if( sp_slam_config_->per_frame_process_time - time_used.count()*1000.0 > 0)
                                        {
                                                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int64_t>(sp_slam_config_->per_frame_process_time - time_used.count()*1000)));
                                        }
                                }
                                else
                                {
                                        set_front_end_status(status);
                                }
                                break;
                        }
                        //TODO(snowden) : if after TRACKING then directly NEED_INSERT_KEYFRAM, or run another turn ? need to decide;
                        case FrontEndStatus::NEED_INSERT_KEYFRAM:
                        {
                                DLOG_INFO << "FrontEndStatus::NEED_INSERT_KEYFRAM " << std::endl;
                                CHECK_EQ(insert_keyframe(), true);
                                set_front_end_status(FrontEndStatus::TRACKING);
                                break;
                        }
                        case FrontEndStatus::LOST:
                        {
                                DLOG_INFO << "FrontEndStatus::LOST " << std::endl;
                                set_front_end_status(FrontEndStatus::RESET);
                                break;
                        }
                        case FrontEndStatus::RESET:
                        {
                                DLOG_INFO << "FrontEndStatus::RESET " << std::endl;
                                CHECK_EQ(reset(), true);
                                set_front_end_status(FrontEndStatus::INITING);
                                break;
                        }
                        case FrontEndStatus::FINISHED:
                        {
                                DLOG_INFO << "FrontEndStatus::FINISHED " << std::endl;
                                wp_map_.lock()->condition_var_is_map_updated_.notify_all();
                                DLOG_INFO << " finished tracker notify all " << std::endl;
                                is_running_.store(false);
                                break;
                        }
                        case FrontEndStatus::UNKONW:
                        {
                                LOG_FATAL << " FrontEndStatus is UNKONW " << std::endl;
                                break;
                        }
                        default:
                        {
                                LOG_FATAL << "illegal FrontEndStatus :  " << static_cast<int>(get_front_end_status())  << std::endl;
                                break;
                        }
                } //switch
        } //while(is_running_)
} //void Tracker::front_end_loop()


/**
 * @brief  
 * @author snowden
 * @date 2021-07-20
 * @version 1.0
 */
bool Tracker::init_front_end()
{
        SHOW_FUNCTION_INFO
        //TODO(snowden): init;
        return true;
}


/**
 * @brief   tracking point in frames ,and give a rough pose and 3d position;
 * @return  if no image , return FINISHED;  if  tracked little point , return NEED_INSERT_KEYFRAM, 
 *                    if tracked too little point, return LOST, if track normal ,return TRACKING
 * @author snowden
 * @date 2021-07-20
 * @version 1.0
 */
Tracker::FrontEndStatus Tracker::tracking()
{
        SHOW_FUNCTION_INFO
        static int counter = 0;
        counter++;
        //TODO(snowden) : tracking;
        if(counter < 2)
        {
                return FrontEndStatus::TRACKING;
        }
        else if (counter < 5)
        {
                return FrontEndStatus::NEED_INSERT_KEYFRAM;
        }
        else if (counter < 8)
        {
                return FrontEndStatus::LOST;
        }
        else if(counter < 10)
        {
                return FrontEndStatus::FINISHED;
        }

}


/**
 * @brief  
 * @author snowden
 * @date 2021-07-20
 * @version 1.0
 */
bool Tracker::insert_keyframe()
{
        SHOW_FUNCTION_INFO
        //TODO(snowden): insert keyframe;
        return true;
}


/**
 * @brief  
 * @author snowden
 * @date 2021-07-20
 * @version 1.0
 */
bool Tracker::reset()
{
        return true;
}





} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam
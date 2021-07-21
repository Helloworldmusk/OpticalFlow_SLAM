#include "algorithm/opticalflow_slam/include/opticalflow_slam.h"

namespace OpticalFlow_SLAM_algorithm_opticalflow_slam{



/**
 * @brief create a op_slam, load system config , camera_config and dataset, 
 * to filling  slam_config_ , camera_config_ and image_left_right
 * input:
 * @param system_config_path run parameters config
 * @param camera_config_path camer's internal parameters
 * @param dataset_path where to read images
 * @param save_map_path where to save the slam result
 * output:
 * @return OP_SLAM object;
 * @exception no
 * @author snowden
 * @date 2021-07-16
 * @version 1.0
 * @property no
 */ 
OP_SLAM::OP_SLAM(const std::string system_config_path, const std::string camera_config_path,
                                            const std::string dataset_path,  const std::string save_map_path) :
        system_config_path_(system_config_path),
        camera_config_path_(camera_config_path),
        dataset_path_(dataset_path),
        save_map_path_(save_map_path)
{
        // SHOW_FUNCTION_INFO
        is_running_.store(true);
}


/**
 * @brief release the op_slam object;
 * @author snowden
 * @date 2021-07-16
 * @version 1.0
 */ 
OP_SLAM::~OP_SLAM()
{
        //TODO(snowden): need to release dynamic memory
};


/**
 * @brief opticalflow_slam_loop
 * @author snowden
 * @date 2021-07-21
 * @version 1.0
 */ 
void OP_SLAM::opticalflow_slam_loop()
{
        while (is_running_.load())
        {
                DLOG_INFO << " slam is running " << std::endl;
                switch (get_status())
                {
                        case OP_SLAM_STATUS::READY:
                        {
                                DLOG_INFO << " OP_SLAM_STATUS::READY " << std::endl;
                                CHECK_EQ(load_system_config(), true);
                                CHECK_EQ(load_camera_config(), true);
                                CHECK_EQ(load_images(), true);
                                set_status(OP_SLAM_STATUS::INITING);
                                break;
                        }
                        case OP_SLAM_STATUS::INITING:
                        {
                                DLOG_INFO << " OP_SLAM_STATUS::INITING " << std::endl;
                                CHECK_EQ(init(), true);
                                set_status(OP_SLAM_STATUS::RUNNING);
                                break;
                        }
                        case OP_SLAM_STATUS::RUNNING:
                        {
                                DLOG_INFO << " OP_SLAM_STATUS::RUNNING " << std::endl;
                                CHECK_EQ(run(), true);
                                set_status(OP_SLAM_STATUS::SAVINGMAP);
                                break;
                        }
                        case OP_SLAM_STATUS::SAVINGMAP:
                        {
                                DLOG_INFO << " OP_SLAM_STATUS::SAVINGMAP " << std::endl;
                                if (!save_map()) 
                                {
                                        LOG_ERROR << " save map failed , please check " << std::endl;
                                }
                                set_status(OP_SLAM_STATUS::FINISHED);
                                break;
                        }
                        case OP_SLAM_STATUS::FINISHED:
                        {
                                DLOG_INFO << " OP_SLAM_STATUS::FINISHED " << std::endl;
                                stop_slam();

                                break;
                        }
                        case OP_SLAM_STATUS::RESET:
                        {
                                DLOG_INFO << " OP_SLAM_STATUS::RESET " << std::endl;
                                set_status(OP_SLAM_STATUS::INITING);
                                break;
                        }
                        case OP_SLAM_STATUS::UNKNOW:
                        {
                                DLOG_INFO << " OP_SLAM_STATUS::UNKNOW " << std::endl;
                                set_status(OP_SLAM_STATUS::READY);
                                break;
                        }
                        default:
                        {
                                LOG_ERROR << "OP_SLAM_STATUS get_status error:  new state is " \
                                << static_cast<int64_t>(get_status()) << std::endl;
                                break;
                        }
                } //switch (get_status())
        } //while (is_running_.load())
        DLOG_INFO << " OP slam run finished " << std::endl;
} //void OP_SLAM::opticalflow_slam_loop()


/**
 * @brief mainly init sp_map_, sp_tracker_, and sp_optimizer_
 * output
 * @author snowden
 * @date 2021-07-16
 * @version 1.0
 */   
bool OP_SLAM::init()
{
         //init Map
         sp_map_ = std::shared_ptr<Map>(new Map());
         //TODO(snowden): parameter need read from config file;
         uint64_t pyrimid_level_num = 7;
         double_t pyrimid_scale = 0.5;
         uint64_t fps = 10;
        sp_slam_config_ = SystemConfig::getSystemConfig(pyrimid_level_num, pyrimid_scale, fps);
         /**
          * @note tracker will notify viewer and optimizer, and optimizer will notify viewer ,so init order must be :
          *  viewer -> optimizer -> tracker; tracker init will trigger slam run, so put tracker in run funciton; 
          */ 
         //start viewer thread;
         sp_viewer_  = std::shared_ptr<Viewer>(new Viewer(sp_map_, sp_tracker_, sp_optimizer_));
         //start optimizer thread;
         sp_optimizer_ = std::shared_ptr<Optimizer>(new Optimizer(sp_map_, sp_slam_config_, sp_camera_config_));

         if((nullptr != sp_map_ )&& (nullptr != sp_slam_config_) && (nullptr != sp_viewer_) && (nullptr != sp_optimizer_))
         {
                 
                 return true;
         }
         else
         {
                 return false;
         }
        
}


/**
 * @brief include a main loop to run slam system;
 * @author snowden
 * @date 2021-07-16
 * @version 1.0
 */
bool OP_SLAM::run()
{
        sp_tracker_ = std::shared_ptr<Tracker>(new Tracker(sp_map_, sp_slam_config_, sp_camera_config_));
        if(nullptr == sp_tracker_)
        {
                return false;
        }
        sp_tracker_->wp_viewer_ = sp_viewer_;
        sp_tracker_->wp_optimizer_ = sp_optimizer_;
        sp_optimizer_->wp_tracker_ = sp_tracker_;
        sp_optimizer_->wp_viewer_ = sp_viewer_;
        //warning: main thread will be blocked here ,until received tracker's finished notify;
        wait_notify_all_tracker_finished();
        return true;

}


/**
 * @brief save map in save_map_path files;
 * @author snowden
 * @date 2021-07-16
 * @version 1.0
 */
bool OP_SLAM::save_map()
{
        SHOW_FUNCTION_INFO
        return true;
}


/**
 * @brief keep main thread ,keep show viewer;
 * @author snowden
 * @date 2021-07-21
 * @version 1.0
 * @note only tracker , optimizer, and viewer all exit,  slam be allowed exit, 
 * however, optimizer and viewer will always keep alive, so , main thread will be blocked here;
 */
bool OP_SLAM::stop_slam()
{
        DLOG_INFO << " stop slam " << std::endl;
        sp_optimizer_->is_running_.store(false);
        sp_viewer_->is_running_.store(false);
        sp_tracker_->stop();
        DLOG_INFO << " break point 1" << std::endl;
        sp_optimizer_->stop();
        DLOG_INFO << " break point 2" << std::endl;
        sp_viewer_->stop(); 
        DLOG_INFO << " break point 3" << std::endl;
        is_running_.store(false);
        DLOG_INFO << "stop slam finished " << std::endl;
}


/**
 * @brief change slam status in some condition
 * @param new_status
 * @author sowden
  * @date 2021-07-16
 * @version 1.0
 */
bool OP_SLAM::set_status(const OP_SLAM_STATUS &new_status)
{
        SHOW_FUNCTION_INFO
        bool is_allowed_change { false } ;
        switch(new_status)
        {
                case OP_SLAM_STATUS::READY:
                {
                        if (slam_status_ == OP_SLAM_STATUS::UNKNOW)
                        {
                                is_allowed_change = true;
                        }
                        break;
                }
                case OP_SLAM_STATUS::INITING:
                {
                        if ((slam_status_  == OP_SLAM_STATUS::READY) || (slam_status_  == OP_SLAM_STATUS::RESET))
                        {
                                is_allowed_change =  true;
                        }
                        break;
                }
                case OP_SLAM_STATUS::RUNNING:
                {
                        if (slam_status_ == OP_SLAM_STATUS::INITING)
                        {
                                is_allowed_change = true;
                        }
                        break;
                }
                case OP_SLAM_STATUS::SAVINGMAP:
                {
                        if (slam_status_ == OP_SLAM_STATUS::RUNNING)
                        {
                                is_allowed_change = true;
                        }
                        break;
                }
                case OP_SLAM_STATUS::FINISHED:
                {
                        if (slam_status_ == OP_SLAM_STATUS::SAVINGMAP)
                        {
                                is_allowed_change = true;
                        }
                        break;
                }
                case OP_SLAM_STATUS::RESET:
                {
                        if ((slam_status_ == OP_SLAM_STATUS::INITING) || (slam_status_ == OP_SLAM_STATUS::RUNNING) || 
                             (slam_status_ == OP_SLAM_STATUS::FINISHED) || (slam_status_ == OP_SLAM_STATUS::SAVINGMAP) || 
                             (slam_status_ == OP_SLAM_STATUS::UNKNOW) 
                        )
                        {
                                is_allowed_change = true;
                        }
                        break;
                }
                default:
                {
                        LOG_ERROR << "give  OP_SLAM_STATUS is not know " << std::endl;
                        return false;
                }
        } 
        if (is_allowed_change)
        {
                slam_status_ = new_status;
        }
        else
        {
                LOG_ERROR << " can't change slam status from " << static_cast<int64_t>(slam_status_) << " to " << static_cast<int64_t>(new_status) << std::endl;
        }
        return is_allowed_change;
}


/**
 * @brief 
 * @author snowden
 * @date 2021-07-18
 * @version 1.0
 */
bool OP_SLAM::load_system_config()
{
        SHOW_FUNCTION_INFO
        return true;
}


/**
 * @brief 
 * @author snowden
 * @date 2021-07-18
 * @version 1.0
 */
bool OP_SLAM::load_camera_config()
{
        SHOW_FUNCTION_INFO
        return true;
}


/**
 * @brief 
 * @author snowden
 * @date 2021-07-18
 * @version 1.0
 */
bool OP_SLAM::load_images()
{
        SHOW_FUNCTION_INFO
        return true;
}


/**
 * @brief  wait tracker finished notify from tracker thread;
 * @author snowden
 * @date 2021-07-21
 * @version 1.0
 */
void OP_SLAM::wait_notify_all_tracker_finished()
{
        sp_tracker_->condition_variable_is_tracker_finished_.wait(sp_tracker_->tracker_finished_lock);
}


} //OpticalFlow_SLAM_algorithm_opticalflow_slam
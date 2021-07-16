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
        SHOW_FUNCTION_INFO
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
 * @brief mainly init sp_map_, sp_tracker_, and sp_optimizer_
 * output
 * @author snowden
 * @date 2021-07-16
 * @version 1.0
 */   
bool OP_SLAM::init()
{
        SHOW_FUNCTION_INFO

        //TODO(snowden):need to get slam status first;
        is_running = true;
}


/**
 * @brief include a main loop to run slam system;
 * @author snowden
 * @date 2021-07-16
 * @version 1.0
 */
bool OP_SLAM::run()
{
        SHOW_FUNCTION_INFO
        while(is_running)
        {
                SHOW_FUNCTION_INFO
                break;
        }
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
}


/**
 * @brief change slam status in some condition
 * @param new_status
 * @author sowden
  * @date 2021-07-16
 * @version 1.0
 */
bool OP_SLAM::set_status(OP_SLAM_STATUS new_status)
{
        SHOW_FUNCTION_INFO
        bool result = false;
        switch(new_status)
        {
                case OP_SLAM_STATUS::INITING:
                {
                        if ((slam_status_  == OP_SLAM_STATUS::READY) && (slam_status_  == OP_SLAM_STATUS::RESET))
                        {
                                slam_status_ = new_status;
                                result =  true;
                        }
                        break;
                }
                case OP_SLAM_STATUS::RUNING:
                {
                        if (slam_status_ == OP_SLAM_STATUS::INITING)
                        {
                                slam_status_ = new_status;
                                result = true;
                        }
                        break;
                }
                case OP_SLAM_STATUS::FINISHED:
                {
                        if (slam_status_ == OP_SLAM_STATUS::RUNING)
                        {
                                slam_status_ = new_status;
                                result = true;
                        }
                        break;
                }
                case OP_SLAM_STATUS::SAVINGMAP:
                {
                        if (slam_status_ == OP_SLAM_STATUS::FINISHED)
                        {
                                slam_status_ = new_status;
                                result = true;
                        }
                        break;
                }
                case OP_SLAM_STATUS::RESET:
                {
                        if ((slam_status_ == OP_SLAM_STATUS::INITING) || (slam_status_ == OP_SLAM_STATUS::RUNING) || 
                             (slam_status_ == OP_SLAM_STATUS::FINISHED) || (slam_status_ == OP_SLAM_STATUS::SAVINGMAP) || 
                             (slam_status_ == OP_SLAM_STATUS::UNKNOW) 
                        )
                        {
                                slam_status_ = new_status;
                                result = true;
                        }
                        break;
                }
                default:
                {
                        //TODO(snowden) : should use log instead of cout;
                        std::cout << "give  OP_SLAM_STATUS is not knowden " << std::endl;
                        return false;
                }
        } 
        if (!result)
        {
                //TODO(snowden) : should use log instead of cout;
                std::cout << " can't change slam status from " << static_cast<int64_t>(slam_status_) << " to " << static_cast<int64_t>(new_status) << std::endl;
        }
        return result;
}

} //OpticalFlow_SLAM_algorithm_opticalflow_slam
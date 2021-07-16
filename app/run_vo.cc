#include<iostream>

#include <memory> //for smart pointer; 

#include "algorithm/opticalflow_slam/include/opticalflow_slam.h"

namespace  SLAM = OpticalFlow_SLAM_algorithm_opticalflow_slam;
int main()
{
        std::string system_config_path = "";
        std::string camera_config_path = "";
        std::string data_set_path = "";
        std::string save_mat_path = "";
        std::unique_ptr<SLAM::OP_SLAM> up_slam { 
                new SLAM::OP_SLAM(system_config_path, camera_config_path, data_set_path, save_mat_path)  };
        up_slam->init();
        up_slam->run();
        up_slam->save_map();
}


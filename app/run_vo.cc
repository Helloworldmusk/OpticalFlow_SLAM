#include <unistd.h>

#include<iostream>

#include <memory> //for smart pointer; 

#include<glog/logging.h>
#include <gflags/gflags.h>

#include "algorithm/opticalflow_slam/include/opticalflow_slam.h"

// DEFINE_int32(logtostderr, 1, "to stderr");
// DEFINE_int32(minloglevel, 0, "start from info");
// DEFINE_int32(stderrthreshold, 0, "start form info");
namespace  SLAM = OpticalFlow_SLAM_algorithm_opticalflow_slam;
int main(int argc, char* argv[])
{       
        //set log to screem;
        FLAGS_logtostderr=1;
        //show log from INFO 
        FLAGS_minloglevel=0 ;
        google::InitGoogleLogging(argv[0]);
        DLOG_INFO << " this is a dlog test " << std::endl;
        std::string work_space_path = "/home/snowden/workplace/OpticalFlow_SLAM/OpticalFlow_SLAM/";
        std::string system_config_path = work_space_path + "config/system_config.yaml";
        std::string camera_config_path ="/home/snowden/workplace/dataset/05/calib.txt";
        std::string data_set_path = "/home/snowden/workplace/dataset/05";
        std::string save_mat_path = "/home/snowden/workplace/OpticalFlow_SLAM/OpticalFlow_SLAM";
        // std::cout << " current dir " << current_path() << std::endl;;
        std::unique_ptr<SLAM::OP_SLAM> op_slam { 
                new SLAM::OP_SLAM(system_config_path, camera_config_path, data_set_path, save_mat_path)  };
        op_slam->opticalflow_slam_loop();
}


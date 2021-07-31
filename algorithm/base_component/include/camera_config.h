#ifndef OPTICALFLOW_SLAM_ALGORITHM_BASE_COMPONENT_CAMERA_CONFIG_H_
#define OPTICALFLOW_SLAM_ALGORITHM_BASE_COMPONENT_CAMERA_CONFIG_H_

#include "algorithm/common_include.h"

namespace OpticalFlow_SLAM_algorithm_opticalflow_slam {

/**
 * @brief Camera config params
 * @author snowden
 * @date 2021-07-16
 * @version 1.0
 */        
struct CameraConfig  {
         EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    public:

        ~CameraConfig() {};
        void show_camera_config_info();

        static std::shared_ptr<CameraConfig>  getCameraConfig();
         double_t fx_left { 0.0 };
         double_t fy_left { 0.0 };
         double_t cx_left { 0.0 };
         double_t cy_left { 0.0 };
         double_t r1_left { 0.0 };
         double_t r2_left { 0.0 };
         double_t r3_left { 0.0 };
         double_t p1_left { 0.0 };
         double_t p2_left { 0.0 };

         double_t fx_right { 0.0 };
         double_t fy_right { 0.0 };
         double_t cx_right { 0.0 };
         double_t cy_right { 0.0 };
         double_t r1_right { 0.0 };
         double_t r2_right { 0.0 };
         double_t r3_right { 0.0 };
         double_t p1_right { 0.0 };
         double_t p2_right { 0.0 };

        //TODO(snowden) : K_left and K_right will be accessed by mutil thread, so need be set private and set lock;
        Mat33 K_left;
        Mat33 K_right;
        /**
         * @warning: base_line may be negative number,  not absolute distance;
         */ 
        Vec3 base_line;
        Mat44 T_left;
        Mat44 T_right;
         

    private:
        //TODO(snowden): need add synchronized operate for mulit thread;
        static std::shared_ptr<CameraConfig> camera_config ;
        CameraConfig() {};

}; //CameraConfig

} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam

#endif //OPTICALFLOW_SLAM_ALGORITHM_BASE_COMPONENT_CAMERA_CONFIG_H_
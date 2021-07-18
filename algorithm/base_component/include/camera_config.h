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
        CameraConfig() {};
        ~CameraConfig() {};

        static CameraConfig* getCameraConfig(double_t d_fx, double_t d_fy, double_t d_cx, double_t d_cy,
                                                                                          double_t d_r1, double_t d_r2, double_t d_r3, double_t d_p1, double_t d_p2);
         double_t fx { 0.0 };
         double_t fy { 0.0 };
         double_t cx { 0.0 };
         double_t cy { 0.0 };

         double_t r1 { 0.0 };
         double_t r2 { 0.0 };
         double_t r3 { 0.0 };

         double_t p1 { 0.0 };
         double_t p2 { 0.0 };

    private:
        //TODO(snowden): need add synchronized operate for mulit thread;
        static CameraConfig* camera_config ;

        CameraConfig(double_t d_fx, double_t d_fy, double_t d_cx, double_t d_cy,
                                       double_t d_r1, double_t d_r2, double_t d_r3, double_t d_p1, double_t d_p2);


}; //CameraConfig

} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam

#endif //OPTICALFLOW_SLAM_ALGORITHM_BASE_COMPONENT_CAMERA_CONFIG_H_
#ifndef OPTICALFLOW_SLAM_ALGORITHM_BASE_COMPONENT_CAMERA_CONFIG_H_
#define OPTICALFLOW_SLAM_ALGORITHM_BASE_COMPONENT_CAMERA_CONFIG_H_

#include "algorithm/common_include.h"

namespace OpticalFlow_SLAM_algorithm_opticalflow_slam {
/**
 * Camera config params;
 */
struct CameraConfig  {
         EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

         double_t fx { 0.0 };
         double_t fy { 0.0 };
         double_t cx { 0.0 };
         double_t cy { 0.0 };

         double_t r1 { 0.0 };
         double_t r2 { 0.0 };

         double_t r3 { 0.0 };
         double_t p1 { 0.0 };
         double_t p2 { 0.0 };

}; //CameraConfig

} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam

#endif //OPTICALFLOW_SLAM_ALGORITHM_BASE_COMPONENT_CAMERA_CONFIG_H_
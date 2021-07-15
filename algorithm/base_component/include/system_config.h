#ifndef OPTICALFLOW_SLAM_ALGORITHM_BASE_COMPONENT_SYSTEM_CONFIG_H_
#define OPTICALFLOW_SLAM_ALGORITHM_BASE_COMPONENT_SYSTEM_CONFIG_H_

#include "algorithm/common_include.h"

namespace OpticalFlow_SLAM_algorithm_opticalflow_slam {
/**
 * System config params;
 */
struct SystemConfig {
         EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        //pyrimid config;
       static constexpr int64_t pyrimid_levels_num_ {4};
        double_t pyrimid_sacle {0.5};
        std::array<double_t,pyrimid_levels_num_>  a_pyrimid_scales_;

        int64_t fps {10};

}; //SystemConfig

} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam

#endif //OPTICALFLOW_SLAM_ALGORITHM_BASE_COMPONENT_SYSTEM_CONFIG_H_
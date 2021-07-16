#ifndef OPTICALFLOW_SLAM_ALGORITHM_BASE_COMPONENT_SYSTEM_CONFIG_H_
#define OPTICALFLOW_SLAM_ALGORITHM_BASE_COMPONENT_SYSTEM_CONFIG_H_

#include "algorithm/common_include.h"


namespace OpticalFlow_SLAM_algorithm_opticalflow_slam {

/**
 * @brief System config params
 * @author snowden
 * @date 2021-07-16
 * @version 1.0
 */  
struct SystemConfig {
         EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    public:
        SystemConfig() {}
        ~SystemConfig();
        static SystemConfig* getSystemConfig(const int64_t &i_pyrimid_levels_num, const double_t &d_pyrimid_scale, 
                                                                                        const int64_t &i_fps );
        //pyrimid config;
        int64_t pyrimid_levels_num {4};
        double_t pyrimid_scale {0.5};
        std::vector<double_t>  v_pyrimid_scales;
        int64_t fps{10};

    private:
        SystemConfig(const int64_t &i_pyrimid_levels_num, const double_t &d_pyrimid_scale, const int64_t &i_fps );

        //TODO(snowden): need add synchronized operate for mulit thread;
        static SystemConfig* system_config;

}; //SystemConfig

} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam

#endif //OPTICALFLOW_SLAM_ALGORITHM_BASE_COMPONENT_SYSTEM_CONFIG_H_
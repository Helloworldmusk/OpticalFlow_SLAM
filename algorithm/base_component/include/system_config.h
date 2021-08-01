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
        
        ~SystemConfig();
        void show_system_config_info();
        static std::shared_ptr<SystemConfig> getSystemConfig( );        
        //pyrimid config;
        int64_t pyrimid_levels_num {4};
        double_t pyrimid_scale {0.5};
        std::vector<double_t>  v_pyrimid_scales;
        int64_t fps{10};
        double_t per_frame_process_time {1000.0 / fps} ;
        int64_t features_expected_nums { 80 };
        int64_t features_init_min_threshold { 50 };
        int64_t features_tracking_min_threshold { 50 };
        int64_t mappoint_init_min_threshold { 50 };
        /** if tracked mappoint nums between features_expected_nums and  mappoint_need_insert_keyframe_min_threshold, 
          *   a  keyframe is needed;
          */
        int64_t mappoint_need_insert_keyframe_min_threshold { 60 };


    private:
        SystemConfig(){};
        //TODO(snowden): need add synchronized operate for mulit thread;
        static std::shared_ptr<SystemConfig>  system_config;

}; //SystemConfig

} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam

#endif //OPTICALFLOW_SLAM_ALGORITHM_BASE_COMPONENT_SYSTEM_CONFIG_H_
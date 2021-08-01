#include "algorithm/base_component/include/system_config.h"

namespace OpticalFlow_SLAM_algorithm_opticalflow_slam {

std::shared_ptr<SystemConfig>  SystemConfig::system_config = nullptr;

// /**
//  * @brief Singleton Pattern, to get a SystemConfig instance
//  * @author snowden
//  * @date 2021-07-16
//  * @version 1.0
//  * @note  need add sysnchronized operation
//  * @warning : if call getSystemConfig twice, the second call will use the first instance, not second 
//  */
// std::shared_ptr<SystemConfig>  SystemConfig::getSystemConfig(const int64_t &i_pyrimid_levels_num, const double_t &d_pyrimid_scale, 
//                                                                                                    const int64_t &i_fps )
// {
        
//         if (nullptr == system_config)
//         {
//                 //TODO(snowden): need add synchronized operate for mulit thread; DCL， synchronized (SystemConfig.class)  reference: https://www.runoob.com/design-pattern/singleton-pattern.html
//                 if (nullptr == system_config)
//                 {
//                         system_config = std::shared_ptr<SystemConfig>(new SystemConfig(i_pyrimid_levels_num, d_pyrimid_scale, i_fps));
//                 }
//         }
//         else if ((system_config->pyrimid_levels_num != i_pyrimid_levels_num) ||
//                        (system_config->pyrimid_scale != d_pyrimid_scale) ||
//                        (system_config->fps != i_fps) )
//         {
//                 //TODO(snowden) : need use log rather than cout ;
//                 std::cout << "Already exist SystemConfig, and you use a diffrent parameters to get another instance ,it is not allowed , so you get a exist config " << std::endl;
//         }

//         return system_config;

// }

/**
 * @brief Singleton Pattern, to get a SystemConfig instance
 * @author snowden
 * @date 2021-07-22
 * @version 1.0
 * @note  need add sysnchronized operation
 * @warning : if call getSystemConfig twice, the second call will use the first instance, not second 
 */
std::shared_ptr<SystemConfig>  SystemConfig::getSystemConfig( )
{
        
        if (nullptr == system_config)
        {
                //TODO(snowden): need add synchronized operate for mulit thread; DCL， synchronized (SystemConfig.class)  reference: https://www.runoob.com/design-pattern/singleton-pattern.html
                if (nullptr == system_config)
                {
                        system_config = std::shared_ptr<SystemConfig>(new SystemConfig());
                }
        }
        return system_config;

}


// /**
//  * @brief SystemConfig
//  * @author snowden
//  * @date 2021-07-16
//  * @version 1.0
//  */  
// SystemConfig::SystemConfig(const int64_t &i_pyrimid_levels_num, const double_t &d_pyrimid_scale, 
//                                                              const int64_t &i_fps ) 
//         : pyrimid_levels_num(i_pyrimid_levels_num),pyrimid_scale(d_pyrimid_scale),fps(i_fps)
// {
//         v_pyrimid_scales.push_back(1.0);
//         for(int i = 0; i < pyrimid_levels_num ; i++ )
//         {
//                 v_pyrimid_scales.push_back(v_pyrimid_scales[v_pyrimid_scales.size() - 1] * pyrimid_scale );
//         }
//         //TODO(snowden) the loop just for test, need to be deleted;
//         // std::cout << " pyrimid_scales: ";
//         // for(int i = 0; i < pyrimid_levels_num ; i++ )
//         // {
//         //         std::cout << v_pyrimid_scales[i] << " -> " << std::endl;
//         // }
// }


/**
 * @brief ~SystemConfig
 * @author snowden
 * @date 2021-07-16
 * @version 1.0
 */  
SystemConfig::~SystemConfig()
{

}


/**
 * @brief show_system_config_info
 * @author snowden
 * @date 2021-07-22
 * @version 1.0
 */  
void SystemConfig::show_system_config_info()
{
        std::cout << "**************************** system config start  ******************************" << std::endl;        
        std::cout << "  pyrimid_levels_num  :" <<  pyrimid_levels_num << std::endl;
        std::cout << "  pyrimid_scale :" <<   pyrimid_scale << std::endl;
        std::cout << "  fps :" << fps << std::endl;
        std::cout << "  per_frame_process_time :" << per_frame_process_time << std::endl; 
        std::cout << "  features_expected_nums :" <<  features_expected_nums << std::endl;
        std::cout << "  features_init_min_threshold :" << features_init_min_threshold << std::endl;
        std::cout << "  features_tracking_min_threshold :" << features_tracking_min_threshold << std::endl;
        std::cout << "  mappoint_init_min_threshold :" << mappoint_init_min_threshold << std::endl;
        std::cout << "  mappoint_need_insert_keyframe_min_threshold :" << mappoint_need_insert_keyframe_min_threshold << std::endl;
        std::cout << "**************************** system config start  ******************************" << std::endl;        
} 


} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam

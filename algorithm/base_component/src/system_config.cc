#include "algorithm/base_component/include/system_config.h"

namespace OpticalFlow_SLAM_algorithm_opticalflow_slam {

std::shared_ptr<SystemConfig>  SystemConfig::system_config = nullptr;

/**
 * @brief Singleton Pattern, to get a SystemConfig instance
 * @author snowden
 * @date 2021-07-16
 * @version 1.0
 * @note  need add sysnchronized operation
 * @warning : if call getSystemConfig twice, the second call will use the first instance, not second 
 */
std::shared_ptr<SystemConfig>  SystemConfig::getSystemConfig(const int64_t &i_pyrimid_levels_num, const double_t &d_pyrimid_scale, 
                                                                                                   const int64_t &i_fps )
{
        
        if (nullptr == system_config)
        {
                //TODO(snowden): need add synchronized operate for mulit thread; DCL， synchronized (SystemConfig.class)  reference: https://www.runoob.com/design-pattern/singleton-pattern.html
                if (nullptr == system_config)
                {
                        system_config = std::shared_ptr<SystemConfig>(new SystemConfig(i_pyrimid_levels_num, d_pyrimid_scale, i_fps));
                }
        }
        else if ((system_config->pyrimid_levels_num != i_pyrimid_levels_num) ||
                       (system_config->pyrimid_scale != d_pyrimid_scale) ||
                       (system_config->fps != i_fps) )
        {
                //TODO(snowden) : need use log rather than cout ;
                std::cout << "Already exist SystemConfig, and you use a diffrent parameters to get another instance ,it is not allowed , so you get a exist config " << std::endl;
        }

        return system_config;

}


/**
 * @brief SystemConfig
 * @author snowden
 * @date 2021-07-16
 * @version 1.0
 */  
SystemConfig::SystemConfig(const int64_t &i_pyrimid_levels_num, const double_t &d_pyrimid_scale, 
                                                             const int64_t &i_fps ) 
        : pyrimid_levels_num(i_pyrimid_levels_num),pyrimid_scale(d_pyrimid_scale),fps(i_fps)
{
        v_pyrimid_scales.push_back(1.0);
        for(int i = 0; i < pyrimid_levels_num ; i++ )
        {
                v_pyrimid_scales.push_back(v_pyrimid_scales[v_pyrimid_scales.size() - 1] * pyrimid_scale );
        }
        //TODO(snowden) the loop just for test, need to be deleted;
        // std::cout << " pyrimid_scales: ";
        // for(int i = 0; i < pyrimid_levels_num ; i++ )
        // {
        //         std::cout << v_pyrimid_scales[i] << " -> " << std::endl;
        // }
}


/**
 * @brief ~SystemConfig
 * @author snowden
 * @date 2021-07-16
 * @version 1.0
 */  
SystemConfig::~SystemConfig()
{

}



} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam


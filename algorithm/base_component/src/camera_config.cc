
#include "algorithm/base_component/include/camera_config.h"

namespace OpticalFlow_SLAM_algorithm_opticalflow_slam {

std::shared_ptr<CameraConfig>  CameraConfig::camera_config  = nullptr;

void CameraConfig::show_camera_config_info()
{
        std::cout << "**************************** camera config start  ******************************" << std::endl;


        std::cout << " fx_left  :  " <<  fx_left  << std::endl;
        std::cout << " fy_left  :  " <<  fy_left  << std::endl;
        std::cout << " cx_left  :  " <<  cx_left  << std::endl;
        std::cout << " cy_left  :  " <<  cy_left  << std::endl;
        std::cout << " r1_left  :  " <<  r1_left  << std::endl;
        std::cout << " r2_left  :  " <<  r2_left  << std::endl;
        std::cout << " r3_left  :  " <<  r3_left  << std::endl;
        std::cout << " p1_left  :  " <<  p1_left  << std::endl;
        std::cout << " p2_left  :  " <<  p2_left  << std::endl;
        std::cout << std::endl;
        std::cout << " fx_right  :  " <<  fx_right  << std::endl;
        std::cout << " fy_right  :  " <<  fy_right  << std::endl;
        std::cout << " cx_right  :  " <<  cx_right  << std::endl;
        std::cout << " cy_right  :  " <<  cy_right  << std::endl;
        std::cout << " r1_right  :  " <<  r1_right  << std::endl;
        std::cout << " r2_right  :  " <<  r2_right  << std::endl;
        std::cout << " r3_right  :  " <<  r3_right  << std::endl;
        std::cout << " p1_right  :  " <<  p1_right  << std::endl;
        std::cout << " p2_right  :  " <<  p2_right  << std::endl;


        std::cout << "baseline : " << base_line(0) << " " << base_line(1) << " " << base_line(2) << std::endl;
        std::cout << "**************************** camera config end  ******************************" << std::endl;
}

// /**
//  * @brief Singleton Pattern, to get a CameraConfig instance
//  * @property public
//  * @author snowden
//  * @date 2021-07-16
//  * @version 1.0
//  * @note  need add sysnchronized operation
//  * @warning : if call getCameraConfig twice, the second call will use the first instance, not second 
//  */        
// CameraConfig* CameraConfig::getCameraConfig(double_t d_fx, double_t d_fy, double_t d_cx, double_t d_cy,
//                                                                                                       double_t d_r1, double_t d_r2, double_t d_r3, double_t d_p1, double_t d_p2)
// {
//         if (nullptr ==  camera_config)
//         {
//                 //TODO(snowden): need add synchronized operate for mulit thread; DCL， synchronized (CameraConfig.class)  reference: https://www.runoob.com/design-pattern/singleton-pattern.html
//                 if (nullptr ==  camera_config)
//                 {
//                         camera_config = new CameraConfig(d_fx,  d_fy,  d_cx,  d_cy, d_r1,  d_r2,  d_r3,  d_p1,  d_p2);
//                 }
//         }
//         else if ( ( camera_config->fx != d_fx) || ( camera_config->fy != d_fy) || ( camera_config->cx != d_cx) || ( camera_config->cy != d_cy) ||
//                         ( camera_config->r1 != d_r1) || ( camera_config->r2 != d_r2) || ( camera_config->r3 != d_r3) || ( camera_config->p1 != d_p1) || 
//                         ( camera_config->p2 != d_p2) )
//         {
//                 //TODO(snowden) : need use log rather than cout ;
//                 std::cout << "Already exist CameraConfig, and you use a diffrent parameters to get another instance ,it is not allowed , so you get a exist config " << std::endl;
//         }

//         return camera_config;
        
// }

/**
 * @brief Singleton Pattern, to get a CameraConfig instance
 * @property public
 * @author snowden
 * @date 2021-07-22
 * @version 1.0
 * @note  need add sysnchronized operation
 * @warning : if call getCameraConfig twice, the second call will use the first instance, not second 
 */        
std::shared_ptr<CameraConfig> CameraConfig::getCameraConfig()
{
        if (nullptr ==  camera_config)
        {
                //TODO(snowden): need add synchronized operate for mulit thread; DCL， synchronized (CameraConfig.class)  reference: https://www.runoob.com/design-pattern/singleton-pattern.html
                if (nullptr ==  camera_config)
                {
                        camera_config = std::shared_ptr<CameraConfig>(new CameraConfig());
                }
        }
        return camera_config;
        
}

// /**
//  * @brief CameraConfig
//  * @property private
//  * @author snowden
//  * @date 2021-07-16
//  * @version 1.0
//  */        
// CameraConfig::CameraConfig()
// {
        
// }


} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam


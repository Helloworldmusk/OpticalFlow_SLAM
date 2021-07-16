#include "algorithm/base_component/include/mappoint3d.h"

namespace OpticalFlow_SLAM_algorithm_opticalflow_slam {

int64_t Mappoint3d::static_next_3d_id_ = -1;

/**
 * @brief 
 * @author snowden
 * @date 2021-07-16
 * @version 1.0
 */
Mappoint3d::Mappoint3d()
{
        id_ = get_new_id();
}


/**
 * @brief 
 * @author snowden
 * @date 2021-07-16
 * @version 1.0
 */
Mappoint3d::Mappoint3d(const int64_t timestamp, const Vec3 position3d) : 
        timestamp_(timestamp), position3d_(position3d)
{
        id_ = get_new_id();
}


/**
 * @brief 
 * @author snowden
 * @date 2021-07-16
 * @version 1.0
 */
Mappoint3d::~Mappoint3d()
{

}


} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam
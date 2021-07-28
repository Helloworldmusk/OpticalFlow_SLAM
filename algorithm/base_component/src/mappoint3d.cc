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


/**
 * @brief 
 * @author snowden
 * @date 2021-07-28
 * @version 1.0
 */
void Mappoint3d::set_position3d(const Vec3& position3d)
{
        std::unique_lock<std::mutex> position3d_lock { position3d_mutex };
        position3d_ = position3d;
}


/**
 * @brief 
 * @author snowden
 * @date 2021-07-28
 * @version 1.0
 */
Vec3 Mappoint3d::get_position3d()
{
        std::unique_lock<std::mutex> position3d_lock { position3d_mutex };
        return position3d_;
}

} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam
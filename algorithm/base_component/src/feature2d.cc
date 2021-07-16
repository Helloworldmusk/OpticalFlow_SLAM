#include "algorithm/base_component/include/feature2d.h"

namespace OpticalFlow_SLAM_algorithm_opticalflow_slam {

 int64_t Feature2d::static_next_2d_id_ = -1;

/**
 * @brief initialize a Feature with (0.0, 0.0); 
 * @author snowden
 * @date 2021-07-16
 * @version 1.0
 */
Feature2d:: Feature2d()
{
        //other member use default value;
        id_ = get_new_id();
}


/**
 * @brief 
 * @author snowden
 * @date 2021-07-16
 * @version 1.0
 */
Feature2d:: Feature2d(const Vec2 position) : position2d_(position)
{
        id_ = get_new_id();
}


/**
 * @brief 
 * @author snowden
 * @date 2021-07-16
 * @version 1.0
 */
Feature2d:: ~Feature2d()
{

}



} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam
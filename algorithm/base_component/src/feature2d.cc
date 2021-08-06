#include "algorithm/base_component/include/feature2d.h"

namespace OpticalFlow_SLAM_algorithm_opticalflow_slam {

 int64_t Feature2d::static_next_2d_id_ = -1;

/**
 * @brief initialize a Feature with (0.0, 0.0); 
 * @author snowden
 * @date 2021-07-16
 * @version 1.0
 */
Feature2d::Feature2d()
{
        //other member use default value;
        id_ = get_new_id();
}


Feature2d::Feature2d(  cv::KeyPoint keypoint) 
        :cv_keypoint_(keypoint)
{
        id_ = get_new_id();
        position2d_.x() = cv_keypoint_.pt.x;
        position2d_.y() = cv_keypoint_.pt.y;
}

/**
 * @brief 
 * @author snowden
 * @date 2021-07-16
 * @version 1.0
 */
Feature2d:: Feature2d(const Vec2 position) : position2d_(position)
{
        cv_keypoint_.pt.x = position2d_.x();
        cv_keypoint_.pt.y = position2d_.y();
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
        std::cout << "destory feature2d : " << std::endl;
}


/**
 *  @brief 
 *  @author snowden
 *  @date 2021-07-28 
 *  @version 1.0
 */
void Feature2d::set_mappoint3d_linked(const std::shared_ptr<Mappoint3d>& mappoint3d)
{
        std::unique_lock<std::mutex> linked_mappoint3d_lock { linked_mappoint3d_mutex_ };
        sp_mappiont3d_ = mappoint3d;
}
/**
 *  @brief 
 *  @author snowden
 *  @date 2021-07-28 
 *  @version 1.0
 */
std::shared_ptr<Mappoint3d> Feature2d::get_mappoint3d_linked()
{
        std::unique_lock<std::mutex> linked_mappoint3d_lock { linked_mappoint3d_mutex_ };
        return sp_mappiont3d_;
}
/**
 *  @brief 
 *  @author snowden
 *  @date 2021-07-28 
 *  @version 1.0
 */
void Feature2d::set_frame_linked(const std::shared_ptr<Frame>& sp_frame)
{
        // std::unique_lock<std::mutex> linked_frame_lock_ { linked_frame_mutex_ };
        wp_frame_ = sp_frame;
}
/**
 *  @brief 
 *  @author snowden
 *  @date 2021-07-28 
 *  @version 1.0
 */
std::weak_ptr<Frame> Feature2d::get_frame_linked()
{
        // std::unique_lock<std::mutex> linked_frame_lock_ { linked_frame_mutex_ };
        // std::cout << " feature2d id :   " <<  this->id_ << std::endl;
        return wp_frame_;
}


} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam
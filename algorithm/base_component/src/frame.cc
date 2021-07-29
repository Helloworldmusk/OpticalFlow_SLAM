#include "algorithm/base_component/include/frame.h"

namespace OpticalFlow_SLAM_algorithm_opticalflow_slam {

int64_t Frame::static_new_id = -1;

/**
 * @brief 
 * @author snowden
 * @date 2021-07-16
 * @version 1.0
 */
Frame::Frame(const int64_t timestamp, const  cv::Mat left_image, const cv::Mat right_image) :
        timestamp_(timestamp), left_image_(left_image), right_image_(right_image)
{
        // id_ = get_new_id();
}


/**
 * @brief 
 * @author snowden
 * @date 2021-07-16
 * @version 1.0
 */
Frame::~Frame()
{

}


SE3 Frame::get_left_pose()
{
    //TODO(snowden) : may not need to lock;
    std::unique_lock<std::mutex> left_pose_lock(left_pose_mutex_);
    return left_pose_;
}


void Frame::set_left_pose(SE3 new_pose)
{
    std::unique_lock<std::mutex> left_pose_lock(left_pose_mutex_);
    left_pose_ = new_pose;
    return ;
}


SE3 Frame::get_right_pose()
{
    std::unique_lock<std::mutex> right_pose_lock(right_pose_mutex_);
    return right_pose_;
}


void Frame::set_right_pose(SE3 new_pose)
{
    std::unique_lock<std::mutex> right_pose_lock(right_pose_mutex_);
    right_pose_ = new_pose;
    return ;
}

} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam
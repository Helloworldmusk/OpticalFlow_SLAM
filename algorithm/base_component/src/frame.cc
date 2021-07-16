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
        id_ = get_new_id();
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


} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam
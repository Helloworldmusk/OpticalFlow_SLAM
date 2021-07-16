#include "algorithm/base_component/include/keyframe.h"

namespace OpticalFlow_SLAM_algorithm_opticalflow_slam {

/**
 *  @brief KeyFrame
 *  @author snowden
 *  @date 2021-07-16 
 *  @version 1.0
 */
KeyFrame::KeyFrame(std::shared_ptr<Frame> sp_frame) : sp_frame_(sp_frame)
{
        //use default data
}

/**
 *  @brief ~KeyFrame
 *  @author snowden
 *  @date 2021-07-16 
 *  @version 1.0
 */
KeyFrame::~KeyFrame()
{

}



} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam
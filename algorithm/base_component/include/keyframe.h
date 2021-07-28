#ifndef OPTICALFLOW_SLAM_ALGORITHM_BASE_COMPONENT_KEYFRAME_H_
#define OPTICALFLOW_SLAM_ALGORITHM_BASE_COMPONENT_KEYFRAME_H_

#include "algorithm/common_include.h"
#include "algorithm/base_component/include/frame.h"

namespace OpticalFlow_SLAM_algorithm_opticalflow_slam {

/**
 * @brief KeyFrame
 * @author snowden
 * @date 2021-07-16
 * @version 1.0
 * @note it's pose will be modified by mulit thread
 */        
class KeyFrame : public Frame {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        KeyFrame() = delete;
        KeyFrame(std::shared_ptr<Frame> sp_frame);
        ~KeyFrame();
        
        bool get_is_actived()  { return is_actived_; }
        void set_is_actived(bool value) { is_actived_ = value; }
        
        std::shared_ptr<Frame> sp_frame_ { nullptr };
        std::atomic<bool> is_actived_ { true };
    protected:

    private:

}; //KeyFrame

} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam

#endif //OPTICALFLOW_SLAM_ALGORITHM_BASE_COMPONENT_KEYFRAME_H_
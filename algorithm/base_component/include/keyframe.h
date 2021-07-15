#ifndef OPTICALFLOW_SLAM_ALGORITHM_BASE_COMPONENT_KEYFRAME_H_
#define OPTICALFLOW_SLAM_ALGORITHM_BASE_COMPONENT_KEYFRAME_H_

#include "algorithm/common_include.h"
#include "algorithm/base_component/include/frame.h"

namespace OpticalFlow_SLAM_algorithm_opticalflow_slam {
/**
 *  KeyFrame 
 * @note it's pose will be modified by mulit thread;
 */
class KeyFrame : public Frame {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        KeyFrame();
        ~KeyFrame();
        
        bool is_actived_ { true };
    protected:

    private:

}; //KeyFrame

} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam

#endif //OPTICALFLOW_SLAM_ALGORITHM_BASE_COMPONENT_KEYFRAME_H_
#ifndef OPTICALFLOW_SLAM_ALGORITHM_BASE_COMPONENT_FRAME_H_
#define OPTICALFLOW_SLAM_ALGORITHM_BASE_COMPONENT_FRAME_H_

#include "algorithm/common_include.h"
#include "algorithm/base_component/include/feature2d.h"
namespace OpticalFlow_SLAM_algorithm_opticalflow_slam {

class Feature2d;

/**
 *  @brief Frame
 *  @author snowden
 *  @date 2021-07-16 
 *  @version 1.0
 */
class Frame {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        static int64_t static_new_id;

        Frame() {};
        Frame(const int64_t timestamp, const  cv::Mat left_image, const cv::Mat right_image);
        ~Frame();
        int64_t get_new_id() { static_new_id++ ; return static_new_id; }

        int64_t id_ { -1 };
        int64_t timestamp_ { -1 };
        cv::Mat left_image_;
        cv::Mat right_image_;
        SE3 pose_  ;
        std::vector<std::shared_ptr<Feature2d>> vsp_left_feature_;
        std::vector<std::shared_ptr<Feature2d>> vsp_right_feature_;
    protected:

    private:

}; //Frame

} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam

#endif //OPTICALFLOW_SLAM_ALGORITHM_BASE_COMPONENT_FRAME_H_
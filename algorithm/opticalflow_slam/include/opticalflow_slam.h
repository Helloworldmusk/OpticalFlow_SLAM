#ifndef OPTICALFLOW_SLAM_ALGORITHM_OPTICALFLOW_SLAM_H_
#define OPTICALFLOW_SLAM_ALGORITHM_OPTICALFLOW_SLAM_H_

#include "algorithm/common_include.h"
#include "algorithm/base_component/include/feature2d.h"
#include "algorithm/base_component/include/frame.h"
#include "algorithm/base_component/include/keyframe.h"
#include "algorithm/base_component/include/mappoint3d.h"
#include "algorithm/base_component/include/camera_config.h"
#include "algorithm/base_component/include/system_config.h"
#include "algorithm/module/include/map.h"
#include "algorithm/module/include/optimizer.h"
#include "algorithm/module/include/tracker.h"
#include "algorithm/module/include/viewer.h"

namespace OpticalFlow_SLAM_algorithm_opticalflow_slam {
/**
 * 光流法里程计opticalflow_slam -> OP_SLAM
 */
class OP_SLAM {
    public:
        //typedefs
        //enum
        //const
        OP_SLAM(){};
        ~OP_SLAM(){};
        //member function

        //data
        std::vector<std::pair<cv::Mat, cv::Mat>> image_left_right;
        std::shared_ptr<Map> sp_map_;
        std::shared_ptr<Tracker> sp_tracker_;
        std::shared_ptr<Optimizer> sp_optimizer_;
    protected:

    private:

        //data
        std::string dataset_path;
        std::string system_config_path;
        std::string camera_config_path;
        std::string save_map_path;

        SystemConfig  slam_config_;
        CameraConfig camera_config_;
        double_t rpe { 0.0 };
        double_t ate { 0.0 };

}; //class OP_SLAM 

} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam

#endif //OPTICALFLOW_SLAM_ALGORITHM_OPTICALFLOW_SLAM_H_
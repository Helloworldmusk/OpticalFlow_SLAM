#ifndef OPTICALFLOW_SLAM_ALGORITHM_MODULE_MAP_H_
#define OPTICALFLOW_SLAM_ALGORITHM_MODULE_MAP_H_

#include "algorithm/common_include.h"
#include "algorithm/base_component/include/feature2d.h"
#include "algorithm/base_component/include/frame.h"
#include "algorithm/base_component/include/keyframe.h"
#include "algorithm/base_component/include/mappoint3d.h"

namespace OpticalFlow_SLAM_algorithm_opticalflow_slam {

/**
 * @brief Map
 * @author snowden
 * @date 2021-07-16
 * @version 1.0
 * @note it's frame pose  and mappoint position will be modified by mulit threads
 */  
class Map {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        static constexpr int64_t actived_frame_num_{ 6 };

        Map();
        ~Map();

        std::vector<std::shared_ptr<Frame>> vsp_frame_;
        std::vector<std::shared_ptr<KeyFrame>> vsp_keyframe_;
        std::vector<std::shared_ptr<Mappoint3d>> vsp_mappoint_;
        std::vector<std::shared_ptr<KeyFrame>> vsp_actived_keyframe_;
        std::vector<std::shared_ptr<Mappoint3d>> vsp_actived_mappoint_; 
        std::weak_ptr<Frame> wp_current_frame_;
        std::weak_ptr<Frame> wp_last_frame_;       
    protected:

    private:

}; //Map

} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam


#endif //OPTICALFLOW_SLAM_ALGORITHM_MODULE_MAP_H_
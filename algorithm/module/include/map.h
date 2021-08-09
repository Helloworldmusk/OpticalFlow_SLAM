#ifndef OPTICALFLOW_SLAM_ALGORITHM_MODULE_MAP_H_
#define OPTICALFLOW_SLAM_ALGORITHM_MODULE_MAP_H_

#include "algorithm/common_include.h"

#include <condition_variable>

#include "algorithm/base_component/include/feature2d.h"
#include "algorithm/base_component/include/frame.h"
#include "algorithm/base_component/include/keyframe.h"
#include "algorithm/base_component/include/mappoint3d.h"
#include "algorithm/opticalflow_slam/include/macro_define.h"


namespace OpticalFlow_SLAM_algorithm_opticalflow_slam {

/**
 * @brief Map
 * @author snowden
 * @date 2021-07-16
 * @version 1.0
 * @note it's frame pose  and mappoint position will be modified by mulit threads
 *  maybe changed by other mulit threads: 
 */  
class Map {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        static constexpr int64_t kMaxActivedFrameNums { 6 };

        Map() { };
        ~Map() { };
        bool add_frame(std::shared_ptr<Frame> sp_frame);
        bool add_keyframe(std::shared_ptr<KeyFrame> sp_keyframe);
        bool add_mappoint(std::shared_ptr<Mappoint3d> sp_mappoint);
        std::vector<std::shared_ptr<Frame>> get_frames();
        std::vector<std::shared_ptr<KeyFrame>> get_keyframes();
        std::vector<std::shared_ptr<Mappoint3d>> get_mappoints();
        std::deque<std::shared_ptr<KeyFrame>> get_actived_keyframes();
        std::deque<std::shared_ptr<Mappoint3d>> get_actived_mappoints();


        std::mutex frames_mutex_;
        // std::unique_lock<std::mutex> frames_lock_{frames_mutex_};       

        std::mutex keyframe_mutex_;
        // std::unique_lock<std::mutex> keyframe_lock_{keyframe_mutex_};       

        std::mutex mappoint_mutex_;
        // std::unique_lock<std::mutex> mappoint_lock_{mappoint_mutex_};       

        std::mutex actived_keyframe_mutex_;
        // std::unique_lock<std::mutex> actived_keyframe_lock_{actived_keyframe_mutex_};       

        std::mutex actived_mappoint_mutex_;
        // std::unique_lock<std::mutex> actived_mappoint_lock_{actived_mappoint_mutex_};       

        std::mutex data_mutex_;
        std::unique_lock<std::mutex> data_lock_{data_mutex_};       

        std::condition_variable condition_var_is_map_updated_;

    protected:

    private:
        std::vector<std::shared_ptr<Frame>> vsp_frames_;
        std::vector<std::shared_ptr<KeyFrame>> vsp_keyframes_;
        std::vector<std::shared_ptr<Mappoint3d>> vsp_mappoints_;
        std::deque<std::shared_ptr<KeyFrame>> dsp_actived_keyframes_;
        std::deque<std::shared_ptr<Mappoint3d>> dsp_actived_mappoints_; 
}; //Map

} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam

#endif //OPTICALFLOW_SLAM_ALGORITHM_MODULE_MAP_H_
#ifndef OPTICALFLOW_SLAM_ALGORITHM_MODULE_TRACKER_H_
#define OPTICALFLOW_SLAM_ALGORITHM_MODULE_TRACKER_H_

#include "algorithm/common_include.h"

#include <thread>
#include <chrono>
#include <atomic>
#include <condition_variable>

#include "algorithm/base_component/include/feature2d.h"
#include "algorithm/base_component/include/frame.h"
#include "algorithm/base_component/include/keyframe.h"
#include "algorithm/base_component/include/mappoint3d.h"
#include "algorithm/base_component/include/system_config.h"
#include "algorithm/base_component/include/camera_config.h"
#include "algorithm/module/include/map.h"
#include "algorithm/module/include/optimizer.h"
#include "algorithm/module/include/viewer.h"
#include "algorithm/opticalflow_slam/include/macro_define.h"

namespace OpticalFlow_SLAM_algorithm_opticalflow_slam {

class Optimizer;
class Viewer;

/**
 * @brief Tracker
 * @author snowden
 * @date 2021-07-16
 * @version 1.0
 * @note include a frontend thread, will interactive with Optimizer  and Map
 */  
class Tracker {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        enum class FrontEndStatus : std::int64_t {
                READY,
                INITING,
                TRACKING,
                NEED_INSERT_KEYFRAM,
                LOST,
                RESET,
                FINISHED,
                UNKNOW,
                NUM
        };
        Tracker( std::weak_ptr<Map> map, const std::shared_ptr<SystemConfig>  sp_slam_config, 
                          const std::shared_ptr<CameraConfig> sp_camera_config);
        ~Tracker() {};
        void stop();
        FrontEndStatus get_front_end_status() { return enum_front_end_status_; }
        bool set_front_end_status(const FrontEndStatus &new_status);

    
        std::weak_ptr<Map> wp_map_;
        std::weak_ptr<Optimizer> wp_optimizer_;
        std::weak_ptr<Viewer> wp_viewer_;
        std::weak_ptr<Frame> wp_current_frame_; 
        std::weak_ptr<Frame> wp_last_frame_;
        std::shared_ptr<SystemConfig>  sp_slam_config_;
        std::shared_ptr<CameraConfig> sp_camera_config_;
        //last frame pose -> current_frame;
        SE3 relative_motion_;
        int64_t tracked_inliers_num_ { -1 };
        //params
        //TODO(snowden) : need be initilized by system_config files;
        // if init success, detected feature num > init_feature_num_threshold_
        int64_t init_feature_num_threshold_ { 10000000 };
        //if tracking normally, the inliers num >  tracking_inliers_num_threshold_
        int64_t tracking_inliers_num_threshold_ { 10000000 };
        //if inliers_num < reset_inliers_num_threshold, tracker will reset;  TODO(snowden) : if reset , how to deal with Map ?
        int64_t reset_inliers_num_threshold_ { 10000000 };
        //if inliers num < need_insert_keyframe_inliers_num_threshold_, insert a keyframe;
        int64_t need_insert_keyframe_inliers_num_threshold_ { 10000000 };
        FrontEndStatus enum_front_end_status_ { FrontEndStatus::UNKNOW };
        std::thread front_end_thread_;
        std::mutex tracker_finished_mutex;
        std::unique_lock<std::mutex> tracker_finished_lock {tracker_finished_mutex};
        //TODO(snowden) : viewer and optimizer also need deal with this notify();
        std::condition_variable condition_variable_is_tracker_finished_;
        std::atomic<bool> is_running_;

    protected:

    private:
        void front_end_loop();
        bool init_front_end();
        FrontEndStatus tracking();
        bool insert_keyframe();
        bool reset();
        void notify_all_updated_map();
        void notify_all_tracker_finished();
        //data




}; //Tracker

} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam


#endif //OPTICALFLOW_SLAM_ALGORITHM_MODULE_TRACKER_H_
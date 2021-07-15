#ifndef OPTICALFLOW_SLAM_ALGORITHM_MODULE_TRACKER_H_
#define OPTICALFLOW_SLAM_ALGORITHM_MODULE_TRACKER_H_

#include "algorithm/common_include.h"
#include "algorithm/base_component/include/feature2d.h"
#include "algorithm/base_component/include/frame.h"
#include "algorithm/base_component/include/keyframe.h"
#include "algorithm/base_component/include/mappoint3d.h"
#include "algorithm/module/include/map.h"
#include "algorithm/module/include/optimizer.h"

namespace OpticalFlow_SLAM_algorithm_opticalflow_slam {

class Optimizer;

/**
 *  Tracker 
 * @note include a frontend thread, will interactive with Optimizer  and Map;
 */
class Tracker {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        enum class TrackerStatus : std::int64_t {
                TRACKER_STATUS_READY,
                TRACKER_STATUS_INITING,
                TRACKER_STATUS_TRACKING,
                TRACKER_STATUS_NEED_INSERT_KEYFRAM,
                TRACKER_STATUS_LOST,
                TRACKER_STATUS_REST,
                TRACKER_STATUS_UNKONW,
                TRACKER_STATUS_NUM
        };
        Tracker();
        ~Tracker();
    
        std::weak_ptr<Map> wp_map_;
        std::weak_ptr<Optimizer> wp_optimizer_;
        std::weak_ptr<Frame> wp_current_frame_; 
        std::weak_ptr<Frame> wp_last_frame_;
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

        TrackerStatus enum_tracker_status_ { TrackerStatus::TRACKER_STATUS_UNKONW };

    protected:

    private:

}; //Tracker

} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam


#endif //OPTICALFLOW_SLAM_ALGORITHM_MODULE_TRACKER_H_
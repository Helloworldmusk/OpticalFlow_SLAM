#ifndef OPTICALFLOW_SLAM_ALGORITHM_MODULE_VIEWER_H_
#define OPTICALFLOW_SLAM_ALGORITHM_MODULE_VIEWER_H_

#include "algorithm/common_include.h"
#include "algorithm/base_component/include/feature2d.h"
#include "algorithm/base_component/include/frame.h"
#include "algorithm/base_component/include/keyframe.h"
#include "algorithm/base_component/include/mappoint3d.h"
#include "algorithm/module/include/map.h"
#include "algorithm/module/include/optimizer.h"
#include "algorithm/module/include/tracker.h"

namespace OpticalFlow_SLAM_algorithm_opticalflow_slam {

class Tracker;
class Optimizer;

/**
 * @brief Viewer
 * @author snowden
 * @date 2021-07-16
 * @version 1.0
 * @note include a viewer thread, will interactive with Tracker  and Map
 */  
class Viewer {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        Viewer(std::weak_ptr<Map> wp_map, std::weak_ptr<Tracker> wp_tracker, std::weak_ptr<Optimizer> wp_optimizer);
        ~Viewer() { };
        void stop();

        std::weak_ptr<Map> wp_map_;
        std::weak_ptr<Optimizer> wp_optimizer_;
        std::weak_ptr<Tracker> wp_tracker_;
        std::weak_ptr<Frame> wp_current_frame_; 
        std::vector<std::weak_ptr<Mappoint3d>> vsp_mappoint_;
        std::thread viewer_thread_;
        std::atomic<bool> is_running_;
    protected:
        
    private:
        void viewer_loop();
        bool wait_update_map_notify();
        bool update_viewer();
        

        

}; //Viewer

} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam


#endif //OPTICALFLOW_SLAM_ALGORITHM_MODULE_VIEWER_H_
#ifndef OPTICALFLOW_SLAM_ALGORITHM_MODULE_VIEWER_H_
#define OPTICALFLOW_SLAM_ALGORITHM_MODULE_VIEWER_H_

#include "pangolin/pangolin.h"

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

        Viewer(std::weak_ptr<Map> wp_map, std::shared_ptr<Tracker> sp_tracker, std::weak_ptr<Optimizer> wp_optimizer);
        ~Viewer() { };
        void stop();

        std::weak_ptr<Map> wp_map_;
        std::weak_ptr<Optimizer> wp_optimizer_;
        std::shared_ptr<Tracker> sp_tracker_;
        std::weak_ptr<Frame> wp_current_frame_; 
        std::vector<std::shared_ptr<Mappoint3d>> vsp_mappoints_;
        std::thread viewer_thread_;
        std::atomic<bool> is_running_;
        const float kBlueColor_[3] {0,0,1};
        const float KRedColor_[3] {1, 0, 0};
        const float kGreenColor_[3] {0, 1, 0};
        const float depth { 1.0 };
        const float line_width { 2.0 };
        const float fx { 400.0 };
        const float fy { 400.0 };
        const float cx { 512.0 };
        const float cy { 384.0 };
        const float width { 1080.0 };
        const float height { 768.0 };
    protected:
        
    private:
        void viewer_loop();
        bool wait_update_map_notify();
        bool update_viewer(pangolin::OpenGlRenderState& camera, pangolin::View& displayer);
        bool draw_frame(const SE3 pose, const float color[3]);
        bool draw_mappoints(const float color[3]);
        void follow_frame(const SE3 pose, pangolin::OpenGlRenderState& camera);

        

}; //Viewer

} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam


#endif //OPTICALFLOW_SLAM_ALGORITHM_MODULE_VIEWER_H_
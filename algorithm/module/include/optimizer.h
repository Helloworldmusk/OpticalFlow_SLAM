#ifndef OPTICALFLOW_SLAM_ALGORITHM_MODULE_OPTIMIZER_H_
#define OPTICALFLOW_SLAM_ALGORITHM_MODULE_OPTIMIZER_H_

#include "algorithm/common_include.h"

#include <thread>
#include <chrono>
#include <atomic>

#include "algorithm/base_component/include/feature2d.h"
#include "algorithm/base_component/include/frame.h"
#include "algorithm/base_component/include/keyframe.h"
#include "algorithm/base_component/include/mappoint3d.h"
#include "algorithm/base_component/include/system_config.h"
#include "algorithm/base_component/include/camera_config.h"
#include "algorithm/module/include/map.h"
#include "algorithm/module/include/tracker.h"
#include "algorithm/module/include/viewer.h"

namespace OpticalFlow_SLAM_algorithm_opticalflow_slam {

class Tracker;

/**
 * @brief Map
 * @author snowden
 * @date 2021-07-16
 * @version 1.0
 * @note include a backend thread, will interactive with Tracker  and Map
 */  
class Optimizer {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        enum class BackEndStatus : std::int64_t {
                READY,
                INIT,
                IDLE,
                OPTIMIZING,
                FINISHED,
                RESET,
                UNKNOW,
                NUM
        };

        Optimizer( std::weak_ptr<Map> map, const std::shared_ptr<SystemConfig>  sp_slam_config, 
                               const std::shared_ptr<CameraConfig> sp_camera_config);
        ~Optimizer()  { };
        void stop();
        BackEndStatus get_back_end_status() { return enum_back_end_status_; }
        bool set_back_end_status(const BackEndStatus &new_status);
    
        std::weak_ptr<Map> wp_map_;
        std::weak_ptr<Tracker> wp_tracker_;
        std::weak_ptr<Viewer> wp_viewer_;
        std::shared_ptr<SystemConfig>  sp_slam_config_;
        std::shared_ptr<CameraConfig> sp_camera_config_;
        std::atomic<bool>  is_running_;

    protected:

    private:

        void back_end_loop();
        bool init_back_end();
        bool wait_update_map_notify();
        void notify_all_updated_map();
        BackEndStatus optimize();
        BackEndStatus enum_back_end_status_ { BackEndStatus::UNKNOW };
        std::thread back_end_thread_;
        

}; //Optimizer

} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam


#endif //OPTICALFLOW_SLAM_ALGORITHM_MODULE_OPTIMIZER_H_
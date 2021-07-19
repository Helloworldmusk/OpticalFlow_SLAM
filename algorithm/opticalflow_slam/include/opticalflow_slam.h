#ifndef OPTICALFLOW_SLAM_ALGORITHM_OPTICALFLOW_SLAM_OPTICALFLOW_SLAM_H_
#define OPTICALFLOW_SLAM_ALGORITHM_OPTICALFLOW_SLAM_OPTICALFLOW_SLAM_H_

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
#include "algorithm/opticalflow_slam/include/macro_define.h"

namespace OpticalFlow_SLAM_algorithm_opticalflow_slam {

/**
 * 光流法里程计opticalflow_slam -> OP_SLAM
 */
class OP_SLAM {
    public:
        //typedefs
        //enum
        enum class OP_SLAM_STATUS : std::int64_t{
            READY,
            INITING,
            RUNING,
            FINISHED,
            SAVINGMAP,
            RESET,
            UNKNOW,
            NUM
        };
        //const
        OP_SLAM(const std::string system_config_path, const std::string camera_config_path,
                            const std::string dataset_path,  const std::string save_map_path);
        ~OP_SLAM();
        //member function
        bool init( );
        bool run();
        bool save_map();
        //set/get
        OP_SLAM_STATUS get_status() { return slam_status_ ;}
        bool set_status(OP_SLAM_STATUS new_status);
        //data
        std::vector<std::pair<cv::Mat, cv::Mat>> image_left_right;
        std::shared_ptr<Map> sp_map_;
        std::shared_ptr<Tracker> sp_tracker_;
        std::shared_ptr<Optimizer> sp_optimizer_;
        std::shared_ptr<Viewer> sp_viewer_;
    protected:

    private:

        bool load_system_config();
        bool load_camera_config();
        bool load_images();

        //data
        std::string system_config_path_;
        std::string camera_config_path_;
        std::string dataset_path_;
        std::string save_map_path_;

        std::atomic<bool> is_running_;
        OP_SLAM_STATUS slam_status_ = OP_SLAM_STATUS::READY;
        std::shared_ptr<SystemConfig>  sp_slam_config_;
        std::shared_ptr<CameraConfig> sp_camera_config_;
        double_t rpe { 0.0 };
        double_t ate { 0.0 };

}; //class OP_SLAM 

} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam

#endif //OPTICALFLOW_SLAM_ALGORITHM_OPTICALFLOW_SLAM_OPTICALFLOW_SLAM_H_
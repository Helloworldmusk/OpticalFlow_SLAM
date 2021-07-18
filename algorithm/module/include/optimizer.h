#ifndef OPTICALFLOW_SLAM_ALGORITHM_MODULE_OPTIMIZER_H_
#define OPTICALFLOW_SLAM_ALGORITHM_MODULE_OPTIMIZER_H_

#include "algorithm/common_include.h"
#include "algorithm/base_component/include/feature2d.h"
#include "algorithm/base_component/include/frame.h"
#include "algorithm/base_component/include/keyframe.h"
#include "algorithm/base_component/include/mappoint3d.h"
#include "algorithm/base_component/include/system_config.h"
#include "algorithm/base_component/include/camera_config.h"
#include "algorithm/module/include/map.h"
#include "algorithm/module/include/tracker.h"

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

        enum class OptimizerStatus : std::int64_t {
                OPTIMIZER_STATUS_READY,
                OPTIMIZER_STATUS_IDLE,
                OPTIMIZER_STATUS_OPTIMIZING,
                OPTIMIZER_STATUS_UNKONW,
                OPTIMIZER_STATUS_NUM
        };

        Optimizer( std::weak_ptr<Map> map, const std::shared_ptr<SystemConfig>  sp_slam_config, 
                               const std::shared_ptr<CameraConfig> sp_camera_config);
        ~Optimizer()  { };
    
        std::weak_ptr<Map> wp_map_;
        std::weak_ptr<Tracker> wp_tracker_;
        std::shared_ptr<SystemConfig>  sp_slam_config_;
        std::shared_ptr<CameraConfig> sp_camera_config_;

    protected:

    private:

}; //Optimizer

} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam


#endif //OPTICALFLOW_SLAM_ALGORITHM_MODULE_OPTIMIZER_H_
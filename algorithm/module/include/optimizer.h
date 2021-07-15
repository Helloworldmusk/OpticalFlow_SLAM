#ifndef OPTICALFLOW_SLAM_ALGORITHM_MODULE_OPTIMIZER_H_
#define OPTICALFLOW_SLAM_ALGORITHM_MODULE_OPTIMIZER_H_

#include "algorithm/common_include.h"
#include "algorithm/base_component/include/feature2d.h"
#include "algorithm/base_component/include/frame.h"
#include "algorithm/base_component/include/keyframe.h"
#include "algorithm/base_component/include/mappoint3d.h"
#include "algorithm/module/include/map.h"
#include "algorithm/module/include/tracker.h"

namespace OpticalFlow_SLAM_algorithm_opticalflow_slam {

class Tracker;

/**
 *  Optimizer 
 * @note include a backend thread, will interactive with Tracker  and Map;
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

        Optimizer();
        ~Optimizer();
    
        std::weak_ptr<Map> wp_map_;
        std::weak_ptr<Tracker> wp_tracker_;

    protected:

    private:

}; //Optimizer

} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam


#endif //OPTICALFLOW_SLAM_ALGORITHM_MODULE_OPTIMIZER_H_
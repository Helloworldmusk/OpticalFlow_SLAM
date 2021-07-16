#ifndef OPTICALFLOW_SLAM_ALGORITHM_BASE_COMPONENT_FEATURE2D_H_
#define OPTICALFLOW_SLAM_ALGORITHM_BASE_COMPONENT_FEATURE2D_H_

#include "algorithm/common_include.h"
#include "algorithm/base_component/include/mappoint3d.h"
#include "algorithm/base_component/include/frame.h"

namespace OpticalFlow_SLAM_algorithm_opticalflow_slam {

class Mappoint3d;
class Frame;

/**
 * @brief Feature points searched in 2d image
 * @author snowden
 * @date 2021-07-16
 * @version 1.0
 */        
class Feature2d {
    public:
         EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        static int64_t static_next_2d_id_;

        Feature2d();
        Feature2d(const Vec2 position);
        ~Feature2d();
        int64_t get_new_id() { static_next_2d_id_++;  return static_next_2d_id_; }

        int64_t id_ = { -1 };
        bool is_outline_ { false };
        Vec2 position2d_ = Vec2::Zero();
        std::weak_ptr<Mappoint3d> wp_mappiont3d_;
        std::weak_ptr<Frame> wp_frame_;

    protected:
    private:
}; //class Feature2d

} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam

#endif //OPTICALFLOW_SLAM_ALGORITHM_BASE_COMPONENT_FEATURE2D_H_
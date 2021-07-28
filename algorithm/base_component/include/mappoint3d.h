#ifndef OPTICALFLOW_SLAM_ALGORITHM_BASE_COMPONENT_MAPPOINT3D_H_
#define OPTICALFLOW_SLAM_ALGORITHM_BASE_COMPONENT_MAPPOINT3D_H_

#include "algorithm/common_include.h"
#include "algorithm/base_component/include/feature2d.h"

namespace OpticalFlow_SLAM_algorithm_opticalflow_slam {

class Feature2d;

/**
 * @brief 3d Mappoint points create by Triangulate
 * @author snowden
 * @date 2021-07-16
 * @version 1.0
 * @note it's pose will be modified by mulit thread
 */  
class Mappoint3d {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        static int64_t static_next_3d_id_;

        Mappoint3d();
        Mappoint3d(const int64_t timestamp, const Vec3 position3d);
        ~Mappoint3d();
        int64_t get_new_id() { static_next_3d_id_++; return static_next_3d_id_; }
        void set_position3d(const Vec3& position3d);
        Vec3 get_position3d();
        //data
         int64_t id_ { -1 };
         //timestamp_ eq frame's  timestamp_, use to decide if  set outline;
         int64_t timestamp_ { -1 };

        std::vector<std::weak_ptr<Feature2d>> vwp_observers_;

    protected:
    private:
        std::mutex position3d_mutex;

        Vec3 position3d_{ Vec3::Zero() };
        std::atomic<bool> is_actived_ { true };
        std::atomic<bool>  is_outline_ { false };
        std::atomic<std::int64_t> observed_times_ { 0 };
};  //class Mappoint3d

} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam

#endif //OPTICALFLOW_SLAM_ALGORITHM_BASE_COMPONENT_MAPPOINT3D_H_
#ifndef OPTICALFLOW_SLAM_ALGORITHM_BASE_COMPONENT_MAPPOINT3D_H_
#define OPTICALFLOW_SLAM_ALGORITHM_BASE_COMPONENT_MAPPOINT3D_H_

#include "algorithm/common_include.h"
#include "algorithm/base_component/include/feature2d.h"

namespace OpticalFlow_SLAM_algorithm_opticalflow_slam {

class Feature2d;

/**
 * 3d Mappoint points create by Triangulate ; 
 * @note It's position will be modified by mulit thread;
 */

class Mappoint3d {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        Mappoint3d();
        ~Mappoint3d();

        //data
         int64_t id_ { -1 };
         int64_t timestamp_ { -1 };
        Vec3 position3d_{ Vec3::Zero() };
        bool is_actived_ { true };
        bool is_outline_ { true };
        int64_t observed_times_ { 0 };
        std::vector<std::weak_ptr<OpticalFlow_SLAM_algorithm_opticalflow_slam::Feature2d>> vwp_observers_;

    protected:
    private:

};  //class Mappoint3d

} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam

#endif //OPTICALFLOW_SLAM_ALGORITHM_BASE_COMPONENT_MAPPOINT3D_H_
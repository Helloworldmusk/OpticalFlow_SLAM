#include "algorithm/base_component/include/tool.h"
namespace OpticalFlow_SLAM_algorithm_opticalflow_slam {


/**
 * @brief 
 * @author snowden
 * @return coordinate in current image coordinate;
 * @date 2021-07-25
 * @version 1.0
 */ 
Vec2 CoordinateTransformWorldToImage(const std::shared_ptr<Mappoint3d>& mappoint3d, const SE3& current_pose, 
                                                                       const Mat33& camera_intrinsics)
{
        Vec3 position = CoordinateTransformWorldToCamera(mappoint3d->position3d_, current_pose);
        Vec2 position_2d ;
        position_2d(0) = camera_intrinsics(0,0) * position(0) / position(2) + camera_intrinsics(0,2);
        position_2d(1) = camera_intrinsics(1,1) * position(1) / position(2) + camera_intrinsics(1,2);
        return position_2d;
}


/**
 * @brief 
 * @author snowden
 * @return coordinate in current camera coordinate system
 * @date 2021-07-25
 * @version 1.0
 */  
Vec3 CoordinateTransformWorldToCamera(const Vec3& world_coordinate_position, const SE3& current_pose)
{
        Vec3 new_position = current_pose * world_coordinate_position;
}


/**
 * @brief 
 * @author snowden
 * @return 3d coordinate in current camera coordinate system
 * @date 2021-07-26
 * @version 1.0
 */  
Vec3 CoordinateTransformImageToCamera(const Vec2& image_coordinate_position, const double_t& depth,
                                                                                          const Mat33& camera_intrinsics)
{
        Vec3 point3d;
        point3d.x() = (image_coordinate_position.x() - camera_intrinsics(0,2)) * depth / camera_intrinsics(0,0);
        point3d.y() = (image_coordinate_position.y() - camera_intrinsics(1,2)) * depth / camera_intrinsics(1,1); 
        point3d.z() = depth;
}


/**
 * @brief 
 * @author snowden
 * @return 3d coordinate in current camera coordinate system and in normalized plane;
 * @date 2021-07-26
 * @version 1.0
 */  
Vec3 CoordinateTransformImageToNormalizedPlane(const Vec2& image_coordinate_position, 
                                                                                                              const Mat33& camera_intrinsics)
{
        return CoordinateTransformImageToCamera(image_coordinate_position, 1.0, camera_intrinsics);
}


/**
 * @brief according two point in normalized plane and pose in two camera coordinate ,to calculate 3d point's position;
 * @author snowden
 * @return point3d position in camera coordiante;
 * @date 2021-07-26
 * @version 1.0
 */  
bool TriangulateNormalizedPoint(const Vec3& point1, const Vec3& point2, const SE3& pose1, const SE3& pose2, Vec3& position3d)
{
        MatXX A(4, 4);
        Vec4   b(4);
        // DLOG_INFO << "******* point1: " << point1.transpose() <<  " point 2 " << point2.transpose() << std::endl;
        b.setZero();
        A.block<1,4>(0,0) = point1.x() * pose1.matrix3x4().row(2) - pose1.matrix3x4().row(0);
        A.block<1,4>(1,0) = point1.y() * pose1.matrix3x4().row(2) - pose1.matrix3x4().row(1);
        A.block<1,4>(2,0) = point2.x() * pose2.matrix3x4().row(2) - pose2.matrix3x4().row(0);
        A.block<1,4>(3,0) = point2.y() * pose2.matrix3x4().row(2) - pose2.matrix3x4().row(1); 
         auto  svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
         position3d = (svd.matrixV().col(3) / svd.matrixV()(3,3)).head<3>();
         if(svd.singularValues()[3] / svd.singularValues()[2] > 1e-2)
         {
                 DLOG_INFO << " result is not good  " << std::endl;
                return false;
         }
         return true;
}


} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam
#include "algorithm/base_component/include/g2o_type.h"
namespace OpticalFlow_SLAM_algorithm_opticalflow_slam {

/******************************************************VertexPose**********************************************************/
/**
 *  @brief update the position of the node from the parameters in v.
 *  @author snowden
 *  @date 2021-07-30 
 *  @version 1.0
 */
void VertexPose::oplusImpl(const number_t* update)
{
    Vec6 dealt;
    dealt << update[0], update[1], update[2], update[3], update[4], update[5];
    _estimate = SE3::exp(dealt) * _estimate;
}


/**
 *  @brief sets the node to the origin (used in the multilevel stuff)
 *  @author snowden
 *  @date 2021-07-30 
 *  @version 1.0
 */
void VertexPose::setToOriginImpl()
{
    _estimate = SE3();
}


/**
 *  @brief read the vertex from a stream, i.e., the internal state of the vertex
 *  @author snowden
 *  @date 2021-07-30 
 *  @version 1.0
 */
bool VertexPose::read(std::istream& is)
{
    return true;
}


/**
 *  @brief write the vertex to a stream
 *  @author snowden
 *  @date 2021-07-30 
 *  @version 1.0
 */
bool VertexPose::write(std::ostream& os) const 
{
    return true;
}
/*****************************************************VertexPose***********************************************************/


/*****************************************************VertexMappoint******************************************************/
/**
 *  @brief update the position of the node from the parameters in v.
 *  @author snowden
 *  @date 2021-08-02
 *  @version 1.0
 */
void VertexMappoint::oplusImpl(const number_t* update)
{
    Vec3 dealt;
    dealt << update[0], update[1], update[2];
    _estimate += dealt;
}


/**
 *  @brief sets the node to the origin (used in the multilevel stuff)
 *  @author snowden
 *  @date 2021-08-02
 *  @version 1.0
 */
void VertexMappoint::setToOriginImpl()
{
    //TODO(snowden) : may need set other value;
    _estimate = Vec3(0,0,0);
}


/**
 *  @brief read the vertex from a stream, i.e., the internal state of the vertex
 *  @author snowden
 *  @date 2021-08-02
 *  @version 1.0
 */
bool VertexMappoint::read(std::istream& is)
{
    return true;
}


/**
 *  @brief write the vertex to a stream
 *  @author snowden
 *  @date 2021-08-02
 *  @version 1.0
 */
bool VertexMappoint::write(std::ostream& os) const 
{
    return true;
}
/*******************************************************VertexMappoint****************************************************/


/*******************************************************UnaryEdgePose****************************************************/
/**
 *  @brief 
 *  @author snowden
 *  @date 2021-07-30 
 *  @version 1.0
 */
UnaryEdgePose::UnaryEdgePose(const Vec3&  mappoint3d, const Mat33& K) : mappoint3d_(mappoint3d), K_(K)
{

}


/**
 *  @brief Linearizes the oplus operator in the vertex, and stores
*                  the result in temporary variables _jacobianOplusXi and _jacobianOplusXj
 *  @author snowden
 *  @date 2021-07-30 
 *  @version 1.0
 */
void UnaryEdgePose::linearizeOplus()
{
    const VertexPose* vertex = static_cast<VertexPose*>(_vertices[0]);
    SE3 T = vertex->estimate();
    Vec3 point3d_in_camera = T * mappoint3d_;
    double_t X = point3d_in_camera[0];
    double_t Y = point3d_in_camera[1];
    double_t Z = point3d_in_camera[2];
    double_t fx = K_(0,0);
    double_t fy = K_(1,1);
    double_t Z_inv = 1.0 /(Z + 1e-20);
    double_t Z_inv2 = Z_inv * Z_inv;
    _jacobianOplusXi << -fx * Z_inv,     0,                       fx * X * Z_inv2,       fx * X * Y * Z_inv2,            -fx  - fx * X * X * Z_inv2,     fx * Y * Z_inv,
                                               0,                       -fy * Z_inv,     fy * Y * Z_inv2,       fy + fy * Y * Y * Z_inv2,     -fy * X * Y * Z_inv2,             -fy * X * Z_inv;
}


/**
 *  @brief computes the error of the edge and stores it in an internal structure
 *  @author snowden
 *  @date 2021-07-30 
 *  @version 1.0
 */
void UnaryEdgePose::computeError()
{
    const VertexPose* vertex = static_cast<VertexPose*>(_vertices[0]);
    SE3 T = vertex->estimate();
    //N = K * (T * P) / Z , N is (x, y , 1);
    Vec3 point_3d_in_image = K_ * (T * mappoint3d_);
    point_3d_in_image = point_3d_in_image / point_3d_in_image[2];
    /**
     * @warning note the direction, _measurement -> point_3d_in_image, the direction will 
     *                        affect to the jacobian matrix
     */ 
    _error = _measurement - point_3d_in_image.head<2>();
}


/**
 *  @brief read the vertex from a stream, i.e., the internal state of the vertex
 *  @author snowden
 *  @date 2021-07-30 
 *  @version 1.0
 */
bool UnaryEdgePose::read(std::istream& is)
{
    return true;
}


/**
 *  @brief write the vertex to a stream
 *  @author snowden
 *  @date 2021-07-30 
 *  @version 1.0
 */
bool UnaryEdgePose::write(std::ostream& os) const 
{
    return true;
}
/******************************************************UnaryEdgePose*****************************************************/


/******************************************************EdgePoseMappoint*************************************************/


/**
 *  @brief 
 *  @author snowden
 *  @date 2021-08-03 
 *  @version 1.0
 *  @param K   camera 's intrinsics;
 *  @param pose TODO(snowden) : 
 */
EdgePoseMappoint::EdgePoseMappoint(const Mat33 K, const SE3 pose) : K_(K), pose_(pose)
{

}


/**
 *  @brief calculate jacobian;
 *  @author snowden
 *  @date 2021-08-03 
 *  @version 1.0
 */
void EdgePoseMappoint::linearizeOplus()
{
    const VertexPose* vertex_pose = static_cast<VertexPose*>(_vertices[0]);
    const VertexMappoint* vertex_mappoint = static_cast<VertexMappoint*>(_vertices[1]);
    Vec3 camera_mappoint_position = vertex_pose->estimate() * vertex_mappoint->estimate();
    double_t fx = K_(0, 0);
    double_t fy = K_(1, 1);
    double_t X = camera_mappoint_position[0];
    double_t Y = camera_mappoint_position[1];
    double_t Z = camera_mappoint_position[2];
    double_t Z_inv = 1.0/(Z + 1e20);
    double_t Z_inv2 = Z_inv * Z_inv;
    _jacobianOplusXi << -fx * Z_inv,     0,                       fx * X * Z_inv2,       fx * X * Y * Z_inv2,            -fx  - fx * X * X * Z_inv2,     fx * Y * Z_inv,
                                               0,                       -fy * Z_inv,     fy * Y * Z_inv2,       fy + fy * Y * Y * Z_inv2,     -fy * X * Y * Z_inv2,             -fy * X * Z_inv;

    _jacobianOplusXj = _jacobianOplusXj.block<2,3>(0,0) * vertex_pose->estimate().rotationMatrix();
}


/**
 *  @brief computes the error of the edge and stores it in an internal structure
 *  @author snowden
 *  @date 2021-08-03 
 *  @version 1.0
 */
void EdgePoseMappoint::computeError()
{
    const VertexPose* vertex_pose = static_cast<VertexPose*>(_vertices[0]);
    const VertexMappoint* vertex_mappoint = static_cast<VertexMappoint*>(_vertices[1]);
    //note that: vertex_pose->estimate() * vertex_mappoint->estimate need calculate firest; otherwise dimension is not matched;
    Vec3 position = K_ * (vertex_pose->estimate() * vertex_mappoint->estimate());  
    Vec3 project_position = position / position[2];
    _error = _measurement - project_position.head<2>();
}


/**
 *  @brief read the vertex from a stream, i.e., the internal state of the vertex
 *  @author snowden
 *  @date 2021-08-03 
 *  @version 1.0
 */
bool EdgePoseMappoint::read(std::istream& is)
{
    return true;
}


/**
 *  @brief write the vertex to a stream
 *  @author snowden
 *  @date 2021-08-03 
 *  @version 1.0
 */
bool EdgePoseMappoint::write(std::ostream& os) const 
{
    return true;
}

/******************************************************EdgePoseMappoint*************************************************/

} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam

#ifndef OPTICALFLOW_SLAM_ALGORITHM_BASE_COMPONENT_G2O_H_
#define OPTICALFLOW_SLAM_ALGORITHM_BASE_COMPONENT_G2O_H_

#include <algorithm/common_include.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_block_matrix.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>


namespace OpticalFlow_SLAM_algorithm_opticalflow_slam {

/**
 *  @brief Pose Vertex define;
 *  @author snowden
 *  @date 2021-07-30 
 *  @version 1.0
 *  @param first minimal dimension of the vertex, e.g., 3 for rotation in 3D
 *  @param second  internal type to represent the estimate, e.g., Quaternion for rotation in 3D
 */
class VertexPose : public g2o::BaseVertex<6,SE3> {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        /**
         * update the position of the node from the parameters in v.
         * Implement in your class!
         */
        virtual void oplusImpl(const number_t* v);
        //! sets the node to the origin (used in the multilevel stuff)
        virtual void setToOriginImpl();
        //! read the vertex from a stream, i.e., the internal state of the vertex
        virtual bool read(std::istream& is);
        //! write the vertex to a stream
        virtual bool write(std::ostream& os) const ;

    private:

};


/**
 *  @brief Unary edge Vertex define;
 *  @author snowden
 *  @date 2021-07-30 
 *  @version 1.0
 *  @param first  dimension of the edge
 *  @param second  typename of the edge
*   @param third  typename of the vertex 
 */
class UnaryEdgePose: public g2o::BaseUnaryEdge<2, Vec2, VertexPose> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        UnaryEdgePose(const Vec3&  mappoint3d, const Mat33& K);
        virtual void linearizeOplus();
        // computes the error of the edge and stores it in an internal structure
        virtual void computeError();
        //! read the vertex from a stream, i.e., the internal state of the vertex
        virtual bool read(std::istream& is);
        //! write the vertex to a stream
        virtual bool write(std::ostream& os) const ;
    private:
        Vec3 mappoint3d_;
        Mat33 K_;
};

} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam

#endif //OPTICALFLOW_SLAM_ALGORITHM_BASE_COMPONENT_G2O_H_
#ifndef OPTICALFLOW_SLAM_ALGORITHM_COMMON_INCLUDE_H_
#define OPTICALFLOW_SLAM_ALGORITHM_COMMON_INCLUDE_H_
//c lib
#include <math.h>
//c++ lib
#include <cstdint>
#include <iostream>
#include <fstream>
#include <vector>
#include <memory>
#include <thread>
#include <mutex>
#include <chrono>
#include <atomic>
#include <condition_variable>
//third lib
#include <opencv2/core/core.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD> 
#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <glog/logging.h>
#include <gflags/gflags.h>
//project lib


/*
 *typedefs for float and double , it may need modified when run in other platform;
 * current paltform : float->4, double->8;
 */
typedef float  float_t;
typedef double double_t;

//typedefs for eigen
//Mat
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatXX;

typedef Eigen::Matrix<double_t, 3, 3> Mat33;
typedef Eigen::Matrix<double_t, 3, 4> Mat34;
typedef Eigen::Matrix<double_t, 4, 4> Mat44;

typedef Eigen::Matrix<float_t, 3, 3> Mat33f;
typedef Eigen::Matrix<float_t, 3, 4> Mat34f;
typedef Eigen::Matrix<float_t, 4, 4> Mat44f;

//vector
typedef Eigen::Matrix<double_t, 4, 1> Vec4;
typedef Eigen::Matrix<double_t, 3, 1> Vec3;
typedef Eigen::Matrix<double_t, 2, 1> Vec2;

typedef Eigen::Matrix<float_t, 4, 1> Vec4f;
typedef Eigen::Matrix<float_t, 3, 1> Vec3f;
typedef Eigen::Matrix<float_t, 2, 1> Vec2f;

// typedefs for Sophus
typedef Sophus::SE3d SE3;
typedef Sophus::SO3d SO3;




#endif //OPTICALFLOW_SLAM_ALGORITHM_COMMON_INCLUDE_H_
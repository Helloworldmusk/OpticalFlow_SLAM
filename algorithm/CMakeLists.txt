set(CMAKE_CURRENT_BINARY_DIR ${PROJECT_SOURCE_DIR})

# Eigen
include_directories("/usr/include/eigen3")

# Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

set(THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES}
        )
 message(" algorithm current source dir : ${PROJECT_SOURCE_DIR}" )

add_library(op_slam SHARED 
        ${PROJECT_SOURCE_DIR}/algorithm/base_component/src/feature2d.cc
        ${PROJECT_SOURCE_DIR}/algorithm/base_component/src/frame.cc
        ${PROJECT_SOURCE_DIR}/algorithm/base_component/src/keyframe.cc
        ${PROJECT_SOURCE_DIR}/algorithm/base_component/src/mappoint3d.cc
        ${PROJECT_SOURCE_DIR}/algorithm/module/src/map.cc
        ${PROJECT_SOURCE_DIR}/algorithm/module/src/tracker.cc
        ${PROJECT_SOURCE_DIR}/algorithm/module/src/optimizer.cc
        ${PROJECT_SOURCE_DIR}/algorithm/module/src/viewer.cc
        ${PROJECT_SOURCE_DIR}/algorithm/opticalflow_slam/src/opticalflow_slam.cc
        )

target_link_libraries(op_slam 
        ${THIRD_PARTY_LIBS}
        )

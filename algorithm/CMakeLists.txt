set(CMAKE_CURRENT_BINARY_DIR ${PROJECT_SOURCE_DIR})

SET(G2O_LIBS "/usr/local/lib/libg2o_csparse_extension.so" "/usr/local/lib/libg2o_stuff.so" "/usr/local/lib/libg2o_core.so" cxsparse)


# Eigen
include_directories("/usr/include/eigen3")

#Suitesparse
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
find_package(CSparse REQUIRED)
include_directories("/usr/include/suitesparse")

# Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# G2O
list( APPEND CMAKE_MODULE_PATH /home/snowden/workplace/slam/slamcode/slambook2/3rdparty/g2o/cmake_modules/ ) 
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

# pangolin
# include_directories("/usr/local/include/pangolin")
include_directories(${Pangolin_INCLUDE_DIRS})
find_package(Pangolin REQUIRED)
message("**********************************${Pangolin_INCLUDE_DIRS}******************************************\n\r")

# OpenCV
find_package(OpenCV 3.1 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#gflags
find_package(gflags REQUIRED)
include_directories (${gflags_INCLUDE_DIR})

#glog
find_package (glog 0.6.0 REQUIRED)

set(THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES}
        ${Pangolin_LIBRARIES}
        # GL
        # GLU 
        # GLEW 
        glog
        gflags
        g2o_core g2o_stuff g2o_types_sba g2o_solver_csparse g2o_csparse_extension
        ${CSPARSE_LIBRARY}
        # ${G2O_LIBS}
        )
 message(" CSPARSE_LIBRARY: " ${CSPARSE_LIBRARY})
 message(" CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH})
 message(" algorithm current source dir ": ${PROJECT_SOURCE_DIR})

 file( GLOB ALGORITHM_SRC */src/*.cc)
message(" ALGORITHM_SRC : " ${ALGORITHM_SRC})
add_library(op_slam SHARED 
        ${ALGORITHM_SRC}
        )

target_link_libraries(op_slam 
        ${THIRD_PARTY_LIBS}
        )

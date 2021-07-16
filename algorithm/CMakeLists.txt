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

 file( GLOB ALGORITHM_SRC */src/*.cc)
message(" ALGORITHM_SRC : " ${ALGORITHM_SRC})
add_library(op_slam SHARED 
        ${ALGORITHM_SRC}
        )

target_link_libraries(op_slam 
        ${THIRD_PARTY_LIBS}
        )

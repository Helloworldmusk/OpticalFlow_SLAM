message(" app current source dir : ${PROJECT_SOURCE_DIR}" )
add_executable(run_vo ${PROJECT_SOURCE_DIR}/app/run_vo.cc)
target_link_libraries(run_vo op_slam)
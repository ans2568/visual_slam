add_executable(slam_example run_kitti_stereo.cpp)
target_link_libraries(slam_example slam ${THIRD_PARTY_LIBS})
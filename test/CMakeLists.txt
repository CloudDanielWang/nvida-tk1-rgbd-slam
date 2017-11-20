#add_executable( run_vo run_vo.cpp )
#target_link_libraries( run_vo myslam )

add_executable( slam_front slam_front.cpp )
target_link_libraries( slam_front front_end )

#add_executable( slam_front slam_front_single_thread.cpp )
#target_link_libraries( slam_front front_end )

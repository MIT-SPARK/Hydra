find_package(rostest REQUIRED)
add_rostest_gtest(test_${PROJECT_NAME} hydra_ros.test main.cpp test_ros_config.cpp)
target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} ${catkin_LIBRARIES})

cmake_minimum_required(VERSION 3.1)
project(hydra_utils)

add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS roscpp roslib geometry_msgs pose_graph_tools tf2_eigen tf2_ros)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(GTSAM REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs pose_graph_tools tf2_eigen tf2_ros
  DEPENDS
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
)

add_library(${PROJECT_NAME}
    src/display_utils.cpp
    src/ros_parser.cpp
    src/yaml_parser.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC yaml-cpp ${catkin_LIBRARIES})

add_executable(pose_graph_publisher_node src/pose_graph_publisher_node.cpp)
target_include_directories(pose_graph_publisher_node PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(pose_graph_publisher_node ${catkin_LIBRARIES} gtsam)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(
    utest_${PROJECT_NAME}
    tests/hydra_utils.test
    tests/utest_main.cpp
    tests/utest_config.cpp
  )
  target_link_libraries(utest_${PROJECT_NAME} ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()

install(
  TARGETS ${PROJECT_NAME} pose_graph_publisher_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

catkin_install_python(
  PROGRAMS nodes/csv_to_tf nodes/odom_to_tf
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

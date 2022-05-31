cmake_minimum_required(VERSION 3.1)
project(hydra_utils)

add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             roslib
             nodelet
             cv_bridge
             image_transport
             geometry_msgs
             kimera_pgmo
             pose_graph_tools
             tf2_eigen
             tf2_ros
)
find_package(GTSAM REQUIRED)
find_package(kimera_dsg REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)

# TODO(nathan) clean up
find_package(PkgConfig REQUIRED)
pkg_check_modules(glog REQUIRED IMPORTED_TARGET libglog)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  cv_bridge
  geometry_msgs
  image_transport
  nodelet
  kimera_pgmo
  pose_graph_tools
  tf2_eigen
  tf2_ros
  DEPENDS OpenCV
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
)

add_library(${PROJECT_NAME}
    src/display_utils.cpp
    src/dsg_streaming_interface.cpp
    src/ros_parser.cpp
    src/timing_utilities.cpp
    src/yaml_parser.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC yaml-cpp ${catkin_LIBRARIES} PRIVATE PkgConfig::glog kimera_dsg::kimera_dsg)

add_executable(pose_graph_publisher_node src/pose_graph_publisher_node.cpp)
target_include_directories(pose_graph_publisher_node PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(pose_graph_publisher_node ${catkin_LIBRARIES} gtsam)

add_library(${PROJECT_NAME}_nodelet src/mask_nodelet.cpp)
target_include_directories(${PROJECT_NAME}_nodelet PUBLIC ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_nodelet ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(
    utest_${PROJECT_NAME}
    tests/hydra_utils.test
    tests/utest_main.cpp
    tests/utest_config.cpp
    tests/utest_timing_utilities.cpp
  )
  target_link_libraries(utest_${PROJECT_NAME} ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()

install(
  TARGETS ${PROJECT_NAME} pose_graph_publisher_node ${PROJECT_NAME}_nodelet
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

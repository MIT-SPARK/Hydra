cmake_minimum_required(VERSION 3.1)
project(hydra_utils)

add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(
  catkin REQUIRED
  COMPONENTS dynamic_reconfigure
             geometry_msgs
             image_transport
             kimera_pgmo
             nodelet
             pose_graph_tools
             roscpp
             tf2_eigen
             tf2_ros
             visualization_msgs
)
find_package(GTSAM REQUIRED)
find_package(spark_dsg REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

# TODO(nathan) clean up
find_package(PkgConfig REQUIRED)
pkg_check_modules(
  glog
  REQUIRED
  IMPORTED_TARGET
  libglog
)

generate_dynamic_reconfigure_options(
  cfg/Colormap.cfg
  cfg/DynamicLayerVisualizer.cfg
  cfg/LayerVisualizer.cfg
  cfg/Visualizer.cfg
)

catkin_package(
  CATKIN_DEPENDS
  dynamic_reconfigure
  geometry_msgs
  image_transport
  kimera_pgmo
  nodelet
  pose_graph_tools
  roscpp
  tf2_eigen
  tf2_ros
  visualization_msgs
  DEPENDS
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
)

add_library(
  ${PROJECT_NAME}
  src/colormap_utils.cpp
  src/display_utils.cpp
  src/dsg_streaming_interface.cpp
  src/ros_parser.cpp
  src/timing_utilities.cpp
  src/dsg_mesh_plugins.cpp
  src/dynamic_scene_graph_visualizer.cpp
  src/visualizer_utils.cpp
  src/yaml_parser.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
  PRIVATE ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC yaml-cpp ${catkin_LIBRARIES} spark_dsg::spark_dsg
  PRIVATE PkgConfig::glog ${OpenCV_LIBRARIES}
)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

add_executable(hydra_visualizer_node src/hydra_visualizer_node.cpp)
target_link_libraries(hydra_visualizer_node ${PROJECT_NAME})

add_executable(rotate_tf_node src/rotate_tf_node.cpp)
target_link_libraries(rotate_tf_node ${catkin_LIBRARIES})
target_include_directories(rotate_tf_node PUBLIC ${catkin_INCLUDE_DIRS})

add_executable(pose_graph_publisher_node src/pose_graph_publisher_node.cpp)
target_include_directories(pose_graph_publisher_node PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(pose_graph_publisher_node ${catkin_LIBRARIES} gtsam)

add_library(${PROJECT_NAME}_nodelet src/mask_nodelet.cpp)
target_include_directories(
  ${PROJECT_NAME}_nodelet PUBLIC ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
                                 ${cv_bridge_INCLUDE_DIRS}
)
target_link_libraries(
  ${PROJECT_NAME}_nodelet
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${cv_bridge_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(
    utest_${PROJECT_NAME} tests/hydra_utils.test
    tests/utest_main.cpp tests/utest_config.cpp tests/utest_timing_utilities.cpp
  )
  target_link_libraries(utest_${PROJECT_NAME} ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()

install(
  TARGETS ${PROJECT_NAME}
          pose_graph_publisher_node
          ${PROJECT_NAME}_nodelet
          hydra_visualizer_node
          rotate_tf_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

catkin_install_python(
  PROGRAMS
  nodes/csv_to_tf
  nodes/odom_to_tf
  DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION}
)

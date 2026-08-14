cmake_minimum_required(VERSION 3.22.1)
project(hydra_visualizer)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

option(BUILD_SHARED_LIBS "Build shared libraries" ON)

find_package(ament_cmake REQUIRED)
find_package(config_utilities REQUIRED)
find_package(config_utilities_ros REQUIRED)
find_package(glog REQUIRED)
find_package(gflags REQUIRED)
find_package(hydra_msgs REQUIRED)
find_package(ianvs REQUIRED)
find_package(kimera_pgmo_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(spark_dsg REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(
  ${PROJECT_NAME}
  src/adapters/edge_color.cpp
  src/adapters/mesh_color.cpp
  src/adapters/node_color.cpp
  src/adapters/text.cpp
  src/color/colormap_utilities.cpp
  src/drawing.cpp
  src/groundtruth/bounding_box_publisher.cpp
  src/groundtruth/region_publisher.cpp
  src/io/graph_file_wrapper.cpp
  src/io/graph_ros_wrapper.cpp
  src/io/graph_zmq_wrapper.cpp
  src/layer_info.cpp
  src/plugins/basis_point_plugin.cpp
  src/plugins/footprint_plugin.cpp
  src/plugins/khronos_object_plugin.cpp
  src/plugins/mesh_plugin.cpp
  src/plugins/mesh_point_plugin.cpp
  src/plugins/places_freespace_plugin.cpp
  src/plugins/pose_plugin.cpp
  src/plugins/region_growing_boundary_plugin.cpp
  src/plugins/traversability_plugin.cpp
  src/scene_graph_renderer.cpp
  src/utils/ear_clipping.cpp
  src/utils/layer_key_selector.cpp
  src/utils/marker_group_pub.cpp
  src/utils/marker_tracker.cpp
  src/utils/polygon_utilities.cpp
  src/visualizer_node.cpp
)
target_include_directories(
  ${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                         "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC config_utilities::config_utilities
         config_utilities_ros::config_utilities_ros
         ianvs::ianvs
         rclcpp::rclcpp
         spark_dsg::spark_dsg
         tf2_ros::tf2_ros
  PRIVATE tf2_eigen::tf2_eigen glog::glog
)
ament_target_dependencies(
  ${PROJECT_NAME}
  PUBLIC
  hydra_msgs
  kimera_pgmo_msgs
  std_msgs
  std_srvs
  visualization_msgs
)

add_executable(hydra_visualizer_node app/hydra_visualizer_node.cpp)
target_link_libraries(
  hydra_visualizer_node
  ${PROJECT_NAME}
  gflags
  glog::glog
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest)
  ament_add_gtest(test_${PROJECT_NAME}
      tests/main.cpp
      tests/test_ear_clipping.cpp
      tests/test_layer_key_selector.cpp
  )
  target_link_libraries(
    test_${PROJECT_NAME}
    ${PROJECT_NAME}
    gflags
    glog::glog
  )
endif()

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS hydra_visualizer_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME}/)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  config_utilities
  config_utilities_ros
  hydra_msgs
  ianvs
  kimera_pgmo_msgs
  rclcpp
  spark_dsg
  std_msgs
  std_srvs
  tf2_ros
  visualization_msgs
)
ament_package()

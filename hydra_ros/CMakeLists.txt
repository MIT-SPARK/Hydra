cmake_minimum_required(VERSION 3.22.1)
project(hydra_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

option(BUILD_SHARED_LIBS "Build shared libraries" ON)

find_package(ament_cmake REQUIRED)
find_package(CLI11 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gflags REQUIRED)
find_package(hydra REQUIRED)
find_package(hydra_msgs REQUIRED)
find_package(hydra_visualizer REQUIRED)
find_package(kimera_pgmo_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(pose_graph_tools_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(semantic_inference_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_library(
  ${PROJECT_NAME}
  src/hydra_ros_pipeline.cpp
  src/active_window/reconstruction_visualizer.cpp
  src/active_window/tsdf_occupancy_publisher.cpp
  src/active_window/tsdf_gradient_occupancy_publisher.cpp
  src/backend/ros_backend_publisher.cpp
  src/backend/gt_room_publisher.cpp
  src/frontend/gvd_occupancy_publisher.cpp
  src/frontend/object_visualizer.cpp
  src/frontend/ros_frontend_publisher.cpp
  src/frontend/traversability_visualizer.cpp
  src/input/feature_receiver.cpp
  src/input/image_receiver.cpp
  src/input/pointcloud_adaptor.cpp
  src/input/pointcloud_receiver.cpp
  src/input/ros_data_receiver.cpp
  src/input/ros_labelspace.cpp
  src/input/ros_input_module.cpp
  src/input/ros_sensors.cpp
  src/places/external_traversability_estimator.cpp
  src/places/gvd_places_visualizer.cpp
  src/places/gvd_visualization_utilities.cpp
  src/odometry/ros_pose_graph_tracker.cpp
  src/openset/ros_embedding_group.cpp
  src/utils/bow_subscriber.cpp
  src/utils/dsg_streaming_interface.cpp
  src/utils/external_loop_closure_subscriber.cpp
  src/utils/input_data_to_messages.cpp
  src/utils/occupancy_publisher.cpp
  src/utils/scene_graph_logger.cpp
  src/utils/status_monitor.cpp
  src/utils/tf_lookup.cpp
  src/visualizer/feature_color_adaptors.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
         "$<INSTALL_INTERFACE:include>"
  PRIVATE ${PCL_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR}
)
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC hydra::hydra
         hydra_visualizer::hydra_visualizer
         kimera_pgmo_ros::kimera_pgmo_ros
         pose_graph_tools_ros::pose_graph_tools_ros
         tf2_eigen::tf2_eigen
         tf2_ros::tf2_ros
  PRIVATE ${PCL_LIBRARIES} cv_bridge::cv_bridge
)
ament_target_dependencies(
  ${PROJECT_NAME}
  PUBLIC
  geometry_msgs
  hydra_msgs
  semantic_inference_msgs
  std_msgs
)

add_executable(hydra_ros_node app/hydra_node.cpp)
target_link_libraries(hydra_ros_node ${PROJECT_NAME} ${gflags_LIBRARIES})

add_executable(color_to_label_node app/color_to_label_node.cpp)
target_link_libraries(color_to_label_node ${PROJECT_NAME} cv_bridge::cv_bridge CLI11::CLI11)

rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN
  hydra::SceneGraphLogger
  EXECUTABLE
  scene_graph_logger
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest)
  ament_add_gtest(test_${PROJECT_NAME} tests/main.cpp tests/test_ros_sensors.cpp)
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} gflags)
endif()

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS color_to_label_node hydra_ros_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(
  PROGRAMS app/dsg_file_publisher
           app/dsg_republisher
           app/odom_to_tf
           app/noisy_tf_publisher
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  geometry_msgs
  hydra
  hydra_msgs
  hydra_visualizer
  kimera_pgmo_ros
  pose_graph_tools_ros
  semantic_inference_msgs
  tf2_eigen
  tf2_ros
)
ament_package()

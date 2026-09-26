cmake_minimum_required(VERSION 3.22)
project(hydra_multi_ros)

add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(config_utilities REQUIRED)
find_package(gflags REQUIRED)
find_package(kimera_pgmo REQUIRED)
find_package(kimera_pgmo_msgs REQUIRED)
find_package(kimera_pgmo_ros REQUIRED)
find_package(hydra REQUIRED)
find_package(hydra_msgs REQUIRED)
find_package(hydra_multi REQUIRED)
find_package(hydra_ros REQUIRED)
find_package(pose_graph_tools_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(spark_dsg REQUIRED)
find_package(tf2_eigen REQUIRED)

add_library(
  ${PROJECT_NAME}
  src/input/ros_dgraph_input.cpp
  src/input/ros_dsg_input.cpp
  src/interface/ros_unit_interface.cpp
  src/common.cpp
  src/multi_backend_publisher.cpp
  src/multi_ros_pipeline.cpp
  src/status_monitor.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
         "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC hydra_multi::hydra_multi
         hydra_ros::hydra_ros
)
ament_target_dependencies(
  ${PROJECT_NAME}
  PUBLIC
  geometry_msgs
  kimera_pgmo_msgs
  hydra_msgs
  std_msgs
  visualization_msgs
  tf2_eigen
)

rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN
  hydra::SceneGraphLogger
  EXECUTABLE
  scene_graph_logger
)

add_executable(hydra_multi_ros_node app/hydra_multi_ros_node.cpp)
target_link_libraries(hydra_multi_ros_node ${PROJECT_NAME} ${gflags_LIBRARIES})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS hydra_multi_ros_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  geometry_msgs
  hydra_multi
  hydra_ros
  kimera_pgmo_msgs
  std_msgs
)
ament_package()

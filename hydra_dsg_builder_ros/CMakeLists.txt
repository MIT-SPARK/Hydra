cmake_minimum_required(VERSION 3.1)
project(hydra_dsg_builder_ros)

add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Boost REQUIRED COMPONENTS timer)
find_package(spark_dsg REQUIRED)
find_package(GTSAM REQUIRED)
find_package(
  catkin REQUIRED
  COMPONENTS dynamic_reconfigure
             hydra_topology
             hydra_dsg_builder
             kimera_semantics_ros
             roscpp
             pcl_ros
             std_msgs
             tf2_eigen
             tf2_ros
             voxblox_ros
)

generate_dynamic_reconfigure_options(cfg/GvdVisualizer.cfg)

catkin_package(
  CATKIN_DEPENDS
  dynamic_reconfigure
  hydra_topology
  hydra_dsg_builder
  kimera_semantics_ros
  roscpp
  pcl_ros
  std_msgs
  tf2_eigen
  tf2_ros
  voxblox_ros
  DEPENDS spark_dsg
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

add_library(
  ${PROJECT_NAME}
  src/hydra_ros_pipeline.cpp
  src/node_utilities.cpp
  src/ros_backend.cpp
  src/ros_frontend.cpp
  src/ros_lcd_registration.cpp
  #src/visualizer_plugins.cpp
  src/gvd_visualization_utilities.cpp
  src/ros_reconstruction.cpp
  src/topology_server_visualizer.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  spark_dsg::spark_dsg
  gtsam
)

add_executable(${PROJECT_NAME}_node src/incremental_dsg_builder_node.cpp)
target_link_libraries(${PROJECT_NAME}_node PUBLIC ${PROJECT_NAME})

add_executable(hydra_topology_node src/hydra_topology_node.cpp)
target_link_libraries(hydra_topology_node PUBLIC ${PROJECT_NAME} Boost::boost)
add_dependencies(
  ${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

add_executable(gvd_validator src/gvd_validator.cpp)
target_link_libraries(gvd_validator PUBLIC ${PROJECT_NAME} yaml-cpp)

add_executable(dsg_optimizer_node src/dsg_optimizer_node.cpp)
target_link_libraries(dsg_optimizer_node ${PROJECT_NAME})

# TODO(nathan) handle install

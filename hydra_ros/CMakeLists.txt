cmake_minimum_required(VERSION 3.1)
project(hydra_ros)

add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Boost REQUIRED COMPONENTS timer)
# find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(spark_dsg REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
# TODO(nathan) find hydra independently
find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             dynamic_reconfigure
             std_msgs
             geometry_msgs
             visualization_msgs
             hydra_msgs
             pcl_ros
             tf2_eigen
             tf2_ros
             kimera_pgmo
             voxblox_ros
             pose_graph_tools
             kimera_semantics_ros
             hydra
)

generate_dynamic_reconfigure_options(
  cfg/Colormap.cfg
  cfg/DynamicLayerVisualizer.cfg
  cfg/GvdVisualizer.cfg
  cfg/LayerVisualizer.cfg
  cfg/Visualizer.cfg
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  dynamic_reconfigure
  std_msgs
  geometry_msgs
  visualization_msgs
  hydra_msgs
  pcl_ros
  tf2_eigen
  tf2_ros
  kimera_pgmo
  voxblox_ros
  pose_graph_tools
  kimera_semantics_ros
  DEPENDS spark_dsg hydra
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

add_library(
  ${PROJECT_NAME}
  src/config/ros_parser.cpp
  src/pipeline/hydra_ros_pipeline.cpp
  src/pipeline/ros_backend.cpp
  src/pipeline/ros_frontend.cpp
  src/pipeline/ros_lcd_registration.cpp
  src/pipeline/ros_reconstruction.cpp
  src/utils/dsg_streaming_interface.cpp
  src/utils/mask_nodelet.cpp
  src/utils/node_utilities.cpp
  src/visualizer/colormap_utilities.cpp
  src/visualizer/dsg_mesh_plugins.cpp
  src/visualizer/dynamic_scene_graph_visualizer.cpp
  src/visualizer/gvd_visualization_utilities.cpp
  src/visualizer/topology_server_visualizer.cpp
  src/visualizer/visualizer_plugins.cpp
  src/visualizer/visualizer_utilities.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  spark_dsg::spark_dsg
  gtsam
)
add_dependencies(
  ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

add_executable(dsg_optimizer_node src/nodes/dsg_optimizer_node.cpp)
target_link_libraries(dsg_optimizer_node ${PROJECT_NAME})

add_executable(gvd_validator_node src/nodes/gvd_validator_node.cpp)
target_link_libraries(gvd_validator_node PUBLIC ${PROJECT_NAME} yaml-cpp)

add_executable(hydra_ros_node src/nodes/hydra_node.cpp)
target_link_libraries(hydra_ros_node PUBLIC ${PROJECT_NAME})

add_executable(hydra_topology_node src/nodes/hydra_topology_node.cpp)
target_link_libraries(hydra_topology_node PUBLIC ${PROJECT_NAME} Boost::boost)

add_executable(hydra_visualizer_node src/nodes/hydra_visualizer_node.cpp)
target_link_libraries(hydra_visualizer_node PUBLIC ${PROJECT_NAME})

add_executable(rotate_tf_node src/nodes/rotate_tf_node.cpp)
target_link_libraries(rotate_tf_node PUBLIC ${PROJECT_NAME})

add_executable(scene_graph_logger_node src/nodes/scene_graph_logger_node.cpp)
target_link_libraries(scene_graph_logger_node PUBLIC ${PROJECT_NAME})

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()

# TODO(nathan) handle install
install(
  TARGETS ${PROJECT_NAME} hydra_ros_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

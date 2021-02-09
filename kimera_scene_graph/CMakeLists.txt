cmake_minimum_required(VERSION 3.1)
project(kimera_scene_graph)

add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Setup and linking for normal files
find_package(catkin REQUIRED COMPONENTS
    message_generation
    roscpp
    dynamic_reconfigure
    pluginlib
    std_msgs
    graph_cmr_ros
    kimera_semantics
    kimera_semantics_ros
    object_db
    voxblox
    voxblox_ros
    voxblox_skeleton)

find_package(OpenCV REQUIRED)
find_package(GTSAM REQUIRED)
find_package(KimeraRPGO REQUIRED)

add_service_files(DIRECTORY srv FILES CentroidErrorRequest.srv)

generate_dynamic_reconfigure_options(cfg/kimera_scene_graph.cfg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
    CATKIN_DEPENDS
        message_runtime
        roscpp
        dynamic_reconfigure
        object_db
        pluginlib
        std_msgs
        graph_cmr_ros
        kimera_semantics
        kimera_semantics_ros
        voxblox
        voxblox_ros
        voxblox_skeleton
    DEPENDS
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME})

# Serialization plugins (mostly to get around lz4 problems with flann)
add_library(humans_serialization_plugins
    src/humans_serialization_plugins.cpp)
target_include_directories(humans_serialization_plugins PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
)

# Main library for kimera Scene-Graph
add_library(${PROJECT_NAME}
  # Include files are just here for IDEs
  include/kimera_scene_graph/utils/voxblox_to_pcl.h
  include/kimera_scene_graph/common.h
  include/kimera_scene_graph/object_finder-inl.h
  include/kimera_scene_graph/object_place_connectivity_finder.h
  include/kimera_scene_graph/places_room_connectivity_finder.h
  include/kimera_scene_graph/scene_graph_node.h
  include/kimera_scene_graph/scene_graph_edge.h
  include/kimera_scene_graph/scene_graph_layer.h
  include/kimera_scene_graph/scene_graph_visualizer.h
  include/kimera_scene_graph/scene_graph.h
  include/kimera_scene_graph/dynamic_scene_graph.h
  include/kimera_scene_graph/dynamic_scene_graph_evaluator.h
  src/object_finder.cpp
  src/room_finder.cpp
  src/object_place_connectivity_finder.cpp
  src/places_room_connectivity_finder.cpp
  src/room_connectivity_finder.cpp
  src/wall_finder.cpp
  src/building_finder.cpp
  src/scene_graph_builder.cpp
  src/scene_graph_node.cpp
  src/scene_graph_edge.cpp
  src/scene_graph_layer.cpp
  src/scene_graph_visualizer.cpp
  src/scene_graph.cpp
  src/dynamic_scene_graph.cpp
  src/dynamic_scene_graph_evaluator.cpp
  src/semantic_ros_publishers.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  KimeraRPGO
  gtsam
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

# ROS node
add_executable(kimera_scene_graph_node
  src/kimera_scene_graph_node.cpp
)
target_link_libraries(kimera_scene_graph_node
  ${PROJECT_NAME}
  ${PCL_LIBRARIES}
)

add_executable(kimera_dynamic_scene_graph_node
  src/kimera_dynamic_scene_graph_node.cpp
)
target_link_libraries(kimera_dynamic_scene_graph_node
  ${PROJECT_NAME}
)

add_executable(kimera_dynamic_scene_graph_eval_node
  src/kimera_dynamic_scene_graph_eval_node.cpp
)
target_link_libraries(kimera_dynamic_scene_graph_eval_node
  ${PROJECT_NAME}
)

# For Visualization of the Scene Graph
add_executable(scene_graph_visualizer
  src/scene_graph_visualizer.cpp
)
target_link_libraries(scene_graph_visualizer ${PROJECT_NAME})

# For Evaluation of the Scene Graph (and construction)
add_executable(scene_graph_eval
  src/scene_graph_eval.cpp
)
target_link_libraries(scene_graph_eval
  ${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  tbb
)

# Add tests
if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(
    testKimeraSceneGraph
    tests/test_kimera_scene_graph.test
    tests/test_kimera_scene_graph.cpp
    tests/test_dynamic_scene_graph.cpp
    tests/test_dynamic_scene_graph_evaluator.cpp
  )
  target_link_libraries(testKimeraSceneGraph ${PROJECT_NAME})
endif()

# TODO(nathan) handle install

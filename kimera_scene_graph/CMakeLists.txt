cmake_minimum_required(VERSION 2.8.3)
project(kimera_scene_graph)

# Setup and linking for normal files
find_package(catkin_simple REQUIRED)
catkin_simple()

find_package(OpenCV REQUIRED)
find_package(GTSAM REQUIRED)

# Main library for kimera Scene-Graph
cs_add_library(${PROJECT_NAME}
  # Include files are just here for IDEs
  include/kimera_scene_graph/utils/voxblox_to_pcl.h
  include/kimera_scene_graph/common.h
  include/kimera_scene_graph/object_finder-inl.h
  include/kimera_scene_graph/scene_graph_node.h
  include/kimera_scene_graph/scene_graph_edge.h
  include/kimera_scene_graph/scene_graph_layer.h
  include/kimera_scene_graph/scene_graph_visualizer.h
  include/kimera_scene_graph/scene_graph.h
  src/object_finder.cpp
  src/room_finder.cpp
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
  #src/dynamic_scene_node.cpp
  #src/dynamic_scene_node_evaluator.cpp
  src/semantic_ros_publishers.cpp
  src/min_cut_solvers/stoer_wagner_min_cut_wrapper.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  tbb
)
target_include_directories(${PROJECT_NAME} PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${CATKIN_INCLUDE_DIRS}
)

# ROS node
cs_add_executable(kimera_scene_graph_node
  src/kimera_scene_graph_node.cpp
)
target_link_libraries(kimera_scene_graph_node
  ${PROJECT_NAME}
  ${PCL_LIBRARIES}
  tbb
)

# For Visualization of the Scene Graph
cs_add_executable(scene_graph_visualizer
  src/scene_graph_visualizer.cpp
)
target_link_libraries(scene_graph_visualizer ${PROJECT_NAME})

# For Evaluation of the Scene Graph (and construction)
cs_add_executable(scene_graph_eval
  src/scene_graph_eval.cpp
)
target_link_libraries(scene_graph_eval
  ${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  tbb
)

# Miscellaneous, todo(toni): remove...
cs_add_executable(min_cut
  src/min_cut_solvers/stoer_wagner_min_cut.cpp
)
target_link_libraries(min_cut
  ${PROJECT_NAME}
  ${PCL_LIBRARIES}
)

# Add tests
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_scene_node tests/test_scene_node.cpp)
  target_link_libraries(test_scene_node ${PROJECT_NAME})

  catkin_add_gtest(test_scene_graph_serialization
    tests/test_scene_graph_serialization.cpp
  )
  target_link_libraries(test_scene_graph_serialization ${PROJECT_NAME})
endif()

# catkin_add_gtest(test_scene_node_evaluator tests/test_scene_node_evaluator.cpp)
# target_link_libraries(test_scene_node_evaluator ${PROJECT_NAME})

cs_install()

cs_export()

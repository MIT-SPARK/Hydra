cmake_minimum_required(VERSION 2.8.3)
project(kimera_scene_graph)

# Setup and linking for normal files
find_package(catkin_simple REQUIRED)
catkin_simple()

find_package(OpenCV REQUIRED)
find_package(GTSAM REQUIRED)
find_package(KimeraRPGO REQUIRED)

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
  include/kimera_scene_graph/dynamic_scene_graph.h
  include/kimera_scene_graph/dynamic_scene_graph_evaluator.h
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
  src/dynamic_scene_graph.cpp
  src/dynamic_scene_graph_evaluator.cpp
  src/semantic_ros_publishers.cpp
  src/min_cut_solvers/stoer_wagner_min_cut_wrapper.cpp
)
target_link_libraries(${PROJECT_NAME} 
  ${OpenCV_LIBRARIES} 
  ${PCL_LIBRARIES} 
  gtsam gtsam_unstable
  tbb
  KimeraRPGO
)
target_include_directories(${PROJECT_NAME} PUBLIC 
  ${OpenCV_INCLUDE_DIRS}
  ${CATKIN_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
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

cs_add_executable(kimera_dynamic_scene_graph_node
  src/kimera_dynamic_scene_graph_node.cpp
)
target_link_libraries(kimera_dynamic_scene_graph_node
  ${PROJECT_NAME}
)

cs_add_executable(kimera_dynamic_scene_graph_eval_node
  src/kimera_dynamic_scene_graph_eval_node.cpp
)
target_link_libraries(kimera_dynamic_scene_graph_eval_node
  ${PROJECT_NAME}
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
  add_rostest_gtest(
    testKimeraSceneGraph
    tests/test_kimera_scene_graph.test
    tests/test_kimera_scene_graph.cpp
    tests/test_dynamic_scene_graph.cpp
    tests/test_dynamic_scene_graph_evaluator.cpp
  )
  target_link_libraries(testKimeraSceneGraph ${PROJECT_NAME})
endif()

cs_install()

cs_export()

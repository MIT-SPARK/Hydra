cmake_minimum_required(VERSION 2.8.3)
project(kimera_scene_graph)

# Setup and linking for normal files
find_package(catkin_simple REQUIRED)
catkin_simple()

find_package(OpenCV REQUIRED)

cs_add_library(${PROJECT_NAME}
  src/object_finder.cpp
  src/room_finder.cpp
  src/scene_graph_server.cpp
  src/semantic_ros_publishers.cpp
  src/min_cut_solvers/stoer_wagner_min_cut.cpp
  src/min_cut_solvers/stoer_wagner_min_cut_wrapper.cpp
)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} gtsam tbb)
target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})

cs_add_executable(kimera_scene_graph_node
  src/kimera_scene_graph.cpp
)
target_link_libraries(kimera_scene_graph_node ${PROJECT_NAME} gtsam)

cs_add_executable(scene_graph_eval
  src/scene_graph_eval.cpp
)
target_link_libraries(scene_graph_eval ${PROJECT_NAME} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

cs_add_executable(min_cut
  src/min_cut_solvers/stoer_wagner_min_cut.cpp
)
target_link_libraries(min_cut ${PROJECT_NAME} ${PCL_LIBRARIES})

# Add tests
catkin_add_gtest(test_scene_node tests/test_scene_node.cpp)
target_link_libraries(test_scene_node ${PROJECT_NAME}) 

cs_install()

cs_export()

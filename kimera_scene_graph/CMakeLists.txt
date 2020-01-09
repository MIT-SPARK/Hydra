cmake_minimum_required(VERSION 2.8.3)
project(kimera_scene_graph)

find_package(catkin_simple REQUIRED)

catkin_simple()

cs_add_library(${PROJECT_NAME}
  src/object_finder.cpp
  src/room_finder.cpp
  src/scene_graph_server.cpp
)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

cs_add_executable(kimera_scene_graph_node
  src/kimera_scene_graph.cpp
)
target_link_libraries(kimera_scene_graph_node ${PROJECT_NAME})

cs_add_executable(scene_graph_eval
  src/scene_graph_eval.cpp
)
target_link_libraries(scene_graph_eval ${PROJECT_NAME} ${PCL_LIBRARIES})

cs_install()

cs_export()

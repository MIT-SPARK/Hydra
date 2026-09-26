target_sources(
  ${PROJECT_NAME}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/deformation_graph_operator.cpp
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/pose_graph_operator.cpp
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/mesh_operator.cpp
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_scene_graph_operator.cpp
)

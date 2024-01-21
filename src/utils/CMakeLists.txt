target_sources(
  ${PROJECT_NAME}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/disjoint_set.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/display_utilities.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/log_utilities.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/mesh_utilities.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/minimum_spanning_tree.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/nearest_neighbor_utilities.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/pose_graph_tracker.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/teaser_params.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/timing_utilities.cpp
)

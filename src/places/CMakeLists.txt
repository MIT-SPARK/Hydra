target_sources(
  ${PROJECT_NAME}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/compression_graph_extractor.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/floodfill_graph_extractor.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/graph_extractor_config.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/graph_extractor_interface.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/graph_extractor_types.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/graph_extractor_utilities.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/gvd_graph.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/gvd_integrator.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/gvd_integrator_config.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/gvd_parent_tracker.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/gvd_thinning.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/gvd_utilities.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/gvd_voxel.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/nearest_voxel_utilities.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/update_statistics.cpp
)

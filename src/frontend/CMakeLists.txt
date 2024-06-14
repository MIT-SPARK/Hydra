target_sources(
  ${PROJECT_NAME}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/frontend_module.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/gvd_place_extractor.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/frontier_extractor.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/mesh_segmenter.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/place_mesh_connector.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/place_2d_segmenter.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/place_2d_split_logic.cpp
)

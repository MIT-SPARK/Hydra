target_sources(
  ${PROJECT_NAME}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/frontend_config.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/frontend_module.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/gvd_place_extractor.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/mesh_segmenter.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/mesh_segmenter_config.cpp
)

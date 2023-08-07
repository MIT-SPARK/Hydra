target_sources(
  ${PROJECT_NAME}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/graph_clustering.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/graph_filtration.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/room_finder.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/room_finder_config.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/room_utilities.cpp
)

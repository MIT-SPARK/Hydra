target_sources(
  ${PROJECT_NAME}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/embedding_distances.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/embedding_group.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/embedding_pooling.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/clustering/agglomerative_clustering.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/clustering/ib_utilities.cpp
)

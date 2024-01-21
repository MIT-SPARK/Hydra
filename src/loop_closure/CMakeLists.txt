target_sources(
  ${PROJECT_NAME}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/descriptor_matching.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/detector.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/graph_registration.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/loop_closure_config.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/loop_closure_module.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/scene_graph_descriptors.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/sensor_descriptor_factory.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/sensor_registration.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/subgraph_extraction.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/teaser_registration.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/visual_descriptors.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/visual_registration.cpp
)

if(HYDRA_ENABLE_GNN)
  target_sources(
    ${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/gnn_descriptors.cpp
  )
endif(HYDRA_ENABLE_GNN)

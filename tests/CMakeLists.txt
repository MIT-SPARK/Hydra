catkin_add_gtest(
  test_${PROJECT_NAME}
  main.cpp
  src/resources.cpp
  src/place_fixtures.cpp
  backend/test_merge_handler.cpp
  backend/test_update_functions.cpp
  config/test_config.cpp
  frontend/test_frontend.cpp
  loop_closure/test_descriptor_matching.cpp
  loop_closure/test_detector.cpp
  loop_closure/test_registration.cpp
  loop_closure/test_scene_graph_descriptors.cpp
  loop_closure/test_subgraph_extraction.cpp
  places/test_compression_graph_extractor.cpp
  places/test_esdf_comparison.cpp
  places/test_floodfill_graph_extractor.cpp
  places/test_graph_extractor_utilities.cpp
  places/test_gvd_incremental.cpp
  places/test_gvd_integrator.cpp
  places/test_gvd_thinning.cpp
  places/test_gvd_utilities.cpp
  reconstruction/test_marching_cubes.cpp
  reconstruction/test_reconstruction_module.cpp
  rooms/test_graph_clustering.cpp
  rooms/test_graph_filtration.cpp
  rooms/test_room_finder.cpp
  rooms/test_room_finder_config.cpp
  rooms/test_room_utilities.cpp
  utils/test_minimum_spanning_tree.cpp
  utils/test_nearest_neighbor_utilities.cpp
  utils/test_timing_utilities.cpp
)
target_include_directories(test_${PROJECT_NAME} PUBLIC include)
target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})

if(HYDRA_GNN)
  target_sources(
    test_${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/loop_closure/test_gnn_descriptors.cpp
  )
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME}_gnn)
endif()

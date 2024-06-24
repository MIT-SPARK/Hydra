# catkin forwards gtest, but eventually we won't have catkin dependencies
if(NOT TARGET gtest)
  include(FetchContent)
  FetchContent_Declare(
    googletest GIT_REPOSITORY https://github.com/google/googletest.git GIT_TAG v1.13.0
  )

  populate_content(googletest BUILD_SHARED_LIBS=OFF)
endif()

configure_file(hydra_test_config.h.in hydra_test_config.h)
include(GoogleTest)
enable_testing()

add_executable(
  test_${PROJECT_NAME}
  main.cpp
  src/resources.cpp
  src/place_fixtures.cpp
  backend/test_update_agents_functor.cpp
  backend/test_update_objects_functor.cpp
  backend/test_update_places_functor.cpp
  backend/test_update_rooms_buildings_functor.cpp
  common/test_config_utilities.cpp
  input/test_camera.cpp
  input/test_lidar.cpp
  input/test_sensor.cpp
  input/test_sensor_utilities.cpp
  loop_closure/test_descriptor_matching.cpp
  loop_closure/test_detector.cpp
  loop_closure/test_registration.cpp
  loop_closure/test_scene_graph_descriptors.cpp
  loop_closure/test_subgraph_extraction.cpp
  places/test_compression_graph_extractor.cpp
  places/test_floodfill_graph_extractor.cpp
  places/test_graph_extractor_utilities.cpp
  places/test_gvd_integrator.cpp
  places/test_gvd_utilities.cpp
  places/test_voxel_templates.cpp
  reconstruction/test_marching_cubes.cpp
  reconstruction/test_reconstruction_module.cpp
  reconstruction/test_semantic_integrator.cpp
  reconstruction/test_tsdf_interpolators.cpp
  reconstruction/test_volumetric_map.cpp
  rooms/test_graph_clustering.cpp
  rooms/test_graph_filtration.cpp
  rooms/test_room_finder.cpp
  rooms/test_room_utilities.cpp
  utils/test_active_window_tracker.cpp
  utils/test_minimum_spanning_tree.cpp
  utils/test_nearest_neighbor_utilities.cpp
  utils/test_pose_graph_tracker.cpp
  utils/test_timing_utilities.cpp
)
target_include_directories(
  test_${PROJECT_NAME}
  PUBLIC include
  PRIVATE ${CMAKE_CURRENT_BINARY_DIR}
)
target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} GTest::gtest_main)
gtest_add_tests(TARGET test_${PROJECT_NAME})

if(HYDRA_ENABLE_GNN)
  target_sources(
    test_${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/loop_closure/test_gnn_descriptors.cpp
           ${CMAKE_CURRENT_SOURCE_DIR}/gnn/test_gnn_interface.cpp
           ${CMAKE_CURRENT_SOURCE_DIR}/gnn/test_tensor.cpp
  )
endif()

if(${HYDRA_ENABLE_ROS_INSTALL_LAYOUT})
  install(TARGETS test_${PROJECT_NAME}
          RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
  )
endif()

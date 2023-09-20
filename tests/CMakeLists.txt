# catkin forwards gtest, but eventually we won't have catkin dependencies
if(NOT TARGET gtest)
  include(FetchContent)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.13.0
  )

  FetchContent_GetProperties(googletest)
  if(NOT googletest_POPULATED)
    FetchContent_Populate(googletest)
    set(BUILD_SHARED_LIBS_THIS ${BUILD_SHARED_LIBS})
    set(BUILD_SHARED_LIBS
        OFF
        CACHE INTERNAL "Build static libraries"
    )
    add_subdirectory(${googletest_SOURCE_DIR} ${googletest_BINARY_DIR} EXCLUDE_FROM_ALL)
    set(BUILD_SHARED_LIBS
        ${BUILD_SHARED_LIBS_THIS}
        CACHE BOOL "Build shared libraries"
    )
  endif()
endif()

add_executable(
  test_${PROJECT_NAME}
  main.cpp
  src/resources.cpp
  src/place_fixtures.cpp
  backend/test_merge_handler.cpp
  backend/test_update_functions.cpp
  common/test_config_utilities.cpp
  # frontend/test_frontend.cpp
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
  reconstruction/test_camera.cpp
  reconstruction/test_lidar.cpp
  reconstruction/test_marching_cubes.cpp
  reconstruction/test_reconstruction_module.cpp
  reconstruction/test_semantic_integrator.cpp
  reconstruction/test_sensor.cpp
  reconstruction/test_sensor_utilities.cpp
  reconstruction/test_volumetric_map.cpp
  rooms/test_graph_clustering.cpp
  rooms/test_graph_filtration.cpp
  rooms/test_room_finder.cpp
  rooms/test_room_utilities.cpp
  utils/test_minimum_spanning_tree.cpp
  utils/test_nearest_neighbor_utilities.cpp
  utils/test_pose_graph_tracker.cpp
  utils/test_timing_utilities.cpp
)
target_include_directories(test_${PROJECT_NAME} PUBLIC include)
target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} GTest::gtest_main)

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

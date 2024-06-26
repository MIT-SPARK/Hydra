target_sources(
  ${PROJECT_NAME}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/active_window_tracker.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/csv_reader.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/disjoint_set.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/display_utilities.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/log_utilities.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/mesh_utilities.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/minimum_spanning_tree.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/nearest_neighbor_utilities.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/pgmo_glog_sink.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/pgmo_mesh_interface.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/pgmo_mesh_traits.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/place_2d_ellipsoid_math.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/timing_utilities.cpp
)

target_sources(
  ${PROJECT_NAME}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/association_strategies.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/backend_module.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/backend_utilities.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/dsg_updater.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/external_loop_closure_receiver.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/merge_proposer.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/merge_tracker.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/mesh_clustering.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/mst_factors.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/pgmo_configs.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/surface_place_utilities.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/update_agents_functor.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/update_buildings_functor.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/update_frontiers_functor.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/update_functions.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/update_objects_functor.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/update_places_functor.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/update_rooms_functor.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/update_surface_places_functor.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/zmq_interfaces.cpp
)

target_sources(
  ${PROJECT_NAME}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/backend_module.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/backend_utilities.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/merge_tracker.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/pgmo_configs.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/surface_place_utilities.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/update_agents_functor.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/update_frontiers_functor.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/update_functions.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/update_objects_functor.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/update_places_functor.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/update_rooms_buildings_functor.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/update_surface_places_functor.cpp
)

target_sources(
  ${PROJECT_NAME}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/integration_masking.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/marching_cubes.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/mesh_integrator.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/mesh_integrator_config.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/projection_interpolators.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/projective_integrator.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/semantic_integrator.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/tsdf_interpolators.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/volumetric_map.cpp
)

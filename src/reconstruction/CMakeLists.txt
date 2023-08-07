target_sources(
  ${PROJECT_NAME}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/combo_integrator.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/marching_cubes.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/mesh_integrator.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/mesh_integrator_config.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/reconstruction_config.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/reconstruction_module.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/reconstruction_output.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/semantic_mesh_layer.cpp
)

target_sources(
  ${PROJECT_NAME}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/backend_module.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/merge_handler.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/pgmo_configs.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/update_functions.cpp
)

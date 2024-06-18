target_sources(
  ${PROJECT_NAME}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/batch_pipeline.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/config_utilities.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/global_info.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/hydra_pipeline.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/label_remapper.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/label_space_config.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/robot_prefix_config.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/semantic_color_map.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/shared_dsg_info.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/shared_module_state.cpp
)

target_sources(
  ${PROJECT_NAME}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/camera.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/data_receiver.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/input_data.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/input_module.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/input_packet.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/lidar.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/sensor.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/sensor_input_packet.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/sensor_utilities.cpp
)

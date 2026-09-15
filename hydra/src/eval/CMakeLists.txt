target_sources(
  ${PROJECT_NAME}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/place_evaluator.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/place_metrics.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/room_evaluator.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/room_io.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/room_metrics.cpp
)

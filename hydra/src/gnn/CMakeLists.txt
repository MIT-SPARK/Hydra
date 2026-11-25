target_sources(
  ${PROJECT_NAME}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/gnn_interface.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/ort_utilities.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/tensor.cpp
)

add_executable(gnn_model_reader gnn_model_reader.cpp)
target_link_libraries(gnn_model_reader ${PROJECT_NAME})

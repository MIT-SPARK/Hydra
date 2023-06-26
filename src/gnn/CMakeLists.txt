configure_file(cmake/ort.CMakeLists.txt.in ort-download/CMakeLists.txt)
execute_process(
  COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" .
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/ort-download" # OUTPUT_QUIET
)
execute_process(
  COMMAND "${CMAKE_COMMAND}" --build .
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/ort-download" # OUTPUT_QUIET
)

add_library(ort::ort IMPORTED INTERFACE)
set_property(
  TARGET ort::ort PROPERTY INTERFACE_INCLUDE_DIRECTORIES
                           "${CMAKE_CURRENT_BINARY_DIR}/ort-src/include"
)
set_property(
  TARGET ort::ort PROPERTY INTERFACE_LINK_LIBRARIES
                           "${CMAKE_CURRENT_BINARY_DIR}/ort-src/lib/libonnxruntime.so"
)

# TODO(nathan) handle glog better (i.e. don't require catkin)
add_library(${PROJECT_NAME}_gnn gnn_interface.cpp ort_utilities.cpp tensor.cpp)
target_include_directories(
  ${PROJECT_NAME}_gnn PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                             $<INSTALL_INTERFACE:include> ${catkin_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_gnn ort::ort ${catkin_LIBRARIES})

add_executable(gnn_model_reader gnn_model_reader.cpp)
target_link_libraries(gnn_model_reader ${PROJECT_NAME}_gnn)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(
    utest_${PROJECT_NAME}_gnn tests/utest_main.cpp tests/utest_gnn_interface.cpp
                              tests/utest_tensor.cpp
  )
  target_link_libraries(utest_${PROJECT_NAME}_gnn ${PROJECT_NAME}_gnn)
endif()

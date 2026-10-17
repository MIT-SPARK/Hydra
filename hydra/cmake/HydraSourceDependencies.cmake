include(FetchContent)

if(HYDRA_ENABLE_GNN)
  # TODO(nathan) fetch content
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
    TARGET ort::ort
    PROPERTY INTERFACE_LINK_LIBRARIES
             "${CMAKE_CURRENT_BINARY_DIR}/ort-src/lib/libonnxruntime.so"
  )
endif()

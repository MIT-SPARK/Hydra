find_package(CUDA REQUIRED)

include(ExternalProject)

# TODO(nathan) architecture dependent URL with cross-compiled version
set(ORT_URL
    "https://github.com/microsoft/onnxruntime/releases/download/v1.13.1/onnxruntime-linux-x64-gpu-1.13.1.tgz"
)

ExternalProject_Add(
  ort
  URL ${ORT_URL}
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ""
)

# TODO(nathan) consider namespacing
ExternalProject_Get_Property(ort SOURCE_DIR)

if(NOT TARGET ort::ort)
  add_library(ort::ort IMPORTED INTERFACE)
  set_property(
    TARGET ort::ort PROPERTY INTERFACE_INCLUDE_DIRECTORIES "${SOURCE_DIR}/include"
  )
  set_property(
    TARGET ort::ort PROPERTY INTERFACE_LINK_LIBRARIES
                             "${SOURCE_DIR}/lib/libonnxruntime.so"
  )
  add_dependencies(ort::ort ort)
endif()

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

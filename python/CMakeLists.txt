cmake_minimum_required(VERSION 3.14)
project(hydra_python)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wno-maybe-uninitialized)

find_package(Python COMPONENTS Interpreter Development)
find_package(pybind11 CONFIG REQUIRED)
if(PROJECT_IS_TOP_LEVEL)
  find_package(hydra REQUIRED)
endif()

pybind11_add_module(
  _hydra_bindings
  bindings/src/bindings.cpp
  bindings/src/color_parser.cpp
  bindings/src/depth_parser.cpp
  bindings/src/glog_utilities.cpp
  bindings/src/label_parser.cpp
  bindings/src/python_batch.cpp
  bindings/src/python_image.cpp
  bindings/src/python_pipeline.cpp
  bindings/src/python_reconstruction.cpp
  bindings/src/python_sensor_input.cpp
  bindings/src/python_sensors.cpp
)
target_link_libraries(_hydra_bindings PRIVATE hydra::hydra)
target_compile_options(_hydra_bindings PRIVATE -Wno-maybe-uninitialized)
target_include_directories(
  _hydra_bindings PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/bindings/include
)

if(DEFINED SKBUILD AND "${SKBUILD}")
  install(TARGETS _hydra_bindings DESTINATION ${PROJECT_NAME})
endif()

cmake_minimum_required(VERSION 3.14)
project(hydra_python)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wno-unused-parameter)

include(FetchContent)
FetchContent_Declare(
  pybind11
  GIT_REPOSITORY https://github.com/pybind/pybind11.git
  GIT_TAG master
)
FetchContent_MakeAvailable(pybind11)

pybind11_add_module(
  _hydra_bindings
  bindings/src/batch.cpp
  bindings/src/bindings.cpp
  bindings/src/color_parser.cpp
  bindings/src/config_bindings.cpp
  bindings/src/depth_parser.cpp
  bindings/src/glog_utilities.cpp
  bindings/src/hydra_python_pipeline.cpp
  bindings/src/label_parser.cpp
  bindings/src/python_config.cpp
  bindings/src/python_image.cpp
  bindings/src/python_reconstruction.cpp
  bindings/src/python_sensor_input.cpp
)
target_link_libraries(_hydra_bindings PRIVATE hydra)
target_include_directories(
  _hydra_bindings PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/bindings/include
)

if(HYDRA_ENABLE_COVERAGE)
  target_compile_options(_hydra_bindings PRIVATE --coverage)
  target_link_options(_hydra_bindings PRIVATE --coverage)
endif()

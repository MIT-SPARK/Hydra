cmake_minimum_required(VERSION 3.14)
project(hydra VERSION 2.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

add_compile_options(-Wall -Wextra)

option(HYDRA_ENABLE_COVERAGE "Build core library with GCC --coverage option" OFF)
option(HYDRA_ENABLE_EVAL "Build evaluation code" ON)
option(HYDRA_ENABLE_GNN "Build GNN interface" OFF)
option(HYDRA_ENABLE_PYTHON "Build Hydra python bindings" OFF)
option(HYDRA_ENABLE_TESTS "Build Hydra unit tests" OFF)
option(HYDRA_ENABLE_ROS_INSTALL_LAYOUT "Install binaries to ROS location" ON)
option(BUILD_SHARED_LIBS "Build shared libs" ON)

find_package(config_utilities REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(kimera_pgmo REQUIRED)
find_package(spatial_hash REQUIRED)
find_package(spark_dsg REQUIRED)
find_package(teaserpp REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
# we turn off PCL precompile internally to get around having vtk linked. Note: kdtree is
# REQUIRED to make sure we link against FLANN (used by euclidean extraction)
find_package(PCL REQUIRED COMPONENTS common kdtree)

include(GNUInstallDirs)
include(HydraBuildConfig)
include(HydraSourceDependencies)

add_library(${PROJECT_NAME})
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC $<INSTALL_INTERFACE:include>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> ${OpenCV_INCLUDE_DIRS}
  PRIVATE ${CMAKE_CURRENT_BINARY_DIR} ${PCL_INCLUDE_DIRS}
)
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC config_utilities::config_utilities
         Eigen3::Eigen
         gtsam
         kimera_pgmo::kimera_pgmo
         spark_dsg::spark_dsg
         spatial_hash::spatial_hash
         teaserpp::teaser_registration
         ${OpenCV_LIBRARIES}
  PRIVATE nanoflann::nanoflann ${PCL_LIBRARIES}
)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
add_library(hydra::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

add_subdirectory(src)
if(HYDRA_ENABLE_GNN)
  add_subdirectory(src/gnn)
  target_link_libraries(${PROJECT_NAME} PRIVATE ort::ort)
endif()

if(HYDRA_ENABLE_COVERAGE)
  target_compile_options(${PROJECT_NAME} PRIVATE --coverage)
  target_link_options(${PROJECT_NAME} PRIVATE --coverage)
endif()

if(HYDRA_ENABLE_EVAL)
  add_subdirectory(eval)
endif()

if(HYDRA_ENABLE_PYTHON)
  add_subdirectory(python)
endif()

if(HYDRA_ENABLE_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

include(HydraInstall)

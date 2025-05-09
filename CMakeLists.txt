cmake_minimum_required(VERSION 3.14)
project(hydra VERSION 2.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

add_compile_options(-Wall -Wextra)

option(HYDRA_ENABLE_EVAL "Build evaluation code" ON)
option(HYDRA_ENABLE_GNN "Build GNN interface" OFF)
option(HYDRA_ENABLE_PYTHON "Build Hydra python bindings" OFF)
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

if(HYDRA_ENABLE_EVAL)
  add_subdirectory(eval)
endif()

if(HYDRA_ENABLE_PYTHON)
  add_subdirectory(python)
endif()

include(CTest)
if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(tests)
endif()

include(GNUInstallDirs)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT hydra-targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT hydra-targets FILE hydraTargets.cmake NAMESPACE hydra::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/hydra
)

include(CMakePackageConfigHelpers)
configure_package_config_file(
  ${CMAKE_CURRENT_LIST_DIR}/cmake/hydraConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/hydraConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/hydra
)
write_basic_package_version_file(
  hydraConfigVersion.cmake VERSION ${PACKAGE_VERSION} COMPATIBILITY AnyNewerVersion
)
install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/hydraConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/hydraConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/hydra
)

find_package(ament_cmake_core QUIET)
if (${ament_cmake_core_FOUND})
  # NOTE(nathan) this will actually likely change once we drop ros1 support, but
  # for now this is a good proxy if we are in ros2 and need to install the config files
  # versus in ros1 (where installing them breaks things)
  install(DIRECTORY config DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})
  ament_package()
endif()

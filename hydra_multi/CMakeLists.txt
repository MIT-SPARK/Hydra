cmake_minimum_required(VERSION 3.22.1)
project(hydra_multi VERSION 2.0.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra)

option(HYDRA_MULTI_ENABLE_TESTS "Build Hydra-Multi unit tests" ON)

find_package(CLI11 REQUIRED)
find_package(config_utilities REQUIRED)
find_package(GTSAM REQUIRED)
find_package(hydra REQUIRED)
find_package(kimera_pgmo REQUIRED)
find_package(spark_dsg REQUIRED)

add_library(
  ${PROJECT_NAME}
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC $<INSTALL_INTERFACE:include>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PRIVATE ${CMAKE_CURRENT_BINARY_DIR}
)
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC config_utilities::config_utilities
         spark_dsg::spark_dsg
         hydra::hydra
         kimera_pgmo::kimera_pgmo
         gtsam
)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
add_library(
  hydra_multi::${PROJECT_NAME} ALIAS ${PROJECT_NAME}
)

add_subdirectory(src)

add_executable(offline_fusion app/offline_fusion.cpp)
target_link_libraries(offline_fusion ${PROJECT_NAME} CLI11::CLI11)
set_target_properties(offline_fusion PROPERTIES OUTPUT_NAME hydra-multi-offline-fusion)

include(CTest)
if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(tests)
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT hydra_multi-targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(TARGETS offline_fusion RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY config DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})

install(
  EXPORT hydra_multi-targets
  FILE hydra_multiTargets.cmake
  NAMESPACE hydra_multi::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/hydra_multi
)

configure_package_config_file(
  ${CMAKE_CURRENT_LIST_DIR}/cmake/hydra_multiConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/hydra_multiConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/hydra_multi
)
write_basic_package_version_file(
  hydra_multiConfigVersion.cmake
  VERSION ${PACKAGE_VERSION}
  COMPATIBILITY AnyNewerVersion
)
install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/hydra_multiConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/hydra_multiConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/hydra_multi
)

find_package(ament_cmake_core QUIET)
if(${ament_cmake_core_FOUND})
  ament_package()
endif()

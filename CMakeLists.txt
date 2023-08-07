cmake_minimum_required(VERSION 3.10)
project(hydra)

add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/Filesystem.cmake)
has_std_filesystem(HYDRA_USE_STANDARD_FS)

option(HYDRA_USE_COVERAGE "Build core library with GCC --coverage option" OFF)
option(HYDRA_GNN "Build GNN interface" ON)
option(HYDRA_ENABLE_EVAL "Build evaluation code" ON)

# TODO(nathan) clean this up
if(HYDRA_GNN)
  set(HYDRA_USE_GNN_CXX_VALUE 1)
else()
  set(HYDRA_USE_GNN_CXX_VALUE 0)
endif()

if(${HYDRA_USE_STANDARD_FS})
  set(HYDRA_USE_STANDARD_FS_CXX_VALUE 1)
else()
  set(HYDRA_USE_STANDARD_FS_CXX_VALUE 0)
endif()

configure_file(hydra_build_config.h.in ${CMAKE_CURRENT_BINARY_DIR}/hydra_build_config.h)

find_package(Boost REQUIRED COMPONENTS timer)
find_package(spark_dsg REQUIRED)
find_package(teaserpp REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(config_utilities REQUIRED)
find_package(
  catkin REQUIRED
  COMPONENTS kimera_pgmo
             kimera_semantics
             tf2_eigen
             voxblox
)

set(PCL_FIND_QUIETLY TRUE)
find_package(PCL REQUIRED COMPONENTS common)

configure_file(cmake/nanoflann.CMakeLists.txt.in nanoflann-download/CMakeLists.txt)
execute_process(
  COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" .
  WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/nanoflann-download"
  OUTPUT_QUIET
)
execute_process(
  COMMAND "${CMAKE_COMMAND}" --build .
  WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/nanoflann-download"
  OUTPUT_QUIET
)

set(NANOFLANN_BUILD_EXAMPLES
    OFF
    CACHE INTERNAL ""
)
set(NANOFLANN_BUILD_TESTS
    OFF
    CACHE INTERNAL ""
)
add_subdirectory(${CMAKE_BINARY_DIR}/nanoflann-src ${CMAKE_BINARY_DIR}/nanoflann-build)

catkin_package(
  CATKIN_DEPENDS
  kimera_pgmo
  kimera_semantics
  tf2_eigen
  voxblox
  DEPENDS spark_dsg config_utilities
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} gtsam
)

add_library(${PROJECT_NAME})
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC include ${catkin_INCLUDE_DIRS}
  PRIVATE ${CMAKE_CURRENT_BINARY_DIR} ${PCL_INCLUDE_DIRS}
)
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC ${catkin_LIBRARIES}
         teaserpp::teaser_registration
         config_utilities::config_utilities
         spark_dsg::spark_dsg
         yaml-cpp
         gtsam
  PRIVATE nanoflann::nanoflann ${PCL_LIBRARIES}
)
if(HYDRA_USE_STANDARD_FS)
  target_link_libraries(${PROJECT_NAME} PRIVATE stdc++fs)
else()
  find_package(
    Boost
    COMPONENTS filesystem
    REQUIRED
  )
  target_link_libraries(${PROJECT_NAME} PRIVATE Boost::filesystem)
endif()

add_subdirectory(src)
if(HYDRA_GNN)
  add_subdirectory(src/gnn)
  target_link_libraries(${PROJECT_NAME} PUBLIC ${PROJECT_NAME}_gnn)
endif()

if(HYDRA_USE_COVERAGE)
  target_compile_options(${PROJECT_NAME} PRIVATE --coverage)
  # TODO(nathan) once we have newer cmake, target link options
  target_link_libraries(${PROJECT_NAME} PRIVATE --coverage)
endif()

if(HYDRA_ENABLE_EVAL)
  add_subdirectory(eval)
endif()

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()

# TODO(nathan) handle install

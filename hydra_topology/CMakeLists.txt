cmake_minimum_required(VERSION 3.1)
project(hydra_topology)

add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(spark_dsg REQUIRED)
find_package(
  catkin REQUIRED
  COMPONENTS dynamic_reconfigure
             hydra_utils
             hydra_msgs
             kimera_semantics_ros
             pcl_ros
             roscpp
             tf2_eigen
             tf2_ros
             voxblox
             voxblox_ros
)

generate_dynamic_reconfigure_options(cfg/GvdVisualizer.cfg)

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
  dynamic_reconfigure
  hydra_utils
  hydra_msgs
  kimera_semantics_ros
  pcl_ros
  roscpp
  tf2_eigen
  tf2_ros
  voxblox
  voxblox_ros
  DEPENDS spark_dsg
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

add_library(
  ${PROJECT_NAME}
  src/graph_extractor.cpp
  src/graph_extractor_types.cpp
  src/graph_extraction_utilities.cpp
  src/gvd_integrator.cpp
  src/gvd_utilities.cpp
  src/gvd_visualization_utilities.cpp
  src/gvd_voxel.cpp
  src/nearest_neighbor_utilities.cpp
  src/topology_server_visualizer.cpp
  src/voxel_aware_marching_cubes.cpp
  src/voxel_aware_mesh_integrator.cpp
)
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC ${catkin_LIBRARIES} spark_dsg::spark_dsg
  PRIVATE nanoflann::nanoflann
)
target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node src/hydra_topology_node.cpp)
target_link_libraries(${PROJECT_NAME}_node PUBLIC ${PROJECT_NAME})
add_dependencies(
  ${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(
    utest_${PROJECT_NAME}
    tests/utest_main.cpp
    tests/src/test_fixtures.cpp
    tests/utest_esdf.cpp
    tests/utest_esdf_helpers.cpp
    tests/utest_graph_extraction_utilities.cpp
    tests/utest_graph_extractor.cpp
    tests/utest_gvd_utilities.cpp
    tests/utest_marching_cubes.cpp
    tests/utest_nearest_neighbor_utilities.cpp
    tests/utest_incremental_gvd.cpp
    tests/utest_incremental_integration.cpp
  )
  target_include_directories(
    utest_${PROJECT_NAME} PUBLIC tests/include ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(utest_${PROJECT_NAME} ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()

install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

cmake_minimum_required(VERSION 3.1)
project(kimera_topology)

add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             dynamic_reconfigure
             kimera_dsg
             kimera_dsg_visualizer
             kimera_semantics_ros
             message_generation
             std_msgs
             tf2_eigen
             voxblox
             voxblox_ros
)
find_package(yaml-cpp REQUIRED)

add_message_files(FILES ActiveLayer.msg)

generate_messages(DEPENDENCIES std_msgs)

generate_dynamic_reconfigure_options(cfg/GvdVisualizerConfig.cfg)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  dynamic_reconfigure
  kimera_dsg
  kimera_dsg_visualizer
  kimera_semantics_ros
  message_runtime
  voxblox
  voxblox_ros
  tf2_eigen
  DEPENDS
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

add_library(
  ${PROJECT_NAME}
  src/config_parser.cpp
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
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC include ${catkin_INCLUDE_DIRS}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/third_party/nanoflann/include
)

add_executable(${PROJECT_NAME}_node src/kimera_topology_node.cpp)
target_link_libraries(${PROJECT_NAME}_node PUBLIC ${PROJECT_NAME} ${catkin_LIBRARIES})
target_include_directories(${PROJECT_NAME}_node PUBLIC include ${catkin_INCLUDE_DIRS})
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(places_eval_node src/places_eval_node.cpp)
target_link_libraries(places_eval_node PUBLIC ${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)
target_include_directories(places_eval_node PUBLIC include ${catkin_INCLUDE_DIRS})

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

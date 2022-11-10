cmake_minimum_required(VERSION 3.1)
project(hydra_dsg_builder)

add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_WITH_COVERAGE "Build core library with GCC --coverage option" ON)
option(HYDRA_GNN "build GNN interface" ON)

find_package(Boost REQUIRED COMPONENTS timer)
find_package(spark_dsg REQUIRED)
find_package(
  catkin REQUIRED
  COMPONENTS hydra_topology
             kimera_pgmo
             kimera_semantics
             std_msgs
             tf2_eigen
             voxblox
)

find_package(teaserpp REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(KimeraRPGO REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  hydra_topology
  kimera_pgmo
  kimera_semantics
  std_msgs
  tf2_eigen
  voxblox
  DEPENDS spark_dsg
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} gtsam
)

if(HYDRA_GNN)
  add_subdirectory(src/gnn)
endif()

add_library(
  ${PROJECT_NAME}
  src/dsg_lcd_descriptors.cpp
  src/dsg_lcd_matching.cpp
  src/dsg_lcd_detector.cpp
  src/dsg_lcd_registration.cpp
  src/dsg_update_functions.cpp
  src/hydra_config.cpp
  src/graph_clustering.cpp
  src/graph_filtration.cpp
  src/incremental_dsg_backend.cpp
  src/incremental_dsg_frontend.cpp
  src/incremental_dsg_lcd.cpp
  src/incremental_mesh_segmenter.cpp
  src/room_colormap.cpp
  src/room_finder.cpp
  src/room_helpers.cpp
  src/merge_handler.cpp
  src/minimum_spanning_tree.cpp
  src/shared_module_state.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC ${catkin_LIBRARIES}
         teaserpp::teaser_registration
         spark_dsg::spark_dsg
         gtsam
)
if(BUILD_WITH_COVERAGE)
  target_compile_options(${PROJECT_NAME} PRIVATE --coverage)
  # TODO(nathan) once we have newer cmake, target link options
  target_link_libraries(${PROJECT_NAME} PRIVATE --coverage)
endif()

if(HYDRA_GNN)
  target_link_libraries(${PROJECT_NAME} PUBLIC ${PROJECT_NAME}_gnn)
endif()

if(CATKIN_ENABLE_TESTING)
  find_package(yaml-cpp)
  catkin_add_gtest(
    utest_${PROJECT_NAME}
    tests/utest_main.cpp
    tests/utest_configs.cpp
    tests/utest_dsg_lcd_registration.cpp
    tests/utest_dsg_lcd_descriptors.cpp
    tests/utest_dsg_lcd_matching.cpp
    tests/utest_dsg_lcd_module.cpp
    tests/utest_dsg_update_functions.cpp
    tests/utest_graph_clustering.cpp
    tests/utest_graph_filtration.cpp
    tests/utest_merge_handler.cpp
    tests/utest_minimum_spanning_tree.cpp
    tests/utest_room_helpers.cpp
    tests/utest_room_finder.cpp
  )
  target_link_libraries(utest_${PROJECT_NAME} ${PROJECT_NAME} yaml-cpp)
endif()

# TODO(nathan) handle install

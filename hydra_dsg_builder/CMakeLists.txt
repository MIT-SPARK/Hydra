cmake_minimum_required(VERSION 3.1)
project(hydra_dsg_builder)

add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(PCL_FIND_QUIETLY TRUE)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS timer)
find_package(spark_dsg REQUIRED)
find_package(
  catkin REQUIRED
  COMPONENTS hydra_topology
             kimera_pgmo
             kimera_semantics
             kimera_semantics_ros
             roscpp
             std_msgs
             tf2_eigen
             voxblox
             voxblox_ros
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
  kimera_semantics_ros
  roscpp
  std_msgs
  tf2_eigen
  voxblox
  voxblox_ros
  DEPENDS spark_dsg
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

add_library(
  ${PROJECT_NAME}_pcl src/incremental_mesh_segmenter.cpp src/object_helpers.cpp
)
target_include_directories(
  ${PROJECT_NAME}_pcl
  PUBLIC include
         ${PCL_INCLUDE_DIRS}
         ${kimera_semantics_INCLUDE_DIRS}
         ${roscpp_INCLUDE_DIRS}
         ${hydra_utils_INCLUDE_DIRS}
)
target_link_libraries(
  ${PROJECT_NAME}_pcl
  PUBLIC ${PCL_LIBRARIES}
         ${kimera_semantics_LIBRARIES}
         ${roscpp_LIBRARIES}
         ${hydra_utils_LIBRARIES}
         spark_dsg::spark_dsg
)

add_library(
  ${PROJECT_NAME}
  src/dsg_lcd_descriptors.cpp
  src/dsg_lcd_matching.cpp
  src/dsg_lcd_detector.cpp
  src/dsg_lcd_registration.cpp
  src/dsg_update_functions.cpp
  src/incremental_dsg_backend.cpp
  src/incremental_dsg_frontend.cpp
  src/incremental_dsg_lcd.cpp
  src/incremental_room_finder.cpp
  src/lcd_visualizer.cpp
  src/minimum_spanning_tree.cpp
  src/visualizer_plugins.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC ${PROJECT_NAME}_pcl
         ${catkin_LIBRARIES}
         teaserpp::teaser_registration
         spark_dsg::spark_dsg
         gtsam
)

add_executable(${PROJECT_NAME}_incremental_node src/incremental_dsg_builder_node.cpp)
target_link_libraries(${PROJECT_NAME}_incremental_node ${PROJECT_NAME})

add_executable(dsg_optimizer_node src/dsg_optimizer_node.cpp)
target_link_libraries(dsg_optimizer_node ${PROJECT_NAME})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(
    utest_${PROJECT_NAME}
    tests/utest_main.cpp
    tests/utest_dsg_lcd_registration.cpp
    tests/utest_dsg_lcd_descriptors.cpp
    tests/utest_dsg_lcd_matching.cpp
    tests/utest_dsg_lcd_module.cpp
    tests/utest_dsg_update_functions.cpp
    tests/utest_incremental_room_finder.cpp
    tests/utest_minimum_spanning_tree.cpp
  )
  target_link_libraries(utest_${PROJECT_NAME} ${PROJECT_NAME})
endif()

# TODO(nathan) handle install

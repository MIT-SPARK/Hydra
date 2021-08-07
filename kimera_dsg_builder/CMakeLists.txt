cmake_minimum_required(VERSION 3.1)
project(kimera_dsg_builder)

add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
option(DSG_USE_ASAN "enable address sanatizer" OFF)

# Setup and linking for normal files
find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             dynamic_reconfigure
             pluginlib
             std_msgs
             kimera_dsg
             kimera_dsg_visualizer
             kimera_pgmo
             kimera_semantics
             kimera_semantics_ros
             object_db
             tf2_eigen
             voxblox
             voxblox_ros
             voxblox_skeleton
)

find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(KimeraRPGO REQUIRED)

generate_dynamic_reconfigure_options(cfg/DsgBuilderConfig.cfg)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  dynamic_reconfigure
  object_db
  pluginlib
  std_msgs
  kimera_dsg
  kimera_dsg_visualizer
  kimera_pgmo
  kimera_semantics
  kimera_semantics_ros
  tf2_eigen
  voxblox
  voxblox_ros
  voxblox_skeleton
  DEPENDS
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

# Serialization plugins (mostly to get around lz4 problems with flann)
# add_library(humans_serialization_plugins src/humans_serialization_plugins.cpp)
# target_include_directories( humans_serialization_plugins PUBLIC include
# ${catkin_INCLUDE_DIRS} )

add_library(
  ${PROJECT_NAME}
  src/connectivity_utils.cpp
  src/object_finder.cpp
  src/room_finder.cpp
  src/wall_finder.cpp
  src/building_finder.cpp
  src/dsg_backend.cpp
  src/scene_graph_builder.cpp
  src/voxblox_utils.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} gtsam)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_offline_node src/kimera_dsg_builder_offline_node.cpp)
target_link_libraries(${PROJECT_NAME}_offline_node ${PROJECT_NAME})

# TODO(nathan) allow choice of sanitizer
if(DSG_USE_ASAN)
  # TODO(nathan) later cmake has better options for this
  set(CMAKE_EXE_LINKER_FLAGS
      "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=address -fno-omit-frame-pointer"
  )
  target_compile_options(
    ${PROJECT_NAME}_offline_node PUBLIC -fsanitize=address -fno-omit-frame-pointer
  )
endif()

# Add tests
if(CATKIN_ENABLE_TESTING)
  # TODO(nathan) clean up
  find_package(rostest REQUIRED)
  add_rostest_gtest(
    testDsgOfflineBuilder tests/test_dsg_offline_builder.test
    tests/utest_main.cpp tests/test_dynamic_scene_graph.cpp
    tests/test_dynamic_scene_graph_evaluator.cpp
  )
  target_link_libraries(testDsgOfflineBuilder ${PROJECT_NAME})
endif()

# TODO(nathan) handle install

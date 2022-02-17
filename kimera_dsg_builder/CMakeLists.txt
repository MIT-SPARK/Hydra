cmake_minimum_required(VERSION 3.1)
project(kimera_dsg_builder)

add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
option(DSG_USE_ASAN "enable address sanatizer" OFF)

find_package(Boost REQUIRED COMPONENTS timer)
find_package(kimera_dsg REQUIRED)
find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             dynamic_reconfigure
             pluginlib
             std_msgs
             kimera_dsg_visualizer
             kimera_pgmo
             kimera_semantics
             kimera_semantics_ros
             kimera_topology
             kimera_vio_ros
             object_db
             tf2_eigen
             voxblox
             voxblox_ros
)

find_package(teaserpp REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(KimeraRPGO REQUIRED)
find_package(yaml-cpp REQUIRED)

generate_dynamic_reconfigure_options(cfg/DsgBuilder.cfg)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  dynamic_reconfigure
  object_db
  pluginlib
  std_msgs
  kimera_dsg_visualizer
  kimera_pgmo
  kimera_semantics
  kimera_semantics_ros
  kimera_topology
  kimera_vio_ros
  tf2_eigen
  voxblox
  voxblox_ros
  DEPENDS kimera_dsg
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

add_library(
  ${PROJECT_NAME}
  src/building_finder.cpp
  src/connectivity_utils.cpp
  src/dsg_lcd_descriptors.cpp
  src/dsg_lcd_matching.cpp
  src/dsg_lcd_module.cpp
  src/dsg_lcd_registration.cpp
  src/dsg_update_functions.cpp
  src/incremental_dsg_backend.cpp
  src/incremental_dsg_frontend.cpp
  src/incremental_mesh_segmenter.cpp
  src/incremental_room_finder.cpp
  src/lcd_visualizer.cpp
  src/minimum_spanning_tree.cpp
  src/object_finder.cpp
  src/offline_dsg_builder.cpp
  src/room_finder.cpp
  src/serialization_helpers.cpp
  src/timing_utilities.cpp
  src/visualizer_plugins.cpp
  src/voxblox_utils.cpp
  src/wall_finder.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gtsam
  teaserpp::teaser_registration
  kimera_dsg::kimera_dsg
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_offline_node src/kimera_dsg_builder_offline_node.cpp)
target_link_libraries(${PROJECT_NAME}_offline_node ${PROJECT_NAME})

add_executable(${PROJECT_NAME}_batch_node src/batch_dsg_builder_node.cpp)
target_link_libraries(${PROJECT_NAME}_batch_node ${PROJECT_NAME})

add_executable(${PROJECT_NAME}_incremental_node src/incremental_dsg_builder_node.cpp)
target_link_libraries(${PROJECT_NAME}_incremental_node ${PROJECT_NAME})

add_executable(dsg_visualizer_node src/dsg_visualizer_node.cpp)
target_link_libraries(dsg_visualizer_node ${PROJECT_NAME} yaml-cpp)

add_executable(room_visualizer_node src/room_visualizer_node.cpp)
target_link_libraries(room_visualizer_node ${PROJECT_NAME} yaml-cpp)

add_executable(room_eval_node src/room_eval_node.cpp)
target_link_libraries(room_eval_node ${PROJECT_NAME} yaml-cpp)

add_executable(pose_graph_publisher_node src/pose_graph_publisher_node.cpp)
target_include_directories(pose_graph_publisher_node PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(pose_graph_publisher_node ${catkin_LIBRARIES} gtsam)

add_executable(dsg_optimizer_node src/dsg_optimizer_node.cpp)
target_link_libraries(dsg_optimizer_node ${PROJECT_NAME})

add_executable(gt_trajectory_optimizer src/gt_trajectory_optimizer.cpp)
target_include_directories(gt_trajectory_optimizer PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(gt_trajectory_optimizer ${PROJECT_NAME} yaml-cpp KimeraRPGO)

# TODO(nathan) allow choice of sanitizer
if(DSG_USE_ASAN)
  # TODO(nathan) later cmake has better options for this
  set(CMAKE_EXE_LINKER_FLAGS
      "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=address -fno-omit-frame-pointer"
  )
  target_compile_options(
    ${PROJECT_NAME} PUBLIC -fsanitize=address -fno-omit-frame-pointer
  )
  target_compile_options(
    ${PROJECT_NAME}_offline_node PUBLIC -fsanitize=address -fno-omit-frame-pointer
  )
  target_compile_options(
    ${PROJECT_NAME}_incremental_node PUBLIC -fsanitize=address -fno-omit-frame-pointer
  )
endif()

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
    tests/utest_timing_utilities.cpp
  )
  target_link_libraries(utest_${PROJECT_NAME} ${PROJECT_NAME})
endif()

# TODO(nathan) handle install

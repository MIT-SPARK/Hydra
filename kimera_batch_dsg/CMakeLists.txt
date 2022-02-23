cmake_minimum_required(VERSION 3.1)
project(kimera_batch_dsg)

add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(kimera_dsg REQUIRED)
find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             dynamic_reconfigure
             kimera_dsg_visualizer
             kimera_pgmo
             kimera_semantics
             kimera_semantics_ros
             kimera_topology
             std_msgs
             visualization_msgs
             voxblox
             voxblox_ros
)
find_package(Boost REQUIRED COMPONENTS timer)
find_package(GTSAM REQUIRED)

generate_dynamic_reconfigure_options(cfg/DsgBuilder.cfg)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  dynamic_reconfigure
  hydra_utils
  kimera_dsg_visualizer
  kimera_pgmo
  kimera_semantics
  kimera_semantics_ros
  kimera_topology
  std_msgs
  visualization_msgs
  voxblox
  voxblox_ros
  DEPENDS
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

add_library(
  ${PROJECT_NAME}
  src/building_finder.cpp
  src/connectivity_utils.cpp
  #src/object_finder.cpp
  src/room_finder.cpp
  src/voxblox_utils.cpp
  src/wall_finder.cpp
  src/offline_dsg_builder.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} gtsam)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node src/batch_dsg_builder_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

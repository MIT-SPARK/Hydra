cmake_minimum_required(VERSION 3.1)
project(hydra_dsg_builder_ros)

add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Boost REQUIRED COMPONENTS timer)
find_package(spark_dsg REQUIRED)
find_package(GTSAM REQUIRED)
find_package(
  catkin REQUIRED
  COMPONENTS hydra_topology
             hydra_dsg_builder
             kimera_semantics_ros
             roscpp
             std_msgs
             tf2_eigen
             voxblox_ros
)

catkin_package(
  CATKIN_DEPENDS
  hydra_topology
  hydra_dsg_builder
  kimera_semantics_ros
  roscpp
  std_msgs
  tf2_eigen
  voxblox_ros
  DEPENDS spark_dsg
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

add_library(
  ${PROJECT_NAME}
  src/lcd_visualizer.cpp
  src/node_utilities.cpp
  src/ros_backend.cpp
  src/ros_frontend.cpp
  src/ros_lcd_registration.cpp
  src/visualizer_plugins.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  spark_dsg::spark_dsg
  gtsam
)

add_executable(${PROJECT_NAME}_node src/incremental_dsg_builder_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

add_executable(dsg_optimizer_node src/dsg_optimizer_node.cpp)
target_link_libraries(dsg_optimizer_node ${PROJECT_NAME})

# TODO(nathan) handle install

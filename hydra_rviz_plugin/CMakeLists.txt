cmake_minimum_required(VERSION 3.1)
project(hydra_rviz_plugin)

add_compile_options(-Wall -Wno-register)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS rviz hydra_msgs)
find_package(spark_dsg REQUIRED)
find_package(
  Qt5
  ${rviz_QT_VERSION}
  EXACT
  REQUIRED
  Core
  Widgets
)

add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
  CATKIN_DEPENDS rviz hydra_msgs
  DEPENDS spark_dsg
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

# n.b. headers are required for moc to work correctly
add_library(
  ${PROJECT_NAME} include/hydra_rviz_plugin/scene_graph_display.h
                  src/scene_graph_display.cpp src/scene_graph_visual.cpp
)
set_target_properties(${PROJECT_NAME} PROPERTIES AUTOMOC TRUE)
target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  spark_dsg::spark_dsg
  Qt5::Widgets
)

# TODO(nathan) handle install

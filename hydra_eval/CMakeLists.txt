cmake_minimum_required(VERSION 3.1)
project(hydra_eval)

add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(kimera_dsg REQUIRED)
find_package(catkin REQUIRED COMPONENTS voxblox kimera_dsg_visualizer visualization_msgs)
find_package(yaml-cpp REQUIRED)

catkin_package(
  CATKIN_DEPENDS voxblox kimera_dsg_visualizer visualization_msgs
  DEPENDS
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

add_library(${PROJECT_NAME}
    src/room_utils.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp kimera_dsg::kimera_dsg)

add_executable(room_eval_node src/room_eval_node.cpp)
target_link_libraries(room_eval_node ${PROJECT_NAME})

# TODO(nathan) load in ply instead, and avoid voxblox utils
add_executable(room_visualizer_node src/room_visualizer_node.cpp)
target_link_libraries(room_visualizer_node ${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME} room_eval_node room_visualizer_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

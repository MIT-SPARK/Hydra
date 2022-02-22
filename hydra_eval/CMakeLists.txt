cmake_minimum_required(VERSION 3.1)
project(hydra_eval)

add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(kimera_dsg REQUIRED)
find_package(catkin REQUIRED COMPONENTS voxblox)
find_package(yaml-cpp REQUIRED)
find_package(GTSAM REQUIRED)

catkin_package(
  CATKIN_DEPENDS voxblox
  DEPENDS
  INCLUDE_DIRS
  LIBRARIES
)

add_executable(room_eval_node src/room_eval_node.cpp)
target_include_directories(room_eval_node PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(room_eval_node ${catkin_LIBRARIES} yaml-cpp kimera_dsg::kimera_dsg)

# TODO(nathan) load in ply instead, and avoid voxblox utils
add_executable(room_visualizer_node src/room_visualizer_node.cpp)
target_include_directories(room_visualizer_node PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(room_visualizer_node ${catkin_LIBRARIES} yaml-cpp gtsam kimera_dsg::kimera_dsg)

install(
  TARGETS room_eval_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

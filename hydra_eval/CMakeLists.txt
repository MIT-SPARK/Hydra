cmake_minimum_required(VERSION 3.1)
project(hydra_eval)

add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(kimera_dsg REQUIRED)
find_package(KimeraRPGO REQUIRED)
find_package(
  catkin REQUIRED
  COMPONENTS voxblox
             kimera_dsg_visualizer
             kimera_pgmo
             kimera_topology
             visualization_msgs
)
find_package(Boost REQUIRED COMPONENTS timer)
find_package(GTSAM REQUIRED)
find_package(yaml-cpp REQUIRED)

configure_file(cmake/nanoflann.CMakeLists.txt.in nanoflann-download/CMakeLists.txt)
execute_process(
  COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" .
  WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/nanoflann-download"
  OUTPUT_QUIET
)
execute_process(
  COMMAND "${CMAKE_COMMAND}" --build .
  WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/nanoflann-download"
  OUTPUT_QUIET
)

set(NANOFLANN_BUILD_EXAMPLES
    OFF
    CACHE INTERNAL ""
)
set(NANOFLANN_BUILD_TESTS
    OFF
    CACHE INTERNAL ""
)
add_subdirectory(${CMAKE_BINARY_DIR}/nanoflann-src ${CMAKE_BINARY_DIR}/nanoflann-build)

# TODO(nathan) fetch content when possible on 20.04
configure_file(cmake/json.CMakeLists.txt.in json-download/CMakeLists.txt)
execute_process(
  COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" .
  WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/json-download"
  OUTPUT_QUIET
)
execute_process(
  COMMAND "${CMAKE_COMMAND}" --build .
  WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/json-download"
  OUTPUT_QUIET
)

set(JSON_BuildTests
    OFF
    CACHE INTERNAL ""
)
set(JSON_Install
    OFF
    CACHE INTERNAL ""
)
add_subdirectory(${CMAKE_BINARY_DIR}/json-src ${CMAKE_BINARY_DIR}/json-build)

catkin_package(
  CATKIN_DEPENDS
  voxblox
  kimera_dsg_visualizer
  kimera_topology
  kimera_pgmo
  visualization_msgs
  DEPENDS
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

add_library(${PROJECT_NAME} src/room_utils.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  yaml-cpp
  kimera_dsg::kimera_dsg
)

add_executable(room_eval_node src/room_eval_node.cpp)
target_link_libraries(room_eval_node ${PROJECT_NAME} nlohmann_json::nlohmann_json)

add_executable(places_eval_node src/places_eval_node.cpp)
target_link_libraries(
  places_eval_node PUBLIC ${PROJECT_NAME} nanoflann::nanoflann
                          nlohmann_json::nlohmann_json
)

add_executable(room_visualizer_node src/room_visualizer_node.cpp)
target_link_libraries(room_visualizer_node ${PROJECT_NAME} gtsam)

add_executable(places_visualizer_node src/places_visualizer_node.cpp)
target_link_libraries(places_visualizer_node PUBLIC ${PROJECT_NAME})

add_executable(gt_trajectory_optimizer src/gt_trajectory_optimizer.cpp)
target_include_directories(gt_trajectory_optimizer PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(
  gt_trajectory_optimizer
  ${catkin_LIBRARIES}
  yaml-cpp
  KimeraRPGO
)

install(
  TARGETS ${PROJECT_NAME} room_eval_node room_visualizer_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

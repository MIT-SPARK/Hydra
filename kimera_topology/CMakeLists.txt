cmake_minimum_required(VERSION 3.1)
project(kimera_topology)

find_package(catkin REQUIRED COMPONENTS roscpp voxblox voxblox_ros)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  voxblox
  voxblox_ros
  DEPENDS
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

add_compile_options(-Wall -Wextra)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(
  ${PROJECT_NAME}
  src/gvd_integrator.cpp
  src/gvd_utilities.cpp
  src/gvd_voxel.cpp
  src/voxel_aware_marching_cubes.cpp
  src/voxel_aware_mesh_integrator.cpp
)
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(
    utest_${PROJECT_NAME}
    tests/utest_main.cpp
    tests/src/test_fixtures.cpp
    tests/src/test_visualizations.cpp
    tests/src/test_helpers.cpp
    tests/utest_esdf.cpp
    tests/utest_esdf_helpers.cpp
    tests/utest_gvd_utilities.cpp
    tests/utest_gvd_visualization.cpp
    tests/utest_marching_cubes.cpp
    tests/utest_incremental_gvd.cpp
  )
  target_include_directories(
    utest_${PROJECT_NAME} PUBLIC tests/include ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(utest_${PROJECT_NAME} ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

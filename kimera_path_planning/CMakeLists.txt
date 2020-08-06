cmake_minimum_required(VERSION 2.8.3)
project(kimera_path_planning)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(GTSAM REQUIRED)

cs_add_library(${PROJECT_NAME}
  # .h just for IDEs
  include/kimera_path_planning/common.h
  include/kimera_path_planning/kimera_path_planning.h
  src/${PROJECT_NAME}.cpp
)
# GTSAM should not be needed, but we have an undefined reference otw..
# undefined reference to symbol '_ZN5gtsam23_multirobotKeyFormatterB5cxx11Em'
target_link_libraries(${PROJECT_NAME} gtsam gtsam_unstable)
target_include_directories(${PROJECT_NAME} PUBLIC ${GTSAM_INCLUDE_DIR})

cs_add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

cs_install()

cs_export()

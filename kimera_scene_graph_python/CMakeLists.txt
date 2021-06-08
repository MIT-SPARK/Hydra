cmake_minimum_required(VERSION 3.1)
project(kimera_scene_graph_python)

add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
    pybind11_catkin
    kimera_scene_graph_definitions
    kimera_semantics)

# allows catkin to install python packages in the right place in the catkin
# workspace
catkin_python_setup()

catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        pybind11_catkin
        kimera_scene_graph_definitions
        kimera_semantics
    DEPENDS
    INCLUDE_DIRS)

include_directories(${catkin_INCLUDE_DIRS})

pybind_add_module(kimera_scene_graph_pybind kimera_scene_graph_python/kimera_scene_graph_python_bindings.cpp)
target_link_libraries(kimera_scene_graph_pybind PUBLIC ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(tests)
endif()

install(
  TARGETS kimera_scene_graph_pybind
  LIBRARY
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})

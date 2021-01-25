cmake_minimum_required(VERSION 2.8.3)
project(kimera_scene_graph_python)

find_package(catkin REQUIRED COMPONENTS
        pybind11_catkin
        kimera_scene_graph
        kimera_semantics)

#allows catkin to install python packages in the right place in the catkin workspace
catkin_python_setup()

catkin_package(
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS pybind11_catkin kimera_scene_graph kimera_semantics
)

include_directories(${catkin_INCLUDE_DIRS})

pybind_add_module(kimera_scene_graph_pybind kimera_scene_graph_python/kimera_scene_graph_python_bindings.cpp)

target_link_libraries(kimera_scene_graph_pybind ${catkin_LIBRARIES} ${PCL_LIBRARIES})

if (CATKIN_ENABLE_TESTING)
    catkin_add_nosetests(tests)
endif()

install(TARGETS kimera_scene_graph_pybind
        LIBRARY DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
        )

message(STATUS ${CATKIN_DEVEL_PREFIX})
#/home/lisa/catkin_ws/devel
message(STATUS ${CATKIN_DEVEL_PREFIX})
#lib/python2.7/dist-packages/kimera_scene_graph_python
message(STATUS ${CATKIN_PACKAGE_PYTHON_DESTINATION})
#lib
message(STATUS ${CATKIN_PACKAGE_LIB_DESTINATION})
#/home/lisa/catkin_ws/build/kimera_scene_graph_python
message(STATUS ${CMAKE_CURRENT_BINARY_DIR})
#lib/kimera_scene_graph_python
message(STATUS ${CATKIN_PACKAGE_BIN_DESTINATION})

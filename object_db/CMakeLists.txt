cmake_minimum_required(VERSION 3.1)
project(object_db)

add_compile_options(-Wall -Wextra -Wno-unused-parameter)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(teaserpp REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    message_generation
    actionlib_msgs
    sensor_msgs
    std_msgs
    pcl_ros)

add_action_files(DIRECTORY action
    FILES ObjectRegistration.action)

generate_messages(DEPENDENCIES std_msgs sensor_msgs actionlib_msgs)

catkin_package(
    CATKIN_DEPENDS
        message_runtime
        actionlib_msgs
        sensor_msgs
        std_msgs
        pcl_ros
    DEPENDS
    INCLUDE_DIRS include
    LIBRARIES object_registration_server matcher)

# Object Registration Server
add_library(object_registration_server
  src/object_registration_server.cpp
  src/ply_io.cpp)
target_include_directories(object_registration_server PUBLIC
    include
    ${catkin_INCLUDE_DIRS})
target_link_libraries(object_registration_server
    matcher
    teaserpp::teaser_registration
    ${catkin_LIBRARIES})
add_dependencies(object_registration_server ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

# PCL Keypoints Module
add_library(matcher src/matcher.cpp)
target_include_directories(matcher PUBLIC
    include
    ${catkin_INCLUDE_DIRS})
target_link_libraries(matcher ${catkin_LIBRARIES})

# Server node
add_executable(server_node src/server_node.cpp)
target_link_libraries(server_node
    object_registration_server
    teaserpp::teaser_registration)

# Sample client code
add_executable(sample_client src/sample_client.cpp)
target_link_libraries(sample_client
    object_registration_server
    teaserpp::teaser_registration)

if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(test_matcher tests/matcher_test.cpp)
    target_link_libraries(test_matcher object_registration_server matcher)
    target_include_directories(test_matcher PRIVATE include)
endif()

# TODO(nathan) install

cmake_minimum_required(VERSION 2.8.3)
project(object_db)

set(CMAKE_CXX_STANDARD 14)


# catkin_simple setup
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(Eigen3 3.2 QUIET REQUIRED NO_MODULE)

# Object Registration Server
cs_add_library(object_registration_server
  src/object_registration_server.cpp
  src/ply_io.cpp
  include/object_db/tinyply.h)
target_link_libraries(object_registration_server
    matcher teaserpp::teaser_registration ${PCL_LIBRARIES})

# PCL Keypoints Module
cs_add_library(matcher src/matcher.cpp)
target_link_libraries(matcher ${PCL_LIBRARIES})

# Server node
cs_add_executable(server_node src/server_node.cpp)
target_link_libraries(server_node
    object_registration_server teaserpp::teaser_registration)

# Sample client code
cs_add_executable(sample_client src/sample_client.cpp)
target_link_libraries(sample_client object_registration_server teaserpp::teaser_registration)

# Add tests
add_subdirectory(test)

# Install & Export
cs_install()
cs_export()

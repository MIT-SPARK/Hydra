cmake_minimum_required(VERSION 3.22)
project(hydra_plusplus)

find_package(ament_cmake REQUIRED)

install(DIRECTORY app/ DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

ament_package()

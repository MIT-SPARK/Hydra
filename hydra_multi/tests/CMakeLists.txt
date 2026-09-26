find_package(GTest REQUIRED)
include(GoogleTest)
enable_testing()

find_package(gflags REQUIRED)

configure_file(hydra_multi_test_config.h.in hydra_multi_test_config.h)

add_executable(
  test_${PROJECT_NAME}
  main.cpp
  test_loop_closure_initial_align.cpp
  test_pose_graph_operator.cpp
  test_mesh_operator.cpp
  test_dsg_operator.cpp
)
target_include_directories(test_${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(
  test_${PROJECT_NAME}
  ${PROJECT_NAME}
  ${gflags_LIBRARIES}
  GTest::gtest_main
)
gtest_add_tests(TARGET test_${PROJECT_NAME})

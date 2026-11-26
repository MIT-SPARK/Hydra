find_package(GTest REQUIRED)

include(GoogleTest)
enable_testing()

add_executable(
  test_${PROJECT_NAME}
  main.cpp
  test_loop_closure_initial_align.cpp
  test_pose_graph_operator.cpp
  test_mesh_operator.cpp
  test_dsg_operator.cpp
  test_multi_dsg_info.cpp
)
target_link_libraries(test_${PROJECT_NAME} PUBLIC ${PROJECT_NAME} GTest::gtest_main)
gtest_add_tests(TARGET test_${PROJECT_NAME})

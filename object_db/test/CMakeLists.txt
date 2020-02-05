cmake_minimum_required(VERSION 2.8.3)
project(object_db_test)

enable_testing()
include(CTest)

catkin_add_gtest(test_matcher matcher_test.cpp)
target_link_libraries(test_matcher object_registration_server matcher)

add_test(all_tests test_matcher)

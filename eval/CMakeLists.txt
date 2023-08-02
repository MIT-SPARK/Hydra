find_package(KimeraRPGO REQUIRED)

add_library(
  ${PROJECT_NAME}_eval
  src/graph_utilities.cpp
  src/place_evaluator.cpp
  src/place_metrics.cpp
  src/room_evaluator.cpp
  src/room_io.cpp
  src/room_metrics.cpp
)
target_include_directories(${PROJECT_NAME}_eval PUBLIC include)
target_link_libraries(
  ${PROJECT_NAME}_eval
  PUBLIC ${PROJECT_NAME}
  PRIVATE nanoflann::nanoflann
)

add_executable(compute_filtrations tools/compute_filtrations.cpp)
target_link_libraries(compute_filtrations ${PROJECT_NAME}_eval)

add_executable(evaluate_places tools/evaluate_places.cpp)
target_link_libraries(evaluate_places ${PROJECT_NAME}_eval)

add_executable(evaluate_rooms tools/evaluate_rooms.cpp)
target_link_libraries(evaluate_rooms ${PROJECT_NAME}_eval)

add_executable(gt_trajectory_optimizer tools/gt_trajectory_optimizer.cpp)
target_link_libraries(gt_trajectory_optimizer ${PROJECT_NAME}_eval KimeraRPGO)

add_executable(merge_graphs tools/merge_graphs.cpp)
target_link_libraries(merge_graphs ${PROJECT_NAME}_eval)

install(
  TARGETS ${PROJECT_NAME}_eval
          compute_filtrations
          evaluate_places
          evaluate_rooms
          gt_trajectory_optimizer
          merge_graphs
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

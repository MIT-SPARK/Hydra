find_package(KimeraRPGO REQUIRED)
find_package(gflags REQUIRED)

add_library(
  ${PROJECT_NAME}_eval
  src/graph_utilities.cpp src/place_evaluator.cpp src/place_metrics.cpp
  src/room_evaluator.cpp src/room_io.cpp src/room_metrics.cpp
)
target_include_directories(${PROJECT_NAME}_eval PUBLIC include)
target_link_libraries(
  ${PROJECT_NAME}_eval PUBLIC ${PROJECT_NAME} ${gflags_LIBRARIES}
  PRIVATE nanoflann::nanoflann
)

add_executable(compress_graph tools/compress_graph.cpp)
target_link_libraries(compress_graph ${PROJECT_NAME}_eval nanoflann::nanoflann)

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

if(${HYDRA_ENABLE_ROS_INSTALL_LAYOUT})
  install(
    TARGETS ${PROJECT_NAME}_eval compress_graph compute_filtrations evaluate_places
            evaluate_rooms gt_trajectory_optimizer merge_graphs
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
  )
endif()

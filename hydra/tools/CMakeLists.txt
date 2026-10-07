find_package(CLI11 CONFIG REQUIRED)

set(HYDRA_TOOL_INSTALL_DIR ${CMAKE_INSTALL_BINDIR})
find_package(ament_cmake_core QUIET)
if (${ament_cmake_core_FOUND})
  set(HYDRA_TOOL_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR}/hydra)
endif()

add_executable(compute_filtrations compute_filtrations.cpp)
target_link_libraries(compute_filtrations PRIVATE hydra::hydra CLI11::CLI11)

add_executable(optimize_graph optimize_graph.cpp)
target_link_libraries(optimize_graph PRIVATE hydra::hydra CLI11::CLI11)

add_executable(reconcile_graph reconcile_graph.cpp)
target_link_libraries(reconcile_graph PRIVATE hydra::hydra CLI11::CLI11)

install(
  TARGETS compute_filtrations optimize_graph reconcile_graph
  RUNTIME DESTINATION ${HYDRA_TOOL_INSTALL_DIR}
)

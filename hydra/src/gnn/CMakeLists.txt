target_sources(
  ${PROJECT_NAME}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/gnn_interface.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/ort_utilities.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/tensor.cpp
)

# TODO(nathan) fix find_package(PkgConfig REQUIRED) pkg_check_modules(
# glog REQUIRED IMPORTED_TARGET libglog
# )

# add_library(${PROJECT_NAME}_gnn STATIC gnn_interface.cpp ort_utilities.cpp tensor.cpp)
# target_include_directories(
# ${PROJECT_NAME}_gnn PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
# $<INSTALL_INTERFACE:include>
# ) TODO(nathan) public linkage for ort not ideal, but should be fine for now...
# target_link_libraries(
# ${PROJECT_NAME}_gnn PUBLIC Eigen3::Eigen ort::ort PRIVATE PkgConfig::glog
# ) set_property(TARGET ${PROJECT_NAME}_gnn PROPERTY POSITION_INDEPENDENT_CODE ON)

# TODO(nathan) install
add_executable(gnn_model_reader gnn_model_reader.cpp)
# target_link_libraries(gnn_model_reader ${PROJECT_NAME}_gnn PkgConfig::libglog)
target_link_libraries(gnn_model_reader ${PROJECT_NAME})

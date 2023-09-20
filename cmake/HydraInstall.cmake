install(
  TARGETS ${PROJECT_NAME}
  EXPORT hydra-targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(
  EXPORT hydra-targets
  FILE hydraTargets.cmake
  NAMESPACE hydra::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/hydra
)

include(CMakePackageConfigHelpers)
configure_package_config_file(
  ${CMAKE_CURRENT_LIST_DIR}/hydraConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/hydraConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/hydra
)
write_basic_package_version_file(
  hydraConfigVersion.cmake
  VERSION ${PACKAGE_VERSION}
  COMPATIBILITY AnyNewerVersion
)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/hydraConfig.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/hydraConfigVersion.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/hydra
)

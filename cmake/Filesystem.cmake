function(has_std_filesystem result)
  try_compile(
    HAS_CPP_FS ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_SOURCE_DIR}/cmake/test_fs.cpp
    LINK_LIBRARIES
      stdc++fs
      CXX_STANDARD
      17
      CXX_STANDARD_REQUIRED
      TRUE
      CXX_EXTENSIONS
      FALSE
      OUTPUT_VARIABLE HAS_CPP_FS_OUTPUT
  )
  set(${result} ${HAS_CPP_FS} PARENT_SCOPE)
endfunction()

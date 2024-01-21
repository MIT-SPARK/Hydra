include(FetchContent)

# Populates content for a FetchContent dependency with applied settings and no
# installation
function(populate_content pkgname)
  FetchContent_GetProperties(${pkgname})
  if(${pkgname}_POPULATED)
    return()
  endif()

  foreach(loop_var IN LISTS ARGN)
    string(REPLACE "=" ";" parsed_var ${loop_var})
    list(LENGTH parsed_var parsed_len)
    if(NOT parsed_len EQUAL 2)
      message(
        WARNING "Invalid argument '${loop_var}' for populate_content('${pkgname}')"
      )
      continue()
    endif()

    list(GET parsed_var 0 varname)
    list(GET parsed_var 1 varval)
    if(DEFINED ${varname})
      # only cache set variables
      set(${varname}_prev ${${varname}})
      list(APPEND parsed_varnames ${varname})
    else()
      list(APPEND unset_varnames ${varname})
    endif()

    set(${varname} ${varval} CACHE INTERNAL "")
  endforeach()

  # note that FetchContent_MakeAvailable automatically installs any added fetch-content
  # project, see:
  # https://stackoverflow.com/questions/65527126/disable-install-for-fetchcontent
  FetchContent_Populate(${pkgname})
  add_subdirectory(${${pkgname}_SOURCE_DIR} ${${pkgname}_BINARY_DIR} EXCLUDE_FROM_ALL)

  foreach(varname IN LISTS parsed_varnames)
    set(${varname} ${${varname}_prev} CACHE INTERNAL "")
  endforeach()

  foreach(varname IN LISTS unset_varnames)
    unset(${varname} CACHE)
  endforeach()
endfunction()

FetchContent_Declare(
  nanoflann GIT_REPOSITORY https://github.com/jlblancoc/nanoflann.git GIT_TAG v1.5.0
)

FetchContent_Declare(
  dbow2
  PATCH_COMMAND git reset --hard && git apply ${CMAKE_SOURCE_DIR}/.patches/dbow2.patch
  GIT_REPOSITORY https://github.com/dorian3d/DBoW2.git
  INSTALL_COMMAND ""
)

populate_content(nanoflann NANOFLANN_BUILD_TESTS=OFF NANOFLANN_BUILD_EXAMPLES=OFF)
populate_content(
  dbow2 BUILD_SHARED_LIBS=OFF CMAKE_POSITION_INDEPENDENT_CODE=ON BUILD_DEMO=OFF
)
if(NOT TARGET DBoW2::DBoW2)
  add_library(DBoW2::DBoW2 ALIAS DBoW2)
endif()

if(HYDRA_ENABLE_GNN)
  # TODO(nathan) fetch content
  configure_file(cmake/ort.CMakeLists.txt.in ort-download/CMakeLists.txt)
  execute_process(
    COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" .
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/ort-download" # OUTPUT_QUIET
  )
  execute_process(
    COMMAND "${CMAKE_COMMAND}" --build .
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/ort-download" # OUTPUT_QUIET
  )

  add_library(ort::ort IMPORTED INTERFACE)
  set_property(
    TARGET ort::ort PROPERTY INTERFACE_INCLUDE_DIRECTORIES
                             "${CMAKE_CURRENT_BINARY_DIR}/ort-src/include"
  )
  set_property(
    TARGET ort::ort
    PROPERTY INTERFACE_LINK_LIBRARIES
             "${CMAKE_CURRENT_BINARY_DIR}/ort-src/lib/libonnxruntime.so"
  )
endif()

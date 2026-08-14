# Exports 0 or 1 for enabled features for better use with c++
#
# Sets ${flag_name}_CXX_VALUE to 0 or 1 depending on the option value
#
# Args: flag_name: Name of the flag to export to c++
macro(EXPORT_CXX_VALUE flag_name)
  if(${flag_name})
    set(${flag_name}_CXX_VALUE 1)
  else()
    set(${flag_name}_CXX_VALUE 0)
  endif()
endmacro()

EXPORT_CXX_VALUE(HYDRA_ENABLE_GNN)
configure_file(cmake/hydra_build_config.h.in ${CMAKE_CURRENT_BINARY_DIR}/hydra_build_config.h)

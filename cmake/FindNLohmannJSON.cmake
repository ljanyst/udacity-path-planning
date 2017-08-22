include(FindPackageHandleStandardArgs)

find_path(
  NLOHMANNJSON_INCLUDE_DIR
  NAMES json.hpp
  HINTS
  ${NLOHMANNJSON_ROOT_DIR}
  PATH_SUFFIXES
  include)

find_package_handle_standard_args(
  NLohmannJSON
  DEFAULT_MSG
  NLOHMANNJSON_INCLUDE_DIR)

mark_as_advanced(NLOHMANNJSON_INCLUDE_DIR)

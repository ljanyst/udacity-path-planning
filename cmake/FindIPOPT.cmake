# Try to find IPOPT
# Once done, this will define
#
# IPOPT_FOUND - system has cppunit
# IPOPT_INCLUDE_DIRS - the cppunit include directories
# IPOPT_LIBRARIES - cppunit libraries directories

find_path(IPOPT_INCLUDE_DIRS coin/IpTNLP.hpp)
find_library(IPOPT_LIBRARIES ipopt)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(IPOPT DEFAULT_MSG IPOPT_INCLUDE_DIRS IPOPT_LIBRARIES)

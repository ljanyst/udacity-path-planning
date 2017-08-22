project(path-planning)
cmake_minimum_required(VERSION 3.5)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

find_package(uWebSockets REQUIRED)
find_package(NLohmannJSON REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(IPOPT REQUIRED)
find_package(CppAD REQUIRED)

if(APPLE)
  find_package(OpenSSL REQUIRED)
  find_package(LibUV REQUIRED)
endif()

include_directories(
  ${UWEBSOCKETS_INCLUDE_DIR}
  ${NLOHMANNJSON_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${CPPAD_INCLUDE_DIR}
  ${OPENSSL_INCLUDE_DIR})

add_definitions(-std=c++11)

add_executable(path-planner
  src/main.cc
  src/map.cc
  src/utils.cc
  src/poly.cc
  src/auto_pilot.cc
  src/mpc.cc
  src/behavior_planner.cc
  src/path_predictor.cc)

target_link_libraries(
  path-planner
  ssl z
  ${UWEBSOCKETS_LIBRARIES}
  ${LIBUV_LIBRARY}
  ${IPOPT_LIBRARIES})

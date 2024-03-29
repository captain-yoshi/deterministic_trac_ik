cmake_minimum_required(VERSION 2.8.3)
project(deterministic_trac_ik_lib)

if(CMAKE_COMPILER_IS_GNUCXX OR ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang"))
    set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
endif()

find_package(catkin REQUIRED
  COMPONENTS
    cmake_modules
    kdl_parser
    roscpp
    urdf
)

find_package(Boost REQUIRED COMPONENTS date_time)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pkg_nlopt REQUIRED nlopt)

find_package(Eigen3 REQUIRED)
#pkg_check_modules(Eigen REQUIRED eigen3)
# TODO: resolve libraries to absolute paths


catkin_package(
  CATKIN_DEPENDS
    kdl_parser
    roscpp
    urdf
  DEPENDS
    Boost
    # purposefully not including nlopt here, see earlier note
  INCLUDE_DIRS
    include ${EIGEN3_INCLUDE_DIR} ${pkg_nlopt_INCLUDE_DIRS}
  LIBRARIES
    deterministic_trac_ik ${pkg_nlopt_LIBRARIES}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${pkg_nlopt_INCLUDE_DIRS}
)

add_library(deterministic_trac_ik
  src/kdl_tl.cpp
  src/nlopt_ik.cpp
  src/deterministic_trac_ik.cpp
  src/utils.cpp)
target_link_libraries(deterministic_trac_ik
  ${catkin_LIBRARIES}
  ${pkg_nlopt_LIBRARIES}
  ${Boost_LIBRARIES})

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

install(TARGETS deterministic_trac_ik
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

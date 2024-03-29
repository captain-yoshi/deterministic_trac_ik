cmake_minimum_required(VERSION 2.8.3)
project(deterministic_trac_ik_kinematics_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang"))
    set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
endif()

find_package(catkin REQUIRED
  COMPONENTS
    moveit_core
    pluginlib
    roscpp
    tf_conversions
    deterministic_trac_ik_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    pluginlib
    roscpp
    tf_conversions
    deterministic_trac_ik_lib
)

set(Deterministic_TRAC_IK_LIBRARY_NAME deterministic_trac_ik_kinematics_plugin)

add_library(${Deterministic_TRAC_IK_LIBRARY_NAME} src/deterministic_trac_ik_kinematics_plugin.cpp)
target_link_libraries(${Deterministic_TRAC_IK_LIBRARY_NAME} ${catkin_LIBRARIES})

install(TARGETS ${Deterministic_TRAC_IK_LIBRARY_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
  FILES
    deterministic_trac_ik_kinematics_description.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

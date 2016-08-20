cmake_minimum_required(VERSION 3.5)

project(apriltags_ros)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(apriltags)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS})

macro(targets)
  if(NOT target_suffix STREQUAL "")
    get_rclcpp_information("${rmw_implementation}" "rclcpp${target_suffix}")
  endif()

  add_executable(test_sub2${target_suffix}
    src/test_sub2.cpp)
  # TODO(dhood): Allow dependencies to be listed in arbitrary order without affecting the selected
  # typesupport. See https://github.com/ros2/ros2/issues/253
  ament_target_dependencies(test_sub2${target_suffix}
    "rclcpp${target_suffix}"
    "sensor_msgs"
    "std_msgs"
    "geometry_msgs")

  install(TARGETS test_sub2${target_suffix}
    DESTINATION bin)

  add_executable(apriltag_detector_node${target_suffix}
    src/apriltag_detector.cpp
    src/apriltag_detector_node.cpp)
  # TODO(dhood): Allow dependencies to be listed in arbitrary order without affecting the selected
  # typesupport. See https://github.com/ros2/ros2/issues/253
  ament_target_dependencies(apriltag_detector_node${target_suffix}
    "rclcpp${target_suffix}"
    "sensor_msgs"
    "apriltags"
    "Eigen3"
    "std_msgs"
    "geometry_msgs")

  install(TARGETS apriltag_detector_node${target_suffix}
    DESTINATION bin)

##  ament_target_dependencies(apriltag_detector_node${target_suffix}
##    "rclcpp${target_suffix}"
##    "sensor_msgs"
##    "geometry_msgs"
##    "std_msgs"
##    "Eigen3"
##    "OpenCV"
##    "apriltags"
##    "example_interfaces")
endmacro()

call_for_each_rmw_implementation(targets GENERATE_DEFAULT)

ament_package()


#cmake_minimum_required(VERSION 3.5)
#
#project(apriltags_ros)
#
#if(NOT WIN32)
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
#endif()
#
##find_package(ament_cmake REQUIRED)
###find_package(Eigen3 REQUIRED)
###find_package(apriltags REQUIRED)
##find_package(rclcpp REQUIRED)
##find_package(example_interfaces REQUIRED)
##find_package(rmw REQUIRED)
##find_package(sensor_msgs REQUIRED)
##find_package(geometry_msgs REQUIRED)
###find_package(OpenCV REQUIRED)
##find_package(rmw_implementation_cmake REQUIRED)
#
#find_package(ament_cmake REQUIRED)
#find_package(example_interfaces REQUIRED)
#find_package(rclcpp REQUIRED)
#find_package(rmw REQUIRED)
#find_package(rmw_implementation_cmake REQUIRED)
#find_package(sensor_msgs REQUIRED)
#find_package(std_msgs REQUIRED)
#find_package(geometry_msgs REQUIRED)
#
#
#include_directories(
#  include
#  ${Eigen_INCLUDE_DIRS}
#  ${apriltags_INCLUDE_DIRS}
#  ${sensor_msgs_INCLUDE_DIRS}
#  ${geometry_msgs_INCLUDE_DIRS}
#  ${std_msgs_INCLUDE_DIRS}
#)
#
#macro(targets)
#  if(NOT target_suffix STREQUAL "")
#    get_rclcpp_information("${rmw_implementation}" "rclcpp${target_suffix}")
#  endif()
#
##  add_executable(apriltag_detector_node${target_suffix}
##    src/apriltag_detector.cpp
##    src/apriltag_detector_node.cpp)
##  ament_target_dependencies(apriltag_detector_node${target_suffix}
##    "rclcpp${target_suffix}"
##    "sensor_msgs"
##    "geometry_msgs"
##    "std_msgs"
##    "Eigen3"
##    "OpenCV"
##    "apriltags"
##    "example_interfaces")
##
##  target_link_libraries(apriltag_detector_node${target_suffix} ${OpenCV_LIBRARIES})
#
#  add_executable(test_sub2${target_suffix}
#    src/test_sub2.cpp)
#  ament_target_dependencies(test_sub2${target_suffix}
#    "rclcpp${target_suffix}"
#    "sensor_msgs")
#
#  install(TARGETS test_sub2${target_suffix}
#          DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
#endmacro()
#
#call_for_each_rmw_implementation(targets GENERATE_DEFAULT)
#
#ament_package()
#

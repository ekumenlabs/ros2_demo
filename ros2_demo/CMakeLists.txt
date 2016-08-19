cmake_minimum_required(VERSION 3.5)

project(ros2_demo)

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

function(custom_executable target)
  add_executable(${target}${target_suffix}
    src/${target}.cpp)
  # TODO(dhood): Allow dependencies to be listed in arbitrary order without affecting the selected
  # typesupport. See https://github.com/ros2/ros2/issues/253
  ament_target_dependencies(${target}${target_suffix}
    "rclcpp${target_suffix}"
    "std_msgs"
    "geometry_msgs"
    "sensor_msgs"
    "example_interfaces")
  install(TARGETS ${target}${target_suffix}
    DESTINATION bin)
endfunction()

macro(targets)
  if(NOT target_suffix STREQUAL "")
    get_rclcpp_information("${rmw_implementation}" "rclcpp${target_suffix}")
  endif()

  custom_executable(driver)
  custom_executable(please_move)
  custom_executable(test_sub)
endmacro()

call_for_each_rmw_implementation(targets GENERATE_DEFAULT)

ament_package()

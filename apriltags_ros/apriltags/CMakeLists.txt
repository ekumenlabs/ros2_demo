cmake_minimum_required(VERSION 2.8.3)
project(apriltags)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include include/AprilTags)
include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(apriltags SHARED
  src/Edge.cc
  src/FloatImage.cc
  src/GLine2D.cc
  src/GLineSegment2D.cc
  src/Gaussian.cc
  src/GrayModel.cc
  src/Homography33.cc
  src/MathUtil.cc
  src/Quad.cc
  src/Segment.cc
  src/TagDetection.cc
  src/TagDetector.cc
  src/TagFamily.cc
  src/UnionFindSimple.cc
)
target_link_libraries(apriltags
  ${OpenCV_LIBRARIES}
)

message(OPENCV_LIBRARIES: ${OpenCV_LIBRARIES})

ament_export_include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})
ament_export_dependencies(Eigen3)
ament_export_dependencies(OpenCV)
ament_export_libraries(apriltags)
ament_package()

install(TARGETS apriltags DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_PREFIX}/include)


#include <rclcpp/rclcpp.hpp>

#include <apriltags_ros/apriltag_detector.h>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto n = rclcpp::node::Node::make_shared("apriltag_detector");
  apriltags_ros::AprilTagDetector detector(n);
  rclcpp::spin(n);
  return 0;
}


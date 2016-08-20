#ifndef APRILTAG_DETECTOR_H
#define APRILTAG_DETECTOR_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <AprilTags/TagDetector.h>

namespace apriltags_ros{


class AprilTagDescription{
 public:
  AprilTagDescription(int id, double size, std::string &frame_name):id_(id), size_(size), frame_name_(frame_name){}
  double size(){return size_;}
  int id(){return id_;} 
  std::string& frame_name(){return frame_name_;} 
 private:
  int id_;
  double size_;
  std::string frame_name_;
};


class AprilTagDetector{
 public:
  AprilTagDetector(rclcpp::node::Node::SharedPtr n);
  ~AprilTagDetector();
 private:
  void imageCb(const sensor_msgs::msg::Image::SharedPtr msg);
  std::map<int, AprilTagDescription> parse_tag_descriptions();

 private:
  rclcpp::node::Node::SharedPtr n_;

  std::map<int, AprilTagDescription> descriptions_;

  rclcpp::subscription::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::publisher::Publisher<sensor_msgs::msg::Image>::SharedPtr detections_image_pub_;
  rclcpp::publisher::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr detection_pose_pub_;

  std::shared_ptr<AprilTags::TagDetector> tag_detector_;
};



}


#endif

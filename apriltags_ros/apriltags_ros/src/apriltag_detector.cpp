#include <apriltags_ros/apriltag_detector.h>
#include <boost/foreach.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <AprilTags/Tag16h5.h>
#include <AprilTags/Tag25h7.h>
#include <AprilTags/Tag25h9.h>
#include <AprilTags/Tag36h9.h>
#include <AprilTags/Tag36h11.h>
#include <iostream>
#include <stdio.h>

namespace apriltags_ros{

AprilTagDetector::AprilTagDetector(rclcpp::node::Node::SharedPtr n) : n_(n) {
  descriptions_ = parse_tag_descriptions();

  std::string tag_family = "36h11";

  const AprilTags::TagCodes* tag_codes;
  if(tag_family == "16h5"){
    tag_codes = &AprilTags::tagCodes16h5;
  }
  else if(tag_family == "25h7"){
    tag_codes = &AprilTags::tagCodes25h7;
  }
  else if(tag_family == "25h9"){
    tag_codes = &AprilTags::tagCodes25h9;
  }
  else if(tag_family == "36h9"){
    tag_codes = &AprilTags::tagCodes36h9;
  }
  else if(tag_family == "36h11"){
    tag_codes = &AprilTags::tagCodes36h11;
  }
  else{
    printf("Invalid tag family specified; defaulting to 36h11");
    tag_codes = &AprilTags::tagCodes36h11;
  }

  tag_detector_= std::shared_ptr<AprilTags::TagDetector>(new AprilTags::TagDetector(*tag_codes));
  image_sub_= n_->create_subscription<sensor_msgs::msg::Image>("image", std::bind(&AprilTagDetector::imageCb, this, std::placeholders::_1), rmw_qos_profile_sensor_data);
  detections_image_pub_ = n_->create_publisher<sensor_msgs::msg::Image>("detections_image", rmw_qos_profile_default);
  detection_pose_pub_ = n_->create_publisher<geometry_msgs::msg::PoseStamped>("poses", rmw_qos_profile_default);
}

AprilTagDetector::~AprilTagDetector() {
}

void AprilTagDetector::imageCb(const sensor_msgs::msg::Image::SharedPtr msg) {
    printf("image received");
  if(msg->encoding != "bgr8")
  {
    printf("received image with unsupported encoding: %s\n", msg->encoding.c_str());
    return;
  }

  // Precompute the sin function for each row and column
  uint32_t image_width = msg->width;
  float x_radians_per_pixel = 60.0/57.0/image_width;
  float sin_pixel_x[image_width];
  for (int x = 0; x < image_width; ++x) {
    sin_pixel_x[x] = sin((x - image_width/ 2.0)  * x_radians_per_pixel);
  }

  uint32_t image_height = msg->height;
  float y_radians_per_pixel = 45.0/57.0/image_width;
  float sin_pixel_y[image_height];
  for (int y = 0; y < image_height; ++y) {
    // Sign opposite x for y up values
    sin_pixel_y[y] = sin((image_height/ 2.0 - y)  * y_radians_per_pixel);
  }

  // Convert image message to OpenCV image
  cv::Mat mat_color(msg->height, msg->width, CV_8UC3, const_cast<uchar*>(&msg->data[0]), msg->step);

  // Convert image go grayscale
  cv::Mat mat_gray;
  cv::cvtColor(mat_color, mat_gray, CV_BGR2GRAY);
  std::vector<AprilTags::TagDetection> detections = tag_detector_->extractTags(mat_gray);
  printf("%d tag detected", (int)detections.size());

  double fx;
  double fy;
  double px;
  double py;

  fx = 1.0/640.0;
  fy = 1.0/480.0;
  px = 320;
  py = 240;

  BOOST_FOREACH(AprilTags::TagDetection detection, detections){
    std::map<int, AprilTagDescription>::const_iterator description_itr = descriptions_.find(detection.id);
    if(description_itr == descriptions_.end()){
      printf("Found tag: %d, but no description was found for it", detection.id);
      continue;
    }
    AprilTagDescription description = description_itr->second;
    double tag_size = description.size();

    detection.draw(mat_color);
    Eigen::Matrix4d transform = detection.getRelativeTransform(tag_size, fx, fy, px, py);
    Eigen::Matrix3d rot = transform.block(0, 0, 3, 3);
    Eigen::Quaternion<double> rot_quaternion = Eigen::Quaternion<double>(rot);

    geometry_msgs::msg::PoseStamped tag_pose;
    tag_pose.pose.position.x = transform(0, 3);
    tag_pose.pose.position.y = transform(1, 3);
    tag_pose.pose.position.z = transform(2, 3);
    tag_pose.pose.orientation.x = rot_quaternion.x();
    tag_pose.pose.orientation.y = rot_quaternion.y();
    tag_pose.pose.orientation.z = rot_quaternion.z();
    tag_pose.pose.orientation.w = rot_quaternion.w();
    tag_pose.header.frame_id = description.frame_name();

    detection_pose_pub_->publish(tag_pose);
  }

  //image_pub_.publish(cv_ptr->toImageMsg());
}


std::map<int, AprilTagDescription> AprilTagDetector::parse_tag_descriptions(){
  std::map<int, AprilTagDescription> descriptions;

  // Hardcoded tags for now. -jbinney
  int id = 6;
  double size = 0.080;
  std::string frame_name = "tag6";
  AprilTagDescription description(id, size, frame_name);
  std::cout << "Loaded tag config: "<<id<<", size: "<<size<<", frame_name: "<< frame_name << std::endl;
  descriptions.insert(std::make_pair(id, description));
  return descriptions;
}


}

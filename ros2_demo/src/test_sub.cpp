#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

void image_cb(const sensor_msgs::msg::Image::SharedPtr msg) {
    printf("Got image message\n");
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto n = rclcpp::node::Node::make_shared("test_sub");
  //rclcpp::subscription::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  auto image_sub_= n->create_subscription<sensor_msgs::msg::Image>("usb_cam_image_raw", image_cb, rmw_qos_profile_sensor_data);
  rclcpp::spin(n);
  return 0;
}

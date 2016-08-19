#include <iostream>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>

rclcpp::publisher::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdpub;
geometry_msgs::msg::Twist move_a_bit;
geometry_msgs::msg::Twist stop;

void chatterCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (msg->data == "Move left please") {
    std::cout << "Got: '" << msg->data << "', moving a bit" << std::endl;
    for (int repeat = 0; repeat < 10; ++repeat) {
      cmdpub->publish(move_a_bit);
    }
  } else {
    cmdpub->publish(stop);
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // This will create a node in the Domain 2
  auto node = rclcpp::node::Node::make_shared("driver#2");

  auto sub = node->create_subscription<std_msgs::msg::String>(
    "please_move", chatterCallback, rmw_qos_profile_default);
  cmdpub = node->create_publisher<geometry_msgs::msg::Twist>(
    "cmd_vel", rmw_qos_profile_default);

  stop = geometry_msgs::msg::Twist();
  move_a_bit = geometry_msgs::msg::Twist();
  move_a_bit.linear.x = 0.1;
  stop.linear.x = 0;
  stop.angular.z = 0;

  rclcpp::spin(node);

  return 0;
}

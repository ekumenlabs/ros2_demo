#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node_domain_2 = rclcpp::node::Node::make_shared("please_move#2");
  auto pub_2 = node_domain_2->create_publisher<std_msgs::msg::String>("please_move", rmw_qos_profile_default);

  rclcpp::WallRate loop_rate(2);

  auto msg = std::make_shared<std_msgs::msg::String>();

  while (rclcpp::ok()) {

    for (int repeat = 0; repeat < 10; ++repeat) {
      msg->data = "Move left please";
      pub_2->publish(msg);
      rclcpp::spin_some(node_domain_2);
      std::cout << "Publishing: '" << msg->data << "' to domain 2" << std::endl;
      loop_rate.sleep();
    }

    msg->data = "Stop";
    pub_2->publish(msg);
    rclcpp::spin_some(node_domain_2);
    std::cout << "Publishing: '" << msg->data << "' to domain 2" << std::endl;

    rclcpp::utilities::sleep_for(std::chrono::seconds(10));
  }

  return 0;
}

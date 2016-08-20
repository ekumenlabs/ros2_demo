#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

std::chrono::time_point<std::chrono::steady_clock> last_detection = std::chrono::steady_clock::now();

void detectionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::cout << "We are detecting the RoDI. We should stop! " << msg << std::endl;
  last_detection = std::chrono::steady_clock::now();
}

bool was_rodi_detected(void) {
  std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
  std::chrono::duration<double> time_since_seen = now-last_detection;
  if (time_since_seen > std::chrono::seconds(10)) {
    std::cout << "We have not seen rodi for 10 seconds, going forward." << std::endl;
    return false;
  } else {
    std::cout << "There is a RoDI! Stopping!." << std::endl;
  }
  return true;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node_rodi = rclcpp::node::Node::make_shared("please_move#2");
  auto node_turtle = rclcpp::node::Node::make_shared("driver_forward#1");

  auto pub_rodi = node_rodi->create_publisher<std_msgs::msg::String>("please_move", rmw_qos_profile_default);
  auto pub_turtle = node_turtle->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rmw_qos_profile_default);
  auto sub = node_turtle->create_subscription<geometry_msgs::msg::PoseStamped>("tag_pose", detectionCallback, rmw_qos_profile_default);

  auto msg = std::make_shared<std_msgs::msg::String>();

  auto move_forward = geometry_msgs::msg::Twist();
  move_forward.linear.x = 0.5;
  auto stop = geometry_msgs::msg::Twist();

  bool rodi_detected = false;
  rclcpp::WallRate loop_rate(10);

  while (rclcpp::ok()) {

    rodi_detected = was_rodi_detected();

    if (rodi_detected) {
      pub_turtle->publish(move_forward);
    } else {
      pub_turtle->publish(stop);
    }
    rclcpp::spin_some(node_rodi);
    rclcpp::spin_some(node_turtle);
    loop_rate.sleep();
  }

  return 0;
}

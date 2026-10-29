#include <functional>
#include <memory>
#include <string>

#include "aruco_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

class ArmandoArucoTargetNode : public rclcpp::Node
{
public:
  ArmandoArucoTargetNode()
  : Node("armando_aruco_target_node")
  {
    declare_parameter<int>("target_id", 0);
    declare_parameter<std::string>("marker_topic", "/aruco_marker_publisher/markers");
    declare_parameter<std::string>("target_pose_topic", "/target_pose");

    target_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      get_parameter("target_pose_topic").as_string(), rclcpp::QoS(10));
    marker_sub_ = create_subscription<aruco_msgs::msg::MarkerArray>(
      get_parameter("marker_topic").as_string(), rclcpp::QoS(10).reliable(),
      std::bind(&ArmandoArucoTargetNode::marker_callback, this, std::placeholders::_1));
  }

private:
  void marker_callback(const aruco_msgs::msg::MarkerArray::SharedPtr msg)
  {
    (void)msg;

    // TODO(classwork):
    // 1. Find the marker whose ID matches target_id.
    // 2. Resolve or transform its pose into base_link.
    // 3. Publish a geometry_msgs::msg::PoseStamped on target_pose_topic.
    // 4. Handle a missing marker without publishing a stale target.
    RCLCPP_DEBUG(get_logger(), "Implement ArUco target extraction.");
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  rclcpp::Subscription<aruco_msgs::msg::MarkerArray>::SharedPtr marker_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmandoArucoTargetNode>());
  rclcpp::shutdown();
  return 0;
}

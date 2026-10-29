#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include "aruco_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class ArmandoArucoTargetNode : public rclcpp::Node
{
public:
  ArmandoArucoTargetNode()
  : Node("armando_aruco_target_node")
  {
    declare_parameter<int>("target_id", 0);
    declare_parameter<std::string>("marker_topic", "/aruco_marker_publisher/markers");
    declare_parameter<std::string>("target_pose_topic", "/target_pose");
    declare_parameter<double>("target_normal_offset", 0.1);

    target_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      get_parameter("target_pose_topic").as_string(), rclcpp::QoS(10));
    marker_sub_ = create_subscription<aruco_msgs::msg::MarkerArray>(
      get_parameter("marker_topic").as_string(), rclcpp::QoS(10).reliable(),
      std::bind(&ArmandoArucoTargetNode::marker_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "ArUco target adapter ready (marker topic: %s, pose topic: %s).",
      get_parameter("marker_topic").as_string().c_str(),
      get_parameter("target_pose_topic").as_string().c_str());
  }

private:
  void marker_callback(const aruco_msgs::msg::MarkerArray::SharedPtr msg)
  {
    const int target_id = static_cast<int>(get_parameter("target_id").as_int());
    if (target_id < 0) {
      return;
    }

    for (const auto & marker : msg->markers) {
      if (marker.id != target_id) {
        continue;
      }

      const double normal_offset = get_parameter("target_normal_offset").as_double();
      if (!std::isfinite(normal_offset)) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000, "target_normal_offset must be finite.");
        return;
      }

      geometry_msgs::msg::PoseStamped target_pose;
      target_pose.header = marker.header;
      target_pose.header.stamp = rclcpp::Time(0);
      target_pose.pose = marker.pose.pose;

      tf2::Quaternion orientation;
      tf2::fromMsg(target_pose.pose.orientation, orientation);
      const double orientation_norm = orientation.length2();
      if (!std::isfinite(orientation_norm) || orientation_norm < 1e-12) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "Selected ArUco marker has invalid orientation.");
        return;
      }
      orientation.normalize();

      const tf2::Vector3 marker_normal =
        tf2::quatRotate(orientation, tf2::Vector3(0.0, 0.0, 1.0));
      target_pose.pose.position.x += normal_offset * marker_normal.x();
      target_pose.pose.position.y += normal_offset * marker_normal.y();
      target_pose.pose.position.z += normal_offset * marker_normal.z();
      target_pose_pub_->publish(target_pose);
      return;
    }
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

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "pose_loader.hpp"

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

class ArmandoController : public rclcpp::Node {
public:
  ArmandoController() : Node("armando_controller") {
    this->declare_parameter<std::string>("poses_file", "");
    this->declare_parameter<std::string>("controller_name", "/joint_trajectory_controller/follow_joint_trajectory");
    std::string poses_file = this->get_parameter("poses_file").as_string();
    if (poses_file.empty()) {
      // Use the installed package share directory path
      std::string pkg_share = ament_index_cpp::get_package_share_directory("armando_controller");
      poses_file = pkg_share + "/config/poses.yaml";
    }
    poses_ = load_poses(poses_file);
    client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, this->get_parameter("controller_name").as_string());
    RCLCPP_INFO(this->get_logger(), "ArmandoController node started.");
  }

  bool has_pose(const std::string& pose_name) const {
    return poses_.count(pose_name) > 0;
  }

  void send_pose_goal(const std::string& pose_name) {
    if (poses_.count(pose_name) == 0) {
      RCLCPP_ERROR(this->get_logger(), "Pose '%s' not found!", pose_name.c_str());
      return;
    }
    std::vector<std::string> joint_names = {"j0", "j1", "j2", "j3"};
    RCLCPP_INFO(this->get_logger(), "Joint names used:");
    for (const auto& j : joint_names) {
      RCLCPP_INFO(this->get_logger(), "  %s", j.c_str());
    }
    // Debug: print positions that will be sent
    const auto& positions = poses_[pose_name];
    if (positions.size() != joint_names.size()) {
      RCLCPP_ERROR(this->get_logger(), "Mismatch between joint count (%zu) and positions count (%zu)", joint_names.size(), positions.size());
      return;
    }
    if (!client_->wait_for_action_server(std::chrono::seconds(20))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      return; 
    }
    auto goal_msg = FollowJointTrajectory::Goal();
    goal_msg.trajectory.joint_names = joint_names;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = positions;
    point.time_from_start = rclcpp::Duration::from_seconds(2.0);
    goal_msg.trajectory.points.push_back(point);
    // Do not set header.stamp: let the controller handle it
    RCLCPP_INFO(this->get_logger(), "Sending goal for pose '%s' with positions:", pose_name.c_str());
    for (size_t i = 0; i < point.positions.size(); ++i) {
      RCLCPP_INFO(this->get_logger(), "  %s: %f", joint_names[i].c_str(), point.positions[i]);
    }
    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    send_goal_options.result_callback = [this, pose_name](const GoalHandleFollowJointTrajectory::WrappedResult & result) {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(this->get_logger(), "Goal for pose '%s' succeeded!", pose_name.c_str());
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "Goal for pose '%s' was aborted!", pose_name.c_str());
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(this->get_logger(), "Goal for pose '%s' was canceled!", pose_name.c_str());
          break;
        default:
          RCLCPP_ERROR(this->get_logger(), "Goal for pose '%s' finished with unknown result code: %d", pose_name.c_str(), static_cast<int>(result.code));
      }
      if (result.result) {
        RCLCPP_INFO(this->get_logger(), "Result error code: %d", result.result->error_code);
        RCLCPP_INFO(this->get_logger(), "Result error string: %s", result.result->error_string.c_str());
      }
      rclcpp::shutdown();
    };
    send_goal_options.feedback_callback = [this](GoalHandleFollowJointTrajectory::SharedPtr, const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback) {
      RCLCPP_INFO(this->get_logger(), "Feedback received: actual positions size = %zu", feedback->actual.positions.size());
    };
    client_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(this->get_logger(), "Goal sent for pose '%s'", pose_name.c_str());
  }

private:
  std::map<std::string, std::vector<double>> poses_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmandoController>();

  // Declare and read the "pose" parameter from command line
  node->declare_parameter<std::string>("pose", "pos0");
  std::string pose_name = node->get_parameter("pose").as_string();
  // Check if pose exists
  if (!node->has_pose(pose_name)) {
    RCLCPP_ERROR(node->get_logger(), "Pose '%s' does not exist! Aborting.", pose_name.c_str());
    rclcpp::shutdown();
    return 1;
  }
  node->send_pose_goal(pose_name);

  rclcpp::spin(node);
  return 0;
}
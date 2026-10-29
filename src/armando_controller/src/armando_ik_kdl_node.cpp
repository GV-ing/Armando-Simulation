#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <kdl/chain.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

class ArmandoIKKDLNode : public rclcpp::Node
{
public:
  ArmandoIKKDLNode()
  : Node("armando_ik_kdl_node")
  {
    declare_parameter<std::string>("target_pose_topic", "/target_pose");
    declare_parameter<std::string>(
      "controller_action_name", "/joint_trajectory_controller/follow_joint_trajectory");
    declare_parameter<std::string>("joint_states_topic", "/joint_states");
    declare_parameter<std::string>("base_link", "base_link");
    declare_parameter<std::string>("end_effector_link", "crawer_base");
    declare_parameter<std::string>("robot_description", "");

    initialize_kinematics();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, get_parameter("controller_action_name").as_string());

    target_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      get_parameter("target_pose_topic").as_string(), rclcpp::QoS(10),
      std::bind(&ArmandoIKKDLNode::target_pose_callback, this, std::placeholders::_1));
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      get_parameter("joint_states_topic").as_string(), rclcpp::QoS(10),
      std::bind(&ArmandoIKKDLNode::joint_state_callback, this, std::placeholders::_1));
    contact_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/contact_sensor_topic", rclcpp::QoS(10),
      std::bind(&ArmandoIKKDLNode::contact_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "KDL IK ready (target topic: %s, base: %s, end effector: %s).",
      get_parameter("target_pose_topic").as_string().c_str(),
      base_link_.c_str(), end_effector_link_.c_str());
  }

private:
  struct JointLimit
  {
    std::string name;
    bool has_position_limits;
    double lower;
    double upper;
  };

  void initialize_kinematics()
  {
    const std::string robot_description = get_parameter("robot_description").as_string();
    if (robot_description.empty()) {
      throw std::runtime_error(
              "robot_description is empty. Pass the expanded URDF using --ros-args -p "
              "robot_description:=...");
    }

    base_link_ = get_parameter("base_link").as_string();
    end_effector_link_ = get_parameter("end_effector_link").as_string();

    urdf::Model urdf_model;
    if (!urdf_model.initString(robot_description)) {
      throw std::runtime_error("Failed to parse robot_description as URDF");
    }

    KDL::Tree tree;
    if (!kdl_parser::treeFromString(robot_description, tree)) {
      throw std::runtime_error("Failed to construct a KDL tree from robot_description");
    }
    if (!tree.getChain(base_link_, end_effector_link_, chain_)) {
      throw std::runtime_error(
              "No KDL chain from '" + base_link_ + "' to '" + end_effector_link_ + "'");
    }

    if (chain_.getNrOfJoints() != joint_names_.size()) {
      throw std::runtime_error(
              "The selected KDL chain must contain exactly the four controller joints j0-j3");
    }

    std::size_t joint_index = 0;
    for (unsigned int segment_index = 0;
      segment_index < chain_.getNrOfSegments(); ++segment_index)
    {
      const auto & joint = chain_.getSegment(segment_index).getJoint();
      if (joint.getType() == KDL::Joint::Fixed) {
        continue;
      }

      if (joint_index >= joint_names_.size() || joint.getName() != joint_names_[joint_index]) {
        throw std::runtime_error(
                "The KDL chain joint order must be j0, j1, j2, j3 to match the controller");
      }

      const auto urdf_joint = urdf_model.getJoint(joint.getName());
      if (!urdf_joint) {
        throw std::runtime_error("Joint '" + joint.getName() + "' is missing from robot_description");
      }

      JointLimit limit{joint.getName(), false, 0.0, 0.0};
      if (urdf_joint->limits) {
        limit.has_position_limits = urdf_joint->type != urdf::Joint::CONTINUOUS;
        limit.lower = urdf_joint->limits->lower;
        limit.upper = urdf_joint->limits->upper;
      }
      joint_limits_.push_back(std::move(limit));
      ++joint_index;
    }

    Eigen::Matrix<double, 6, 1> task_weights = Eigen::Matrix<double, 6, 1>::Zero();
    task_weights[0] = 1.0;
    task_weights[1] = 1.0;
    task_weights[2] = 1.0;
    ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(chain_, task_weights);
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    for (std::size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
      current_joint_positions_[msg->name[i]] = msg->position[i];
    }
  }

  void contact_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data && !target_touched_) {
      target_touched_ = true;
      RCLCPP_INFO(get_logger(), "Contact detected; pausing target tracking.");
    }
  }

  void target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (target_touched_ || is_moving_) {
      return;
    }
    process_target_pose(*msg);
  }

  void process_target_pose(const geometry_msgs::msg::PoseStamped & input_pose)
  {
    geometry_msgs::msg::PoseStamped target_pose;
    try {
      target_pose = tf_buffer_->transform(
        input_pose, base_link_, tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Could not transform target pose: %s", ex.what());
      return;
    }

    const auto & point = target_pose.pose.position;
    const double target_x = point.x;
    const double target_y = point.y;
    const double target_z = point.z;
    if (!std::isfinite(target_x) || !std::isfinite(target_y) || !std::isfinite(target_z)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Target pose position must be finite.");
      return;
    }
    const double dx = target_x - last_target_position_[0];
    const double dy = target_y - last_target_position_[1];
    const double dz = target_z - last_target_position_[2];
    if (has_valid_target_ && std::sqrt(dx * dx + dy * dy + dz * dz) < 0.01) {
      return;
    }

    std::vector<double> joint_positions;
    if (!solve_position_ik(target_x, target_y, target_z, joint_positions)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "KDL could not find a valid joint solution at (%.3f, %.3f, %.3f).",
        target_x, target_y, target_z);
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "KDL solution at (%.3f, %.3f, %.3f); sending trajectory.",
      target_x, target_y, target_z);
    send_trajectory(joint_positions);
    last_target_position_ = {target_x, target_y, target_z};
    has_valid_target_ = true;
  }

  bool solve_position_ik(double x, double y, double z, std::vector<double> & positions)
  {
    KDL::JntArray initial(chain_.getNrOfJoints());
    KDL::JntArray solution(chain_.getNrOfJoints());

    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
      const auto current = current_joint_positions_.find(joint_names_[i]);
      initial(static_cast<unsigned int>(i)) =
        current == current_joint_positions_.end() ? 0.0 : current->second;
    }

    const KDL::Frame target(KDL::Rotation::Identity(), KDL::Vector(x, y, z));
    if (ik_solver_->CartToJnt(initial, target, solution) < 0) {
      return false;
    }

    positions.resize(joint_names_.size());
    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
      const double position = solution(static_cast<unsigned int>(i));
      if (!std::isfinite(position)) {
        return false;
      }
      if (joint_limits_[i].has_position_limits &&
        (position < joint_limits_[i].lower || position > joint_limits_[i].upper))
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "KDL solution for joint %s is outside its URDF limits.",
          joint_limits_[i].name.c_str());
        return false;
      }
      positions[i] = position;
    }
    return true;
  }

  void send_trajectory(const std::vector<double> & target_positions)
  {
    if (target_positions.size() != joint_names_.size()) {
      RCLCPP_ERROR(get_logger(), "Refusing to send a trajectory with an invalid joint count.");
      return;
    }
    if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_ERROR(get_logger(), "Trajectory action server is not available.");
      return;
    }

    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names.assign(joint_names_.begin(), joint_names_.end());

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = target_positions;
    point.velocities.resize(joint_names_.size(), 0.0);
    point.time_from_start = rclcpp::Duration::from_seconds(3.0);
    goal.trajectory.points.push_back(point);

    auto options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    options.goal_response_callback =
      [this](const GoalHandleFollowJointTrajectory::SharedPtr & goal_handle) {
        is_moving_ = static_cast<bool>(goal_handle);
        if (!goal_handle) {
          RCLCPP_ERROR(get_logger(), "Trajectory goal was rejected.");
        }
      };
    options.result_callback =
      [this](const GoalHandleFollowJointTrajectory::WrappedResult & result) {
        is_moving_ = false;
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_WARN(get_logger(), "Trajectory did not complete successfully.");
        }
      };
    action_client_->async_send_goal(goal, options);
  }

  const std::array<std::string, 4> joint_names_{"j0", "j1", "j2", "j3"};
  std::string base_link_;
  std::string end_effector_link_;
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
  std::vector<JointLimit> joint_limits_;
  std::unordered_map<std::string, double> current_joint_positions_;
  std::array<double, 3> last_target_position_{0.0, 0.0, 0.0};
  bool target_touched_ = false;
  bool is_moving_ = false;
  bool has_valid_target_ = false;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr contact_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ArmandoIKKDLNode>());
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("armando_ik_kdl_node"), "%s", ex.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

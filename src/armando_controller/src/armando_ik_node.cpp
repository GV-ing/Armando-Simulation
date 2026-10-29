#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "aruco_msgs/msg/marker_array.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

class ArmandoIKNode : public rclcpp::Node {
public:
    ArmandoIKNode() : Node("armando_ik_node") {
        // Initial parameters
        this->declare_parameter("target_id", 0);
        this->declare_parameter<std::string>("controller_action_name", "/joint_trajectory_controller/follow_joint_trajectory");

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, this->get_parameter("controller_action_name").as_string());

        rclcpp::QoS qos_profile(10);
        qos_profile.reliable();
        qos_profile.durability_volatile();

        marker_sub_ = this->create_subscription<aruco_msgs::msg::MarkerArray>(
            "/aruco_marker_publisher/markers", qos_profile,
            std::bind(&ArmandoIKNode::marker_callback, this, std::placeholders::_1));

        contact_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/contact_sensor_topic", 10, 
            std::bind(&ArmandoIKNode::contact_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "=== Armando IK EXPERT (Full Version) Online ===");
        RCLCPP_INFO(this->get_logger(), "Use 'ros2 param set /armando_ik_node target_id -1' to return HOME.");
    }

private:
    void contact_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data && !target_touched_) {
            target_touched_ = true;
            RCLCPP_INFO(this->get_logger(), "!!! CONTACT DETECTED !!!");
        }
    }

    void marker_callback(const aruco_msgs::msg::MarkerArray::SharedPtr msg) {
        int current_target_id = this->get_parameter("target_id").as_int();
        
        // If the user requests target -1, send the robot HOME and exit
        if (current_target_id == -1) {
            if (last_target_id_ != -1) {
                RCLCPP_INFO(this->get_logger(), "HOME command received. Returning...");
                send_action_goal({0.0, 0.0, 0.0, 0.0});
                last_target_id_ = -1;
                has_valid_target_ = false;
            }
            return;
        }

        if (target_touched_ || is_moving_) return;

        // Dynamic target change detection
        if (current_target_id != last_target_id_) {
            RCLCPP_INFO(this->get_logger(), "Switching to Target ID: %d", current_target_id);
            last_target_id_ = current_target_id;
            has_valid_target_ = false;
            last_target_x_ = -999.0;
        }

        if (msg->markers.empty()) return;

        for (const auto & marker : msg->markers) {
            if (marker.id == current_target_id) { 
                process_target(marker);
                return; 
            }
        }
    }

    void process_target(const aruco_msgs::msg::Marker & marker) {
        geometry_msgs::msg::PoseStamped in_pose, out_pose;
        in_pose.header.frame_id = marker.header.frame_id;
        in_pose.header.stamp = rclcpp::Time(0);
        in_pose.pose = marker.pose.pose;

        try {
            out_pose = tf_buffer_->transform(in_pose, "base_link", tf2::durationFromSec(0.1));
        } catch (const tf2::TransformException & ex) {
            return;
        }
        
        double tx = out_pose.pose.position.x;
        double ty = out_pose.pose.position.y;
        double tz = out_pose.pose.position.z;

        double dist = std::sqrt(std::pow(tx - last_target_x_, 2) + std::pow(ty - last_target_y_, 2));
        
        if (!has_valid_target_ || dist >= 0.01) {
            std::vector<double> q_out;
            if (solve_ik_analytical(tx, ty, tz, q_out)) {
                RCLCPP_INFO(this->get_logger(), "Target ID %d Reachable. Sending trajectory...", last_target_id_);
                send_action_goal(q_out);
                last_target_x_ = tx;
                last_target_y_ = ty;
                has_valid_target_ = true;
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                    "ID %d OUT OF PHYSICAL RANGE (X:%.2f Y:%.2f Z:%.2f)", last_target_id_, tx, ty, tz);
            }
        }
    }

    bool solve_ik_analytical(double x, double y, double z, std::vector<double>& q_out) {
        q_out.resize(4, 0.0);
        
        // Real Parameters + Elasticity Margin (0.004m) for table limits
        double z_off = 0.1015; 
        double l1 = 0.1150; 
        double l2 = 0.1150; 
        double l3 = 0.0750;

        // 1. Base Calculation (j0) with URDF Offset Correction
        double q0 = std::atan2(y, x); 
        
        // Correction based on your rpy="0 0 -1.57" in the URDF
        // If the arm goes to the opposite side, change + to -
        double q0_cmd = q0 - (M_PI / 2.0); 
        
        while (q0_cmd > M_PI) q0_cmd -= 2.0 * M_PI;
        while (q0_cmd < -M_PI) q0_cmd += 2.0 * M_PI;

        // 2. 2D Kinematics on the radial plane
        double r = std::sqrt(x*x + y*y);
        double phi = -M_PI / 2.0; 

        double rw = r - l3 * std::cos(phi);
        double zw = (z - z_off) - l3 * std::sin(phi);
        double d = std::sqrt(rw*rw + zw*zw);

        if (d > (l1 + l2) || d < std::abs(l1 - l2)) return false;

        // 3. Internal angle calculations
        double cos_q2 = (d*d - l1*l1 - l2*l2) / (2 * l1 * l2);
        if (cos_q2 > 1.0) cos_q2 = 1.0; 
        if (cos_q2 < -1.0) cos_q2 = -1.0;
        
        double q2 = -std::acos(cos_q2); 

        double alpha = std::atan2(zw, rw);
        double beta = std::acos((l1*l1 + d*d - l2*l2) / (2 * l1 * d));
        double q1 = alpha + beta;

        // 4. Mapping to Armando joints
        q_out[0] = q0_cmd; 
        q_out[1] = q1 - (M_PI / 2.0); 
        q_out[2] = q2;
        q_out[3] = phi - (q1 + q2);

        return true;
    }

    void send_action_goal(const std::vector<double>& target_positions) {
        if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available!");
            return;
        }

        auto goal_msg = FollowJointTrajectory::Goal();
        goal_msg.trajectory.joint_names = {"j0", "j1", "j2", "j3"};

        int current_target_id = this->get_parameter("target_id").as_int();

        // Create the two trajectory points
        trajectory_msgs::msg::JointTrajectoryPoint p1;
        trajectory_msgs::msg::JointTrajectoryPoint p2;

        p1.velocities.resize(4, 0.0);
        p2.velocities.resize(4, 0.0);
        p2.positions = target_positions; // The final point is always the IK target

        if (current_target_id >= 0) {
            // ORDER: j0 FIRST, then the others
            // Point 1: only j0 moves to target, others stay (assume 0 or current position)
            // Note: Ideally you should have the "current" position of joints j1, j2, j3 here
            p1.positions = {target_positions[0], 0.0, 0.0, 0.0}; 
            p1.time_from_start = rclcpp::Duration::from_seconds(1.5); // j0 arrives here
            
            p2.time_from_start = rclcpp::Duration::from_seconds(3.0); // All others finish here
            RCLCPP_INFO(this->get_logger(), "Sequence: j0 -> Others (Target >= 0)");
        } 
        else {
            // ORDER: Others FIRST, then j0 (HOME or Target -1)
            // Point 1: j0 stays still (0.0), others move to target
            p1.positions = {0.0, target_positions[1], target_positions[2], target_positions[3]};
            p1.time_from_start = rclcpp::Duration::from_seconds(1.5);
            
            p2.time_from_start = rclcpp::Duration::from_seconds(3.0);
            RCLCPP_INFO(this->get_logger(), "Sequence: Others -> j0 (Target -1)");
        }

        goal_msg.trajectory.points.push_back(p1);
        goal_msg.trajectory.points.push_back(p2);

        // ... remaining send logic (callbacks) stays the same ...
        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        // (Keep your existing callbacks here)
        send_goal_options.goal_response_callback = [this](GoalHandleFollowJointTrajectory::SharedPtr goal_handle) {
            this->is_moving_ = !!goal_handle;
        };
        send_goal_options.result_callback = [this](const GoalHandleFollowJointTrajectory::WrappedResult & result) {
            this->is_moving_ = false;
        };

        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    // State memory
    int last_target_id_ = -1;
    bool target_touched_ = false;
    bool is_moving_ = false;
    bool has_valid_target_ = false;
    double last_target_x_ = -999.0, last_target_y_ = -999.0;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
    rclcpp::Subscription<aruco_msgs::msg::MarkerArray>::SharedPtr marker_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr contact_sub_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmandoIKNode>());
    rclcpp::shutdown();
    return 0;
}
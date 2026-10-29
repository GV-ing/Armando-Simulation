#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "std_msgs/msg/empty.hpp"

using namespace std::chrono_literals;

class ArmandoAutomator : public rclcpp::Node {
public:
    ArmandoAutomator() : Node("armando_automator") {
        trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", 10);

        // Publisher initialization for all 4 objects
        attach_a_pub_ = this->create_publisher<std_msgs::msg::Empty>("/gripper/attach_a", 10);
        detach_a_pub_ = this->create_publisher<std_msgs::msg::Empty>("/gripper/detach_a", 10);
        attach_b_pub_ = this->create_publisher<std_msgs::msg::Empty>("/gripper/attach_b", 10);
        detach_b_pub_ = this->create_publisher<std_msgs::msg::Empty>("/gripper/detach_b", 10);
        attach_c_pub_ = this->create_publisher<std_msgs::msg::Empty>("/gripper/attach_c", 10);
        detach_c_pub_ = this->create_publisher<std_msgs::msg::Empty>("/gripper/detach_c", 10);
        attach_d_pub_ = this->create_publisher<std_msgs::msg::Empty>("/gripper/attach_d", 10);
        detach_d_pub_ = this->create_publisher<std_msgs::msg::Empty>("/gripper/detach_d", 10);

        RCLCPP_INFO(this->get_logger(), "Full Armando C++ Automator node started.");
        timer_ = this->create_wall_timer(2s, std::bind(&ArmandoAutomator::execute_sequence, this));
    }

private:
    struct Step {
        std::vector<double> pos;
        std::string action; 
        double duration;
    };

    void execute_sequence() {
        timer_->cancel();
  
        // YOUR FULL SEQUENCE
        std::vector<Step> mission = {
            {{-0.854, 1.035, 2.105, -1.621}, "none", 4.0},     // pos0
            {{-0.854, 1.300, 1.0, -0.6},     "attach_a", 3.0}, // pos1 + ATTACH A
            {{-0.854, 0.0, 0.0, 0.0},        "none", 3.0},     // pos2
            {{-2.1, 1.0, 0.25, 0.25},        "detach_a", 4.0}, // pos3 + DETACH A
            {{0.854, 0.0, 0.0, 0.0},         "none", 3.0},     // pos4
            {{0.854, 1.035, 1.105, 1.0},     "attach_b", 3.5}, // pos5 + ATTACH B
            {{-0.854, 0.0, 0.0, 0.0},        "none", 3.0},     // pos6
            {{-0.854, 0.80, 0.45, 0.25},     "detach_b", 3.0}, // pos7 + DETACH B
            {{-2.03, 0.0, 0.0, 0.0},         "none", 3.0},     // pos8
            {{-1.9, 1.4, 0.50, 0.30},        "attach_c", 4.0}, // pos9 + ATTACH C
            {{-2.0, 0.0, 0.0, 0.0},          "none", 3.0},     // pos10
            {{0.8, 0.5, 0.5, 0.5},           "detach_c", 3.0}, // pos11 + DETACH C
            {{2.354, 0.0, 0.0, 0.0},         "none", 3.0},     // pos12
            {{2.354, 0.5, 1.5, 1.3},         "attach_d", 3.5}, // pos13 + ATTACH D
            {{1.9, 0.0, 0.0, 0.0},           "none", 3.0},     // pos14
            {{-0.854, 1.300, 0.0, -0.6},     "detach_d", 3.5}  // pos15 + DETACH D
        };

        for (size_t i = 0; i < mission.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "Executing Step %zu...", i);
            if (!move_to(mission[i].pos, mission[i].duration)) return;
            execute_gripper_action(mission[i].action);
        }
        RCLCPP_INFO(this->get_logger(), "FINAL MISSION COMPLETED!");
    }

    bool move_to(const std::vector<double> &positions, double duration) {
        auto msg = trajectory_msgs::msg::JointTrajectory();
        msg.joint_names = {"j0", "j1", "j2", "j3"};
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = positions;
        point.time_from_start = rclcpp::Duration::from_seconds(duration);
        msg.points.push_back(point);
        trajectory_pub_->publish(msg);
        rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>((duration + 0.5) * 1000)));
        return true; 
    }

    void execute_gripper_action(const std::string &action) {
        auto empty_msg = std_msgs::msg::Empty();
        if (action == "none") return;

        if (action == "attach_a") attach_a_pub_->publish(empty_msg);
        else if (action == "detach_a") detach_a_pub_->publish(empty_msg);
        else if (action == "attach_b") attach_b_pub_->publish(empty_msg);
        else if (action == "detach_b") detach_b_pub_->publish(empty_msg);
        else if (action == "attach_c") attach_c_pub_->publish(empty_msg);
        else if (action == "detach_c") detach_c_pub_->publish(empty_msg);
        else if (action == "attach_d") attach_d_pub_->publish(empty_msg);
        else if (action == "detach_d") detach_d_pub_->publish(empty_msg);

        RCLCPP_INFO(this->get_logger(), "-> Action: %s", action.c_str());
        rclcpp::sleep_for(1s);
    }

    // Publisher Members
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr attach_a_pub_, detach_a_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr attach_b_pub_, detach_b_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr attach_c_pub_, detach_c_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr attach_d_pub_, detach_d_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmandoAutomator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
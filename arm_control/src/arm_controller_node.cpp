#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class ArmControllerNode : public rclcpp::Node {
public:
    ArmControllerNode() : Node("arm_controller_node") {
        // Subscriber al topic /joint_states
        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&ArmControllerNode::jointStateCallback, this, std::placeholders::_1));

        // Publisher al topic /position_controller/command
        position_command_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/position_controller/command", 10);

        RCLCPP_INFO(this->get_logger(), "Arm Controller Node started.");
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received joint states:");
        for (size_t i = 0; i < msg->name.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "Joint %s: Position = %f", msg->name[i].c_str(), msg->position[i]);
        }

        
        auto command_msg = std_msgs::msg::Float64MultiArray();
        command_msg.data = {0.0, 0.0, 0.0, 0.0}; 
        position_command_publisher_->publish(command_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_command_publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmControllerNode>());
    rclcpp::shutdown();
    return 0;
}


#include "ros2_drone_core/px4_wrapper.hpp"

PX4Wrapper::PX4Wrapper(rclcpp::Node* node) : node_(node) {
    pub_cmd_ = node_->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
    pub_mode_ = node_->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
}

void PX4Wrapper::arm() {
    publish_cmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(node_->get_logger(), "⚠️ Sending ARM Command!");
}

void PX4Wrapper::set_offboard_mode() {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = true;
    msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    pub_mode_->publish(msg);
}

void PX4Wrapper::publish_cmd(uint16_t command, float p1, float p2) {
    px4_msgs::msg::VehicleCommand msg{};
    msg.command = command;
    msg.param1 = p1;
    msg.param2 = p2;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    pub_cmd_->publish(msg);
}

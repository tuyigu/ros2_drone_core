#ifndef PX4_WRAPPER_HPP_
#define PX4_WRAPPER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>

class PX4Wrapper {
public:
    explicit PX4Wrapper(rclcpp::Node* node);
    void arm();
    void set_offboard_mode();
private:
    rclcpp::Node* node_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr pub_cmd_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr pub_mode_;
    void publish_cmd(uint16_t command, float p1 = 0.0, float p2 = 0.0);
};
#endif

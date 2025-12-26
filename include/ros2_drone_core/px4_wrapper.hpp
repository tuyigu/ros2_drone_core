#ifndef PX4_WRAPPER_HPP_
#define PX4_WRAPPER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
class PX4Wrapper {
public:
    explicit PX4Wrapper(rclcpp::Node* node);
    void arm();
    void set_offboard_mode();
    float get_current_altitude() const { return current_alt_; }
private:
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_local_pos_;
    float current_alt_ = 0.0f;

    // 【新增】回调函数
    void local_pos_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    rclcpp::Node* node_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr pub_cmd_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr pub_mode_;
    void publish_cmd(uint16_t command, float p1 = 0.0, float p2 = 0.0);
};
#endif

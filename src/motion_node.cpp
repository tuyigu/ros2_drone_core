#include <rclcpp/rclcpp.hpp>
#include "ros2_drone_core/px4_wrapper.hpp"

class MotionController : public rclcpp::Node {
public:
    MotionController() : Node("motion_controller") {
        this->declare_parameter("arm_delay_sec", 5.0);
        this->declare_parameter("auto_arm", false);

        px4_ = std::make_shared<PX4Wrapper>(this);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&MotionController::loop, this));
        RCLCPP_INFO(this->get_logger(), "✅ System Ready. Waiting for param config...");
    }

private:
    std::shared_ptr<PX4Wrapper> px4_;
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_ = 0;
    bool params_loaded_ = false;
    double delay_ = 0.0;
    bool auto_arm_ = false;

    void loop() {
        px4_->set_offboard_mode(); // 心跳

        if (!params_loaded_) {
            delay_ = this->get_parameter("arm_delay_sec").as_double();
            auto_arm_ = this->get_parameter("auto_arm").as_bool();
            RCLCPP_INFO(this->get_logger(), "⚙️ Config: Delay=%.1fs, AutoArm=%d", delay_, auto_arm_);
            params_loaded_ = true;
        }

        if (auto_arm_ && counter_ == (int)(delay_ * 20)) {
            px4_->arm();
        }
        if (counter_ <= (int)(delay_ * 20) + 1) counter_++;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotionController>());
    rclcpp::shutdown();
    return 0;
}

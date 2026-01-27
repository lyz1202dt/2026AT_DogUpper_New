#include "robot_interfaces/msg/robot.hpp"
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <string>
#include <fstream>
#include <chrono>

namespace dog_controller {
class DogController : public controller_interface::ControllerInterface {
public:
    DogController();
    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

private:
    rclcpp::Publisher<robot_interfaces::msg::Robot>::SharedPtr state_publisher;
    rclcpp::Subscription<robot_interfaces::msg::Robot>::SharedPtr target_subscriber;
    std::vector<std::string> joints_name_;
    rclcpp_lifecycle::LifecycleNode::OnSetParametersCallbackHandle::SharedPtr param_cb_;

    robot_interfaces::msg::Robot joints_target;
    robot_interfaces::msg::Robot joints_state;

    double joint_torque_filter_gate{0.8};
    double joint_omega_filter_gate{0.8};
    double joint_kp[3],joint_kd[3];
    std::ofstream csv_file_;
    bool csv_initialized_;
    std::chrono::steady_clock::time_point start_time_;
    std::chrono::duration<double> soft_start_duration_{5.0};
    std::chrono::steady_clock::time_point soft_start_start_time;
    int debug_cnt=0;
};
} // namespace dog_controller

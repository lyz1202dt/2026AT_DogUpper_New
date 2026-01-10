#include "dog_controller/dog_controller.hpp"
#include "robot_interfaces/msg/robot.hpp"
#include <controller_interface/controller_interface.hpp>
// pluginlib export macro
#include <pluginlib/class_list_macros.hpp>

namespace dog_controller {
DogController::DogController() {}

controller_interface::CallbackReturn DogController::on_init() {
    state_publisher   = get_node()->create_publisher<robot_interfaces::msg::Robot>("legs_status", 10);
    target_subscriber = get_node()->create_subscription<robot_interfaces::msg::Robot>(
        "legs_target", 10, [this](const robot_interfaces::msg::Robot& msg) { joints_target = msg; });
    joints_name_ = {"lf_joint1", "lf_joint2", "lf_joint3", "rf_joint1", "rf_joint2", "rf_joint3",
                    "lb_joint1", "lb_joint2", "lb_joint3", "rb_joint1", "rb_joint2", "rb_joint3"};
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DogController::on_configure(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    return controller_interface::ControllerInterface::CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn DogController::on_activate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    return controller_interface::ControllerInterface::CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn DogController::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    return controller_interface::ControllerInterface::CallbackReturn::SUCCESS;
}

controller_interface::return_type DogController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
    auto joints_num = joints_name_.size();
    robot_interfaces::msg::Robot state_msg;

    for (size_t i = 0; i < joints_num; i++) // 从接口读取关节状态
    {
        state_msg.legs[i / 3].joints[i % 3].rad    = state_interfaces_[i * 3 + 0].get_value();
        state_msg.legs[i / 3].joints[i % 3].omega  = state_interfaces_[i * 3 + 1].get_value();
        state_msg.legs[i / 3].joints[i % 3].torque = state_interfaces_[i * 3 + 2].get_value();
    }
    state_publisher->publish(state_msg);    // 发布关节状态

    for (size_t i = 0; i < joints_num; i++) // 将计算结果写入硬件层
    {
        command_interfaces_[i * 3 + 0].set_value((double)joints_target.legs[i / 3].joints[i % 3].rad);
        command_interfaces_[i * 3 + 1].set_value((double)joints_target.legs[i / 3].joints[i % 3].omega);
        command_interfaces_[i * 3 + 2].set_value((double)joints_target.legs[i / 3].joints[i % 3].torque);
    }
    return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration DogController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& name : joints_name_) {
        cfg.names.push_back(name + "/position");
        cfg.names.push_back(name + "/velocity");
        cfg.names.push_back(name + "/effort");
    }
    return cfg;
}

controller_interface::InterfaceConfiguration DogController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& name : joints_name_) {
        cfg.names.push_back(name + "/position");
        cfg.names.push_back(name + "/velocity");
        cfg.names.push_back(name + "/effort");
    }
    return cfg;
}
} // namespace dog_controller

PLUGINLIB_EXPORT_CLASS(dog_controller::DogController, controller_interface::ControllerInterface)
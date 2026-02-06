#ifndef DRIVE_HPP
#define DRIVE_HPP

#include <memory>
#include <thread> 
#include <rclcpp/rclcpp.hpp>
#include "cdc_trans.hpp"
#include "arm_data.h"
#include "robot_interfaces/msg/motor_target.hpp"
#include "robot_interfaces/msg/motor_state.hpp"

class DriveNode : public rclcpp::Node {
public:
    DriveNode();
    ~DriveNode();
private:
    bool exit_thread{false};
    bool first_update{true};
    int state_log_print_cnt{0};
    int target_log_print_cnt{0};
    int state_log_update_cnt{50};
    int target_log_update_cnt{50};
    bool enable_control{false};
    void publishState(const state_pack_t* legs_state);
    void subscribStarget(const robot_interfaces::msg::MotorTarget &msg);
    target_pack_t pack;
    std::unique_ptr<CDCTrans> cdc_trans;
    std::unique_ptr<std::thread> usb_event_handle_thread;
    rclcpp::Publisher<robot_interfaces::msg::MotorState>::SharedPtr arm_pub;
    rclcpp::Subscription<robot_interfaces::msg::MotorTarget>::SharedPtr arm_sub;
};
#endif
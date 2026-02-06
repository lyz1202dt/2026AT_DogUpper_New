#include "arm_drive.hpp"
#include "cdc_trans.hpp"
#include <chrono>
#include <memory>
#include <rclcpp/logging.hpp>
#include <robot_interfaces/msg/motor_state.hpp>
#include <robot_interfaces/msg/motor_target.hpp>
#include <thread>

using namespace std::chrono_literals;

DriveNode::DriveNode() : rclcpp::Node("driver_node") {
    arm_pub = this->create_publisher<robot_interfaces::msg::MotorState>("arm_status", 10);
    arm_sub = this->create_subscription<robot_interfaces::msg::MotorTarget>("arm_target", 10, std::bind(&DriveNode::subscribStarget, this, std::placeholders::_1));
    cdc_trans = std::make_unique<CDCTrans>();                           // 创建CDC传输对象
    cdc_trans->regeiser_recv_cb([this](const uint8_t* data, int size) { // 注册接收回调
        // RCLCPP_INFO(this->get_logger(), "接收到了数据包,长度%d", size);
        if (size == sizeof(state_pack_t)) 
        {
            const state_pack_t* pack = reinterpret_cast<const state_pack_t*>(data);
            if (pack->pack_type == 0)         // 确认包类型正确
                publishState(pack);        // 一旦接收，立即发布狗腿状态
            else
                RCLCPP_ERROR(this->get_logger(), "接收到错误的数据包类型%d", pack->pack_type);
        }
    });
}

DriveNode::~DriveNode() {
    // 请求线程退出并等待其结束，保证安全关闭
    exit_thread = true;
    if (usb_event_handle_thread && usb_event_handle_thread->joinable()) {
        usb_event_handle_thread->join();
    }
    if (cdc_trans) {
        cdc_trans->close();
    }
}

void DriveNode::publishState(const state_pack_t* arm_state) {
    robot_interfaces::msg::MotorState msg;
    msg.pos = arm_state->robstride01.state.rad;
    msg.vel = arm_state->robstride01.state.omega;
    msg.torque = arm_state->robstride01.state.torque;
    msg.gmpositions = arm_state->GM6020.Angle_DEG;
    msg.gmvel = arm_state->GM6020.Speed;
    msg.upper = arm_state->servo2.up;
    msg.lower = arm_state->servo2.low;
    arm_pub->publish(msg);
}
void DriveNode::subscribStarget(const robot_interfaces::msg::MotorTarget &msg)
{
    pack.rob01.except_pos = msg.except_pos;
    pack.rob01.except_omega = msg.except_omega;
    pack.rob01.except_torque = msg.except_torque;
    pack.rob02.target_vel = msg.gm_except_speed;
    pack.servo1.up = msg.except_up;
    pack.servo1.low = msg.except_low;
    target_log_print_cnt++;
    if (target_log_update_cnt == target_log_print_cnt) {
        target_log_print_cnt = 0;
        RCLCPP_INFO(this->get_logger(), "订阅到电机目标值");
    }
    first_update = false;
    cdc_trans->send_struct(pack);
}
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DriveNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
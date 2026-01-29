#include "serialnode.hpp"
#include "cdc_trans.hpp"
#include "data_pack.h"
#include <rclcpp/logging.hpp>
#include <robot_interfaces/msg/robot.hpp>
#include <memory>
#include <thread>
#include <chrono>
#include <tf2/LinearMath/Quaternion.h> 
using namespace std::chrono_literals;

SerialNode::SerialNode()
    : Node("driver_node") {

    // 初始化状态
    exit_thread = false;
    legs_target.pack_type=0x00;

    this->declare_parameter("enable_control",false);

        param_server_ =
        this->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            RCLCPP_INFO(this->get_logger(), "更新PID参数");
            for (const auto& param : params) {
                if(param.get_name() == "enable_control")
                    enable_control=param.as_bool();
            }
            return result;
        });


    // 先创建 publisher/subscriber，确保回调中 publish 时 publisher 已就绪
    robot_pub = this->create_publisher<robot_interfaces::msg::Robot>("legs_status", 10);
    imu_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/imu_pose_sensor/pose",rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local());
    imu_angular_vel_pub=this->create_publisher<geometry_msgs::msg::Vector3>("/imu_imu_sensor/imu",rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local());
    robot_sub = this->create_subscription<robot_interfaces::msg::Robot>(
        "legs_target", 10, std::bind(&SerialNode::legsSubscribCb, this, std::placeholders::_1));



    cdc_trans = std::make_unique<CDCTrans>();                           // 创建CDC传输对象
    cdc_trans->regeiser_recv_cb([this](const uint8_t* data, int size) { // 注册接收回调
        //RCLCPP_INFO(this->get_logger(), "接收到了数据包,长度%d", size);
        if (size == sizeof(MotorStatePack_t)) // 验证包长度，可以被视作四条腿的状态数据包
        {
            const MotorStatePack_t* pack = reinterpret_cast<const MotorStatePack_t*>(data);
            if (pack->pack_type == 0)  // 确认包类型正确
                publishLegState(pack); // 一旦接收，立即发布狗腿状态
            else RCLCPP_ERROR(this->get_logger(), "接收到错误的数据包类型%d", pack->pack_type);
        }
    });
    if(!cdc_trans->open(0x0483, 0x5740))                                // 开启USB_CDC传输接口
        exit_thread=true;

    // 创建线程处理CDC消息（在 open 之后、publisher 创建之后）
    usb_event_handle_thread = std::make_unique<std::thread>([this]() {
        do{
            cdc_trans->process_once();
        }while (!exit_thread);
    });

    RCLCPP_INFO(this->get_logger(),"通信节点初始化完成");
}

SerialNode::~SerialNode() {
    // 请求线程退出并等待其结束，保证安全关闭
    exit_thread = true;
    if (usb_event_handle_thread && usb_event_handle_thread->joinable()) {
        usb_event_handle_thread->join();
    }
    if (cdc_trans) {
        cdc_trans->close();
    }
}

void SerialNode::publishLegState(const MotorStatePack_t* legs_state) {
    robot_interfaces::msg::Robot msg;
   // RCLCPP_INFO(this->get_logger(), "发布电机当前状态");
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            msg.legs[i].joints[j].rad    = legs_state->leg[i].joint[j].rad;
            msg.legs[i].joints[j].omega  = legs_state->leg[i].joint[j].omega;
            msg.legs[i].joints[j].torque = legs_state->leg[i].joint[j].torque;
        }
        msg.legs[i].wheel.omega=legs_state->leg[i].wheel.omega;
        msg.legs[i].wheel.torque=legs_state->leg[i].wheel.torque;
    }

    geometry_msgs::msg::PoseStamped imu_msg;
    tf2::Quaternion q;
    q.setRPY(legs_state->JY61.Angle.Roll, legs_state->JY61.Angle.Pitch, legs_state->JY61.Angle.Yaw);
    imu_msg.pose.orientation.x = q.x();
    imu_msg.pose.orientation.y = q.y();
    imu_msg.pose.orientation.z = q.z();
    imu_msg.pose.orientation.w = q.w();
    geometry_msgs::msg::Vector3 imu_angular_vel_msg;
    imu_angular_vel_msg.x = legs_state->JY61.AngularVelocity.X;
    imu_angular_vel_msg.y = legs_state->JY61.AngularVelocity.Y;
    imu_angular_vel_msg.z = legs_state->JY61.AngularVelocity.Z;

    state_log_print_cnt++;
    if(state_log_update_cnt==state_log_print_cnt)
    {
        state_log_print_cnt=0;
        RCLCPP_INFO(this->get_logger(), "发布电机状态");
    }

    robot_pub->publish(msg);
    imu_pub->publish(imu_msg);
    imu_angular_vel_pub->publish(imu_angular_vel_msg);

}

void SerialNode::legsSubscribCb(const robot_interfaces::msg::Robot& msg) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            legs_target.leg[i].joint[j].rad    = msg.legs[i].joints[j].rad;
            legs_target.leg[i].joint[j].omega  = msg.legs[i].joints[j].omega;
            legs_target.leg[i].joint[j].torque = msg.legs[i].joints[j].torque;
            legs_target.leg[i].joint[j].kp     = msg.legs[i].joints[j].kp;
            legs_target.leg[i].joint[j].kd     = msg.legs[i].joints[j].kd;
        }
        legs_target.leg[i].wheel.omega=msg.legs[i].wheel.omega;
        legs_target.leg[i].wheel.torque=msg.legs[i].wheel.torque;
    }
    cdc_trans->send_struct(legs_target); // 一旦订阅到最新的包，立即发送到下位机
    target_log_print_cnt++;
    if(target_log_update_cnt==target_log_print_cnt)
    {
        target_log_print_cnt=0;
        RCLCPP_INFO(this->get_logger(), "订阅到电机目标值");
    }
        
    first_update=false;
}

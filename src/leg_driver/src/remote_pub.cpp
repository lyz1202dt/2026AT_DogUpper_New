#include "serialnode.hpp"
#include "cdc_trans.hpp"
#include "data_pack.h"
#include <rclcpp/logging.hpp>
#include <robot_interfaces/msg/robot.hpp>
#include <memory>
#include <thread>
#include <chrono>
#include <tf2/LinearMath/Quaternion.h> 
#include "robot_interfaces/msg/move_cmd.hpp"
#include <cmath> 
using namespace std::chrono_literals;

class RemoteNode :public rclcpp::Node
{
    public:
    RemoteNode():Node("remote_node")
    {  
        remote_cdc_trans = std::make_unique<CDCTrans>();
          exit_thread=false;
          remote_pub= this->create_publisher<robot_interfaces::msg::MoveCmd>("robot_move_cmd", 10);
           remote_cdc_trans->regeiser_recv_cb([this](const uint8_t* data, int size) { // 注册接收回调
        RCLCPP_INFO(this->get_logger(), "接收到了数据包,长度%d", size);
        if (size == sizeof(MotorStatePack_t)) // 验证包长度，可以被视作四条腿的状态数据包
        {
            const MotorStatePack_t* pack = reinterpret_cast<const MotorStatePack_t*>(data);
            if (pack->pack_type == 0)  // 确认包类型正确
                publishremote(pack); // 一旦接收，立即发布狗腿状态
            else RCLCPP_ERROR(this->get_logger(), "接收到错误的数据包类型%d", pack->pack_type);
        }
    });
    if(!remote_cdc_trans->open(0x0483, 0x5740))                                // 开启USB_CDC传输接口
        exit_thread=true;
    // 创建线程处理CDC消息（在 open 之后、publisher 创建之后）
      usb_event_handle_thread = std::make_unique<std::thread>([this]() {
        do{
            remote_cdc_trans->process_once();
        }while (!exit_thread);
    });
}
    ~RemoteNode() {
    // 请求线程退出并等待其结束，保证安全关闭
    exit_thread = true;
    if (usb_event_handle_thread && usb_event_handle_thread->joinable()) {
        usb_event_handle_thread->join();
    }
    if (remote_cdc_trans) {
        remote_cdc_trans->close();
    }
}
        void publishremote(const MotorStatePack_t* legs_remote)
        {
        robot_interfaces::msg::MoveCmd remote_msg;
        remote_msg.vx =  legs_remote->remote.vx;
        remote_msg.vy =  legs_remote->remote.vy;
        remote_msg.vz = legs_remote->remote.omega;
        remote_msg.wheel_vel = legs_remote->remote.wheel_v;
        const float step_speed_limit= 0.2; 
        const float wheel_speed_limit =0.1;
    if (remote_msg.vx == 0 && remote_msg.vy == 0 && remote_msg.wheel_vel == 0) {
        remote_msg.step_mode = 1; 
    }
    if (std::fabs(remote_msg.vx)< step_speed_limit && std::fabs(remote_msg.vy) < step_speed_limit) {
            remote_msg.step_mode = 1; 
    }
    if (std::fabs(remote_msg.vx) > wheel_speed_limit) {
        remote_msg.step_mode = 2; 
    }
    
    if (std::fabs(remote_msg.vx) > 1.0f || std::fabs(remote_msg.vy) > 1.0) {
        remote_msg.step_mode = 1; 
    }
    if(remote_msg.wheel_vel > wheel_speed_limit){
        remote_msg.step_mode = 2;
    }
        RCLCPP_INFO(this->get_logger(), "legs_remote->remote.vx %f, legs_remote->remote.vy %f, legs_remote->remote.omega %f, legs_remote->remote.wheel_v %f", 
        legs_remote->remote.vx, legs_remote->remote.vy, legs_remote->remote.omega, legs_remote->remote.wheel_v);
        remote_pub->publish(remote_msg);
        }
    private:
    rclcpp::Publisher<robot_interfaces::msg::MoveCmd>::SharedPtr remote_pub;
    std::unique_ptr<CDCTrans> remote_cdc_trans;
    std::atomic<bool> exit_thread{false};
    std::unique_ptr<std::thread> usb_event_handle_thread;
};
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto remote_node = std::make_shared<RemoteNode>();
    rclcpp::spin(remote_node);
    rclcpp::shutdown();
    return 0;
}
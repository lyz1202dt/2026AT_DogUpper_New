#ifndef __SERIALNODE_HPP__
#define __SERIALNODE_HPP__

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "cdc_trans.hpp"
#include <robot_interfaces/msg/robot.hpp>
#include "data_pack.h"
#include <thread>
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <robot_interfaces/msg/move_cmd.hpp>



class SerialNode : public rclcpp::Node
{
public:
    SerialNode();
    ~SerialNode();

private:
    bool exit_thread;
    bool first_update{true};
    int state_log_print_cnt{0};
    int target_log_print_cnt{0};
    int state_log_update_cnt{50};
    int target_log_update_cnt{50};
    bool enable_control{false};
    void legsSubscribCb(const robot_interfaces::msg::Robot &msg);
    void publishLegState(const MotorStatePack_t *legs_state);
    void publishremote(const MotorStatePack_t* legs_remote);

    std::unique_ptr<CDCTrans> cdc_trans;
    std::unique_ptr<std::thread> usb_event_handle_thread;
    MotorTargetPack_t legs_target;
    rclcpp::Publisher<robot_interfaces::msg::Robot>::SharedPtr robot_pub;
    rclcpp::Subscription<robot_interfaces::msg::Robot>::SharedPtr robot_sub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr imu_pub;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr imu_angular_vel_pub;
    rclcpp::Publisher<robot_interfaces::msg::MoveCmd>::SharedPtr remote_pub;
    
    OnSetParametersCallbackHandle::SharedPtr param_server_;

    double joint_kp[3],joint_kd[3];
};

#endif
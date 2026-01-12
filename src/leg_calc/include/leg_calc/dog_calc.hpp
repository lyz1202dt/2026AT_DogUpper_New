#pragma once

#include "leg_calc.hpp"
#include "step.h"
#include "vmc.hpp"
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <chrono>
#include <ctime>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <memory>
#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <robot_interfaces/msg/robot.hpp>
#include <robot_interfaces/msg/move_cmd.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tuple>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>



class RobotCalcNode {
public:
    RobotCalcNode(const rclcpp::Node::SharedPtr node);
    ~RobotCalcNode();


    enum DogState {                // 机器人状态机
        DOG_IDEL,
        DOG_STOP,
        DOG_STARTING,
        DOG_SETP,
        DOG_ENDING
    };

    enum DogReqState{  //请求的机器人状态
        DOG_REQ_IDEL,
        DOG_REQ_STOP,
        DOG_REQ_RUN
    };

private:
    void show_callback();
    std::tuple<Vector3D, Vector3D, Vector3D> signal_leg_calc(
        const Vector3D& exp_cart_pos, const Vector3D& exp_cart_vel, const Vector3D& exp_cart_acc, const Vector3D& exp_cart_force,
        std::shared_ptr<LegCalc> leg_calc);
    void legs_update();

    rclcpp::Node::SharedPtr node_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_server_;

    std::shared_ptr<VMC> lf_z_vmc, lf_x_vmc, lf_y_vmc;
    std::shared_ptr<VMC> rf_z_vmc, rf_x_vmc, rf_y_vmc;
    std::shared_ptr<VMC> lb_z_vmc, lb_x_vmc, lb_y_vmc;
    std::shared_ptr<VMC> rb_z_vmc, rb_x_vmc, rb_y_vmc;

    bool enable_vmc{false};
    double force_filter_gate{0.8};
    
    rclcpp::TimerBase::SharedPtr ui_update_timer;
    rclcpp::TimerBase::SharedPtr legs_update_timer;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher;
    rclcpp::Publisher<robot_interfaces::msg::Robot>::SharedPtr legs_target_pub;
    rclcpp::Subscription<robot_interfaces::msg::Robot>::SharedPtr legs_state_sub;
    rclcpp::Subscription<robot_interfaces::msg::MoveCmd>::SharedPtr move_cmd_sub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr rviz_joint_publisher;
    rclcpp::SyncParametersClient::SharedPtr robot_description_param_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> robot_tf_broadcaster;

    std::vector<std::string> joint_names = {"lf_joint1", "lf_joint2", "lf_joint3", "rf_joint1", "rf_joint2", "rf_joint3",
                                            "lb_joint1", "lb_joint2", "lb_joint3", "rb_joint1", "rb_joint2", "rb_joint3"};

    // 解算部分
    KDL::Tree tree;
    std::string urdf_xml;
    KDL::Chain lf_leg_chain;
    KDL::Chain rf_leg_chain;
    KDL::Chain lb_leg_chain;
    KDL::Chain rb_leg_chain;
    std::shared_ptr<LegCalc> lf_leg_calc;
    std::shared_ptr<LegCalc> rf_leg_calc;
    std::shared_ptr<LegCalc> lb_leg_calc;
    std::shared_ptr<LegCalc> rb_leg_calc;

    // 步态规划部分
    Vector2D lf_exp_vel,rf_exp_vel,lb_exp_vel,rb_exp_vel;
    LegStep lf_leg_step,rf_leg_step,lb_leg_step,rb_leg_step;

    double step_time{0.8};  //整个对角步态全程的时间
    double step_height{0.08};
    double foot_base_height{0.0};
    double foot_pos_base_offset{-0.25};
    double step_support_rate{0.55};
    rclcpp::Time main_phrase_start_time,slave_phrase_start_time;   //第一、二相位步态开始时间
    rclcpp::Time slave_phrase_stop_time;
    bool step1_support_updated{false};
    bool step2_support_updated{false};
    bool step1_flight_updated{false};
    bool step2_flight_updated{false};

    // 数据缓存部分
    Eigen::Vector3d lf_joint_pos, lf_joint_vel;
    Eigen::Vector3d rf_joint_pos, rf_joint_vel;
    Eigen::Vector3d lb_joint_pos, lb_joint_vel;
    Eigen::Vector3d rb_joint_pos, rb_joint_vel;
    Eigen::Vector3d lf_forward_torque, lf_joint_torque;
    Eigen::Vector3d rf_forward_torque, rf_joint_torque;
    Eigen::Vector3d lb_forward_torque, lb_joint_torque;
    Eigen::Vector3d rb_forward_torque, rb_joint_torque;
    int rviz2_update_cnt{0};

    Eigen::Vector3d lf_leg_stop_pos,rf_leg_stop_pos,lb_leg_stop_pos,rb_leg_stop_pos;

    sensor_msgs::msg::JointState joint_display_msg;

    // 机器人状态
    DogState robot_state{DOG_IDEL};
    DogReqState robot_req_state{DOG_REQ_IDEL}; //请求的机器人状态
    double robot_lf_grivate{0.0};
    double robot_rf_grivate{0.0};
    double robot_lb_grivate{0.0};
    double robot_rb_grivate{0.0};
    double robot_lf_dx{0.25};
    double robot_rf_dx{0.25};
    double robot_lb_dx{-0.23};
    double robot_rb_dx{-0.23};
    
    tf2::Quaternion robot_rotation;                    //机器人姿态
    geometry_msgs::msg::Twist robot_velocity;          //机器人速度信息
};

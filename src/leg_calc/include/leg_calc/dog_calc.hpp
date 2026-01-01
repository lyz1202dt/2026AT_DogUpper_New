#pragma once

#include "leg_calc.hpp"
#include "step.h"
#include "vmc.hpp"
#include <Eigen/Dense>
#include <chrono>
#include <ctime>
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
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tuple>
#include <visualization_msgs/msg/marker.hpp>

class RobotCalcNode {
public:
    RobotCalcNode(const rclcpp::Node::SharedPtr node);
    ~RobotCalcNode();

private:
    enum DogState {                // 机器人状态机
        DOG_STOP,
        DOG_STARTINT,
        DOG_SETP_1,
        DOG_STEP_2,
        DOG_ENDING
    };

    void show_callback();
    std::tuple<Vector3D, Vector3D, Vector3D> signal_leg_calc(
        const Vector3D& exp_cart_pos, const Vector3D& exp_cart_vel, const Vector3D& exp_cart_acc, const Vector3D& exp_cart_force,
        std::shared_ptr<LegCalc> leg_calc);
    void signal_leg_state();
    void legs_update();
    void legs_update2();

    rclcpp::Node::SharedPtr node_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_server_;

    VMC* vmc;
    std::shared_ptr<VMC> lf_z_vmc,lf_x_vmc,lf_y_vmc;
    std::shared_ptr<VMC> rf_z_vmc,rf_x_vmc,rf_y_vmc;
    std::shared_ptr<VMC> lb_z_vmc,lb_x_vmc,lb_y_vmc;
    std::shared_ptr<VMC> rb_z_vmc,rb_x_vmc,rb_y_vmc;

    bool enable_vmc{false};
    double force_filter_gate{0.8};

    bool update_flag{true};

    CycloidStep_t cycloid_step[4]; // 4个脚的步态
    float leg_run_time;            // 一个脚步的时间
    rclcpp::Time last_step1_reset_time, last_step2_reset_time;

    rclcpp::TimerBase::SharedPtr ui_update_timer;
    rclcpp::TimerBase::SharedPtr legs_update_timer;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher;
    rclcpp::Publisher<robot_interfaces::msg::Robot>::SharedPtr legs_target_pub;
    rclcpp::Subscription<robot_interfaces::msg::Robot>::SharedPtr legs_state_sub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr rviz_joint_publisher;
    rclcpp::SyncParametersClient::SharedPtr robot_description_param_;

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

    CycloidStep_t step_line1, step_line2;
    StepTrajectory_t air_step_line;
    SupportTrajectory_t gnd_step_line;
    bool last_switch{false};

    Eigen::Vector3d lf_joint_pos,lf_joint_vel;
    Eigen::Vector3d rf_joint_pos,rf_joint_vel;
    Eigen::Vector3d lb_joint_pos,lb_joint_vel;
    Eigen::Vector3d rb_joint_pos,rb_joint_vel;
    Eigen::Vector3d lf_forward_torque,lf_joint_torque;
    Eigen::Vector3d rf_forward_torque,rf_joint_torque;
    Eigen::Vector3d lb_forward_torque,lb_joint_torque;
    Eigen::Vector3d rb_forward_torque,rb_joint_torque;

    sensor_msgs::msg::JointState joint_display_msg;

    // 机器人状态
    DogState robot_state{DOG_STOP};
    DogState last_robot_state{DOG_STOP};
    double robot_lf_grivate{40.0};
    double robot_rf_grivate{40.0};
    double robot_lb_grivate{40.0};
    double robot_rb_grivate{40.0};
};
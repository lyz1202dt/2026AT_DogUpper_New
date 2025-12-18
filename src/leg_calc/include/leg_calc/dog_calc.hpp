#pragma once

#include "step.h"
#include "vmc.hpp"
#include "leg_calc.hpp"
#include <Eigen/Dense>
#include <chrono>
#include <ctime>
#include <geometry_msgs/msg/point.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <memory>
#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <robot_interfaces/msg/robot.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tuple>
#include <visualization_msgs/msg/marker.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>



class RobotCalcNode {
public:
    RobotCalcNode(const rclcpp::Node::SharedPtr node);
    ~RobotCalcNode();
private:
    void show_callback();
    void legs_update();


    rclcpp::Node::SharedPtr node_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_server_;

    VMC* vmc;
    
    bool enable_vmc{false};
    double force_filter_gate{0.8};
    double joint1_kp;
    double joint1_kd;
    double joint2_kp;
    double joint2_kd;
    double joint3_kp;
    double joint3_kd;

    bool update_flag{true};


    CycloidStep_t cycloid_step[4];  //4个脚的步态
    float leg_run_time;                                                  // 一个脚步的时间
    rclcpp::Time last_step1_reset_time,last_step2_reset_time;

    rclcpp::TimerBase::SharedPtr ui_update_timer;
    rclcpp::TimerBase::SharedPtr legs_update_timer;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher;
    rclcpp::Publisher<robot_interfaces::msg::Robot>::SharedPtr legs_target_pub;
    rclcpp::Subscription<robot_interfaces::msg::Robot>::SharedPtr legs_state_sub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr rviz_joint_publisher;
    rclcpp::SyncParametersClient::SharedPtr robot_description_param_;

    std::vector<std::string> joint_names = {"lf_joint1", "lf_joint2", "lf_joint3",
                                                "rf_joint1", "rf_joint2", "rf_joint3",
                                                "lb_joint1", "lb_joint2", "lb_joint3",
                                                "rb_joint1", "rb_joint2", "rb_joint3"};

    //解算部分
    KDL::Tree tree;
    std::string urdf_xml;
    KDL::Chain lf_leg_chain;
    KDL::Chain rf_leg_chain;
    KDL::Chain lb_leg_chain;
    KDL::Chain rb_leg_chain;
    std::unique_ptr<LegCalc> lf_leg_calc;
    std::unique_ptr<LegCalc> rf_leg_calc;
    std::unique_ptr<LegCalc> lb_leg_calc;
    std::unique_ptr<LegCalc> rb_leg_calc;

    CycloidStep_t step_line1,step_line2;
    StepTrajectory_t air_step_line;
    SupportTrajectory_t gnd_step_line;
    bool last_switch{false};
    

    Eigen::Vector3d lf_cart_target;
    Eigen::Vector3d rf_cart_target;
    Eigen::Vector3d lb_cart_target;
    Eigen::Vector3d rb_cart_target;

    sensor_msgs::msg::JointState joint_msg;

};
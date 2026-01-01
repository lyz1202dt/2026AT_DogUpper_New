#include "dog_calc.hpp"
#include "step.h"
#include <Eigen/src/Core/Matrix.h>
#include <chrono>
#include <kdl/frames.hpp>
#include <rclcpp/duration.hpp>
#include <robot_interfaces/msg/detail/robot__struct.hpp>
#include <tuple>

using namespace std::chrono_literals;

RobotCalcNode::RobotCalcNode(const rclcpp::Node::SharedPtr node) {
    node_    = node;
    vmc      = new VMC(200, 60, 5.0, 0.5, 0.2, 0.1, 20ms);                                      // 创建VMC计算对象
    lf_z_vmc = std::make_shared<VMC>(200, 60, 4.0, 0.5, 0.2, 0.1, 10ms);
    rf_z_vmc = std::make_shared<VMC>(200, 60, 4.0, 0.5, 0.2, 0.1, 10ms);
    lf_z_vmc = std::make_shared<VMC>(200, 60, 4.0, 0.5, 0.2, 0.1, 10ms);
    rf_z_vmc = std::make_shared<VMC>(200, 60, 4.0, 0.5, 0.2, 0.1, 10ms);

    node_->declare_parameter("force_filter_gate", 0.8);
    node_->declare_parameter("enable_vmc", false);
    node_->declare_parameter("vmc_kp", 350.0);
    node_->declare_parameter("vmc_kd", 50.0);
    node_->declare_parameter("vmc_mass", 5.0);

    node_->declare_parameter("lf_grivate", 40.0);
    node_->declare_parameter("rf_grivate", 40.0);
    node_->declare_parameter("lb_grivate", 40.0);
    node_->declare_parameter("rb_grivate", 40.0);

    param_server_ = node_->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        RCLCPP_INFO(node_->get_logger(), "更新参数");
        for (const auto& param : params) {
            if (param.get_name() == "enable_vmc")
                enable_vmc = param.as_bool();
            else if (param.get_name() == "force_filter_gate")
                force_filter_gate = param.as_double();
            else if (param.get_name() == "vmc_kp")
                vmc->kp = param.as_double();
            else if (param.get_name() == "vmc_kd")
                vmc->kd = param.as_double();
            else if (param.get_name() == "vmc_mass")
                vmc->mass = param.as_double();
            else if (param.get_name() == "lf_grivate")
                robot_lf_grivate = param.as_double();
            else if (param.get_name() == "rf_grivate")
                robot_rf_grivate = param.as_double();
            else if (param.get_name() == "lb_grivate")
                robot_lb_grivate = param.as_double();
            else if (param.get_name() == "rb_grivate")
                robot_rb_grivate = param.as_double();
        }
        return result;
    });

    marker_publisher = node_->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

    rviz_joint_publisher = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    legs_target_pub = node_->create_publisher<robot_interfaces::msg::Robot>("legs_target", 10); // 创建期望位置发布者

    legs_state_sub = node_->create_subscription<robot_interfaces::msg::Robot>("legs_status", 10, [this](const robot_interfaces::msg::Robot& msg) {
        for (int i = 0; i < 3; i++) {
            lf_joint_pos[i] = (double)msg.legs[0].joints[i].rad;
            rf_joint_pos[i] = (double)msg.legs[1].joints[i].rad;
            lb_joint_pos[i] = (double)msg.legs[2].joints[i].rad;
            rb_joint_pos[i] = (double)msg.legs[3].joints[i].rad;

            lf_joint_vel[i] = (double)msg.legs[0].joints[i].omega;
            rf_joint_vel[i] = (double)msg.legs[1].joints[i].omega;
            lb_joint_vel[i] = (double)msg.legs[2].joints[i].omega;
            rb_joint_vel[i] = (double)msg.legs[3].joints[i].omega;

            lf_joint_torque[i] = (double)msg.legs[0].joints[i].torque;
            rf_joint_torque[i] = (double)msg.legs[1].joints[i].torque;
            lb_joint_torque[i] = (double)msg.legs[2].joints[i].torque;
            rb_joint_torque[i] = (double)msg.legs[3].joints[i].torque;
        }
    });

    robot_description_param_ = std::make_shared<rclcpp::SyncParametersClient>(node_, "/robot_state_publisher");

    auto params = robot_description_param_->get_parameters({"robot_description"});
    urdf_xml    = params[0].as_string();
    if (urdf_xml.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "无法读取URDF文件，不能进行动力学计算");
        return;
    }

    kdl_parser::treeFromString(urdf_xml, tree);                                                 // 解析四条腿的KDL树结构
    tree.getChain("body_link", "lf_link4", lf_leg_chain);
    tree.getChain("body_link", "rf_link4", rf_leg_chain);
    tree.getChain("body_link", "lb_link4", lb_leg_chain);
    tree.getChain("body_link", "rb_link4", rb_leg_chain);

    // 初始化狗腿解算器，定义足端中性点位置
    lf_leg_calc = std::make_shared<LegCalc>(lf_leg_chain);
    lf_leg_calc->pos_offset << 0.21, 0.16, -0.25;

    rf_leg_calc = std::make_shared<LegCalc>(rf_leg_chain);
    rf_leg_calc->pos_offset << 0.21, -0.16, -0.25;

    lb_leg_calc = std::make_shared<LegCalc>(lb_leg_chain);
    lb_leg_calc->pos_offset << -0.21, 0.16, -0.25;

    rb_leg_calc = std::make_shared<LegCalc>(rb_leg_chain);
    rb_leg_calc->pos_offset << -0.21, -0.16, -0.25;

    joint_display_msg.name = {"lf_joint1", "lf_joint2", "lf_joint3", "rf_joint1", "rf_joint2", "rf_joint3",
                              "lb_joint1", "lb_joint2", "lb_joint3", "rb_joint1", "rb_joint2", "rb_joint3"};
    joint_display_msg.position.resize(12);

    last_step1_reset_time = node_->get_clock()->now();
    last_step2_reset_time = node_->get_clock()->now();

    // ui_update_timer =
    //     node_->create_wall_timer(50ms, std::bind(&RobotCalcNode::show_callback, this));
    legs_update_timer = node_->create_wall_timer(10ms, std::bind(&RobotCalcNode::legs_update, this));

    RCLCPP_INFO(node_->get_logger(), "初始化完成");
    // UpdateCycloidStep(Eigen::Vector2d(0.1, 0.0), &step_line1, 2.0, 0.08); //
    // 首次启动先规划一次步态 UpdateCycloidStep(Eigen::Vector2d(0.0, 0.0), &step_line2, 2.0, 0.0);
    // //步态曲线2规划为静止 UpdateAirStepLine(const Vector3D &cur_pos, const Vector3D &cur_vel,
    // const Vector2D &exp_vel, StepTrajectory_t *line, float time, float step_height)
    UpdateGndStepLine(Vector3D(0.0, 0.0, 0.0), Vector2D(0.05, 0.0), &gnd_step_line, 2.0);
}

RobotCalcNode::~RobotCalcNode() { delete vmc; }

void RobotCalcNode::show_callback() {
    visualization_msgs::msg::Marker dot_marker;
    dot_marker.header.frame_id = "body_link"; // 设置坐标系
    dot_marker.header.stamp    = node_->get_clock()->now();
    dot_marker.ns              = "points";
    dot_marker.id              = 0;
    dot_marker.type            = visualization_msgs::msg::Marker::SPHERE;
    dot_marker.action          = visualization_msgs::msg::Marker::ADD;

    // dot_marker.pose.position.x = lf_leg_calc->pos_offset[0] + lf_cart_target[0];
    // dot_marker.pose.position.y = lf_leg_calc->pos_offset[1] + lf_cart_target[0];
    // dot_marker.pose.position.z = lf_leg_calc->pos_offset[2] + lf_cart_target[0];
    //  设置球体的尺寸
    dot_marker.scale.x = 0.1;
    dot_marker.scale.y = 0.1;
    dot_marker.scale.z = 0.1;
    // 设置颜色
    dot_marker.color.a = 1.0;              // 不透明
    dot_marker.color.r = 1.0;
    dot_marker.color.g = 0.0;
    dot_marker.color.b = 0.0;

    marker_publisher->publish(dot_marker); // 发布点标记（狗腿足端位置）

    // visualization_msgs::msg::Marker arraw_marker;
    // arraw_marker.header.frame_id = "body_link"; // 选择你在 TF 树中有的 frame
    // arraw_marker.header.stamp    = node_->get_clock()->now();
    // arraw_marker.ns              = "arrows";
    // arraw_marker.id              = 0;
    // // 类型：箭头
    // arraw_marker.type   = visualization_msgs::msg::Marker::ARROW;
    // arraw_marker.action = visualization_msgs::msg::Marker::ADD;
    // geometry_msgs::msg::Point p_start;
    // // p_start.x = foot_pos[0];
    // // p_start.y = foot_pos[1];
    // // p_start.z = foot_pos[2];

    // geometry_msgs::msg::Point p_end;
    // // p_end.x = p_start.x+foot_force[0]*0.05f;
    // // p_end.y = p_start.y+foot_force[1]*0.05f;
    // // p_end.z = p_start.z+foot_force[2]*0.05f;
    // // p_end.x = p_start.x+foot_force[0]*0.05;
    // // p_end.y = p_start.y+foot_force[1]*0.05;
    // // p_end.z = p_start.z+leg_virtual_force*0.05;

    // arraw_marker.points.push_back(p_start);
    // arraw_marker.points.push_back(p_end);

    // arraw_marker.scale.x = 0.03;
    // arraw_marker.scale.y = 0.03;
    // arraw_marker.scale.z = 0.03;
    // // 设置颜色
    // arraw_marker.color.a = 1.0; // 不透明
    // arraw_marker.color.r = 1.0;
    // arraw_marker.color.g = 1.0;
    // arraw_marker.color.b = 0.0;

    // arraw_marker.lifetime = rclcpp::Duration(0, 0);

    // // marker_publisher->publish(arraw_marker);    //发布箭头标记（狗腿足端受力）*/

    // RCLCPP_INFO(node_->get_logger(), "kp=%lf,kd=%lf,mass=%lf", vmc->kp, vmc->kd, vmc->mass);
}

std::tuple<Vector3D, Vector3D, Vector3D> signal_leg_calc(
    const Vector3D& exp_cart_pos, const Vector3D& exp_cart_vel, const Vector3D& exp_cart_acc, const Vector3D& exp_cart_force,
    std::shared_ptr<LegCalc> leg_calc) {
    Vector3D joint_pos, joint_omega, joint_torque;

    (void)exp_cart_acc;                                                               // 目前还没有实现将笛卡尔的加速度转为关节空间的加速度，所以这个参数先不用

    int result;
    joint_pos    = leg_calc->joint_pos(exp_cart_pos, &result);                        // 一般这个位置不可能会迭代失败，所以不再对result进行处理
    joint_omega  = leg_calc->joint_vel(joint_pos, exp_cart_vel);
    joint_torque = leg_calc->joint_torque_foot_force(joint_pos, exp_cart_force);
    joint_torque += leg_calc->joint_torque_dynamic(joint_pos, joint_omega, Vector3D(0.0, 0.0, 0.0));
    return std::make_tuple(joint_pos, joint_omega, joint_torque);
}

void RobotCalcNode::legs_update() {

    if (node_->get_clock()->now() - last_step1_reset_time > rclcpp::Duration(1, 0)) { // 足端轨迹更新
        last_step1_reset_time = node_->get_clock()->now();
        if (last_switch)                                                              // 规划并执行支撑步态
        {
            last_switch = false;
            RCLCPP_INFO(node_->get_logger(), "规划支撑相");
            UpdateGndStepLine(Vector3D(0.025, 0.0, 0.0), Vector2D(0.05, 0.0), &gnd_step_line, 1.0);
        } else {                                                                      // 规划并执行摆动步态
            last_switch = true;
            RCLCPP_INFO(node_->get_logger(), "规划摆动相");
            UpdateAirStepLine(Vector3D(-0.025, 0.0, 0.0), Vector3D(-0.1, 0.0, 0.0), Vector2D(0.05, 0.0), &air_step_line, 1.0f, 0.07f);
        }
    }

    // 足端位置解算
    std::tuple<Vector3D, Vector3D, Vector3D> target1;
    double now_s = (node_->get_clock()->now() - last_step1_reset_time).seconds();
    if (last_switch) {
        target1 = GetQuinticStep(air_step_line, now_s);
    } else {
        target1 = GetSupportStep(gnd_step_line, now_s);
    }

    RCLCPP_INFO(node_->get_logger(), "当前期望坐标:(%lf,%lf,%lf)", std::get<0>(target1)[0], std::get<0>(target1)[1], std::get<0>(target1)[2]);
    // TODO:计算关节空间期望
    int result               = 0;
    auto lf_joint_target_pos = lf_leg_calc->joint_pos(std::get<0>(target1), &result);
    // auto rb_joint_target_pos=rb_leg_calc->joint_pos(lfrb_cart_target, &result);
    // auto rf_joint_target_pos = rf_leg_calc->joint_pos(rflb_cart_target, &result);
    // auto lb_joint_target_pos=lb_leg_calc->joint_pos(rflb_cart_target, &result);

    // TODO:将解算结果发往下位机
    robot_interfaces::msg::Robot robot_msg;
    robot_msg.legs[0].joints[0].rad = (float)lf_joint_target_pos[0];
    robot_msg.legs[0].joints[1].rad = (float)lf_joint_target_pos[1];
    robot_msg.legs[0].joints[2].rad = (float)lf_joint_target_pos[2];

    // robot_msg.legs[0].joints[1].rad=0.9f;
    robot_msg.legs[1].joints[1].rad = -4.97f / 6.33;
    robot_msg.legs[2].joints[1].rad = 4.97f / 6.33;
    robot_msg.legs[3].joints[1].rad = -4.97f / 6.33;

    legs_target_pub->publish(robot_msg);

    // TODO:写入目标并发布，通过RVIZ2可视化
    joint_display_msg.position[0] = lf_joint_target_pos[0];
    joint_display_msg.position[1] = lf_joint_target_pos[1];
    joint_display_msg.position[2] = lf_joint_target_pos[2];
    // joint_msg.position[3] = rf_joint_target_pos[0];
    // joint_msg.position[4] = rf_joint_target_pos[1];
    // joint_msg.position[5] = rf_joint_target_pos[2];
    // joint_msg.position[6] = lb_joint_target_pos[0];
    // joint_msg.position[7] = lb_joint_target_pos[1];
    // joint_msg.position[8] = lb_joint_target_pos[2];
    // joint_msg.position[9] = rb_joint_target_pos[0];
    // joint_msg.position[10] = rb_joint_target_pos[1];
    // joint_msg.position[11] = rb_joint_target_pos[2];

    // RCLCPP_INFO(node_->get_logger(),"完成解算");

    joint_display_msg.header.stamp = node_->get_clock()->now();
    rviz_joint_publisher->publish(joint_display_msg);
}

void RobotCalcNode::legs_update2() {
    auto lf_foot_exp_pos   = Vector3D(0.0, 0.0, 0.0);
    auto lf_foot_exp_vel   = Vector3D(0.0, 0.0, 0.0);
    auto lf_foot_exp_force = Vector3D(0.0, 0.0, 0.0);

    auto rf_foot_exp_pos   = Vector3D(0.0, 0.0, 0.0);
    auto rf_foot_exp_vel   = Vector3D(0.0, 0.0, 0.0);
    auto rf_foot_exp_force = Vector3D(0.0, 0.0, 0.0);

    auto lb_foot_exp_pos   = Vector3D(0.0, 0.0, 0.0);
    auto lb_foot_exp_vel   = Vector3D(0.0, 0.0, 0.0);
    auto lb_foot_exp_force = Vector3D(0.0, 0.0, 0.0);

    auto rb_foot_exp_pos   = Vector3D(0.0, 0.0, 0.0);
    auto rb_foot_exp_vel   = Vector3D(0.0, 0.0, 0.0);
    auto rb_foot_exp_force = Vector3D(0.0, 0.0, 0.0);

    if (robot_state == DOG_STOP) {
        auto lf_cart_pos   = lf_leg_calc->foot_pos(lf_joint_pos);
        auto lf_cart_vel   = lf_leg_calc->foot_vel(lf_joint_pos, lf_joint_vel);
        auto lf_cart_force = lf_leg_calc->foot_force(lf_joint_pos, lf_joint_torque, lf_forward_torque);
        double lf_foot_acc; // ACC计算目前暂时还不可用
        std::tie(lf_foot_exp_pos[2], lf_foot_exp_vel[2], lf_foot_acc) = lf_z_vmc->targetUpdate(0.0, lf_cart_pos[2], 0.0, lf_cart_vel[2], -lf_cart_force[2]);
        lf_foot_exp_force = Vector3D(0.0, 0.0, -robot_lf_grivate);


        auto rf_cart_pos   = rf_leg_calc->foot_pos(rf_joint_pos);
        auto rf_cart_vel   = rf_leg_calc->foot_vel(rf_joint_pos, rf_joint_vel);
        auto rf_cart_force = rf_leg_calc->foot_force(rf_joint_pos, rf_joint_torque, rf_forward_torque);
        double rf_foot_acc;
        std::tie(rf_foot_exp_pos[2], rf_foot_exp_vel[2], rf_foot_acc) = rf_z_vmc->targetUpdate(0.0, rf_cart_pos[2], 0.0, rf_cart_vel[2], -rf_cart_force[2]);
        rf_foot_exp_force = Vector3D(0.0, 0.0, -robot_rf_grivate);

        auto lb_cart_pos   = lb_leg_calc->foot_pos(lb_joint_pos);
        auto lb_cart_vel   = lb_leg_calc->foot_vel(lb_joint_pos, lb_joint_vel);
        auto lb_cart_force = lb_leg_calc->foot_force(lb_joint_pos, lb_joint_torque, lb_forward_torque);
        double lb_foot_acc;
        std::tie(lb_foot_exp_pos[2], lb_foot_exp_vel[2], lb_foot_acc) = lb_z_vmc->targetUpdate(0.0, lb_cart_pos[2], 0.0, lb_cart_vel[2], -lb_cart_force[2]);
        lb_foot_exp_force = Vector3D(0.0, 0.0, -robot_lb_grivate);

        auto rb_cart_pos   = rb_leg_calc->foot_pos(rb_joint_pos);
        auto rb_cart_vel   = rb_leg_calc->foot_vel(rb_joint_pos, rb_joint_vel);
        auto rb_cart_force = rb_leg_calc->foot_force(rb_joint_pos, rb_joint_torque, rb_forward_torque);
        double rb_foot_acc;
        std::tie(rb_foot_exp_pos[2], rb_foot_exp_vel[2], rb_foot_acc) = rb_z_vmc->targetUpdate(0.0, rb_cart_pos[2], 0.0, rb_cart_vel[2], -rb_cart_force[2]);
        rb_foot_exp_force = Vector3D(0.0, 0.0, -robot_rb_grivate);
    }

    auto lf_leg_joints_target = signal_leg_calc(lf_foot_exp_pos, lf_foot_exp_vel, Vector3D(0.0, 0.0, 0.0), lf_foot_exp_force, lf_leg_calc);
    auto rf_leg_joints_target = signal_leg_calc(rf_foot_exp_pos, rf_foot_exp_vel, Vector3D(0.0, 0.0, 0.0), rf_foot_exp_force, rf_leg_calc);
    auto lb_leg_joints_target = signal_leg_calc(lb_foot_exp_pos, lb_foot_exp_vel, Vector3D(0.0, 0.0, 0.0), lb_foot_exp_force, lb_leg_calc);
    auto rb_leg_joints_target = signal_leg_calc(rb_foot_exp_pos, rb_foot_exp_vel, Vector3D(0.0, 0.0, 0.0), rb_foot_exp_force, rb_leg_calc);

    lf_forward_torque = std::get<2>(lf_leg_joints_target);
    rf_forward_torque = std::get<2>(rf_leg_joints_target);
    lb_forward_torque = std::get<2>(lb_leg_joints_target);
    rb_forward_torque = std::get<2>(rb_leg_joints_target);

    // 填写关节目标并发布
    robot_interfaces::msg::Robot joints_target;
    for (int i = 0; i < 3; i++) {
        joints_target.legs[0].joints[i].rad    = (float)std::get<0>(lf_leg_joints_target)[i];
        joints_target.legs[0].joints[i].omega  = (float)std::get<1>(lf_leg_joints_target)[i];
        joints_target.legs[0].joints[i].torque = (float)std::get<2>(lf_leg_joints_target)[i];

        joints_target.legs[1].joints[i].rad    = (float)std::get<0>(rf_leg_joints_target)[i];
        joints_target.legs[1].joints[i].omega  = (float)std::get<1>(rf_leg_joints_target)[i];
        joints_target.legs[1].joints[i].torque = (float)std::get<2>(rf_leg_joints_target)[i];

        joints_target.legs[2].joints[i].rad    = (float)std::get<0>(lb_leg_joints_target)[i];
        joints_target.legs[2].joints[i].omega  = (float)std::get<1>(lb_leg_joints_target)[i];
        joints_target.legs[2].joints[i].torque = (float)std::get<2>(lb_leg_joints_target)[i];

        joints_target.legs[3].joints[i].rad    = (float)std::get<0>(rb_leg_joints_target)[i];
        joints_target.legs[3].joints[i].omega  = (float)std::get<1>(rb_leg_joints_target)[i];
        joints_target.legs[3].joints[i].torque = (float)std::get<2>(rb_leg_joints_target)[i];
    }
    legs_target_pub->publish(joints_target);
}

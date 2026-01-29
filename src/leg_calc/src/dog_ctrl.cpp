#include "dog_calc.hpp"
#include "step.h"
#include "vmc.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/AngleAxis.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <algorithm>
#include <chrono>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <kdl/frames.hpp>
#include <rclcpp/duration.hpp>
#include <robot_interfaces/msg/detail/robot__struct.hpp>
#include <robot_interfaces/msg/robot.hpp>
#include <sensor_msgs/msg/detail/imu__struct.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tuple>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std::chrono_literals;

RobotCalcNode::RobotCalcNode(const rclcpp::Node::SharedPtr node) {
    node_    = node;
    lf_z_vmc = std::make_shared<SimpleVMC>(500, 120, 40.0);
    rf_z_vmc = std::make_shared<SimpleVMC>(500, 120, 40.0);
    lb_z_vmc = std::make_shared<SimpleVMC>(500, 120, 40.0);
    rb_z_vmc = std::make_shared<SimpleVMC>(500, 120, 40.0);

    lf_x_vmc = std::make_shared<SimpleVMC>(160, 60, 20.0);
    lf_y_vmc = std::make_shared<SimpleVMC>(160, 60, 20.0);
    rf_x_vmc = std::make_shared<SimpleVMC>(160, 60, 20.0);
    rf_y_vmc = std::make_shared<SimpleVMC>(160, 60, 20.0);
    lb_x_vmc = std::make_shared<SimpleVMC>(160, 60, 20.0);
    lb_y_vmc = std::make_shared<SimpleVMC>(160, 60, 20.0);
    rb_x_vmc = std::make_shared<SimpleVMC>(160, 60, 20.0);
    rb_y_vmc = std::make_shared<SimpleVMC>(160, 60, 20.0);

    // 狗身平衡VMC
    roll_vmc  = std::make_shared<SimpleVMC>(-200.0, 0.0, 100);
    pitch_vmc = std::make_shared<SimpleVMC>(500.0, 100.0, 100);

    node_->declare_parameter("direction_filter_gate", 0.2);
    node_->declare_parameter("vertical_vmc_kp", 200.0);
    node_->declare_parameter("vertical_vmc_kd", 13.0);

    node_->declare_parameter("horizontal_vmc_kp", 200.0);
    node_->declare_parameter("horizontal_vmc_kd", 2.0);

    node_->declare_parameter("roll_vmc_kp", -60.0);
    node_->declare_parameter("roll_vmc_kd", -4.0);
    node_->declare_parameter("pitch_vmc_kp", 100.0);
    node_->declare_parameter("pitch_vmc_kd", 5.0);
    node_->declare_parameter("roll_balance_force_compen", 1.0);
    node_->declare_parameter("pitch_balance_force_compen", 1.0);

    node_->declare_parameter("lf_dx", 0.0);
    node_->declare_parameter("rf_dx", 0.0);
    node_->declare_parameter("lb_dx", 0.0);
    node_->declare_parameter("rb_dx", 0.0);

    node_->declare_parameter("step_support_rate", 0.6); // 支撑相时间
    node_->declare_parameter("step_time", 0.6);         // 一个完整步态时间（0.7共振，0.8太慢容易失衡，最好0.6）
    node_->declare_parameter("step_height", 0.14);

    node_->declare_parameter("target_roll", 0.0);       // 狗子期望的当前俯仰角
    node_->declare_parameter("target_pitch", 0.0);
    node_->declare_parameter("body_height", 0.21);

    node_->declare_parameter("joint1_kp", 50.0);
    node_->declare_parameter("joint1_kd", 3.0);
    node_->declare_parameter("joint2_kp", 50.0);
    node_->declare_parameter("joint2_kd", 3.0);
    node_->declare_parameter("joint3_kp", 50.0);
    node_->declare_parameter("joint3_kd", 3.0);


    param_server_ = node_->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        RCLCPP_INFO(node_->get_logger(), "更新参数");
        std::string name;
        for (const auto& param : params) {
            name = param.get_name();
            if (name == "horizontal_vmc_kp") {
                double value = param.as_double();
                lf_x_vmc->kp = value;
                lf_y_vmc->kp = value;
                rf_x_vmc->kp = value;
                rf_y_vmc->kp = value;
                lb_x_vmc->kp = value;
                lb_y_vmc->kp = value;
                rb_x_vmc->kp = value;
                rb_y_vmc->kp = value;
            } else if (name == "horizontal_vmc_kd") {
                double value = param.as_double();
                lf_x_vmc->kd = value;
                lf_y_vmc->kd = value;
                rf_x_vmc->kd = value;
                rf_y_vmc->kd = value;
                lb_x_vmc->kd = value;
                lb_y_vmc->kd = value;
                rb_x_vmc->kd = value;
                rb_y_vmc->kd = value;
            } else if (name == "roll_vmc_kp") {
                roll_vmc->kp = param.as_double();
            } else if (name == "roll_vmc_kd") {
                roll_vmc->kd = param.as_double();
            } else if (name == "roll_balance_force_compen") {
                roll_balance_force_compen = param.as_double();
            } else if (name == "pitch_balance_force_compen") {
                pitch_balance_force_compen = param.as_double();
            } else if (name == "pitch_vmc_kp") {
                pitch_vmc->kp = param.as_double();
            } else if (name == "pitch_vmc_kd") {
                pitch_vmc->kd = param.as_double();
            } else if (name == "direction_filter_gate") {
                direction_filter_gate = param.as_double();
                direction_filter_gate = std::clamp(direction_filter_gate, 0.0, 1.0);
            }

            else if (name == "vertical_vmc_kp") {
                lf_z_vmc->kp = param.as_double();
                rf_z_vmc->kp = param.as_double();
                lb_z_vmc->kp = param.as_double();
                rb_z_vmc->kp = param.as_double();
            } else if (name == "vertical_vmc_kd") {
                lf_z_vmc->kd = param.as_double();
                rf_z_vmc->kd = param.as_double();
                lb_z_vmc->kd = param.as_double();
                rb_z_vmc->kd = param.as_double();
            }else if(name=="joint1_kp"){
                stand_joints_kp[0]=param.as_double();
            }else if(name=="joint1_kd"){
                stand_joints_kd[0]=param.as_double();
            }else if(name=="joint2_kp"){
                stand_joints_kp[1]=param.as_double();
            }else if(name=="joint2_kd"){
                stand_joints_kd[1]=param.as_double();
            }else if(name=="joint3_kp"){
                stand_joints_kp[2]=param.as_double();
            }else if(name=="joint3_kd"){
                stand_joints_kd[2]=param.as_double();
            }else if (name == "lf_dx")
                lf_base_offset[0] = 0.25 + param.as_double();
            else if (name == "rf_dx")
                rf_base_offset[0] = 0.25 + param.as_double();
            else if (name == "lb_dx")
                lb_base_offset[0] = -0.23 + param.as_double();
            else if (name == "rb_dx")
                rb_base_offset[0] = -0.23 + param.as_double();
            else if (name == "step_support_rate")
                step_support_rate = param.as_double();
            else if (name == "step_time")
                step_time = param.as_double();
            else if (name == "step_height")
                step_height = param.as_double();
            else if (name == "body_height") {
                double temp = param.as_double();
                if (temp < 0.32 && temp > 0.1)
                    body_height = temp;
                else {
                    result.successful = false;
                    result.reason     = "狗身期望高度过大或过小";
                }
            }
        }
        return result;
    });

    // 读取所有默认参数
    // ========== direction filter ==========
    direction_filter_gate = node_->get_parameter("direction_filter_gate").as_double();
    direction_filter_gate = std::clamp(direction_filter_gate, 0.0, 1.0);

    // ========== vertical VMC (Z) ==========
    lf_z_vmc->kp = node_->get_parameter("vertical_vmc_kp").as_double();
    rf_z_vmc->kp = lb_z_vmc->kp = rb_z_vmc->kp = lf_z_vmc->kp;

    lf_z_vmc->kd = node_->get_parameter("vertical_vmc_kd").as_double();
    rf_z_vmc->kd = lb_z_vmc->kd = rb_z_vmc->kd = lf_z_vmc->kd;

    // ========== horizontal VMC (X/Y) ==========
    double horizontal_kp, horizontal_kd;
    horizontal_kp = node_->get_parameter("horizontal_vmc_kp").as_double();
    horizontal_kd = node_->get_parameter("horizontal_vmc_kd").as_double();

    lf_x_vmc->kp = lf_y_vmc->kp = rf_x_vmc->kp = rf_y_vmc->kp = lb_x_vmc->kp = lb_y_vmc->kp = rb_x_vmc->kp = rb_y_vmc->kp = horizontal_kp;
    lf_x_vmc->kd = lf_y_vmc->kd = rf_x_vmc->kd = rf_y_vmc->kd = lb_x_vmc->kd = lb_y_vmc->kd = rb_x_vmc->kd = rb_y_vmc->kd = horizontal_kd;

    // ========== roll / pitch VMC ==========
    roll_vmc->kp = node_->get_parameter("roll_vmc_kp").as_double();
    roll_vmc->kd = node_->get_parameter("roll_vmc_kd").as_double();
    pitch_vmc->kp = node_->get_parameter("pitch_vmc_kp").as_double();
    pitch_vmc->kd = node_->get_parameter("pitch_vmc_kd").as_double();

    // ========== foot x offset ==========

    lf_base_offset[0] = 0.25 + node_->get_parameter("lf_dx").as_double();
    rf_base_offset[0] = 0.25 + node_->get_parameter("rf_dx").as_double();
    lb_base_offset[0] = -0.25 + node_->get_parameter("lb_dx").as_double();
    rb_base_offset[0] = -0.25 + node_->get_parameter("rb_dx").as_double();

    // ========== gait ==========
    step_support_rate = node_->get_parameter("step_support_rate").as_double();
    step_time = node_->get_parameter("step_time").as_double();
    step_height = node_->get_parameter("step_height").as_double();

    // ========== body height (FIXED BUG) ==========
    body_height = node_->get_parameter("body_height").as_double();

    stand_joints_kp[0]=node_->get_parameter("joint1_kp").as_double();
    stand_joints_kp[1]=node_->get_parameter("joint2_kp").as_double();
    stand_joints_kp[2]=node_->get_parameter("joint3_kp").as_double();
    stand_joints_kd[0]=node_->get_parameter("joint1_kd").as_double();
    stand_joints_kd[1]=node_->get_parameter("joint2_kd").as_double();
    stand_joints_kd[2]=node_->get_parameter("joint3_kd").as_double();


    robot_rotation.setRPY(0.0, 0.0, 0.0);

    marker_publisher = node_->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);

    rviz_joint_publisher = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    legs_target_pub = node_->create_publisher<robot_interfaces::msg::Robot>("legs_target", 10); // 创建期望位置发布者

    // pose_sensor 发布的是 PoseStamped（见 mujoco_ros2_control/src/pose_sensor.cpp），这里必须用 PoseStamped 才能收到消息
    // 同时 QoS 需要与发布端兼容（发布端当前是 RELIABLE + TRANSIENT_LOCAL）
    imu_sub = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/imu_pose_sensor/pose", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local(),
        [this](const geometry_msgs::msg::PoseStamped& msg) {
            const auto& q_msg = msg.pose.orientation;

            double qw = robot_rotation.getW();
            double qx = robot_rotation.getX();
            double qy = robot_rotation.getY();
            double qz = robot_rotation.getZ();

            quaternionLowPassFilter(qw, qx, qy, qz, q_msg.w, q_msg.x, q_msg.y, q_msg.z, direction_filter_gate); //(0-强滤波、1-不滤波)

            robot_rotation.setW(qw);
            robot_rotation.setX(qx);
            robot_rotation.setY(qy);
            robot_rotation.setZ(qz);
        });

    imu_angular_vel_sub = node_->create_subscription<geometry_msgs::msg::Vector3>(
        "/imu_imu_sensor/imu", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local(),
        [this](const geometry_msgs::msg::Vector3& msg) {
            robot_velocity.angular.x = robot_velocity.angular.x + direction_filter_gate * (msg.x - robot_velocity.angular.x);
            robot_velocity.angular.y = robot_velocity.angular.y + direction_filter_gate * (msg.y - robot_velocity.angular.y);
            robot_velocity.angular.z = robot_velocity.angular.z + direction_filter_gate * (msg.z - robot_velocity.angular.z);
        });
    legs_state_sub =
        node_->create_subscription<robot_interfaces::msg::Robot>("legs_status", 10, [this](const robot_interfaces::msg::Robot& msg) {
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

    // 订阅机器人的运动期望
    move_cmd_sub =
        node_->create_subscription<robot_interfaces::msg::MoveCmd>("control/robot_move_cmd", 10, [this](const robot_interfaces::msg::MoveCmd& msg) {
            if (msg.step_mode == DOG_REQ_STOP) { // 请求状态为停止
                robot_req_state = DOG_REQ_STOP;
            }else if (msg.step_mode == DOG_REQ_IDEL) {
                robot_req_state = DOG_REQ_IDEL;
            } else if (msg.step_mode == DOG_REQ_RUN) {
                robot_req_state = DOG_REQ_RUN;
                Vector3D v_body(msg.vx, msg.vy, 0.0);
                Vector3D omega(0.0, 0.0, msg.vz);

                // LF
                Vector3D v_lf = v_body + omega.cross(lf_leg_calc->pos_offset);
                lf_exp_vel    = Vector2D(v_lf[0], v_lf[1]);

                // RF
                Vector3D v_rf = v_body + omega.cross(rf_leg_calc->pos_offset);
                rf_exp_vel    = Vector2D(v_rf[0], v_rf[1]);

                // LB
                Vector3D v_lb = v_body + omega.cross(lb_leg_calc->pos_offset);
                lb_exp_vel    = Vector2D(v_lb[0], v_lb[1]);

                // RB
                Vector3D v_rb = v_body + omega.cross(rb_leg_calc->pos_offset);
                rb_exp_vel    = Vector2D(v_rb[0], v_rb[1]);
            }
            //RCLCPP_INFO(node_->get_logger(), "接收到期望更新消息:lf:(%lf,%lf),type=%d", lf_exp_vel[0], lf_exp_vel[1], msg.step_mode);
        });

    robot_description_param_ = std::make_shared<rclcpp::SyncParametersClient>(node_, "/robot_state_publisher");

    auto params          = robot_description_param_->get_parameters({"robot_description"});
    std::string urdf_xml = params[0].as_string();
    if (urdf_xml.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "无法读取URDF文件，不能进行动力学计算");
        return;
    }

    robot_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

    kdl_parser::treeFromString(urdf_xml, tree); // 解析四条腿的KDL树结构
    tree.getChain("body_link", "lf_link4", lf_leg_chain);
    tree.getChain("body_link", "rf_link4", rf_leg_chain);
    tree.getChain("body_link", "lb_link4", lb_leg_chain);
    tree.getChain("body_link", "rb_link4", rb_leg_chain);

    // 初始化狗腿解算器，定义足端中性点位置
    lf_leg_calc = std::make_shared<LegCalc>(lf_leg_chain);
    lf_base_offset << 0.25, 0.18, -body_height;

    rf_leg_calc = std::make_shared<LegCalc>(rf_leg_chain);
    rf_base_offset << 0.25, -0.18, -body_height;

    lb_leg_calc = std::make_shared<LegCalc>(lb_leg_chain);
    lb_base_offset << -0.23, 0.18, -body_height;

    rb_leg_calc = std::make_shared<LegCalc>(rb_leg_chain);
    rb_base_offset << -0.23, -0.18, -body_height;


    lf_leg_calc->pos_offset = lf_base_offset;
    rf_leg_calc->pos_offset = rf_base_offset;
    lb_leg_calc->pos_offset = lb_base_offset;
    rb_leg_calc->pos_offset = rb_base_offset;


    joint_display_msg.name = {"lf_joint1", "lf_joint2", "lf_joint3", "rf_joint1", "rf_joint2", "rf_joint3",
                              "lb_joint1", "lb_joint2", "lb_joint3", "rb_joint1", "rb_joint2", "rb_joint3"};
    joint_display_msg.position.resize(12);

    comm_pos =
        get_grivate_center_pose(Vector3D(0.0, -0.78, 0.0), Vector3D(0.0, 0.78, 0.0), Vector3D(0.0, -0.78, 0.0), Vector3D(0.0, 0.78, 0.0));


    ui_update_timer   = node_->create_wall_timer(50ms, std::bind(&RobotCalcNode::show_callback, this));
    legs_update_timer = node_->create_wall_timer(10ms, std::bind(&RobotCalcNode::legs_update, this));


    RCLCPP_INFO(node_->get_logger(), "初始化完成");
}

RobotCalcNode::~RobotCalcNode() {}

void RobotCalcNode::show_callback() {

    visualization_msgs::msg::Marker com_marker;
    com_marker.header.frame_id = "body_link";
    com_marker.header.stamp    = node_->get_clock()->now();
    com_marker.ns              = "center_of_mass";
    com_marker.id              = 1;
    com_marker.type            = visualization_msgs::msg::Marker::SPHERE;
    com_marker.action          = visualization_msgs::msg::Marker::ADD;

    // 设置质心位置
    com_marker.pose.position.x    = comm_pos[0];
    com_marker.pose.position.y    = comm_pos[1];
    com_marker.pose.position.z    = comm_pos[2];
    com_marker.pose.orientation.x = 0.0;
    com_marker.pose.orientation.y = 0.0;
    com_marker.pose.orientation.z = 0.0;
    com_marker.pose.orientation.w = 1.0;

    // 设置球体尺寸
    com_marker.scale.x = 0.08;
    com_marker.scale.y = 0.08;
    com_marker.scale.z = 0.08;

    // 设置颜色 - 红色表示质心
    com_marker.color.a = 1.0;
    com_marker.color.r = 1.0;
    com_marker.color.g = 0.0;
    com_marker.color.b = 0.0;

    com_marker.lifetime = rclcpp::Duration(0, 0);


    const auto now = node_->get_clock()->now();

    visualization_msgs::msg::MarkerArray mark_array;

    auto make_force_arrow = [&](int id, const Vector3D& foot_pos, const Vector3D& pos_offset, const Vector3D& foot_force) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "body_link";
        m.header.stamp    = now;
        m.ns              = "foot_force";
        m.id              = id;
        m.type            = visualization_msgs::msg::Marker::ARROW;
        m.action          = visualization_msgs::msg::Marker::ADD;

        m.pose.orientation.w = 1.0;

        m.scale.x = 0.02;
        m.scale.y = 0.04;
        m.scale.z = 0.06;


        geometry_msgs::msg::Point p_start, p_end;
        p_start.x = foot_pos[0] + pos_offset[0];
        p_start.y = foot_pos[1] + pos_offset[1];
        p_start.z = foot_pos[2] + pos_offset[2];

        p_end.x = p_start.x + foot_force[0] * 0.05;
        p_end.y = p_start.y + foot_force[1] * 0.05;
        p_end.z = p_start.z + foot_force[2] * 0.05;

        m.points.clear();
        m.points.push_back(p_start);
        m.points.push_back(p_end);

        m.color.a = 1.0;
        m.color.r = 1.0;
        m.color.g = 1.0;
        m.color.b = 0.0;

        m.lifetime = rclcpp::Duration(0, 0);

        mark_array.markers.push_back(m);
    };

    // LF
    make_force_arrow(
        0, lf_leg_calc->foot_pos(lf_joint_pos), lf_leg_calc->pos_offset,
        lf_leg_calc->foot_force(lf_joint_pos, lf_joint_torque));

    // RF
    make_force_arrow(
        1, rf_leg_calc->foot_pos(rf_joint_pos), rf_leg_calc->pos_offset,
        rf_leg_calc->foot_force(rf_joint_pos, rf_joint_torque));

    // LB
    make_force_arrow(
        2, lb_leg_calc->foot_pos(lb_joint_pos), lb_leg_calc->pos_offset,
        lb_leg_calc->foot_force(lb_joint_pos, lb_joint_torque));

    // RB
    make_force_arrow(
        3, rb_leg_calc->foot_pos(rb_joint_pos), rb_leg_calc->pos_offset,
        rb_leg_calc->foot_force(rb_joint_pos, rb_joint_torque));

    mark_array.markers.push_back(com_marker);

    marker_publisher->publish(mark_array);
}

std::tuple<Vector3D, Vector3D, Vector3D> RobotCalcNode::signal_leg_calc(
    const Vector3D& exp_cart_pos, const Vector3D& exp_cart_vel, const Vector3D& exp_cart_acc, const Vector3D& exp_cart_force,
    std::shared_ptr<LegCalc> leg_calc) {
    Vector3D joint_pos, joint_omega, joint_torque;

    int result;
    joint_pos    = leg_calc->joint_pos(exp_cart_pos, &result); // 一般这个位置不可能会迭代失败，所以不再对result进行处理
    joint_omega  = leg_calc->joint_vel(joint_pos, exp_cart_vel);
    joint_torque = leg_calc->joint_torque_foot_force(joint_pos, exp_cart_force);
    joint_torque += leg_calc->joint_torque_dynamic(joint_pos, joint_omega, exp_cart_acc);
    return std::make_tuple(joint_pos, joint_omega, joint_torque);
}


void RobotCalcNode::legs_update() {
    // auto lf_foot_exp_pos   = Vector3D(0.0, 0.0, 0.0);
    // auto lf_foot_exp_vel   = Vector3D(0.0, 0.0, 0.0);
    // auto lf_foot_exp_acc   = Vector3D(0.0, 0.0, 0.0);
    // auto lf_foot_exp_force = Vector3D(0.0, 0.0, 0.0);

    // auto rf_foot_exp_pos   = Vector3D(0.0, 0.0, 0.0);
    // auto rf_foot_exp_vel   = Vector3D(0.0, 0.0, 0.0);
    // auto rf_foot_exp_acc   = Vector3D(0.0, 0.0, 0.0);
    // auto rf_foot_exp_force = Vector3D(0.0, 0.0, 0.0);

    // auto lb_foot_exp_pos   = Vector3D(0.0, 0.0, 0.0);
    // auto lb_foot_exp_vel   = Vector3D(0.0, 0.0, 0.0);
    // auto lb_foot_exp_acc   = Vector3D(0.0, 0.0, 0.0);
    // auto lb_foot_exp_force = Vector3D(0.0, 0.0, 0.0);

    // auto rb_foot_exp_pos   = Vector3D(0.0, 0.0, 0.0);
    // auto rb_foot_exp_vel   = Vector3D(0.0, 0.0, 0.0);
    // auto rb_foot_exp_acc   = Vector3D(0.0, 0.0, 0.0);
    // auto rb_foot_exp_force = Vector3D(0.0, 0.0, 0.0);



    double cur_roll, cur_pitch, cur_yaw;
    tf2::Matrix3x3(robot_rotation).getRPY(cur_roll, cur_pitch, cur_yaw);

    roll_offset_virtual_torque  = roll_vmc->update(cur_roll, robot_velocity.angular.x);
    pitch_offset_virtual_torque = pitch_vmc->update(cur_pitch, robot_velocity.angular.y);
    // RCLCPP_INFO(node_->get_logger(),"roll:%lf,pitch:%lf",cur_roll,cur_pitch);

    // TODO:计算四个足端的期望的平衡虚拟力(pitch)
    //  lf_foot_exp_force[2] += pitch_offset_virtual_torque * lf_leg_calc->pos_offset[0];
    //  rf_foot_exp_force[2] += pitch_offset_virtual_torque * rf_leg_calc->pos_offset[0];
    //  lb_foot_exp_force[2] += pitch_offset_virtual_torque * lb_leg_calc->pos_offset[0];
    //  rb_foot_exp_force[2] += pitch_offset_virtual_torque * rb_leg_calc->pos_offset[0];

    // lf_foot_exp_force[0]+= pitch_offset_virtual_torque*std::sin(cur_pitch)*pitch_balance_force_compen;
    // rf_foot_exp_force[0]+= pitch_offset_virtual_torque*std::sin(cur_pitch)*pitch_balance_force_compen;
    // lb_foot_exp_force[0]+= pitch_offset_virtual_torque*std::sin(cur_pitch)*pitch_balance_force_compen;
    // rb_foot_exp_force[0]+= pitch_offset_virtual_torque*std::sin(cur_pitch)*pitch_balance_force_compen;

    // // TODO:计算四个足端的期望的平衡虚拟力(roll)
    // lf_foot_exp_force[2] += roll_offset_virtual_torque * lf_leg_calc->pos_offset[1];
    // rf_foot_exp_force[2] += roll_offset_virtual_torque * rf_leg_calc->pos_offset[1];
    // lb_foot_exp_force[2] += roll_offset_virtual_torque * lb_leg_calc->pos_offset[1];
    // rb_foot_exp_force[2] += roll_offset_virtual_torque * rb_leg_calc->pos_offset[1];

    // lf_foot_exp_force[1]+= pitch_offset_virtual_torque*std::sin(cur_roll)*roll_balance_force_compen;
    // rf_foot_exp_force[1]+= pitch_offset_virtual_torque*std::sin(cur_roll)*roll_balance_force_compen;
    // lb_foot_exp_force[1]+= pitch_offset_virtual_torque*std::sin(cur_roll)*roll_balance_force_compen;
    // rb_foot_exp_force[1]+= pitch_offset_virtual_torque*std::sin(cur_roll)*roll_balance_force_compen;

    lf_leg_calc->pos_offset = lf_base_offset;
    rf_leg_calc->pos_offset = rf_base_offset;
    lb_leg_calc->pos_offset = lb_base_offset;
    rb_leg_calc->pos_offset = rb_base_offset;
    
    robot_interfaces::msg::Robot joints_target;
    if (robot_state == DOG_IDEL)                  // 单位置控制
    {
        if (robot_req_state == DOG_REQ_STOP)
            robot_state = DOG_STOP;

        int result = 0;
        Vector3D legs_motor_rad[4];
        legs_motor_rad[0] = lf_leg_calc->joint_pos(Vector3D(0.0, 0.0, 0.0), &result);
        legs_motor_rad[1] = rf_leg_calc->joint_pos(Vector3D(0.0, 0.0, 0.0), &result);
        legs_motor_rad[2] = lb_leg_calc->joint_pos(Vector3D(0.0, 0.0, 0.0), &result);
        legs_motor_rad[3] = rb_leg_calc->joint_pos(Vector3D(0.0, 0.0, 0.0), &result);

        for (int i = 0; i < 4; i++)               // 填写计算结果
        {
            for (int j = 0; j < 3; j++) {
                joints_target.legs[i].joints[j].rad =legs_motor_rad[i][j];
                joints_target.legs[i].joints[j].omega =0.0f;
                joints_target.legs[i].joints[j].torque =0.0f;
                joints_target.legs[i].joints[j].rad =legs_motor_rad[i][j];
                joints_target.legs[i].joints[j].kp  = stand_joints_kp[j];
                joints_target.legs[i].joints[j].kd  = stand_joints_kd[j];
            }
            joints_target.legs[i].wheel.omega=0.0f;
            joints_target.legs[i].wheel.torque=0.0f;
        }

        //RCLCPP_INFO(node_->get_logger(),"%f,%f,%f",joints_target.legs[0].joints[0].kp,joints_target.legs[0].joints[1].kp,joints_target.legs[0].joints[2].kp);

        command_and_state_publish(joints_target); // 发送
        return;
    } else if (robot_state == DOG_STOP) {         // 狗保持站立（纯力控）
        // 获取足端位置和速度
        auto lf_foot_cart_pos = lf_leg_calc->foot_pos(lf_joint_pos);
        auto lf_foot_cart_vel = lf_leg_calc->foot_vel(lf_joint_pos, lf_joint_vel);
        auto rf_foot_cart_pos = rf_leg_calc->foot_pos(rf_joint_pos);
        auto rf_foot_cart_vel = rf_leg_calc->foot_vel(rf_joint_pos, rf_joint_vel);
        auto lb_foot_cart_pos = lb_leg_calc->foot_pos(lb_joint_pos);
        auto lb_foot_cart_vel = lb_leg_calc->foot_vel(lb_joint_pos, lb_joint_vel);
        auto rb_foot_cart_pos = rb_leg_calc->foot_pos(rb_joint_pos);
        auto rb_foot_cart_vel = rb_leg_calc->foot_vel(rb_joint_pos, rb_joint_vel);

        // 计算水平方向的 VMC 力
        double lf_vmc_x_force = lf_x_vmc->update(lf_foot_cart_pos[0], lf_foot_cart_vel[0]);
        double lf_vmc_y_force = lf_y_vmc->update(lf_foot_cart_pos[1], lf_foot_cart_vel[1]);
        double rf_vmc_x_force = rf_x_vmc->update(rf_foot_cart_pos[0], rf_foot_cart_vel[0]);
        double rf_vmc_y_force = rf_y_vmc->update(rf_foot_cart_pos[1], rf_foot_cart_vel[1]);
        double lb_vmc_x_force = lb_x_vmc->update(lb_foot_cart_pos[0], lb_foot_cart_vel[0]);
        double lb_vmc_y_force = lb_y_vmc->update(lb_foot_cart_pos[1], lb_foot_cart_vel[1]);
        double rb_vmc_x_force = rb_x_vmc->update(rb_foot_cart_pos[0], rb_foot_cart_vel[0]);
        double rb_vmc_y_force = rb_y_vmc->update(rb_foot_cart_pos[1], rb_foot_cart_vel[1]);

        // 计算竖直方向的 VMC 力
        double lf_vmc_z_force = lf_z_vmc->update(lf_foot_cart_pos[2], lf_foot_cart_vel[2]);
        double rf_vmc_z_force = rf_z_vmc->update(rf_foot_cart_pos[2], rf_foot_cart_vel[2]);
        double lb_vmc_z_force = lb_z_vmc->update(lb_foot_cart_pos[2], lb_foot_cart_vel[2]);
        double rb_vmc_z_force = rb_z_vmc->update(rb_foot_cart_pos[2], rb_foot_cart_vel[2]);

        RCLCPP_INFO(node_->get_logger(),"VMC:%lf",lf_vmc_z_force);

        // 使用 CompleteOrthogonalDecomposition 计算克服重力所需的足端力
        // 计算机器人总质量
        double total_mass = 0.0;
        auto accumulate_mass = [&total_mass](const KDL::Chain& chain) {
            for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
                total_mass += chain.getSegment(i).getInertia().getMass();
            }
        };
        accumulate_mass(lf_leg_chain);
        accumulate_mass(rf_leg_chain);
        accumulate_mass(lb_leg_chain);
        accumulate_mass(rb_leg_chain);
        
        // 加上 body_link 质量
        const auto it = tree.getSegment("body_link");
        if (it != tree.getSegments().end()) {
            total_mass += it->second.segment.getInertia().getMass();
        }

        // 构建约束矩阵求解四足端力
        // 约束：力平衡 + 力矩平衡（相对于质心）
        // f_lf_z + f_rf_z + f_lb_z + f_rb_z = m * g
        // (r_lf - r_com) × [0, 0, f_lf_z] + ... = 0
        
        Eigen::Matrix<double, 3, 4> A;
        Eigen::Vector3d b;
        
        // 力平衡约束（Z方向）
        A(0, 0) = 1.0;  // lf
        A(0, 1) = 1.0;  // rf
        A(0, 2) = 1.0;  // lb
        A(0, 3) = 1.0;  // rb
        b(0) = total_mass * 9.81;
        
        // 力矩平衡约束（X轴 - 由Y方向位置和Z方向力产生）
        Vector3D lf_foot_pos_global = lf_foot_cart_pos + lf_leg_calc->pos_offset;
        Vector3D rf_foot_pos_global = rf_foot_cart_pos + rf_leg_calc->pos_offset;
        Vector3D lb_foot_pos_global = lb_foot_cart_pos + lb_leg_calc->pos_offset;
        Vector3D rb_foot_pos_global = rb_foot_cart_pos + rb_leg_calc->pos_offset;
        
        A(1, 0) = lf_foot_pos_global[1] - comm_pos[1];  // lf
        A(1, 1) = rf_foot_pos_global[1] - comm_pos[1];  // rf
        A(1, 2) = lb_foot_pos_global[1] - comm_pos[1];  // lb
        A(1, 3) = rb_foot_pos_global[1] - comm_pos[1];  // rb
        b(1) = 0.0;
        
        // 力矩平衡约束（Y轴 - 由X方向位置和Z方向力产生，注意符号）
        A(2, 0) = -(lf_foot_pos_global[0] - comm_pos[0]);  // lf
        A(2, 1) = -(rf_foot_pos_global[0] - comm_pos[0]);  // rf
        A(2, 2) = -(lb_foot_pos_global[0] - comm_pos[0]);  // lb
        A(2, 3) = -(rb_foot_pos_global[0] - comm_pos[0]);  // rb
        b(2) = 0.0;
        
        // 使用 CompleteOrthogonalDecomposition 求解
        Eigen::CompleteOrthogonalDecomposition<Eigen::Matrix<double, 3, 4>> cod(A);
        Eigen::Vector4d gravity_forces = cod.solve(b);
        
        // 构建笛卡尔空间的足端力
        auto lf_cart_force = Vector3D(lf_vmc_x_force, lf_vmc_y_force, -gravity_forces(0) + lf_vmc_z_force);
        auto rf_cart_force = Vector3D(rf_vmc_x_force, rf_vmc_y_force, -gravity_forces(1) + rf_vmc_z_force);
        auto lb_cart_force = Vector3D(lb_vmc_x_force, lb_vmc_y_force, -gravity_forces(2) + lb_vmc_z_force);
        auto rb_cart_force = Vector3D(rb_vmc_x_force, rb_vmc_y_force, -gravity_forces(3) + rb_vmc_z_force);


        //平衡控制
        double cur_roll, cur_pitch, cur_yaw;
        tf2::Matrix3x3(robot_rotation).getRPY(cur_roll, cur_pitch, cur_yaw);

        roll_offset_virtual_torque  = roll_vmc->update(cur_roll, robot_velocity.angular.x);
        pitch_offset_virtual_torque = pitch_vmc->update(cur_pitch, robot_velocity.angular.y);

        lf_cart_force[2] += pitch_offset_virtual_torque * lf_leg_calc->pos_offset[0];
        rf_cart_force[2] += pitch_offset_virtual_torque * rf_leg_calc->pos_offset[0];
        lb_cart_force[2] += pitch_offset_virtual_torque * lb_leg_calc->pos_offset[0];
        rb_cart_force[2] += pitch_offset_virtual_torque * rb_leg_calc->pos_offset[0];

        lf_cart_force[0] += pitch_offset_virtual_torque * std::sin(cur_pitch) * pitch_balance_force_compen;
        rf_cart_force[0] += pitch_offset_virtual_torque * std::sin(cur_pitch) * pitch_balance_force_compen;
        lb_cart_force[0] += pitch_offset_virtual_torque * std::sin(cur_pitch) * pitch_balance_force_compen;
        rb_cart_force[0] += pitch_offset_virtual_torque * std::sin(cur_pitch) * pitch_balance_force_compen;

        // TODO:计算四个足端的期望的平衡力
        lf_cart_force[2] += roll_offset_virtual_torque * lf_leg_calc->pos_offset[1];
        rf_cart_force[2] += roll_offset_virtual_torque * rf_leg_calc->pos_offset[1];
        lb_cart_force[2] += roll_offset_virtual_torque * lb_leg_calc->pos_offset[1];
        rb_cart_force[2] += roll_offset_virtual_torque * rb_leg_calc->pos_offset[1];

        lf_cart_force[1] += pitch_offset_virtual_torque * std::sin(cur_roll) * roll_balance_force_compen;
        rf_cart_force[1] += pitch_offset_virtual_torque * std::sin(cur_roll) * roll_balance_force_compen;
        lb_cart_force[1] += pitch_offset_virtual_torque * std::sin(cur_roll) * roll_balance_force_compen;
        rb_cart_force[1] += pitch_offset_virtual_torque * std::sin(cur_roll) * roll_balance_force_compen;


        Vector3D legs_motor_torque[4];
        legs_motor_torque[0] = lf_leg_calc->joint_torque_foot_force(lf_joint_pos, lf_cart_force);
        legs_motor_torque[1] = rf_leg_calc->joint_torque_foot_force(rf_joint_pos, rf_cart_force);
        legs_motor_torque[2] = lb_leg_calc->joint_torque_foot_force(lb_joint_pos, lb_cart_force);
        legs_motor_torque[3] = rb_leg_calc->joint_torque_foot_force(rb_joint_pos, rb_cart_force);

        for (int i = 0; i < 4; i++)           // 填写计算结果
        {
            for (int j = 0; j < 3; j++)
                joints_target.legs[i].joints[j].torque = static_cast<float>(legs_motor_torque[i][j]);
        }

        if (robot_req_state == DOG_REQ_RUN)   // 如果请求转移到行走状态，那么机器人状态先跳转到开始行走状态
            robot_state = DOG_STARTING;
        else if (robot_req_state == DOG_REQ_IDEL)
        {
            robot_state = DOG_IDEL;
        }
        
        command_and_state_publish(joints_target);
        return;
    }/*else if (robot_state == DOG_STARTING) { // 狗处于开始前进状态，规划一次初相位轨迹
        auto now                = node_->get_clock()->now();
        main_phrase_start_time  = now;
        slave_phrase_start_time = now;
        slave_phrase_stop_time =
            now
            + rclcpp::Duration(
                std::chrono::duration<double>(
                    (std::abs(2.0 * step_support_rate - 1.0) * 0.5 + 1.0 - step_support_rate) * step_time)); // 预规划从相位支撑相结束时间
        lf_leg_step.update_flight_trajectory(
            lf_leg_calc->foot_pos(lf_joint_pos), Vector3D(0.0, 0.0, 0.0), lf_exp_vel, ((1.0 - step_support_rate) * step_time), step_height);
        rf_leg_step.update_support_trajectory(
            rf_leg_calc->foot_pos(rf_joint_pos), rf_exp_vel,
            (std::abs(2.0 * step_support_rate - 1.0) * 0.5 + 1.0 - step_support_rate) * step_time);
        lb_leg_step.update_support_trajectory(
            lb_leg_calc->foot_pos(lb_joint_pos), lb_exp_vel,
            (std::abs(2.0 * step_support_rate - 1.0) * 0.5 + 1.0 - step_support_rate) * step_time);
        rb_leg_step.update_flight_trajectory(
            lf_leg_calc->foot_pos(lf_joint_pos), Vector3D(0.0, 0.0, 0.0), lf_exp_vel, ((1.0 - step_support_rate) * step_time), step_height);
        step1_support_updated = false;                                                                       // 设置足端轨迹更新状态
        step1_flight_updated  = true;
        step2_flight_updated  = false;
        step2_support_updated = true;

        // 无条件跳转到行走态
        robot_state = DOG_SETP;         // 初始相
    } else if (robot_state == DOG_SETP) // 机器人正在正常执行步态
    {
        auto now = node_->get_clock()->now();
        // TODO:利用LegStep类的轨迹计算是否成功的判据来决定是否开启
        if (step1_flight_updated && (!step1_support_updated)) {    // 处于足端飞行相
            if (now - main_phrase_start_time > rclcpp::Duration(
                    std::chrono::duration<double>(
                        (1.0 - step_support_rate) * step_time))) { // 如果主相位飞行相已经结束，那么立即规划主相位支撑相
                step1_support_updated = true;                      // 设置足端轨迹更新状态
                step1_flight_updated  = false;
                slave_phrase_stop_time =
                    now                                            // 从相位支撑相结束时间等于主相位飞行相结束时间+T*(2*α-1)/2
                    + rclcpp::Duration(std::chrono::duration<double>(std::abs(2.0 * step_support_rate - 1.0) * step_time * 0.5));

                // TODO:根据姿态更新足端中性点位置,求在平面上的投影与基偏移量叠加作为新的足端中性点
                auto vertical_v = Vector3D(0.0, 0.0, -body_height); // 足端垂直向量
                tf2::Quaternion q;
                q.setRPY(cur_roll, cur_pitch, 0.0);
                Eigen::Quaterniond e_q(q);
                Eigen::Matrix3d R_mat   = e_q.toRotationMatrix().transpose();
                Vector3D rot_pos_offset = R_mat * vertical_v;
                rot_pos_offset[2]       = 0.0;

                // lf_leg_calc->pos_offset=lf_base_offset+rot_pos_offset;      //在规划轨迹前更改足端中性点，不会引起系统冲击
                // rb_leg_calc->pos_offset=rb_base_offset+rot_pos_offset;

                lf_leg_step.update_support_trajectory(lf_leg_calc->foot_pos(lf_joint_pos), lf_exp_vel, step_support_rate * step_time);
                // 主相对角腿也需要同步进入支撑相（右后）
                rb_leg_step.update_support_trajectory(rb_leg_calc->foot_pos(rb_joint_pos), rb_exp_vel, step_support_rate * step_time);
                main_phrase_start_time = now;
                RCLCPP_INFO(node_->get_logger(), "主相位支撑相规划");
            }
        } else if (step1_support_updated && (!step1_flight_updated)) {               // 处于足端支撑相
            if (now - main_phrase_start_time > rclcpp::Duration(
                    std::chrono::duration<double>(step_support_rate) * step_time)) { // 如果主相位飞行相已经结束，那么立即规划主相位飞行相
                step1_support_updated = false;                                       // 设置足端轨迹更新状态
                step1_flight_updated  = true;
                lf_leg_step.update_flight_trajectory(
                    lf_leg_calc->foot_pos(lf_joint_pos), -Vector3D(lf_exp_vel[0], lf_exp_vel[1], 0.0), lf_exp_vel,
                    step_time * (1.0 - step_support_rate), step_height);
                // 主相对角腿也需要规划飞行轨迹（右后）
                rb_leg_step.update_flight_trajectory(
                    rb_leg_calc->foot_pos(rb_joint_pos), -Vector3D(rb_exp_vel[0], rb_exp_vel[1], 0.0), rb_exp_vel,
                    step_time * (1.0 - step_support_rate), step_height);
                main_phrase_start_time = now;
                RCLCPP_INFO(node_->get_logger(), "主相位摆动相规划");
            }
        }


        if (step2_flight_updated && (!step2_support_updated)) {    // 如果从相位处于飞行相
            if (now - slave_phrase_start_time > rclcpp::Duration(
                    std::chrono::duration<double>(
                        (1.0 - step_support_rate) * step_time))) { // 如果主相位飞行相已经结束，那么立即规划主相位支撑相
                step2_support_updated = true;                      // 设置足端轨迹更新状态
                step2_flight_updated  = false;
                // TODO:根据姿态更新足端中性点位置

                auto vertical_v = Vector3D(0.0, 0.0, -body_height); // 足端垂直向量
                tf2::Quaternion q;
                q.setRPY(cur_roll, cur_pitch, 0.0);
                Eigen::Quaterniond e_q(q);
                Eigen::Matrix3d R_mat   = e_q.toRotationMatrix().transpose();
                Vector3D rot_pos_offset = R_mat * vertical_v;
                rot_pos_offset[2]       = 0.0;

                // rf_leg_calc->pos_offset=rf_base_offset+rot_pos_offset;      //在规划轨迹前更改足端中性点，不会引起系统冲击
                // lb_leg_calc->pos_offset=lb_base_offset+rot_pos_offset;


                rf_leg_step.update_support_trajectory(
                    rf_leg_calc->foot_pos(rf_joint_pos), rf_exp_vel,
                    step_support_rate * step_time); // 预更新支撑相(精确结束时间由主相位确定)
                // 从相对角腿也同步进入支撑相（左后）
                lb_leg_step.update_support_trajectory(lb_leg_calc->foot_pos(lb_joint_pos), lb_exp_vel, step_support_rate * step_time);
                slave_phrase_start_time = now;
                slave_phrase_stop_time  = now + rclcpp::Duration(std::chrono::duration<double>(step_support_rate * step_time));
                if (robot_req_state == DOG_REQ_STOP) {                 // 请求状态为停止，那么状态机跳转到正在停止
                    robot_state = DOG_ENDING;
                }
                RCLCPP_INFO(node_->get_logger(), "从相位支撑相规划");
            }
        } else if (step2_support_updated && (!step2_flight_updated)) { // 如果从相位处于支撑相(调相位)
            if (now > slave_phrase_stop_time)                          // 如果到达了由主相位确定的从相位支撑相结束时间，那么更新从相位飞行相
            {
                step2_support_updated = false;                         // 设置足端轨迹更新状态
                step2_flight_updated  = true;
                // 从相两条腿同时进入飞行相（右前 & 左后）
                rf_leg_step.update_flight_trajectory(
                    rf_leg_calc->foot_pos(rf_joint_pos), -Vector3D(rf_exp_vel[0], rf_exp_vel[1], 0.0), rf_exp_vel,
                    (1.0 - step_support_rate) * step_time, step_height);
                lb_leg_step.update_flight_trajectory(
                    lb_leg_calc->foot_pos(lb_joint_pos), -Vector3D(lb_exp_vel[0], lb_exp_vel[1], 0.0), lb_exp_vel,
                    (1.0 - step_support_rate) * step_time, step_height);
                slave_phrase_start_time = now;
                RCLCPP_INFO(node_->get_logger(), "从相位摆动相规划");
            }
        }*/

        // bool success[4];
        // std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) =
        //     lf_leg_step.get_target((now - main_phrase_start_time).seconds(), success[0]); // 得到狗腿当前期望
        // std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) =
        //     rf_leg_step.get_target((now - slave_phrase_start_time).seconds(), success[1]);
        // std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) =
        //     lb_leg_step.get_target((now - slave_phrase_start_time).seconds(), success[2]);
        // std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) =
        //     rb_leg_step.get_target((now - main_phrase_start_time).seconds(), success[3]);

        // RCLCPP_INFO(node_->get_logger(),"rf:(%lf,%lf,%lf),success=(%d,%d,%d,%d)",rf_foot_exp_pos[0],rf_foot_exp_pos[1],rf_foot_exp_pos[2],success[0],success[1],success[2],success[3]);

        // auto lf_cart_pos   = lf_leg_calc->foot_pos(lf_joint_pos);
        // auto lf_cart_vel   = lf_leg_calc->foot_vel(lf_joint_pos, lf_joint_vel);
        // auto lf_cart_force = lf_leg_calc->foot_force(lf_joint_pos, lf_joint_torque, lf_forward_torque);
        // auto rb_cart_pos   = rb_leg_calc->foot_pos(rb_joint_pos);
        // auto rb_cart_vel   = rb_leg_calc->foot_vel(rb_joint_pos, rb_joint_vel);
        // auto rb_cart_force = rb_leg_calc->foot_force(rb_joint_pos, rb_joint_torque, rb_forward_torque);
        // auto rf_cart_pos   = rf_leg_calc->foot_pos(rf_joint_pos);
        // auto rf_cart_vel   = rf_leg_calc->foot_vel(rf_joint_pos, rf_joint_vel);
        // auto rf_cart_force = rf_leg_calc->foot_force(rf_joint_pos, rf_joint_torque, rf_forward_torque);
        // auto lb_cart_pos   = lb_leg_calc->foot_pos(lb_joint_pos);
        // auto lb_cart_vel   = lb_leg_calc->foot_vel(lb_joint_pos, lb_joint_vel);
        // auto lb_cart_force = lb_leg_calc->foot_force(lb_joint_pos, lb_joint_torque, lb_forward_torque);

        // if (step1_support_updated) {           // 主相位需要VMC计算
        //     std::tie(lf_foot_exp_pos[2], lf_foot_exp_vel[2], lf_foot_exp_acc[2]) =
        //         lf_z_vmc->targetUpdate(lf_foot_exp_pos[2], lf_cart_pos[2], lf_foot_exp_vel[2], lf_cart_vel[2], -lf_cart_force[2]);

        //     std::tie(rb_foot_exp_pos[2], rb_foot_exp_vel[2], rb_foot_exp_acc[2]) =
        //         rb_z_vmc->targetUpdate(rb_foot_exp_pos[2], rb_cart_pos[2], rb_foot_exp_vel[2], rb_cart_vel[2], -rb_cart_force[2]);

        //     if (step2_support_updated) {       // 如果从相位也需要VMC计算，说明此时四足触底，每个脚的向下的力为一倍，否则为两倍
        //         lf_foot_exp_force += Vector3D(0.0, 0.0, -robot_lf_grivate);
        //         rb_foot_exp_force += Vector3D(0.0, 0.0, -robot_rb_grivate);
        //     } else {
        //         lf_foot_exp_force += Vector3D(0.0, 0.0, -2.0 * robot_lf_grivate);
        //         rb_foot_exp_force += Vector3D(0.0, 0.0, -2.0 * robot_rb_grivate);
        //     }



        //     std::tie(lf_foot_exp_pos[0], lf_foot_exp_vel[0], lf_foot_exp_acc[0]) = lf_x_vmc->targetUpdate(
        //         lf_foot_exp_pos[0], lf_cart_pos[0], lf_foot_exp_vel[0], lf_cart_vel[0],
        //         -lf_cart_force[0]);            // 实际这个lf_cart_force是足端本身要施加的力，不是受到的力
        //     std::tie(lf_foot_exp_pos[1], lf_foot_exp_vel[1], lf_foot_exp_acc[1]) =
        //         lf_y_vmc->targetUpdate(lf_foot_exp_pos[1], lf_cart_pos[1], lf_foot_exp_vel[1], lf_cart_vel[1], -lf_cart_force[1]);
        //     std::tie(rb_foot_exp_pos[0], rb_foot_exp_vel[0], rb_foot_exp_acc[0]) =
        //         rb_x_vmc->targetUpdate(rb_foot_exp_pos[0], rb_cart_pos[0], rb_foot_exp_vel[0], rb_cart_vel[0], -rb_cart_force[0]);
        //     std::tie(rb_foot_exp_pos[1], rb_foot_exp_vel[1], rb_foot_exp_acc[1]) =
        //         rb_y_vmc->targetUpdate(rb_foot_exp_pos[1], rb_cart_pos[1], rb_foot_exp_vel[1], rb_cart_vel[1], -rb_cart_force[1]);
        // }
        // if (step2_support_updated) {           // 从相位需要VMC计算

        //     std::tie(rf_foot_exp_pos[2], rf_foot_exp_vel[2], rf_foot_exp_acc[2]) =
        //         rf_z_vmc->targetUpdate(rf_foot_exp_pos[2], rf_cart_pos[2], rf_foot_exp_vel[2], rf_cart_vel[2], -rf_cart_force[2]);

        //     std::tie(lb_foot_exp_pos[2], lb_foot_exp_vel[2], lb_foot_exp_acc[2]) =
        //         lb_z_vmc->targetUpdate(lb_foot_exp_pos[2], lb_cart_pos[2], lb_foot_exp_vel[2], lb_cart_vel[2], -lb_cart_force[2]);

        //     if (step1_support_updated) {
        //         rf_foot_exp_force += Vector3D(0.0, 0.0, -robot_rf_grivate);
        //         lb_foot_exp_force += Vector3D(0.0, 0.0, -robot_lb_grivate);
        //     } else {
        //         rf_foot_exp_force += Vector3D(0.0, 0.0, -2.0 * robot_rf_grivate);
        //         lb_foot_exp_force += Vector3D(0.0, 0.0, -2.0 * robot_lb_grivate);
        //     }

        //     std::tie(rf_foot_exp_pos[0], rf_foot_exp_vel[0], rf_foot_exp_acc[0]) =
        //         rf_x_vmc->targetUpdate(rf_foot_exp_pos[0], rf_cart_pos[0], rf_foot_exp_vel[0], rf_cart_vel[0], -rf_cart_force[0]);
        //     std::tie(rf_foot_exp_pos[1], rf_foot_exp_vel[1], rf_foot_exp_acc[1]) =
        //         rf_y_vmc->targetUpdate(rf_foot_exp_pos[1], rf_cart_pos[1], rf_foot_exp_vel[1], rf_cart_vel[1], -rf_cart_force[1]);
        //     std::tie(lb_foot_exp_pos[0], lb_foot_exp_vel[0], lb_foot_exp_acc[0]) =
        //         lb_x_vmc->targetUpdate(lb_foot_exp_pos[0], lb_cart_pos[0], lb_foot_exp_vel[0], lb_cart_vel[0], -lb_cart_force[0]);
        //     std::tie(lb_foot_exp_pos[1], lb_foot_exp_vel[1], lb_foot_exp_acc[1]) =
        //         lb_y_vmc->targetUpdate(lb_foot_exp_pos[1], lb_cart_pos[1], lb_foot_exp_vel[1], lb_cart_vel[1], -lb_cart_force[1]);
        // }
    // } else if (robot_state == DOG_ENDING) {
    //     lf_leg_stop_pos = lf_leg_calc->foot_pos(lf_joint_pos);
    //     rf_leg_stop_pos = rf_leg_calc->foot_pos(rf_joint_pos);
    //     lb_leg_stop_pos = lb_leg_calc->foot_pos(lb_joint_pos);
    //     rb_leg_stop_pos = rb_leg_calc->foot_pos(rb_joint_pos);


        // lf_foot_exp_pos = lf_leg_stop_pos; // 更新位置目标为当前位置,
        // rf_foot_exp_pos = rf_leg_stop_pos;
        // lb_foot_exp_pos = lb_leg_stop_pos;
        // rb_foot_exp_pos = rb_leg_stop_pos;

    //     robot_state = DOG_STOP;
    // }



    // auto lf_leg_joints_target = signal_leg_calc(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc, lf_foot_exp_force, lf_leg_calc);
    // auto rf_leg_joints_target = signal_leg_calc(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc, rf_foot_exp_force, rf_leg_calc);
    // auto lb_leg_joints_target = signal_leg_calc(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc, lb_foot_exp_force, lb_leg_calc);
    // auto rb_leg_joints_target = signal_leg_calc(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc, rb_foot_exp_force, rb_leg_calc);
    // RCLCPP_INFO(node_->get_logger(),"x:%lf,y:%lf,z:%lf",lf_foot_exp_pos[0],lf_foot_exp_pos[1],lf_foot_exp_pos[2]);

    // lf_forward_torque = std::get<2>(lf_leg_joints_target); // 更新本周期计算的前馈力矩准备用作下一个控制周期的计算
    // rf_forward_torque = std::get<2>(rf_leg_joints_target);
    // lb_forward_torque = std::get<2>(lb_leg_joints_target);
    // rb_forward_torque = std::get<2>(rb_leg_joints_target);
}


void RobotCalcNode::command_and_state_publish(const robot_interfaces::msg::Robot &joints_target) {
    legs_target_pub->publish(joints_target);

    rviz2_update_cnt++;
    if (rviz2_update_cnt == 5) {                           // 发布话题在RVIZ2中查看
        rviz2_update_cnt = 0;

        joint_display_msg.position[0] = lf_joint_pos[0];
        joint_display_msg.position[1] = lf_joint_pos[1];
        joint_display_msg.position[2] = lf_joint_pos[2];

        joint_display_msg.position[3] = rf_joint_pos[0];
        joint_display_msg.position[4] = rf_joint_pos[1];
        joint_display_msg.position[5] = rf_joint_pos[2];

        joint_display_msg.position[6] = lb_joint_pos[0];
        joint_display_msg.position[7] = lb_joint_pos[1];
        joint_display_msg.position[8] = lb_joint_pos[2];

        joint_display_msg.position[9]  = rb_joint_pos[0];
        joint_display_msg.position[10] = rb_joint_pos[1];
        joint_display_msg.position[11] = rb_joint_pos[2];

        joint_display_msg.header.stamp = node_->get_clock()->now();
        rviz_joint_publisher->publish(joint_display_msg);
    }

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp            = node_->get_clock()->now();
    t.header.frame_id         = "world";
    t.child_frame_id          = "body_link";
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;

    t.transform.rotation.x = robot_rotation.x();
    t.transform.rotation.y = robot_rotation.y();
    t.transform.rotation.z = robot_rotation.z();
    t.transform.rotation.w = robot_rotation.w();

    robot_tf_broadcaster->sendTransform(t);
}


Vector3D RobotCalcNode::get_grivate_center_pose(
    const Vector3D& lf_joint_pos, const Vector3D& rf_joint_pos, const Vector3D& lb_joint_pos, const Vector3D& rb_joint_pos) {
    // 使用 KDL 计算全身质心（在 body_link 坐标系下）
    // 注意：这里用各 link 的 RigidBodyInertia 来做质量加权平均。
    //      需要 URDF 中每个 link 都有 inertial 标签，否则质量会为 0。

    auto accumulate_chain_com = [](const KDL::Chain& chain, const Vector3D& q_eigen, double& mass_sum, KDL::Vector& com_sum) {
        KDL::JntArray q(chain.getNrOfJoints());
        for (unsigned int i = 0; i < chain.getNrOfJoints() && i < 3; ++i) {
            q(i) = q_eigen[i];
        }

        KDL::Frame T = KDL::Frame::Identity();

        unsigned int joint_idx = 0;
        for (unsigned int seg_idx = 0; seg_idx < chain.getNrOfSegments(); ++seg_idx) {
            const auto& seg = chain.getSegment(seg_idx);

            // 计算该段末端在基座下的位姿
            if (seg.getJoint().getType() != KDL::Joint::None) {
                T = T * seg.pose(q(joint_idx));
                joint_idx++;
            } else {
                T = T * seg.pose(0.0);
            }

            // 该 segment 的刚体惯量（在 segment 坐标系下）
            const KDL::RigidBodyInertia& rbi = seg.getInertia();
            const double m                   = rbi.getMass();
            if (m <= 0.0) {
                continue;
            }

            // COM 在 segment 坐标系下的位置
            const KDL::Vector c_seg = rbi.getCOG();
            // 转到基座坐标系
            const KDL::Vector c_base = T * c_seg;

            mass_sum += m;
            com_sum = com_sum + c_base * m;
        }
    };

    double mass_sum = 0.0;
    KDL::Vector com_sum(0.0, 0.0, 0.0);

    // 4 条腿分别从 body_link 到足端 link4；它们共享 body_link，但各自 inertia 不重复（body_link 本体要单独加）
    accumulate_chain_com(lf_leg_chain, lf_joint_pos, mass_sum, com_sum);
    accumulate_chain_com(rf_leg_chain, rf_joint_pos, mass_sum, com_sum);
    accumulate_chain_com(lb_leg_chain, lb_joint_pos, mass_sum, com_sum);
    accumulate_chain_com(rb_leg_chain, rb_joint_pos, mass_sum, com_sum);

    // 加上 body_link 自身的质量与质心（树中 body_link 是 root，不在各腿 chain 的 segment 中）
    {
        const auto it = tree.getSegment("body_link");
        if (it != tree.getSegments().end()) {
            const auto body_seg = it->second.segment;
            const auto& rbi     = body_seg.getInertia();
            const double m      = rbi.getMass();
            if (m > 0.0) {
                mass_sum += m;
                com_sum = com_sum + rbi.getCOG() * m; // body_link 在 body_link 坐标系下
            }
        }
    }

    if (mass_sum <= 1e-9) {
        return Vector3D(0.0, 0.0, 0.0);
    }

    const KDL::Vector com = com_sum / mass_sum;
    return Vector3D(com.x(), com.y(), com.z());
}


void RobotCalcNode::quaternionLowPassFilter(
    double& w, double& x, double& y, double& z, double w1, double x1, double y1, double z1, double alpha) {

    auto normalizeQuaternion = [](double& w, double& x, double& y, double& z) {
        double norm = std::sqrt(w * w + x * x + y * y + z * z);
        if (norm < 1e-12) {
            // 退化情况，回到单位四元数
            w = 1.0;
            x = y = z = 0.0;
            return;
        }
        w /= norm;
        x /= norm;
        y /= norm;
        z /= norm;
    };


    // 1. 单位化输入（防御性编程）
    normalizeQuaternion(w, x, y, z);
    normalizeQuaternion(w1, x1, y1, z1);

    // 2. 点积，判断是否需要取反（双覆盖问题）
    double dot = w * w1 + x * x1 + y * y1 + z * z1;
    if (dot < 0.0) {
        w1  = -w1;
        x1  = -x1;
        y1  = -y1;
        z1  = -z1;
        dot = -dot;
    }

    // 3. 小角度近似（避免 acos / sin 数值不稳定）
    const double DOT_THRESHOLD = 0.9995;
    if (dot > DOT_THRESHOLD) {
        // 线性插值
        w = w + alpha * (w1 - w);
        x = x + alpha * (x1 - x);
        y = y + alpha * (y1 - y);
        z = z + alpha * (z1 - z);
        normalizeQuaternion(w, x, y, z);
        return;
    }

    // 4. 标准 SLERP
    double theta     = std::acos(dot);
    double sin_theta = std::sin(theta);

    double w_a = std::sin((1.0 - alpha) * theta) / sin_theta;
    double w_b = std::sin(alpha * theta) / sin_theta;

    double w_new = w_a * w + w_b * w1;
    double x_new = w_a * x + w_b * x1;
    double y_new = w_a * y + w_b * y1;
    double z_new = w_a * z + w_b * z1;

    w = w_new;
    x = x_new;
    y = y_new;
    z = z_new;

    // 5. 再次单位化（强烈建议保留）
    normalizeQuaternion(w, x, y, z);
}

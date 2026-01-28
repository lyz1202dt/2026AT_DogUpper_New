#include "dog_calc.hpp"
#include "step.h"
#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <chrono>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <kdl/frames.hpp>
#include <rclcpp/duration.hpp>
#include <robot_interfaces/msg/robot.hpp>
#include <sensor_msgs/msg/detail/imu__struct.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tuple>

using namespace std::chrono_literals;

RobotCalcNode::RobotCalcNode(const rclcpp::Node::SharedPtr node) {
    node_    = node;
    lf_z_vmc = std::make_shared<VMC>(500, 120, 4.0, 0.5, 0.2, 0.1, 10ms);
    rf_z_vmc = std::make_shared<VMC>(500, 120, 4.0, 0.5, 0.2, 0.1, 10ms);
    lb_z_vmc = std::make_shared<VMC>(500, 120, 4.0, 0.5, 0.2, 0.1, 10ms);
    rb_z_vmc = std::make_shared<VMC>(500, 120, 4.0, 0.5, 0.2, 0.1, 10ms);

    lf_x_vmc=std::make_shared<VMC>(160,60,3.0,0.5,0.2,0.1,10ms);
    lf_y_vmc=std::make_shared<VMC>(160,60,3.0,0.5,0.2,0.1,10ms);
    rf_x_vmc=std::make_shared<VMC>(160,60,3.0,0.5,0.2,0.1,10ms);
    rf_y_vmc=std::make_shared<VMC>(160,60,3.0,0.5,0.2,0.1,10ms);
    lb_x_vmc=std::make_shared<VMC>(160,60,3.0,0.5,0.2,0.1,10ms);
    lb_y_vmc=std::make_shared<VMC>(160,60,3.0,0.5,0.2,0.1,10ms);
    rb_x_vmc=std::make_shared<VMC>(160,60,3.0,0.5,0.2,0.1,10ms);
    rb_y_vmc=std::make_shared<VMC>(160,60,3.0,0.5,0.2,0.1,10ms);


    node_->declare_parameter("direction_filter_gate", 0.2);
    node_->declare_parameter("enable_vmc", false);
    node_->declare_parameter("vmc_kp", 300.0);
    node_->declare_parameter("vmc_kd", 100.0);
    node_->declare_parameter("vmc_mass", 4.0);

    node_->declare_parameter("lf_grivate", 8.0);
    node_->declare_parameter("rf_grivate", 8.0);
    node_->declare_parameter("lb_grivate", 8.0);
    node_->declare_parameter("rb_grivate", 8.0);
    node_->declare_parameter("lf_dx", 0.0);
    node_->declare_parameter("rf_dx", 0.0);
    node_->declare_parameter("lb_dx", -0.05);
    node_->declare_parameter("rb_dx", -0.05);

    node_->declare_parameter("step_support_rate", 0.55);                                        // 支撑相时间
    node_->declare_parameter("step_time", 0.7);                                                 // 一个完整步态时间
    node_->declare_parameter("step_height", 0.12);
    node_->declare_parameter("base_height",0.0);


    param_server_ = node_->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        RCLCPP_INFO(node_->get_logger(), "更新参数");
        for (const auto& param : params) {
            if (param.get_name() == "enable_vmc")
                enable_vmc = param.as_bool();
            else if (param.get_name() == "direction_filter_gate")
            {
                direction_filter_gate = param.as_double();
                direction_filter_gate=std::clamp(direction_filter_gate,0.0,1.0);
            }
                
            else if (param.get_name() == "vmc_kp")
            {
                lf_z_vmc->kp = param.as_double();
                rf_z_vmc->kp = param.as_double();
                lb_z_vmc->kp = param.as_double();
                rb_z_vmc->kp = param.as_double();
            }
            else if (param.get_name() == "vmc_kd")
            {
                lf_z_vmc->kd = param.as_double();
                rf_z_vmc->kd = param.as_double();
                lb_z_vmc->kd = param.as_double();
                rb_z_vmc->kd = param.as_double();
            }
            else if (param.get_name() == "vmc_mass")
            {
                lf_z_vmc->mass = param.as_double();
                rf_z_vmc->mass = param.as_double();
                lb_z_vmc->mass = param.as_double();
                rb_z_vmc->mass = param.as_double();
            }
            else if (param.get_name() == "lf_grivate")
                robot_lf_grivate = param.as_double();
            else if (param.get_name() == "rf_grivate")
                robot_rf_grivate = param.as_double();
            else if (param.get_name() == "lb_grivate")
                robot_lb_grivate = param.as_double();
            else if (param.get_name() == "rb_grivate")
                robot_rb_grivate = param.as_double();
            else if(param.get_name()=="lf_dx")
                robot_lf_dx=0.25+param.as_double();
            else if(param.get_name()=="rf_dx")
                robot_rf_dx=0.25+param.as_double();
            else if(param.get_name()=="lb_dx")
                robot_lb_dx=-0.23+param.as_double();
            else if(param.get_name()=="rb_dx")
                robot_rb_dx=-0.23+param.as_double();
            else if (param.get_name() == "step_support_rate")
                step_support_rate = param.as_double();
            else if (param.get_name() == "step_time")
                step_time = param.as_double();
            else if (param.get_name() == "step_height")
                step_height = param.as_double();
            else if(param.get_name()=="base_height")
                foot_base_height=param.as_double();
        }
        return result;
    });

    // 读取所有默认参数
    node_->get_parameter("direction_filter_gate", direction_filter_gate);
    node_->get_parameter("enable_vmc", enable_vmc);
    node_->get_parameter("vmc_kp", lf_z_vmc->kp);
    lf_z_vmc->kp = lf_z_vmc->kp;
    rf_z_vmc->kp = lf_z_vmc->kp;
    lb_z_vmc->kp = lf_z_vmc->kp;
    rb_z_vmc->kp = lf_z_vmc->kp;
    node_->get_parameter("vmc_kd", lf_z_vmc->kd);
    rf_z_vmc->kd = lf_z_vmc->kd;
    lb_z_vmc->kd = lf_z_vmc->kd;
    rb_z_vmc->kd = lf_z_vmc->kd;
    node_->get_parameter("vmc_mass", lf_z_vmc->mass);
    rf_z_vmc->mass = lf_z_vmc->mass;
    lb_z_vmc->mass = lf_z_vmc->mass;
    rb_z_vmc->mass = lf_z_vmc->mass;

    node_->get_parameter("lf_grivate", robot_lf_grivate);
    node_->get_parameter("rf_grivate", robot_rf_grivate);
    node_->get_parameter("lb_grivate", robot_lb_grivate);
    node_->get_parameter("rb_grivate", robot_rb_grivate);
    double lf_dx_temp, rf_dx_temp, lb_dx_temp, rb_dx_temp;
    node_->get_parameter("lf_dx", lf_dx_temp);
    robot_lf_dx = 0.25 + lf_dx_temp;
    node_->get_parameter("rf_dx", rf_dx_temp);
    robot_rf_dx = 0.25 + rf_dx_temp;
    node_->get_parameter("lb_dx", lb_dx_temp);
    robot_lb_dx = -0.23 + lb_dx_temp;
    node_->get_parameter("rb_dx", rb_dx_temp);
    robot_rb_dx = -0.23 + rb_dx_temp;

    node_->get_parameter("step_support_rate", step_support_rate);
    node_->get_parameter("step_time", step_time);
    node_->get_parameter("step_height", step_height);
    node_->get_parameter("base_height", foot_base_height);

    robot_rotation.setRPY(0.0,0.0,0.0);

    marker_publisher = node_->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);

    rviz_joint_publisher = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    legs_target_pub = node_->create_publisher<robot_interfaces::msg::Robot>("legs_target", 10); // 创建期望位置发布者

    // pose_sensor 发布的是 PoseStamped（见 mujoco_ros2_control/src/pose_sensor.cpp），这里必须用 PoseStamped 才能收到消息
    // 同时 QoS 需要与发布端兼容（发布端当前是 RELIABLE + TRANSIENT_LOCAL）
    imu_sub = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/imu_pose_sensor/pose",
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local(),
        [this](const geometry_msgs::msg::PoseStamped &msg) {
            const auto &q_msg = msg.pose.orientation;

            double qw = robot_rotation.getW();
            double qx = robot_rotation.getX();
            double qy = robot_rotation.getY();
            double qz = robot_rotation.getZ();

            quaternionLowPassFilter(qw, qx, qy, qz, q_msg.w, q_msg.x, q_msg.y, q_msg.z, direction_filter_gate);  //(0-强滤波、1-不滤波)

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
            // robot_rotation.setRPY(0.0, 0.0, 0.0);
            // robot_velocity.angular.x=0.0;
            // robot_velocity.angular.y=0.0;
            // robot_velocity.angular.z=0.0;
            //机器人的线速度需要在别的地方计算
        });

    // 订阅机器人的运动期望
    move_cmd_sub =
        node_->create_subscription<robot_interfaces::msg::MoveCmd>("robot_move_cmd", 10, [this](const robot_interfaces::msg::MoveCmd& msg) {
            if (msg.step_mode == DOG_REQ_STOP){ // 请求状态为停止
                robot_req_state = DOG_REQ_STOP;
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
            else if(robot_req_state ==DOG_REQ_IDEL)
            {
                robot_req_state=DOG_REQ_IDEL;
            }
            // RCLCPP_INFO(node_->get_logger(), "接收到期望更新消息:lf:(%lf,%lf),type=%d", lf_exp_vel[0], lf_exp_vel[1], msg.step_mode);
        });

    robot_description_param_ = std::make_shared<rclcpp::SyncParametersClient>(node_, "/robot_state_publisher");

    auto params = robot_description_param_->get_parameters({"robot_description"});
    urdf_xml    = params[0].as_string();
    if (urdf_xml.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "无法读取URDF文件，不能进行动力学计算");
        return;
    }

    robot_tf_broadcaster=std::make_unique<tf2_ros::TransformBroadcaster>(node_);

    kdl_parser::treeFromString(urdf_xml, tree); // 解析四条腿的KDL树结构
    tree.getChain("body_link", "lf_link4", lf_leg_chain);
    tree.getChain("body_link", "rf_link4", rf_leg_chain);
    tree.getChain("body_link", "lb_link4", lb_leg_chain);
    tree.getChain("body_link", "rb_link4", rb_leg_chain);

    // 初始化狗腿解算器，定义足端中性点位置
    lf_leg_calc = std::make_shared<LegCalc>(lf_leg_chain);
    lf_leg_calc->pos_offset << 0.25, 0.18, foot_pos_base_offset;

    rf_leg_calc = std::make_shared<LegCalc>(rf_leg_chain);
    rf_leg_calc->pos_offset << 0.25, -0.18, foot_pos_base_offset;

    lb_leg_calc = std::make_shared<LegCalc>(lb_leg_chain);
    lb_leg_calc->pos_offset << -0.21, 0.18, foot_pos_base_offset;

    rb_leg_calc = std::make_shared<LegCalc>(rb_leg_chain);
    rb_leg_calc->pos_offset << -0.21, -0.18, foot_pos_base_offset;

    joint_display_msg.name = {"lf_joint1", "lf_joint2", "lf_joint3", "rf_joint1", "rf_joint2", "rf_joint3",
                              "lb_joint1", "lb_joint2", "lb_joint3", "rb_joint1", "rb_joint2", "rb_joint3"};
    joint_display_msg.position.resize(12);

    ui_update_timer   = node_->create_wall_timer(50ms, std::bind(&RobotCalcNode::show_callback, this));
    legs_update_timer = node_->create_wall_timer(10ms, std::bind(&RobotCalcNode::legs_update, this));

    comm_pos=get_grivate_center_pose(Vector3D(0.0,0.0,0.0), Vector3D(0.0,0.0,0.0), Vector3D(0.0,0.0,0.0), Vector3D(0.0,0.0,0.0));
    


    RCLCPP_INFO(node_->get_logger(), "初始化完成");
}

RobotCalcNode::~RobotCalcNode() {}

void RobotCalcNode::show_callback() {

    visualization_msgs::msg::Marker com_marker;
    com_marker.header.frame_id = "body_link";
    com_marker.header.stamp = node_->get_clock()->now();
    com_marker.ns = "center_of_mass";
    com_marker.id = 1;
    com_marker.type = visualization_msgs::msg::Marker::SPHERE;
    com_marker.action = visualization_msgs::msg::Marker::ADD;
    
    // 设置质心位置
    com_marker.pose.position.x = comm_pos[0];
    com_marker.pose.position.y = comm_pos[1];
    com_marker.pose.position.z = comm_pos[2];
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

    auto make_force_arrow =
        [&](int id,
            const Vector3D &foot_pos,
            const Vector3D &pos_offset,
            const Vector3D &foot_force)
    {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "body_link";
        m.header.stamp = now;
        m.ns = "foot_force";
        m.id = id;
        m.type = visualization_msgs::msg::Marker::ARROW;
        m.action = visualization_msgs::msg::Marker::ADD;

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
        0,
        lf_leg_calc->foot_pos(lf_joint_pos),
        lf_leg_calc->pos_offset,
        lf_leg_calc->foot_force(lf_joint_pos, lf_joint_torque, lf_forward_torque));

    // RF
    make_force_arrow(
        1,
        rf_leg_calc->foot_pos(rf_joint_pos),
        rf_leg_calc->pos_offset,
        rf_leg_calc->foot_force(rf_joint_pos, rf_joint_torque, rf_forward_torque));

    // LB
    make_force_arrow(
        2,
        lb_leg_calc->foot_pos(lb_joint_pos),
        lb_leg_calc->pos_offset,
        lb_leg_calc->foot_force(lb_joint_pos, lb_joint_torque, lb_forward_torque));

    // RB
    make_force_arrow(
        3,
        rb_leg_calc->foot_pos(rb_joint_pos),
        rb_leg_calc->pos_offset,
        rb_leg_calc->foot_force(rb_joint_pos, rb_joint_torque, rb_forward_torque));

    mark_array.markers.push_back(com_marker);
    
    marker_publisher->publish(mark_array);
    
    // RCLCPP_INFO(node_->get_logger(), "kp=%lf,kd=%lf,mass=%lf", vmc->kp, vmc->kd, vmc->mass);
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
    auto lf_foot_exp_pos   = Vector3D(0.0, 0.0, 0.0);
    auto lf_foot_exp_vel   = Vector3D(0.0, 0.0, 0.0);
    auto lf_foot_exp_acc   = Vector3D(0.0, 0.0, 0.0);
    auto lf_foot_exp_force = Vector3D(0.0, 0.0, 0.0);

    auto rf_foot_exp_pos   = Vector3D(0.0, 0.0, 0.0);
    auto rf_foot_exp_vel   = Vector3D(0.0, 0.0, 0.0);
    auto rf_foot_exp_acc   = Vector3D(0.0, 0.0, 0.0);
    auto rf_foot_exp_force = Vector3D(0.0, 0.0, 0.0);

    auto lb_foot_exp_pos   = Vector3D(0.0, 0.0, 0.0);
    auto lb_foot_exp_vel   = Vector3D(0.0, 0.0, 0.0);
    auto lb_foot_exp_acc   = Vector3D(0.0, 0.0, 0.0);
    auto lb_foot_exp_force = Vector3D(0.0, 0.0, 0.0);

    auto rb_foot_exp_pos   = Vector3D(0.0, 0.0, 0.0);
    auto rb_foot_exp_vel   = Vector3D(0.0, 0.0, 0.0);
    auto rb_foot_exp_acc   = Vector3D(0.0, 0.0, 0.0);
    auto rb_foot_exp_force = Vector3D(0.0, 0.0, 0.0);


    lf_leg_calc->pos_offset[2]=foot_pos_base_offset+foot_base_height;   //更新步态高度
    rf_leg_calc->pos_offset[2]=foot_pos_base_offset+foot_base_height;
    lb_leg_calc->pos_offset[2]=foot_pos_base_offset+foot_base_height;
    rb_leg_calc->pos_offset[2]=foot_pos_base_offset+foot_base_height;
    
    lf_leg_calc->pos_offset[0]=robot_lf_dx;     //更新支撑相中性点位置
    rf_leg_calc->pos_offset[0]=robot_rf_dx;
    lb_leg_calc->pos_offset[0]=robot_lb_dx;
    rb_leg_calc->pos_offset[0]=robot_rb_dx;

    if(robot_state==DOG_IDEL)   //单位置控制
    {
        lf_foot_exp_pos = lf_leg_stop_pos=Vector3D(0.0,0.0,0.0);
        rf_foot_exp_pos = rf_leg_stop_pos=Vector3D(0.0,0.0,0.0);
        lb_foot_exp_pos = lb_leg_stop_pos=Vector3D(0.0,0.0,0.0);
        rb_foot_exp_pos = rb_leg_stop_pos=Vector3D(0.0,0.0,0.0);
        if(robot_req_state==DOG_REQ_STOP)
            robot_state=DOG_STOP;
    }
    else if (robot_state == DOG_STOP) {                             // 狗保持站立

        lf_foot_exp_pos = lf_leg_stop_pos;
        rf_foot_exp_pos = rf_leg_stop_pos;
        lb_foot_exp_pos = lb_leg_stop_pos;
        rb_foot_exp_pos = rb_leg_stop_pos;

        auto lf_cart_pos   = lf_leg_calc->foot_pos(lf_joint_pos);
        auto lf_cart_vel   = lf_leg_calc->foot_vel(lf_joint_pos, lf_joint_vel);
        auto lf_cart_force = lf_leg_calc->foot_force(lf_joint_pos, lf_joint_torque, lf_forward_torque);
        std::tie(lf_foot_exp_pos[2], lf_foot_exp_vel[2], lf_foot_exp_acc[2]) =
            lf_z_vmc->targetUpdate(0.0, lf_cart_pos[2], 0.0, lf_cart_vel[2], -lf_cart_force[2]);
        lf_foot_exp_force = Vector3D(0.0, 0.0, -robot_lf_grivate);

        auto rf_cart_pos   = rf_leg_calc->foot_pos(rf_joint_pos);
        auto rf_cart_vel   = rf_leg_calc->foot_vel(rf_joint_pos, rf_joint_vel);
        auto rf_cart_force = rf_leg_calc->foot_force(rf_joint_pos, rf_joint_torque, rf_forward_torque);
        std::tie(rf_foot_exp_pos[2], rf_foot_exp_vel[2], rf_foot_exp_acc[2]) =
            rf_z_vmc->targetUpdate(0.0, rf_cart_pos[2], 0.0, rf_cart_vel[2], -rf_cart_force[2]);
        rf_foot_exp_force = Vector3D(0.0, 0.0, -robot_rf_grivate);

        auto lb_cart_pos   = lb_leg_calc->foot_pos(lb_joint_pos);
        auto lb_cart_vel   = lb_leg_calc->foot_vel(lb_joint_pos, lb_joint_vel);
        auto lb_cart_force = lb_leg_calc->foot_force(lb_joint_pos, lb_joint_torque, lb_forward_torque);
        std::tie(lb_foot_exp_pos[2], lb_foot_exp_vel[2], lb_foot_exp_acc[2]) =
            lb_z_vmc->targetUpdate(0.0, lb_cart_pos[2], 0.0, lb_cart_vel[2], -lb_cart_force[2]);
        lb_foot_exp_force = Vector3D(0.0, 0.0, -robot_lb_grivate);

        auto rb_cart_pos   = rb_leg_calc->foot_pos(rb_joint_pos);
        auto rb_cart_vel   = rb_leg_calc->foot_vel(rb_joint_pos, rb_joint_vel);
        auto rb_cart_force = rb_leg_calc->foot_force(rb_joint_pos, rb_joint_torque, rb_forward_torque);
        std::tie(rb_foot_exp_pos[2], rb_foot_exp_vel[2], rb_foot_exp_acc[2]) =
            rb_z_vmc->targetUpdate(0.0, rb_cart_pos[2], 0.0, rb_cart_vel[2], -rb_cart_force[2]);
        rb_foot_exp_force = Vector3D(0.0, 0.0, -robot_rb_grivate);

        if (robot_req_state == DOG_REQ_RUN)                   // 如果请求转移到行走状态，那么机器人状态先跳转到开始行走状态
            robot_state = DOG_STARTING;
        else if(robot_req_state==DOG_REQ_IDEL)
            robot_state=DOG_IDEL;
    } else if (robot_state == DOG_STARTING) {                  // 狗处于开始前进状态，规划一次初相位轨迹
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
        step1_support_updated = false;             // 设置足端轨迹更新状态
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
                    + rclcpp::Duration(std::chrono::duration<double>(std::abs(2.0 * step_support_rate - 1.0) * step_time*0.5));
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
                    lf_leg_calc->foot_pos(lf_joint_pos),-Vector3D(lf_exp_vel[0],lf_exp_vel[1],0.0), lf_exp_vel,
                    step_time * (1.0 - step_support_rate), step_height);
                // 主相对角腿也需要规划飞行轨迹（右后）
                rb_leg_step.update_flight_trajectory(
                    rb_leg_calc->foot_pos(rb_joint_pos), -Vector3D(rb_exp_vel[0],rb_exp_vel[1],0.0), rb_exp_vel,
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
                rf_leg_step.update_support_trajectory(
                    rf_leg_calc->foot_pos(rf_joint_pos), rf_exp_vel,
                    step_support_rate * step_time);                // 预更新支撑相(精确结束时间由主相位确定)
                // 从相对角腿也同步进入支撑相（左后）
                lb_leg_step.update_support_trajectory(lb_leg_calc->foot_pos(lb_joint_pos), lb_exp_vel, step_support_rate * step_time);
                slave_phrase_start_time = now;
                slave_phrase_stop_time  = now + rclcpp::Duration(std::chrono::duration<double>(step_support_rate * step_time));
                if (robot_req_state == DOG_REQ_STOP){                   // 请求状态为停止，那么状态机跳转到正在停止
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
                    rf_leg_calc->foot_pos(rf_joint_pos), -Vector3D(rf_exp_vel[0],rf_exp_vel[1],0.0), rf_exp_vel,
                    (1.0 - step_support_rate) * step_time, step_height);
                lb_leg_step.update_flight_trajectory(
                    lb_leg_calc->foot_pos(lb_joint_pos), -Vector3D(lb_exp_vel[0],lb_exp_vel[1],0.0), lb_exp_vel,
                    (1.0 - step_support_rate) * step_time, step_height);
                slave_phrase_start_time = now;
                RCLCPP_INFO(node_->get_logger(), "从相位摆动相规划");
            }
        }

        bool success[4];
        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) =
            lf_leg_step.get_target((now - main_phrase_start_time).seconds(), success[0]); // 得到狗腿当前期望
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) =
            rf_leg_step.get_target((now - slave_phrase_start_time).seconds(), success[1]);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) =
            lb_leg_step.get_target((now - slave_phrase_start_time).seconds(), success[2]);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) =
            rb_leg_step.get_target((now - main_phrase_start_time).seconds(), success[3]);

        // RCLCPP_INFO(node_->get_logger(),"rf:(%lf,%lf,%lf),success=(%d,%d,%d,%d)",rf_foot_exp_pos[0],rf_foot_exp_pos[1],rf_foot_exp_pos[2],success[0],success[1],success[2],success[3]);

        auto lf_cart_pos   = lf_leg_calc->foot_pos(lf_joint_pos);
        auto lf_cart_vel   = lf_leg_calc->foot_vel(lf_joint_pos, lf_joint_vel);
        auto lf_cart_force = lf_leg_calc->foot_force(lf_joint_pos, lf_joint_torque, lf_forward_torque);
        auto rb_cart_pos   = rb_leg_calc->foot_pos(rb_joint_pos);
        auto rb_cart_vel   = rb_leg_calc->foot_vel(rb_joint_pos, rb_joint_vel);
        auto rb_cart_force = rb_leg_calc->foot_force(rb_joint_pos, rb_joint_torque, rb_forward_torque);
        auto rf_cart_pos   = rf_leg_calc->foot_pos(rf_joint_pos);
        auto rf_cart_vel   = rf_leg_calc->foot_vel(rf_joint_pos, rf_joint_vel);
        auto rf_cart_force = rf_leg_calc->foot_force(rf_joint_pos, rf_joint_torque, rf_forward_torque);
        auto lb_cart_pos   = lb_leg_calc->foot_pos(lb_joint_pos);
        auto lb_cart_vel   = lb_leg_calc->foot_vel(lb_joint_pos, lb_joint_vel);
        auto lb_cart_force = lb_leg_calc->foot_force(lb_joint_pos, lb_joint_torque, lb_forward_torque);

        if (step1_support_updated) { // 主相位需要VMC计算
            std::tie(lf_foot_exp_pos[2], lf_foot_exp_vel[2], lf_foot_exp_acc[2]) =
                lf_z_vmc->targetUpdate(lf_foot_exp_pos[2], lf_cart_pos[2], lf_foot_exp_vel[2], lf_cart_vel[2], -lf_cart_force[2]);

            std::tie(rb_foot_exp_pos[2], rb_foot_exp_vel[2], rb_foot_exp_acc[2]) =
                rb_z_vmc->targetUpdate(rb_foot_exp_pos[2], rb_cart_pos[2], rb_foot_exp_vel[2], rb_cart_vel[2], -rb_cart_force[2]);

            if (step2_support_updated) { // 如果从相位也需要VMC计算，说明此时四足触底，每个脚的向下的力为一倍，否则为两倍
                lf_foot_exp_force = Vector3D(0.0, 0.0, -robot_lf_grivate);
                rb_foot_exp_force = Vector3D(0.0, 0.0, -robot_rb_grivate);
            } else {
                lf_foot_exp_force = Vector3D(0.0, 0.0, -2.0 * robot_lf_grivate);
                rb_foot_exp_force = Vector3D(0.0, 0.0, -2.0 * robot_rb_grivate);
            }
        }
        if (step2_support_updated) {     // 从相位需要VMC计算
            
            std::tie(rf_foot_exp_pos[2], rf_foot_exp_vel[2], rf_foot_exp_acc[2]) =
                rf_z_vmc->targetUpdate(rf_foot_exp_pos[2], rf_cart_pos[2], rf_foot_exp_vel[2], rf_cart_vel[2], -rf_cart_force[2]);
            
            std::tie(lb_foot_exp_pos[2], lb_foot_exp_vel[2], lb_foot_exp_acc[2]) =
                lb_z_vmc->targetUpdate(lb_foot_exp_pos[2], lb_cart_pos[2], lb_foot_exp_vel[2], lb_cart_vel[2], -lb_cart_force[2]);

            if (step1_support_updated) {
                rf_foot_exp_force = Vector3D(0.0, 0.0, -robot_rf_grivate);
                lb_foot_exp_force = Vector3D(0.0, 0.0, -robot_lb_grivate);
            } else {
                rf_foot_exp_force = Vector3D(0.0, 0.0, -2.0 * robot_rf_grivate);
                lb_foot_exp_force = Vector3D(0.0, 0.0, -2.0 * robot_lb_grivate);
            }
        }

        //TODO:水平方向VMC
        // std::tie(lf_foot_exp_pos[0],lf_foot_exp_vel[0],lf_foot_exp_acc[0])=lf_x_vmc->targetUpdate(lf_foot_exp_pos[0], lf_cart_pos[0], lf_foot_exp_vel[0], lf_cart_vel[0], -lf_cart_force[0]);   //实际这个lf_cart_force是足端施加的力，不是受到的力
        // std::tie(lf_foot_exp_pos[1],lf_foot_exp_vel[1],lf_foot_exp_acc[1])=lf_y_vmc->targetUpdate(lf_foot_exp_pos[1], lf_cart_pos[1], lf_foot_exp_vel[1], lf_cart_vel[1], -lf_cart_force[1]);

        // std::tie(rf_foot_exp_pos[0],rf_foot_exp_vel[0],rf_foot_exp_acc[0])=rf_x_vmc->targetUpdate(rf_foot_exp_pos[0], rf_cart_pos[0], rf_foot_exp_vel[0], rf_cart_vel[0], -rf_cart_force[0]);
        // std::tie(rf_foot_exp_pos[1],rf_foot_exp_vel[1],rf_foot_exp_acc[1])=rf_y_vmc->targetUpdate(rf_foot_exp_pos[1], rf_cart_pos[1], rf_foot_exp_vel[1], rf_cart_vel[1], -rf_cart_force[1]);

        // std::tie(lb_foot_exp_pos[0],lb_foot_exp_vel[0],lb_foot_exp_acc[0])=lb_x_vmc->targetUpdate(lb_foot_exp_pos[0], lb_cart_pos[0], lb_foot_exp_vel[0], lb_cart_vel[0], -lb_cart_force[0]);
        // std::tie(lb_foot_exp_pos[1],lb_foot_exp_vel[1],lb_foot_exp_acc[1])=lb_y_vmc->targetUpdate(lb_foot_exp_pos[1], lb_cart_pos[1], lb_foot_exp_vel[1], lb_cart_vel[1], -lb_cart_force[1]);

        // std::tie(rb_foot_exp_pos[0],rb_foot_exp_vel[0],rb_foot_exp_acc[0])=rb_x_vmc->targetUpdate(rb_foot_exp_pos[0], rb_cart_pos[0], rb_foot_exp_vel[0], rb_cart_vel[0], -rb_cart_force[0]);
        // std::tie(rb_foot_exp_pos[1],rb_foot_exp_vel[1],rb_foot_exp_acc[1])=rb_y_vmc->targetUpdate(rb_foot_exp_pos[1], rb_cart_pos[1], rb_foot_exp_vel[1], rb_cart_vel[1], -rb_cart_force[1]);
        
    } else if (robot_state == DOG_ENDING) {
        lf_leg_stop_pos = lf_leg_calc->foot_pos(lf_joint_pos);
        rf_leg_stop_pos = rf_leg_calc->foot_pos(rf_joint_pos);
        lb_leg_stop_pos = lb_leg_calc->foot_pos(lb_joint_pos);
        rb_leg_stop_pos = rb_leg_calc->foot_pos(rb_joint_pos);


        lf_foot_exp_pos = lf_leg_stop_pos; // 更新位置目标为当前位置,
        rf_foot_exp_pos = rf_leg_stop_pos;
        lb_foot_exp_pos = lb_leg_stop_pos;
        rb_foot_exp_pos = rb_leg_stop_pos;

        robot_state = DOG_STOP;
    }
    else if(robot_state == DOG_CLIMB_STEPS)  //需要登上台阶
    {
        //TODO:计算机器人质心位置
        //TODO:计算质心在水平面上的投影
        //TODO:根据质心位置计算足端施加力的方向和大小（在地面坐标系下垂直向上，向上的力的大小按照到质心的距离分配）
        //TODO:执行VMC
        //TODO:将VMC计算的力转换到机器人坐标系下
        //TODO:(可选)使用pitch和roll方向简单的PD控制器增加对机器人姿态的控制
    }
    else if(robot_state == DOG_CROSS_WALL)  //需要跨过高墙
    {
        //TODO:狗子左前褪抬起来，其它三条腿支撑，左前褪搭上高墙
        //TODO:右前腿搭上高墙
        //TODO:前面两条腿往后蹬，把狗身往前移
        //TODO:后腿由后面抬起，经过侧面摆到前面，利用小腿推高墙，进而通过高墙
    }

    auto lf_leg_joints_target = signal_leg_calc(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc, lf_foot_exp_force, lf_leg_calc);
    auto rf_leg_joints_target = signal_leg_calc(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc, rf_foot_exp_force, rf_leg_calc);
    auto lb_leg_joints_target = signal_leg_calc(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc, lb_foot_exp_force, lb_leg_calc);
    auto rb_leg_joints_target = signal_leg_calc(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc, rb_foot_exp_force, rb_leg_calc);

    lf_forward_torque = std::get<2>(lf_leg_joints_target);      //更新本周期计算的前馈力矩准备用作下一个控制周期的计算
    rf_forward_torque = std::get<2>(rf_leg_joints_target);
    lb_forward_torque = std::get<2>(lb_leg_joints_target);
    rb_forward_torque = std::get<2>(rb_leg_joints_target);

    //发布话题控制ROS2_control控制器、物理机器人
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

    rviz2_update_cnt++;
    if (rviz2_update_cnt == 5) {    //发布话题在RVIZ2中查看
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

        // for(int i=0;i<12;i++)   //在RVIZ2中显示期望
        //     joint_display_msg.position[i]=joints_target.legs[i/3].joints[i%3].rad;

        joint_display_msg.header.stamp = node_->get_clock()->now();
        rviz_joint_publisher->publish(joint_display_msg);

#if 0
        // 离线模拟、认为关节立即到达发布的目标位置
        robot_interfaces::msg::Robot msg = joints_target;
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
#endif
    }

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = node_->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "body_link";
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;

    t.transform.rotation.x = robot_rotation.x();
    t.transform.rotation.y = robot_rotation.y();
    t.transform.rotation.z = robot_rotation.z();
    t.transform.rotation.w = robot_rotation.w();

    robot_tf_broadcaster->sendTransform(t);

}


Vector3D RobotCalcNode::get_grivate_center_pose(const Vector3D& lf_joint_pos, const Vector3D& rf_joint_pos, const Vector3D& lb_joint_pos, const Vector3D& rb_joint_pos) {
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
            const double m                  = rbi.getMass();
            if (m <= 0.0) {
                continue;
            }

            // COM 在 segment 坐标系下的位置
            const KDL::Vector c_seg = rbi.getCOG();
            // 转到基座坐标系
            const KDL::Vector c_base = T * c_seg;

            mass_sum += m;
            com_sum  = com_sum + c_base * m;
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
                com_sum  = com_sum + rbi.getCOG() * m; // body_link 在 body_link 坐标系下
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
    double& w,  double& x,  double& y,  double& z,
    double  w1, double  x1, double  y1, double  z1,
    double alpha)
{

    auto normalizeQuaternion=[](double& w, double& x, double& y, double& z){
        double norm = std::sqrt(w*w + x*x + y*y + z*z);
    if (norm < 1e-12) {
        // 退化情况，回到单位四元数
        w = 1.0; x = y = z = 0.0;
        return;
    }
    w /= norm;
    x /= norm;
    y /= norm;
    z /= norm;
    };


    // 1. 单位化输入（防御性编程）
    normalizeQuaternion(w,  x,  y,  z);
    normalizeQuaternion(w1, x1, y1, z1);

    // 2. 点积，判断是否需要取反（双覆盖问题）
    double dot = w*w1 + x*x1 + y*y1 + z*z1;
    if (dot < 0.0) {
        w1 = -w1;
        x1 = -x1;
        y1 = -y1;
        z1 = -z1;
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
    double theta = std::acos(dot);
    double sin_theta = std::sin(theta);

    double w_a = std::sin((1.0 - alpha) * theta) / sin_theta;
    double w_b = std::sin(alpha * theta) / sin_theta;

    double w_new = w_a * w  + w_b * w1;
    double x_new = w_a * x  + w_b * x1;
    double y_new = w_a * y  + w_b * y1;
    double z_new = w_a * z  + w_b * z1;

    w = w_new;
    x = x_new;
    y = y_new;
    z = z_new;

    // 5. 再次单位化（强烈建议保留）
    normalizeQuaternion(w, x, y, z);
}

#include "dog_calc.hpp"
#include "step.h"
#include <Eigen/src/Core/Matrix.h>
#include <chrono>
#include <kdl/frames.hpp>
#include <rclcpp/duration.hpp>
#include <robot_interfaces/msg/robot.hpp>
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


    node_->declare_parameter("force_filter_gate", 0.8);
    node_->declare_parameter("enable_vmc", false);
    node_->declare_parameter("vmc_kp", 300.0);
    node_->declare_parameter("vmc_kd", 100.0);
    node_->declare_parameter("vmc_mass", 4.0);

    node_->declare_parameter("lf_grivate", 15.0);
    node_->declare_parameter("rf_grivate", 15.0);
    node_->declare_parameter("lb_grivate", 20.0);
    node_->declare_parameter("rb_grivate", 20.0);
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
            else if (param.get_name() == "force_filter_gate")
                force_filter_gate = param.as_double();
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
    node_->get_parameter("force_filter_gate", force_filter_gate);
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

    marker_publisher = node_->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

    rviz_joint_publisher = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    legs_target_pub = node_->create_publisher<robot_interfaces::msg::Robot>("legs_target", 10); // 创建期望位置发布者

    imu_sub=node_->create_subscription<sensor_msgs::msg::Imu>("imu",rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::Imu &msg){
        //.//RCLCPP_INFO(node_->get_logger(),"posture:%lf,%lf,%lf,%lf",msg.orientation.w,msg.orientation.x,msg.orientation.y,msg.orientation.z);
        robot_rotation.setW(msg.orientation.w);
        robot_rotation.setX(msg.orientation.x);
        robot_rotation.setY(msg.orientation.y);
        robot_rotation.setZ(msg.orientation.z);
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

    RCLCPP_INFO(node_->get_logger(), "初始化完成");
}

RobotCalcNode::~RobotCalcNode() {}

void RobotCalcNode::show_callback() {
    // visualization_msgs::msg::Marker dot_marker;
    // dot_marker.header.frame_id = "body_link"; // 设置坐标系
    // dot_marker.header.stamp    = node_->get_clock()->now();
    // dot_marker.ns              = "points";
    // dot_marker.id              = 0;
    // dot_marker.type            = visualization_msgs::msg::Marker::SPHERE;
    // dot_marker.action          = visualization_msgs::msg::Marker::ADD;

    // dot_marker.pose.position.x = lf_leg_calc->pos_offset[0] + lf_cart_target[0];
    // dot_marker.pose.position.y = lf_leg_calc->pos_offset[1] + lf_cart_target[0];
    // dot_marker.pose.position.z = lf_leg_calc->pos_offset[2] + lf_cart_target[0];
    // //  设置球体的尺寸
    // dot_marker.scale.x = 0.1;
    // dot_marker.scale.y = 0.1;
    // dot_marker.scale.z = 0.1;
    // // 设置颜色
    // dot_marker.color.a = 1.0;              // 不透明
    // dot_marker.color.r = 1.0;
    // dot_marker.color.g = 0.0;
    // dot_marker.color.b = 0.0;

    // marker_publisher->publish(dot_marker); // 发布点标记（狗腿足端位置）

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


    visualization_msgs::msg::Marker arraw_marker;
    arraw_marker.header.frame_id = "body_link"; // 选择你在 TF 树中有的 frame
    arraw_marker.header.stamp    = node_->get_clock()->now();
    arraw_marker.ns              = "arrows";
    arraw_marker.id              = 0;
    // 类型：箭头
    arraw_marker.type   = visualization_msgs::msg::Marker::ARROW;
    arraw_marker.action = visualization_msgs::msg::Marker::ADD;

    auto foot_pos   = rf_leg_calc->foot_pos(rf_joint_pos);
    auto foot_force = rf_leg_calc->foot_force(rf_joint_pos, rf_joint_torque, rf_forward_torque);

    geometry_msgs::msg::Point p_start, p_end;
    p_start.x = foot_pos[0] + rf_leg_calc->pos_offset[0];
    p_start.y = foot_pos[1] + rf_leg_calc->pos_offset[1];
    p_start.z = foot_pos[2] + rf_leg_calc->pos_offset[2];

    p_end.x = p_start.x + foot_force[0] * 0.05f;
    p_end.y = p_start.y + foot_force[1] * 0.05f;
    p_end.z = p_start.z + foot_force[2] * 0.05f;

    arraw_marker.points.push_back(p_start);
    arraw_marker.points.push_back(p_end);

    arraw_marker.scale.x = 0.03;
    arraw_marker.scale.y = 0.03;
    arraw_marker.scale.z = 0.03;
    // 设置颜色
    arraw_marker.color.a = 1.0;              // 不透明
    arraw_marker.color.r = 1.0;
    arraw_marker.color.g = 1.0;
    arraw_marker.color.b = 0.0;

    arraw_marker.lifetime = rclcpp::Duration(0, 0);

    marker_publisher->publish(arraw_marker); // 发布箭头标记（狗腿足端受力）*/

    // RCLCPP_INFO(node_->get_logger(), "kp=%lf,kd=%lf,mass=%lf", vmc->kp, vmc->kd, vmc->mass);
}

std::tuple<Vector3D, Vector3D, Vector3D> RobotCalcNode::signal_leg_calc(
    const Vector3D& exp_cart_pos, const Vector3D& exp_cart_vel, const Vector3D& exp_cart_acc, const Vector3D& exp_cart_force,
    std::shared_ptr<LegCalc> leg_calc) {
    Vector3D joint_pos, joint_omega, joint_torque;

    (void)exp_cart_acc;                                        // 目前还没有实现将笛卡尔的加速度转为关节空间的加速度，所以这个参数先不用

    int result;
    joint_pos    = leg_calc->joint_pos(exp_cart_pos, &result); // 一般这个位置不可能会迭代失败，所以不再对result进行处理
    joint_omega  = leg_calc->joint_vel(joint_pos, exp_cart_vel);
    joint_torque = leg_calc->joint_torque_foot_force(joint_pos, exp_cart_force);
    joint_torque += leg_calc->joint_torque_dynamic(joint_pos, joint_omega, Vector3D(0.0, 0.0, 0.0));
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

        if (step1_support_updated) { // 主相位需要VMC计算
            auto lf_cart_pos   = lf_leg_calc->foot_pos(lf_joint_pos);
            auto lf_cart_vel   = lf_leg_calc->foot_vel(lf_joint_pos, lf_joint_vel);
            auto lf_cart_force = lf_leg_calc->foot_force(lf_joint_pos, lf_joint_torque, lf_forward_torque);
            std::tie(lf_foot_exp_pos[2], lf_foot_exp_vel[2], lf_foot_exp_acc[2]) =
                lf_z_vmc->targetUpdate(lf_foot_exp_pos[2], lf_cart_pos[2], lf_foot_exp_vel[2], lf_cart_vel[2], -lf_cart_force[2]);

            // 主相为左前与右后对角支撑，同时对右后腿使用VMC进行竖直方向的修正
            auto rb_cart_pos   = rb_leg_calc->foot_pos(rb_joint_pos);
            auto rb_cart_vel   = rb_leg_calc->foot_vel(rb_joint_pos, rb_joint_vel);
            auto rb_cart_force = rb_leg_calc->foot_force(rb_joint_pos, rb_joint_torque, rb_forward_torque);
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
            // 从相为右前与左后对角支撑，对右前和左后腿进行竖直方向VMC修正
            auto rf_cart_pos   = rf_leg_calc->foot_pos(rf_joint_pos);
            auto rf_cart_vel   = rf_leg_calc->foot_vel(rf_joint_pos, rf_joint_vel);
            auto rf_cart_force = rf_leg_calc->foot_force(rf_joint_pos, rf_joint_torque, rf_forward_torque);
            std::tie(rf_foot_exp_pos[2], rf_foot_exp_vel[2], rf_foot_exp_acc[2]) =
                rf_z_vmc->targetUpdate(rf_foot_exp_pos[2], rf_cart_pos[2], rf_foot_exp_vel[2], rf_cart_vel[2], -rf_cart_force[2]);


            auto lb_cart_pos   = lb_leg_calc->foot_pos(lb_joint_pos);
            auto lb_cart_vel   = lb_leg_calc->foot_vel(lb_joint_pos, lb_joint_vel);
            auto lb_cart_force = lb_leg_calc->foot_force(lb_joint_pos, lb_joint_torque, lb_forward_torque);
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
}

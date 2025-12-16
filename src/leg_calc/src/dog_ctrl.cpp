#include "dog_calc.hpp"
#include <chrono>

using namespace std::chrono_literals;

RobotCalcNode::RobotCalcNode(const rclcpp::Node::SharedPtr node) {
    node_ = node;
    vmc   = new VMC(200, 60, 5.0, 0.5, 0.2, 0.1, 20ms); // 创建VMC计算对象

    node_->declare_parameter("joint1_kp", 2.4);
    node_->declare_parameter("joint2_kp", 3.2);
    node_->declare_parameter("joint3_kp", 1.5);
    node_->declare_parameter("joint1_kd", 0.15);
    node_->declare_parameter("joint2_kd", 0.16);
    node_->declare_parameter("joint3_kd", 0.18);
    node_->declare_parameter("force_filter_gate", 0.8);
    node_->declare_parameter("enable_vmc", false);
    node_->declare_parameter("vmc_kp", 350.0);
    node_->declare_parameter("vmc_kd", 50.0);
    node_->declare_parameter("vmc_mass", 5.0);

    param_server_ =
        node_->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            RCLCPP_INFO(node_->get_logger(), "更新参数");
            for (const auto& param : params) {
                if (param.get_name() == "enable_vmc")
                    enable_vmc = param.as_bool();
                else if (param.get_name() == "joint1_kp")
                    joint1_kp = param.as_double();
                else if (param.get_name() == "joint1_kd")
                    joint1_kd = param.as_double();
                else if (param.get_name() == "joint2_kp")
                    joint2_kp = param.as_double();
                else if (param.get_name() == "joint2_kd")
                    joint2_kd = param.as_double();
                else if (param.get_name() == "joint3_kp")
                    joint3_kp = param.as_double();
                else if (param.get_name() == "joint3_kd")
                    joint3_kd = param.as_double();
                else if (param.get_name() == "force_filter_gate")
                    force_filter_gate = param.as_double();
                else if (param.get_name() == "vmc_kp")
                    vmc->kp = param.as_double();
                else if (param.get_name() == "vmc_kd")
                    vmc->kd = param.as_double();
                else if (param.get_name() == "vmc_mass")
                    vmc->mass = param.as_double();
            }
            return result;
        });

    joint1_kp = node_->get_parameter("joint1_kp").as_double();
    joint1_kd = node_->get_parameter("joint1_kd").as_double();
    joint2_kp = node_->get_parameter("joint2_kp").as_double();
    joint2_kd = node_->get_parameter("joint2_kd").as_double();
    joint3_kp = node_->get_parameter("joint3_kp").as_double();
    joint3_kd = node_->get_parameter("joint3_kd").as_double();

    marker_publisher =
        node_->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

    rviz_joint_publisher = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    legs_target_pub = node_->create_publisher<robot_interfaces::msg::Robot>(
        "legs_target", 10); // 创建期望位置发布者

    legs_state_sub = node_->create_subscription<robot_interfaces::msg::Robot>(
        "legs_status", 10, [this](const robot_interfaces::msg::Robot& msg) {
            //RCLCPP_INFO(this->get_logger(), "订阅到最新的电机状态");
            
        });

    robot_description_param_ =
        std::make_shared<rclcpp::SyncParametersClient>(node_, "/robot_state_publisher");

    auto params = robot_description_param_->get_parameters({"robot_description"});
    urdf_xml    = params[0].as_string();
    if (urdf_xml.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "无法读取URDF文件，不能进行动力学计算");
        return;
    }

    kdl_parser::treeFromString(urdf_xml, tree);         // 解析四条腿的KDL树结构
    tree.getChain("body_link", "lf_link3", lf_leg_chain);
    tree.getChain("body_link", "rf_link3", rf_leg_chain);
    tree.getChain("body_link", "lb_link3", lb_leg_chain);
    tree.getChain("body_link", "rb_link3", rb_leg_chain);

    // 初始化狗腿解算器
    lf_leg_calc = std::make_unique<LegCalc>(lf_leg_chain);
    rf_leg_calc = std::make_unique<LegCalc>(rf_leg_chain);
    lb_leg_calc = std::make_unique<LegCalc>(lb_leg_chain);
    rb_leg_calc = std::make_unique<LegCalc>(rb_leg_chain);
    lf_leg_calc->pos_offset<<0.21,0.16,-0.25;   //设置足端到机器人中心的偏移


    joint_msg.name = {"lf_joint1", "lf_joint2", "lf_joint3", "rf_joint1", "rf_joint2", "rf_joint3",
                      "lb_joint1", "lb_joint2", "lb_joint3", "rb_joint1", "rb_joint2", "rb_joint3"};
    joint_msg.position.resize(12);


    lf_leg_current_rad.resize(3);
    lf_leg_current_omega.resize(3);
    lf_leg_current_torque.resize(3);
    lf_leg_target_rad.resize(3);
    lf_leg_target_omega.resize(3);
    lf_leg_target_torque.resize(3);

    lf_leg_current_rad(2)=1.6;
    lf_leg_target_rad(2)=1.6;

    lf_cart_target={0.0,0.0,0.0};

    last_step_reset_time=node_->get_clock()->now();


    ui_update_timer =
        node_->create_wall_timer(50ms, std::bind(&RobotCalcNode::show_callback, this));
    legs_update_timer =
        node_->create_wall_timer(50ms, std::bind(&RobotCalcNode::legs_update, this));

    
    current_joint_pos.reserve(12);
    RCLCPP_INFO(node_->get_logger(),"初始化完成");
}

RobotCalcNode::~RobotCalcNode() {
    delete vmc;
}

void RobotCalcNode::show_callback() {
    visualization_msgs::msg::Marker dot_marker;
    dot_marker.header.frame_id = "body_link";     // 设置坐标系
    dot_marker.header.stamp    = node_->get_clock()->now();
    dot_marker.ns              = "points";
    dot_marker.id              = 0;
    dot_marker.type            = visualization_msgs::msg::Marker::SPHERE;
    dot_marker.action          = visualization_msgs::msg::Marker::ADD;

    dot_marker.pose.position.x = lf_cart_target.x()+lf_leg_calc->pos_offset[0];
    dot_marker.pose.position.y = lf_cart_target.y()+lf_leg_calc->pos_offset[1];
    dot_marker.pose.position.z = lf_cart_target.z()+lf_leg_calc->pos_offset[2];
    //  设置球体的尺寸
    dot_marker.scale.x = 0.1;
    dot_marker.scale.y = 0.1;
    dot_marker.scale.z = 0.1;
    // 设置颜色
    dot_marker.color.a = 1.0;               // 不透明
    dot_marker.color.r = 1.0;
    dot_marker.color.g = 0.0;
    dot_marker.color.b = 0.0;

    marker_publisher->publish(dot_marker);  // 发布点标记（狗腿足端位置）

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

    //RCLCPP_INFO(node_->get_logger(), "kp=%lf,kd=%lf,mass=%lf", vmc->kp, vmc->kd, vmc->mass);
}

void RobotCalcNode::legs_update() {
    // TODO:更新当前个狗腿状态
    lf_leg_calc->set_leg_state(lf_leg_current_rad, lf_leg_current_omega, lf_leg_current_torque);
    // TODO:生成步态

    if(node_->get_clock()->now()-last_step_reset_time>rclcpp::Duration(2,0))    //如果时间差大于2s
    {
        lf_cart_target={0.0,0.0,0.0};
    }
    //auto cur_step_time=node_->get_clock()->now()-last_step_reset_time;
    //lf_cart_target[2]=cur_step_time.seconds()*0.1f; //没秒上升0.1m
    
    // TODO:计算关节空间期望
    lf_cart_target.x(0.3);
    int ret = lf_leg_calc->joint_pos(lf_cart_target, lf_leg_target_rad);
    // TODO:写入目标并发布
    if (ret) {
        joint_msg.position[0] = lf_leg_target_rad(0);
        joint_msg.position[1] = lf_leg_target_rad(1);
        joint_msg.position[2] = lf_leg_target_rad(2);
    } else {
        RCLCPP_WARN(node_->get_logger(), "左前腿逆解失败");
    }
    RCLCPP_INFO(node_->get_logger(),"求解完成，发布关节状态,ret=%d",ret);

    // Debug：将狗腿状态直接设为期望
    lf_leg_current_rad = lf_leg_target_rad;


    joint_msg.header.stamp = node_->get_clock()->now();
    rviz_joint_publisher->publish(joint_msg);
}

#include "arm_calc.hpp"
#include <algorithm>
#include <chrono>
#include <kdl/frames.hpp>
#include <rclcpp/duration.hpp>

using namespace std::chrono_literals;
ArmCalcNode::ArmCalcNode(KDL::Chain& chain) : rclcpp::Node("arm_calc_node"),
   fk_solver(KDL::Chain()),
    ik_pos_solver(KDL::Chain(), Eigen::Vector<double,6>(1.0, 1.0, 1.0, 0.0, 0.0, 0.0), 1e-6, 150, 1e-10),
    jacobain_solver(KDL::Chain()),
    _temp_joint4_array(4),
    last_exp_joint_pos(4),
    temp_jacobain(4)
{
    // 初始化发布者（修正类型匹配）
    motor_target_pub = this->create_publisher<robot_interfaces::msg::MotorTarget>("motor_target", 10);
    
    // 初始化订阅者
    motor_state_sub = this->create_subscription<robot_interfaces::msg::MotorState>(
        "motor_state", 10, std::bind(&ArmCalcNode::motor_state_callback, this, std::placeholders::_1));

        this->declare_parameter("exp_end_pos,", 0.0);
        param_server = this->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        RCLCPP_INFO(this->get_logger(), "更新参数");
        std::string name;
        for (const auto& param : params) {
            name = param.get_name();
            if (name == "exp_end_pos[0]") {
                RCLCPP_INFO(this->get_logger(), "x更新");
               exp_end_pos[0] = param.as_double();
            }
            if (name == "exp_end_pos1[1]") {
                RCLCPP_INFO(this->get_logger(), "x更新");
               exp_end_pos[1] = param.as_double();
            }
            if (name == "exp_end_pos[2]") {
                RCLCPP_INFO(this->get_logger(), "x更新");
               exp_end_pos[2] = param.as_double();
            }
}
return result;
}
);
    this->get_parameter("exp_end_pos[0]", exp_end_pos[0]);
    this->get_parameter("exp_end_pos[1]", exp_end_pos[1]);
    this->get_parameter("exp_end_pos[2]", exp_end_pos[2]);

  
 _temp_joint4_array.resize(4);
  last_exp_joint_pos.resize(4);
}

ArmCalcNode::~ArmCalcNode(){} 

void ArmCalcNode::motor_state_callback(const robot_interfaces::msg::MotorState::SharedPtr msg)
{
//1
    joint_pos[0] = msg->gmpositions; 
    joint_pos[1] = msg->pos;          
    joint_pos[2] = std::clamp(msg->upper / 20000.0 * 180.0, 0.0, 180.0);
    joint_pos[3] = std::clamp(msg->lower / 20000.0 * 180.0, 0.0, 180.0);
    joint_vel[0] = msg->gmvel;
    joint_vel[1] = msg->vel;

 //2   
    current_end_pos = arm_end_pos(joint_pos);
    current_end_vel = arm_end_vel(joint_pos, joint_vel);
    update_flight_trajectory(current_end_pos, current_end_vel, exp_end_pos, traj_total_time);
    //3
    int result = 0;
    bool success = false;
    std::tie(target_end_pos, target_end_vel, target_end_acc) = get_target(current_t, success);
    target_joint_pos = joints_pos(target_end_pos, &result);
    target_joint_omega[0] = msg->gmvel;
    target_joint_omega[1] = msg->vel;
//4
    robot_interfaces::msg::MotorTarget motor_target;
    motor_target.except_pos = target_joint_pos[1];
    motor_target.gm_except_position = target_joint_pos[0];
    motor_target.except_omega = target_joint_omega[1];
    motor_target.gm_except_speed = target_joint_omega[0];
    motor_target.except_torque = msg->torque;
    motor_target.except_up = target_joint_pos[2]*20000/180;
    motor_target.except_low = target_joint_pos[3]*20000/180;
    motor_target_pub->publish(motor_target);

 
}
static inline double get_quintic_value(const ArmCalcNode::QuinticLineParam_t& line, const double time) {
    return line.a + line.b * time + line.c * time * time + line.d * time * time * time + line.e * time * time * time * time
         + line.f * time * time * time * time * time;
}


static inline double get_quintic_dt(const ArmCalcNode::QuinticLineParam_t& line, const double time) {
    return line.b + 2.0f * line.c * time + 3.0f * line.d * time * time + 4.0f * line.e * time * time * time
         + 5.0f * line.f * time * time * time * time;
}
static inline double get_quintic_dtdt(const ArmCalcNode::QuinticLineParam_t& line, const double time) {
    return 2.0f * line.c + 6.0f * line.d * time + 12.0f * line.e * time * time + 20.0f * line.f * time * time * time;
}

void ArmCalcNode::update_flight_trajectory(
    const Vector3D& cur_pos,
    const Vector3D& cur_vel,
    const Vector3D& exp_pos,
    const double time)
{
    flight_trajectory.time = time;
    set_quintic(flight_trajectory.lx, cur_pos[0], cur_vel[0], 0.0, exp_pos[0], 0.0, 0.0, time);
    set_quintic(flight_trajectory.ly, cur_pos[1], cur_vel[1], 0.0, exp_pos[1], 0.0, 0.0, time);
    set_quintic(flight_trajectory.lz, cur_pos[2], cur_vel[2], 0.0, exp_pos[2], 0.0, 0.0, time);

    flight_trajectory_is_available = true;
}


void ArmCalcNode::set_quintic(QuinticLineParam_t& seg,
                             double p0, double v0, double a0,
                             double pT, double vT, double aT,
                             double dt)
{
    double T  = dt;
    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T3 * T;
    double T5 = T4 * T;

    seg.a = p0;
    seg.b = v0;
    seg.c = 0.5 * a0;

    seg.d = (10 * (pT - p0) - (6 * v0 + 4 * vT) * T - (1.5 * a0 - 0.5 * aT) * T2) / T3;
    seg.e = (-15 * (pT - p0) + (8 * v0 + 7 * vT) * T + (1.5 * a0 - aT) * T2) / T4;
    seg.f = (6 * (pT - p0) - (3 * v0 + 3 * vT) * T - (0.5 * a0 - 0.5 * aT) * T2) / T5;
}

std::tuple<Vector3D, Vector3D, Vector3D> ArmCalcNode::get_target(double time, bool& success)
{
    Vector3D pos, vel, acc;
    success = false;
    if (flight_trajectory_is_available) {
        // 限制时间范围
        double t = std::clamp(time, 0.0, flight_trajectory.time);
        success = (t < flight_trajectory.time);

        pos[0] = get_quintic_value(flight_trajectory.lx, t);
        vel[0] = get_quintic_dt(flight_trajectory.lx, t);
        acc[0] = get_quintic_dtdt(flight_trajectory.lx, t);

        pos[1] = get_quintic_value(flight_trajectory.ly, t);
        vel[1] = get_quintic_dt(flight_trajectory.ly, t);
        acc[1] = get_quintic_dtdt(flight_trajectory.ly, t);

        pos[2] = get_quintic_value(flight_trajectory.lz, t);
        vel[2] = get_quintic_dt(flight_trajectory.lz, t);
        acc[2] = get_quintic_dtdt(flight_trajectory.lz, t);
    }

    return std::make_tuple(pos, vel, acc);
}

// 机械臂正解：关节角度→末端位置（复用原有KDL逻辑）
Vector3D ArmCalcNode::arm_end_pos(const Eigen::Vector4d& joint_rad)
{
    KDL::Frame frame;
    _temp_joint4_array(0) = joint_rad[0];
    _temp_joint4_array(1) = joint_rad[1];
    _temp_joint4_array(2) = joint_rad[2] * M_PI / 180.0;  // 舵机角度转弧度
    _temp_joint4_array(3) = joint_rad[3] * M_PI / 180.0;

    fk_solver.JntToCart(_temp_joint4_array, frame);

    Vector3D temp;
    temp[0] = frame.p.x();
    temp[1] = frame.p.y();
    temp[2] = frame.p.z();

    return temp;
}

// 机械臂速度正解：关节角速度→末端速度（复用雅可比矩阵）
Vector3D ArmCalcNode::arm_end_vel(const Eigen::Vector4d& joint_rad, const Vector2D& joint_omega)
{
    _temp_joint4_array(0) = joint_rad[0];
    _temp_joint4_array(1) = joint_rad[1];
    _temp_joint4_array(2) = joint_rad[2] * M_PI / 180.0;
    _temp_joint4_array(3) = joint_rad[3] * M_PI / 180.0;

    jacobain_solver.JntToJac(_temp_joint4_array, temp_jacobain);

    // 提取位置相关雅可比矩阵（3x4），仅取前两轴速度（电机）
    Eigen::Matrix<double, 3, 2> jacobian;
    jacobian.block<3,2>(0,0) = temp_jacobain.data.block<3,2>(0,0);

    return jacobian * joint_omega;
}

// 机械臂逆解：末端位置→关节角度（复用原有KDL逻辑）
Eigen::Vector4d ArmCalcNode::joints_pos(const Vector3D& end_pos, int* result)
{
    KDL::Frame frame;
    frame.p.x(end_pos[0]);
    frame.p.y(end_pos[1]);
    frame.p.z(end_pos[2]);
    frame.M = KDL::Rotation::Identity();

    *result = ik_pos_solver.CartToJnt(last_exp_joint_pos, frame, _temp_joint4_array);
    if (*result == 0) {
        last_exp_joint_pos = _temp_joint4_array;  // 缓存成功解
    } else {
        _temp_joint4_array = last_exp_joint_pos;  // 失败则用上次解
    }

    Eigen::Vector4d joint_pos;
    joint_pos[0] = _temp_joint4_array(0);
    joint_pos[1] = _temp_joint4_array(1);
    joint_pos[2] = _temp_joint4_array(2) * 180.0 / M_PI;
    joint_pos[3] = _temp_joint4_array(3) * 180.0 / M_PI;

    return joint_pos;
}



// 主函数：ROS2节点启动
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmCalcNode>());
    rclcpp::shutdown();
    return 0;
}

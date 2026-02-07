#pragma once
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <chrono>
#include <ctime>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <robot_interfaces/msg/detail/leg__struct.hpp>
#include <robot_interfaces/msg/motor_state.hpp>
#include <robot_interfaces/msg/motor_target.hpp>
#include <sensor_msgs/msg/detail/imu__struct.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tuple>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include "step.h"
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>   
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>

// 补充必要的类型定义（避免未定义报错）
typedef Eigen::Vector3d Vector3D;
typedef Eigen::Vector2d Vector2D;

class ArmCalcNode : public rclcpp::Node
{
public:
    ArmCalcNode();
    ArmCalcNode(KDL::Chain &chain);
    ~ArmCalcNode() = default;

    // 保留原有结构体定义
    typedef struct {
        double a;
        double b;
        double c;
        double d;
        double e;
        double f;
    } QuinticLineParam_t;

    typedef struct {
        double k;
        double b;
    } StraightLineParam_t;

    typedef struct {
        QuinticLineParam_t lx;
        QuinticLineParam_t ly;
        QuinticLineParam_t lz;
        double time;
    } FlightTrajectory_t;

private:
    // ROS2通信相关
    rclcpp::TimerBase::SharedPtr timmer;
    rclcpp::Subscription<robot_interfaces::msg::MotorState>::SharedPtr motor_state_sub;
    rclcpp::Publisher<robot_interfaces::msg::MotorTarget>::SharedPtr motor_target_pub;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_server;
    rclcpp::SyncParametersClient::SharedPtr robot_description_param_;

    // KDL相关（求解器改为智能指针，支持构造函数内动态初始化）
    std::string urdf_xml;
    KDL::Tree tree;
    KDL::Chain chain;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
    std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_pos_solver;
    std::unique_ptr<KDL::ChainJntToJacSolver> jacobain_solver;

    // KDL数据结构
    KDL::JntArray q_min;
    KDL::JntArray q_max;
    KDL::JntArray _temp_joint4_array;
    KDL::JntArray last_exp_joint_pos;
    KDL::Jacobian temp_jacobain;
    KDL::JntArray _temp_joint2_array;

    // 控制参数
    double control_dt = 0.01;
    double current_t = 0.0;
    double traj_total_time = 1.0;

    // 关节/末端状态
    Eigen::Vector4d joint_pos{0,0,0,0};
    Eigen::Vector2d joint_vel{0,0};
    Eigen::Vector4d target_joint_pos{0,0,0,0};
    Eigen::Vector2d target_joint_omega{0,0};
    Vector3D current_end_pos{0,0,0};
    Vector3D current_end_vel{0,0,0};
    Vector3D exp_end_pos{0,0,0};
    Vector3D exp_cart_vel{0,0,0};
    Vector3D target_end_pos{0,0,0};
    Vector3D target_end_vel{0,0,0};
    Vector3D target_end_acc{0,0,0};

    // 轨迹规划
    FlightTrajectory_t flight_trajectory;
    bool flight_trajectory_is_available = false;
    robot_interfaces::msg::MotorTarget motor_target;

    // 回调与核心函数（保留原有）
    void updata();
    void update_flight_trajectory(const Vector3D& cur_pos, const Vector3D& cur_vel, const Vector3D& exp_pos, const double time);
    void set_quintic(QuinticLineParam_t& seg, double p0, double v0, double a0, double pT, double vT, double aT, double dt);
    std::tuple<Vector3D, Vector3D, Vector3D> get_target(double time, bool& success);
    Vector3D arm_end_pos(const Eigen::Vector4d& joint_rad);
    Vector3D arm_end_vel(const Eigen::Vector4d& joint_rad, const Vector2D& joint_omega);
    Eigen::Vector4d joints_pos(const Vector3D& end_pos, int* result);
};

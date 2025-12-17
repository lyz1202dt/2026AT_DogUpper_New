#pragma once

#include "step.h"
#include "vmc.hpp"
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <chrono>
#include <ctime>
#include <geometry_msgs/msg/point.hpp>
#include <kdl/jacobian.hpp>
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
#include <kdl/chaindynparam.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>   // ← 你缺的就是它
#include <kdl/chainiksolverpos_lma.hpp>

class LegCalc{
public:
    LegCalc(KDL::Chain &chain);
    ~LegCalc();
    void set_leg_state(KDL::JntArray &rad, KDL::JntArray &omega, KDL::JntArray &torque);    //在一个控制周期内，应首先调用它

    int joint_pos(KDL::JntArray &joint_rad, KDL::Vector &foot_pos,KDL::JntArray &result);
    int joint_pos(KDL::Vector &foot_pos,KDL::JntArray &result);       //稍后需要在线安装IK求解器（手推的解析求解器或者数值迭代器）

    void joint_vel(KDL::JntArray &joint_rad, KDL::Vector &foot_vel,KDL::JntArray &result);

    //void joint_acc(KDL::JntArray &joint_rad, KDL::JntArray &joint_vel,KDL::Vector foot_acc,KDL::JntArray &result);

    void joint_torque(KDL::JntArray &joint_rad, KDL::JntArray &joint_vel, KDL::JntArray &joint_acc,KDL::Vector &result);

    void joint_torque_foot_force(KDL::JntArray &joint_rad,KDL::Vector &foot_force,KDL::JntArray result);    //由足端期望力计算的关节力矩

    void foot_force(KDL::JntArray &joint_rad,KDL::JntArray &joint_torque, KDL::Vector &result);

    void foot_vel(KDL::JntArray &joint_rad, KDL::JntArray &joint_vel, KDL::Vector &result);
    
    void foot_pos(KDL::JntArray &joint_rad,KDL::Vector &result);

    Eigen::Vector3d pos_offset; // 足端位置到机器人中心的偏移
private:
    Eigen::Matrix<double, 3, 3> get_3x3_jacobian_(KDL::Jacobian &full_jacobian);    //从KDL库中求出我们感兴趣的3*3位置雅可比矩阵

    unsigned long int control_tick{0};  //控制周期计数器
    KDL::Chain chain;
    KDL::ChainFkSolverPos_recursive fk_solver;  //关节位置->足端位置
    KDL::ChainJntToJacSolver jacobain_solver;        //求解雅可比矩阵
    KDL::ChainIkSolverVel_pinv vel_solver;     //
    KDL::ChainIkSolverPos_LMA *ik_pos_solver;    //计算期望关节位置
    
    KDL::ChainDynParam dynamin_solver;         //关节运动状态->关节力矩
    KDL::ChainJntToJacSolver force_solver;       //关节力矩->足端力(通常需要减去动力学给的力)

    KDL::JntSpaceInertiaMatrix M;
    KDL::JntArray C;
    KDL::JntArray G;

    KDL::JntArray cur_joint_pos;
    KDL::JntArray cur_joint_vel;
    KDL::JntArray cur_joint_torque;
    KDL::JntArray exp_joint_pos;
};

#pragma once

#include "step.h"
#include "vmc.hpp"
#include <Eigen/Dense>
#include <chrono>
#include <ctime>
#include <geometry_msgs/msg/point.hpp>
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
    int joint_pos(KDL::JntArray &joint_rad, KDL::Vector &foot_pos,KDL::JntArray &result);
    int joint_pos(KDL::Vector &foot_pos,KDL::JntArray &result);

    void joint_vel(KDL::JntArray &joint_rad, KDL::JntArray &joint_vel, KDL::Vector &foot_vel);

    void joint_torque(KDL::JntArray &joint_rad, KDL::JntArray &joint_vel, KDL::Vector &foot_acc, KDL::Vector &foot_force);

    void foot_force(KDL::JntArray &joint_rad, KDL::JntArray &joint_vel, KDL::JntArray &joint_torque, KDL::Vector &foot_force);

    void foot_vel(KDL::JntArray &joint_rad, KDL::JntArray &joint_vel, KDL::Vector &foot_vel);
    
    void foot_pos(KDL::JntArray &joint_rad,KDL::Frame &foot_pos);

    void set_leg_state(KDL::JntArray &rad, KDL::JntArray &omega, KDL::JntArray &torque);

    Eigen::Vector3d pos_offset; // 足端位置到机器人中心的偏移
private:


    KDL::Chain chain;
    KDL::ChainFkSolverPos_recursive *fk_solver; //计算足端位置
    KDL::ChainIkSolverVel_pinv *vel_solver;     //计算期望关节速度
    KDL::ChainIkSolverPos_LMA *ik_pos_solver;    //计算期望关节位置
    KDL::ChainDynParam *dynamin_solver;         //动力学求解
    KDL::ChainJntToJacSolver *jac_solver;       //计算足端力

    Eigen::Matrix<double, 6, 1> ik_weights;      //求解器权重向量

    KDL::JntSpaceInertiaMatrix M;
    KDL::JntArray C;
    KDL::JntArray G;

    KDL::JntArray cur_joint_pos;
    KDL::JntArray cur_joint_vel;
    KDL::JntArray cur_joint_torque;
    KDL::JntArray exp_joint_pos;
};

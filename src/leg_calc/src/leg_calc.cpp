#include "leg_calc.hpp"
#include <chrono>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

using namespace std::chrono_literals;

LegCalc::LegCalc(KDL::Chain& chain)
    : chain(chain) {

    ik_weights << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0;

    fk_solver  = new KDL::ChainFkSolverPos_recursive(chain); // 已知关节期望位置，计算足端位置
    vel_solver = new KDL::ChainIkSolverVel_pinv(chain);      // 已知足端期望速度，计算关节期望角速度
    ik_pos_solver =new KDL::ChainIkSolverPos_LMA(chain, ik_weights,1e-5,200,1e-10);    //求解关节期望
    dynamin_solver = new KDL::ChainDynParam(
        chain, KDL::Vector(0, 0, -9.81)); // 已知足端期望位置/速度/加速度，计算关节期望力矩
    jac_solver = new KDL::ChainJntToJacSolver(chain); // 已知足端期望力，计算关节期望力矩
}

LegCalc::~LegCalc() {
    delete ik_pos_solver;
    delete fk_solver;
    delete vel_solver;
    delete dynamin_solver;
    delete jac_solver;
}

void LegCalc::set_leg_state(KDL::JntArray& rad, KDL::JntArray& omega, KDL::JntArray& torque) {
    cur_joint_pos    = rad;
    cur_joint_vel    = omega;
    cur_joint_torque = torque;
}

int LegCalc::joint_pos(KDL::JntArray& joint_rad, KDL::Vector& foot_pos, KDL::JntArray& result) {
    KDL::Frame foot_frame(foot_pos);
    foot_frame.p.x(foot_frame.p.x() + pos_offset[0]);
    foot_frame.p.y(foot_frame.p.y() + pos_offset[1]);
    foot_frame.p.z(foot_frame.p.z() + pos_offset[2]);
    foot_frame.M = KDL::Rotation::Identity();
    return ik_pos_solver->CartToJnt(joint_rad, foot_frame, result);
}
int LegCalc::joint_pos(KDL::Vector& foot_pos, KDL::JntArray& result) {
    KDL::Frame foot_frame(foot_pos);
    foot_frame.p.x(foot_frame.p.x() + pos_offset[0]);
    foot_frame.p.y(foot_frame.p.y() + pos_offset[1]);
    foot_frame.p.z(foot_frame.p.z() + pos_offset[2]);
    foot_frame.M = KDL::Rotation::Identity();

    return ik_pos_solver->CartToJnt(cur_joint_pos, foot_frame, result);
}

void LegCalc::joint_vel(KDL::JntArray& joint_rad, KDL::JntArray& joint_vel, KDL::Vector& foot_vel) {

}

void LegCalc::joint_torque(
    KDL::JntArray& joint_rad, KDL::JntArray& joint_vel, KDL::Vector& foot_acc,
    KDL::Vector& foot_force) {}

void LegCalc::foot_force(
    KDL::JntArray& joint_rad, KDL::JntArray& joint_vel, KDL::JntArray& joint_torque,
    KDL::Vector& foot_force) {}

void LegCalc::foot_vel(KDL::JntArray& joint_rad, KDL::JntArray& joint_vel, KDL::Vector& foot_vel) {
    KDL::Jacobian jac;
    jac_solver->JntToJac(joint_rad, jac);
}

void LegCalc::foot_pos(KDL::JntArray& joint_rad, KDL::Frame& foot_pos) {
    fk_solver->JntToCart(joint_rad, foot_pos);
}

#include "leg_calc.hpp"
#include <chrono>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <rclcpp/logger.hpp>

using namespace std::chrono_literals;

LegCalc::LegCalc(KDL::Chain& chain)
    : chain(chain)
    , fk_solver(chain)
    , jacobain_solver(chain)
    , vel_solver(chain)
    , dynamin_solver(chain, KDL::Vector(0, 0, -9.81))
    , force_solver(chain){

    Eigen::Vector<double,6> ik_weights;
    ik_weights << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0;
    ik_pos_solver =new KDL::ChainIkSolverPos_LMA(chain, ik_weights,1e-5,200,1e-10);
}

LegCalc::~LegCalc() {}

void LegCalc::set_leg_state(KDL::JntArray& rad, KDL::JntArray& omega, KDL::JntArray& torque) {
    cur_joint_pos    = rad;
    cur_joint_vel    = omega;
    cur_joint_torque = torque;
    control_tick=control_tick+1;
}

int LegCalc::joint_pos(KDL::JntArray& joint_rad, KDL::Vector& foot_pos, KDL::JntArray& result) {
    //之后加载解析求解器
    return 0;
}

int LegCalc::joint_pos(KDL::Vector& foot_pos, KDL::JntArray& result) {
    KDL::Frame frame;
    frame.p=foot_pos;
    // frame.p.x(frame.p.x()+pos_offset[0]);
    // frame.p.y(frame.p.y()+pos_offset[1]);
    // frame.p.z(frame.p.z()+pos_offset[2]);
    RCLCPP_INFO(rclcpp::get_logger("leg_calc"),"期望位置:(%lf,%lf,%lf)",frame.p.x(),frame.p.y(),frame.p.z());
    frame.M=KDL::Rotation::Identity();
    return ik_pos_solver->CartToJnt(cur_joint_pos, KDL::Frame(foot_pos), result);
}

void LegCalc::joint_vel(KDL::JntArray &joint_rad, KDL::Vector &foot_vel,KDL::JntArray &result) {
    KDL::Jacobian temp_jac;
    jacobain_solver.JntToJac(joint_rad,temp_jac);
    Eigen::Matrix<double, 3, 3> jacobian = get_3x3_jacobian_(temp_jac);
    Eigen::Vector3d foot_vel_eigen(foot_vel.x(),foot_vel.y(),foot_vel.z());
    auto result_eigen=jacobian.inverse()*foot_vel_eigen;
    result(0)=result_eigen[0];
    result(1)=result_eigen[1];
    result(2)=result_eigen[2];
}

Eigen::Matrix<double, 3, 3> LegCalc::get_3x3_jacobian_(KDL::Jacobian &full_jacobian)     //只关心前三行的映射关系
{
    Eigen::Matrix<double, 3, 3> jacobian_3x3;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            jacobian_3x3(i, j)       = full_jacobian(i, j);
        }
    }
    return jacobian_3x3;
}


void LegCalc::joint_torque(
    KDL::JntArray &joint_rad, KDL::JntArray &joint_vel, KDL::JntArray &joint_acc,KDL::Vector &result) {
    dynamin_solver.JntToGravity(joint_rad, G);
    dynamin_solver.JntToCoriolis(joint_rad, joint_vel, C);
    dynamin_solver.JntToMass(joint_rad, M);

    // 6. 转换 KDL 输出到 Eigen，方便矩阵运算
    Eigen::Matrix<double, 3, 3> M_;
    Eigen::Matrix<double, 3, 1> C_, G_, ddq_;

    for (int i = 0; i < 3; ++i) {
        C_(i)   = C(i);
        G_(i)   = G(i);
        ddq_(i) = joint_acc(i);
        for (int j = 0; j < 3; ++j) {
            M_(i, j) = M(i, j);
        }
    }
    // 7. 计算前馈力矩 tau
    auto temp=(M_ * ddq_ + C_ + G_);
    result ={temp(0),temp(1),temp(2)};
}

void LegCalc::joint_torque_foot_force(KDL::JntArray &joint_rad,KDL::Vector &foot_force,KDL::JntArray result){
    KDL::Jacobian temp_jac;
    jacobain_solver.JntToJac(joint_rad,temp_jac);
    Eigen::Matrix<double, 3, 3> jacobian = get_3x3_jacobian_(temp_jac);
    Eigen::Vector3d torque(foot_force(0),foot_force(1),foot_force(2));
    auto result_eigen=jacobian.transpose()* torque;
    result(0)=result_eigen[0];
    result(1)=result_eigen[1];
    result(2)=result_eigen[2];
}

void LegCalc::foot_force(KDL::JntArray &joint_rad,KDL::JntArray &joint_torque, KDL::Vector &result) {
    KDL::Jacobian temp_jac;
    jacobain_solver.JntToJac(joint_rad,temp_jac);
    Eigen::Matrix<double, 3, 3> jacobian = get_3x3_jacobian_(temp_jac);
    Eigen::Vector3d torque(joint_torque(0),joint_torque(1),joint_torque(2));
    auto result_eigen=jacobian.transpose()* torque;
    result(0)=result_eigen[0];
    result(1)=result_eigen[1];
    result(2)=result_eigen[2];
}

void LegCalc::foot_vel(KDL::JntArray &joint_rad, KDL::JntArray &joint_vel, KDL::Vector &result) {
    KDL::Jacobian temp_jac;    //在一个控制周期内，应首先调用它;
    jacobain_solver.JntToJac(joint_rad,temp_jac);
    Eigen::Matrix<double, 3, 3> jacobian = get_3x3_jacobian_(temp_jac);
    Eigen::Vector3d foot_vel_eigen(joint_vel(0),joint_vel(1),joint_vel(2));
    auto result_eigen=jacobian*foot_vel_eigen;
    result(0)=result_eigen[0];
    result(1)=result_eigen[1];
    result(2)=result_eigen[2];
}

void LegCalc::foot_pos(KDL::JntArray& joint_rad, KDL::Vector & result) {
    KDL::Frame frame;
    fk_solver.JntToCart(joint_rad, frame);
    result.x(frame.p.x());
    result.y(frame.p.y());
    result.z(frame.p.z());
}

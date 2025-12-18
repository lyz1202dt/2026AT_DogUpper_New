#ifndef __VMC_H__
#define __VMC_H__

#include <chrono>
#include <cmath>
#include <tuple>

class VMC{
public:
    //虚拟质量-弹簧-阻尼系统参数
    VMC(double kp,double kd,double mass,double max_acc,double max_vel,double max_pos,std::chrono::high_resolution_clock::duration dt,
        double start_pos=0.0,double start_vel=0.0);
    //进行VMC计算，并返回新的期望位置/速度/力
    std::tuple<double,double,double> targetUpdate(double exp_pos,double cur_pos,double exp_vel,double cur_vel,double cur_force);
    void reset(double virtual_pos,double virtual_vel);
    double kp;
    double kd;
    double mass;
private:
    double virtual_pos;
    double virtual_vel;

    double max_pos;
    double max_vel;
    double max_acc;
    double dt;
};

class SimpleVMC{
    public:
    SimpleVMC(const double kp,const double kd,const double out_limit=100000000000.0);
    double update(const double pos_in,const double vel_in,const double pos_exp=0.0,const double vel_exp=0.0);
    double kp;
    double kd;
    double out_limit;
};

#endif
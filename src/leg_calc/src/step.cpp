#include "step.h"
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tuple>

static inline void set_quintic(
    QuinticLineParam_t& seg, double p0, double v0, double a0, double pT, double vT, double aT,
    float dt) {
    double T  = dt;
    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T3 * T;
    double T5 = T4 * T;

    seg.a = p0;
    seg.b = v0;
    seg.c = 0.5 * a0;

    // 五次多项式：p(t) = a + b*t + c*t^2 + d*t^3 + e*t^4 + f*t^5
    // 边界条件：p(0)=p0, v(0)=v0, a(0)=a0, p(T)=pT, v(T)=vT, a(T)=aT
    // 解方程得到：
    seg.d = (10 * (pT - p0) - (6 * v0 + 4 * vT) * T - (1.5 * a0 - 0.5 * aT) * T2) / T3;
    seg.e = (-15 * (pT - p0) + (8 * v0 + 7 * vT) * T + (1.5 * a0 - aT) * T2) / T4;
    seg.f = (6 * (pT - p0) - (3 * v0 + 3 * vT) * T - (0.5 * a0 - 0.5 * aT) * T2) / T5;
}

static inline float get_quintic_value(const QuinticLineParam_t& line, const float time) {
    return line.a + line.b * time + line.c * time * time + line.d * time * time * time
         + line.e * time * time * time * time + line.f * time * time * time * time * time;
}

static inline float get_quintic_dt(const QuinticLineParam_t& line, const float time) {
    return line.b + 2.0f * line.c * time + 3.0f * line.d * time * time
         + 4.0f * line.e * time * time * time + 5.0f * line.f * time * time * time * time;
}

static inline float get_quintic_dtdt(const QuinticLineParam_t& line, const float time) {
    return 2.0f * line.c + 6.0f * line.d * time + 12.0f * line.e * time * time
         + 20.0f * line.f * time * time * time;
}

std::tuple<Vector3D, Vector3D, Vector3D> GetPolynomialStep(StepTrajectory_t* line, float time) {
    Vector3D pos;
    Vector3D vel;
    Vector3D acc;

    // X 方向五次多项式
    pos[0] = get_quintic_value(line->lx, time);
    vel[0] = get_quintic_dt(line->lx, time);
    acc[0] = get_quintic_dtdt(line->lx, time);

    // Y 方向五次多项式
    pos[1] = get_quintic_value(line->ly, time);
    vel[1] = get_quintic_dt(line->ly, time);
    acc[1] = get_quintic_dtdt(line->ly, time);

    // Z 方向分两段（前半抬腿，后半落腿）
    if (time < line->time * 0.5f) {
        pos[2] = get_quintic_value(line->l1_z, time);
        vel[2] = get_quintic_dt(line->l1_z, time);
        acc[2] = get_quintic_dtdt(line->l1_z, time);
    } else {
        pos[2] = get_quintic_value(line->l2_z, time - line->time * 0.5f);
        vel[2] = get_quintic_dt(line->l2_z, time - line->time * 0.5f);
        acc[2] = get_quintic_dtdt(line->l2_z, time - line->time * 0.5f);
    }
    return std::make_tuple(pos, vel, acc);
}

std::tuple<Vector3D, Vector3D, Vector3D> GetSupportStep(SupportTrajectory_t& line, float time) {
    Vector3D pos;
    Vector3D vel;

    pos[0] = line.lx.b + line.lx.k * time;
    vel[0] = line.lx.k;

    pos[1] = line.ly.b + line.ly.k * time;
    vel[1] = line.ly.k;

    pos[2] = line.lz.b + line.lz.k * time;
    vel[2] = line.lz.k;

    return std::make_tuple(pos, vel, Vector3D(0.0, 0.0, 0.0));
}

bool UpdateGndStepLine(
    const Vector3D& cur_pos, const Vector2D& exp_vel, SupportTrajectory_t& line, float time) {
    double target_x = -exp_vel[0] * time * 0.5; // 理想情况下，足端轨迹中心应该过足端坐标系的中点
    double target_y = -exp_vel[1] * time * 0.5;

    line.time = time;                           // 记录一个步态相位的时间

    line.lx.k = (target_x - cur_pos[0]) / time;
    line.lx.b = cur_pos[0];

    line.ly.k = (target_y - cur_pos[1]) / time;
    line.ly.b = cur_pos[1];

    line.lz.k =  - cur_pos[2] / time;
    line.lz.b = cur_pos[2];

    return true;
}

bool UpdateAirStepLine(
    const Vector3D& cur_pos, const Vector3D& cur_vel, const Vector2D& exp_vel,
    StepTrajectory_t* line, float time, float step_height) {

    double target_x =exp_vel[0] * time * 0.5;
    double target_y = exp_vel[1] * time * 0.5;

    set_quintic(
        line->lx, cur_pos[0], cur_vel[0], 0.0,    // 起点
        target_x, -exp_vel[0], 0.0, time); // 终点
    // y方向轨迹
    set_quintic(line->ly, cur_pos[1], cur_vel[1], 0.0, target_y, -exp_vel[1], 0.0, time);
    // z方向分为两段：抬腿 -> 落腿
    // 第一段：从当前z抬到最高点
    set_quintic(line->l1_z, cur_pos[2], cur_vel[2], 0.0, step_height, 0.0, 0.0, time * 0.5f);

    // 第二段：从最高点落到地面
    set_quintic(line->l2_z, step_height, 0.0, 0.0, 0.0, 0.0, 0.0, time * 0.5f);

    return true;
}

bool UpdateCycloidStep(
    const Vector2D& exp_vel, CycloidStep_t* line, float time, float step_height) {
    line->Lx =
        exp_vel[0]
        * time; // 步长应该是期望速度乘以半个步态周期，但是转换到机器人坐标系后需要乘以整个步态周期因为机器人在向前移动
    line->Ly = exp_vel[1] * time;
    line->H  = step_height;
    line->T  = time * 0.5f;    // 实际上，摆动相只占整个步态的一半时间

    line->exp_vx = exp_vel[0]; // 当前速度估计（认为机器人足端位置的速度就是期望速度）
    line->exp_vy = exp_vel[1];
    return true;
}

std::tuple<Vector3D, Vector3D, Vector3D> GetCycloidStep(float time, CycloidStep_t& line) {
    const double pi = 3.14159265358979323846;

    float s = time / line.T;

    Vector3D pos, vel, acc;

    if (s <= 1.0f) {           // 摆动相
        // 位置
        pos[0] = line.Lx * (2 * pi * s - std::sin(2 * pi * s)) / (2 * pi) - line.exp_vx * time;
        pos[1] = line.Ly * (2 * pi * s - std::sin(2 * pi * s)) / (2 * pi) - line.exp_vy * time;
        pos[2] = line.H * (1 - std::cos(2 * pi * s)) / 2.0;

        // 速度
        vel[0] = line.Lx * (1 - std::cos(2 * pi * s)) / line.T - line.exp_vx;
        vel[1] = line.Ly * (1 - std::cos(2 * pi * s)) / line.T - line.exp_vy;
        vel[2] = line.H * pi * std::sin(2 * pi * s) / line.T;

        // 加速度
        acc[0] = line.Lx * (2 * pi * std::sin(2 * pi * s)) / (line.T * line.T);
        acc[1] = line.Ly * (2 * pi * std::sin(2 * pi * s)) / (line.T * line.T);
        acc[2] = line.H * 2 * pi * pi * std::cos(2 * pi * s) / (line.T * line.T);
    } else { // 1<s<2,支撑相
        pos[0] = line.Lx * 0.5 - line.exp_vx * (time - line.T);
        pos[1] = line.Ly * 0.5 - line.exp_vy * (time - line.T);
        pos[2] = 0.0;

        vel[0] = -line.exp_vx;
        vel[1] = -line.exp_vy;
        vel[2] = 0.0;

        acc[0] = 0.0;
        acc[1] = 0.0;
        acc[2] = 0.0;
    }

    return {pos, vel, acc};
}

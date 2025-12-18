#ifndef __STEP_H__
#define __STEP_H__

#include <Eigen/Dense>
#include <tuple>

typedef Eigen::Vector2d Vector2D;
typedef Eigen::Vector3d Vector3D;

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
    QuinticLineParam_t l1_z;
    QuinticLineParam_t l2_z;
    float time;        // 摆动相全程时间
} StepTrajectory_t;

typedef struct {
    StraightLineParam_t lx;
    StraightLineParam_t ly;
    StraightLineParam_t lz;
    float time;
} SupportTrajectory_t; // 支撑相全程时间

typedef struct {
    double exp_vx;
    double exp_vy;
    double H;          // 摆动轨迹半径
    double Lx;         // x方向步长
    double Ly;         // x方向步长
    float T;
} CycloidStep_t;

// 步态规划
bool UpdateGndStepLine(
    const Vector3D& cur_pos, const Vector2D& exp_vel, SupportTrajectory_t* line, float time);

    
bool UpdateAirStepLine(
    const Vector3D& cur_pos, const Vector3D& cur_vel, const Vector2D& exp_vel,
    StepTrajectory_t* line, float time, float step_height);


bool UpdateAirStepLine(
    const Vector3D& cur_pos, const Vector3D& cur_vel, const Vector2D& exp_vel, const Vector3D final_pos,
    StepTrajectory_t* line, float time, float step_height);

bool UpdateGndStepLine(
    const Vector3D& cur_pos, const Vector3D final_pos, SupportTrajectory_t* line, float time);


// 步态执行

std::tuple<Vector3D, Vector3D, Vector3D> GetQuinticStep(StepTrajectory_t& line, float time);


std::tuple<Vector3D, Vector3D, Vector3D> GetSupportStep(SupportTrajectory_t& line, float time);

bool UpdateCycloidStep(const Vector2D& exp_vel, CycloidStep_t* line, float time, float step_height);
std::tuple<Vector3D, Vector3D, Vector3D> GetCycloidStep(float time, CycloidStep_t& line);

#endif
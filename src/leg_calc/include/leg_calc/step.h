#ifndef __STEP_H__
#define __STEP_H__

#include <Eigen/Dense>
#include <tuple>

typedef Eigen::Vector2d Vector2D;
typedef Eigen::Vector3d Vector3D;

typedef struct {
    double exp_vx;
    double exp_vy;
    double H;              // 摆动轨迹半径
    double Lx;             // x方向步长
    double Ly;             // x方向步长
    float T;
} CycloidStep_t;

bool UpdateCycloidStep(const Vector2D& exp_vel, CycloidStep_t* line, float time, float step_height);
std::tuple<Vector3D, Vector3D, Vector3D> GetCycloidStep(float time, CycloidStep_t& line);

class LegStep {
public:
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
        double time;       // 摆动相全程时间
    } StepTrajectory_t;

    typedef struct {
        StraightLineParam_t lx;
        StraightLineParam_t ly;
        StraightLineParam_t lz;
        double time;
    } SupportTrajectory_t; // 支撑相全程时间



    LegStep();
    void update_flight_trajectory(const Vector3D& cur_pos, const Vector3D& cur_vel, const Vector2D& exp_vel, const double time, const double step_height,const double target_height=0.0);
    void update_flight_trajectory(
        const Vector3D& cur_pos, const Vector3D& cur_vel, const Vector3D& exp_pos, const Vector2D& exp_vel, const double time, const double step_height);
    void update_support_trajectory(const Vector3D& cur_pos, const Vector2D& exp_vel, double time);
    void update_support_trajectory(const Vector3D& cur_pos, const Vector3D final_pos, double time);
    std::tuple<Vector3D, Vector3D, Vector3D> get_target(double time,bool &success);

private:
    bool flight_trajectory_is_available{false};
    bool support_trajectory_is_available{false};
    StepTrajectory_t flight_trajectory;
    SupportTrajectory_t support_trajectory;
};

#endif
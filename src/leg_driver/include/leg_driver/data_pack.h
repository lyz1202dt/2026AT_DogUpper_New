#ifndef __DATAPACK_H__
#define __DATAPACK_H__

#include <stdint.h>

#pragma pack(1)

typedef struct{
    float rad;
    float omega;
    float torque;
    float kp;
    float kd;
}MotorTarget_t;

typedef struct{
    float omega;
    float torque;
}WheelTarget_t;

typedef struct{
    MotorTarget_t joint[3];
    WheelTarget_t wheel;
}LegTarget_t;

typedef struct{
    int pack_type;
    LegTarget_t leg[4];
}MotorTargetPack_t;



typedef struct {
    float X, Y, Z;
} Vector3D_Typedef;

typedef struct {
  Vector3D_Typedef AngularVelocity;
  struct {
    float Yaw, Pitch, Roll;
  } Angle;
} JY61_Typedef;

typedef struct{
    float rad;
    float omega;
    float torque;
}MotorState_t;

typedef struct{
    float omega;
    float torque;
}WheelState_t;

typedef struct{
    MotorState_t joint[3];
    WheelState_t wheel;
}LegState_t;

typedef struct{
    float vx;
    float vy;
    float omega;
    float wheel_v;
}Remotepack_t;

typedef struct{
    int pack_type;
    LegState_t leg[4];
    JY61_Typedef JY61;
    Remotepack_t remote;
    uint16_t motor_state;
}MotorStatePack_t;



#pragma pack()

#endif
#ifndef __DATAPACK_H__
#define __DATAPACK_H__

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
    float vz;
    float omega_z;
    float leg0_wheel_speed; 
    float leg1_wheel_speed;
    float leg2_wheel_speed;
    float leg3_wheel_speed;

}Remote_pack_t;

typedef struct{
    int pack_type;
    LegState_t leg[4];
    JY61_Typedef JY61;
    Remote_pack_t remote;
}MotorStatePack_t;



#pragma pack()

#endif
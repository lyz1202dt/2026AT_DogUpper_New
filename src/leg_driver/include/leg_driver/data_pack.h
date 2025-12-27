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




typedef struct{
    float rad;
    float omega;
    float torque;
}MotorState_t;

typedef struct{
    float omega;
}WheelState_t;

typedef struct{
    MotorTarget_t joint[3];
    WheelTarget_t wheel;
}LegState_t;

typedef struct{
    int pack_type;
    LegTarget_t leg[4];
}MotorStatePack_t;


#pragma pack()

#endif
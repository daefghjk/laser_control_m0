#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#include "MOTOR.h"
#include "ENCODER.h"

// 电机控制参数结构体
typedef struct {
    float kp;                // 位置环比例系数
    float ki;                // 位置环积分系数
    float kd;                // 位置环微分系数
    float velocity_kp;       // 速度环比例系数
    float velocity_ki;       // 速度环积分系数
    float max_output;        // 最大输出限幅
    float max_integral;      // 积分限幅
} MotorControl_Params_t;

// 电机控制句柄结构体
typedef struct {
    Motor_Handle *motor;            // 电机句柄
    Encoder_Handle_t *encoder;      // 编码器句柄
    MotorControl_Params_t params;   // 控制参数
    float target_position;          // 目标位置
    float target_velocity;          // 目标速度
    float position_error;           // 位置误差
    float velocity_error;           // 速度误差
    float integral;                 // 积分项
    float last_error;              // 上次误差
} MotorControl_Handle_t;

// 初始化函数
void MotorControl_Init(MotorControl_Handle_t *handle, 
                      Motor_Handle *motor,
                      Encoder_Handle_t *encoder,
                      const MotorControl_Params_t *params);

// 设置目标位置
void MotorControl_SetTargetPosition(MotorControl_Handle_t *handle, float position);

// 设置目标速度
void MotorControl_SetTargetVelocity(MotorControl_Handle_t *handle, float velocity);

// 位置控制更新（在定时器中断中调用）
void MotorControl_PositionUpdate(MotorControl_Handle_t *handle);

// 速度控制更新（在定时器中断中调用）
void MotorControl_VelocityUpdate(MotorControl_Handle_t *handle);

// 获取当前位置误差
float MotorControl_GetPositionError(const MotorControl_Handle_t *handle);

// 获取当前速度误差
float MotorControl_GetVelocityError(const MotorControl_Handle_t *handle);

#endif

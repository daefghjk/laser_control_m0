#ifndef __GIMBAL_CONTROL_H__
#define __GIMBAL_CONTROL_H__

#include "STEPPER.h"
#include "JOYSTICK.h"
#include <stddef.h>  // 添加此行以获取 NULL 定义

// 云台状态结构体
typedef struct {
    float pan_angle;     // 水平角度（度）
    float tilt_angle;    // 俯仰角度（度）
    float pan_speed;     // 水平速度（度/秒）
    float tilt_speed;    // 俯仰速度（度/秒）
} GimbalState_t;

// 云台限位配置
typedef struct {
    float pan_min_angle;     // 水平最小角度
    float pan_max_angle;     // 水平最大角度
    float tilt_min_angle;    // 俯仰最小角度
    float tilt_max_angle;    // 俯仰最大角度
    float max_pan_speed;     // 最大水平速度
    float max_tilt_speed;    // 最大俯仰速度
} GimbalLimits_t;

// 云台控制句柄
typedef struct {
    STEPPER_HANDLE_T *pan_stepper;     // 水平电机句柄
    STEPPER_HANDLE_T *tilt_stepper;    // 俯仰电机句柄
    GimbalState_t state;               // 当前状态
    GimbalLimits_t limits;             // 限位配置
} GimbalControl_t;

// 基本控制函数
void GIMBAL_Init(GimbalControl_t *handle, 
                STEPPER_HANDLE_T *pan_stepper,
                STEPPER_HANDLE_T *tilt_stepper,
                const GimbalLimits_t *limits);

void GIMBAL_Enable(GimbalControl_t *handle);
void GIMBAL_Disable(GimbalControl_t *handle);
void GIMBAL_Lock(GimbalControl_t *handle);
void GIMBAL_Unlock(GimbalControl_t *handle);

// 角度控制函数
void GIMBAL_SetPanAngle(GimbalControl_t *handle, float angle);
void GIMBAL_SetTiltAngle(GimbalControl_t *handle, float angle);
void GIMBAL_SetAngles(GimbalControl_t *handle, float pan_angle, float tilt_angle);

// 速度控制函数
void GIMBAL_SetPanSpeed(GimbalControl_t *handle, float speed);
void GIMBAL_SetTiltSpeed(GimbalControl_t *handle, float speed);
void GIMBAL_SetSpeeds(GimbalControl_t *handle, float pan_speed, float tilt_speed);

// 状态查询函数
const GimbalState_t* GIMBAL_GetState(const GimbalControl_t *handle);

// 为将来扩展预留的坐标转换接口
typedef struct {
    float x;    // 笛卡尔坐标 X
    float y;    // 笛卡尔坐标 Y
    float z;    // 笛卡尔坐标 Z
} CartesianPoint_t;

void GIMBAL_SetTargetPoint(GimbalControl_t *handle, const CartesianPoint_t *point);

#endif

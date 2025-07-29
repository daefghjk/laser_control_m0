#include "GIMBAL_CONTROL.h"
#include <math.h>
#include <stddef.h>  // 添加此行以获取 NULL 定义

// 角度限幅函数
static float clamp_angle(float angle, float min, float max) {
    if (angle < min) return min;
    if (angle > max) return max;
    return angle;
}

// 速度限幅函数
static float clamp_speed(float speed, float max_speed) {
    if (speed < -max_speed) return -max_speed;
    if (speed > max_speed) return max_speed;
    return speed;
}

void GIMBAL_Init(GimbalControl_t *handle, 
                STEPPER_HANDLE_T *pan_stepper,
                STEPPER_HANDLE_T *tilt_stepper,
                const GimbalLimits_t *limits)
{
    if (!handle || !pan_stepper || !tilt_stepper || !limits) return;

    handle->pan_stepper = pan_stepper;
    handle->tilt_stepper = tilt_stepper;
    handle->limits = *limits;

    // 初始化状态
    handle->state.pan_angle = 0.0f;
    handle->state.tilt_angle = 0.0f;
    handle->state.pan_speed = 0.0f;
    handle->state.tilt_speed = 0.0f;

    // 确保两个电机都处于失能状态
    STEPPER_Disable(pan_stepper);
    STEPPER_Disable(tilt_stepper);
}

void GIMBAL_Enable(GimbalControl_t *handle)
{
    if (!handle) return;
    STEPPER_Enable(handle->pan_stepper);
    STEPPER_Enable(handle->tilt_stepper);
}

void GIMBAL_Disable(GimbalControl_t *handle)
{
    if (!handle) return;
    STEPPER_Disable(handle->pan_stepper);
    STEPPER_Disable(handle->tilt_stepper);
}

void GIMBAL_Lock(GimbalControl_t *handle)
{
    if (!handle) return;
    STEPPER_Lock(handle->pan_stepper);
    STEPPER_Lock(handle->tilt_stepper);
}

void GIMBAL_Unlock(GimbalControl_t *handle)
{
    if (!handle) return;
    STEPPER_Unlock(handle->pan_stepper);
    STEPPER_Unlock(handle->tilt_stepper);
}

void GIMBAL_SetPanAngle(GimbalControl_t *handle, float angle)
{
    if (!handle) return;
    float target_angle = clamp_angle(angle, 
                                   handle->limits.pan_min_angle,
                                   handle->limits.pan_max_angle);
    
    float delta_angle = target_angle - handle->state.pan_angle;
    STEPPER_MoveAngle(handle->pan_stepper, delta_angle);
    handle->state.pan_angle = target_angle;
}

void GIMBAL_SetTiltAngle(GimbalControl_t *handle, float angle)
{
    if (!handle) return;
    float target_angle = clamp_angle(angle, 
                                   handle->limits.tilt_min_angle,
                                   handle->limits.tilt_max_angle);
    
    float delta_angle = target_angle - handle->state.tilt_angle;
    STEPPER_MoveAngle(handle->tilt_stepper, delta_angle);
    handle->state.tilt_angle = target_angle;
}

void GIMBAL_SetAngles(GimbalControl_t *handle, float pan_angle, float tilt_angle)
{
    if (!handle) return;
    GIMBAL_SetPanAngle(handle, pan_angle);
    GIMBAL_SetTiltAngle(handle, tilt_angle);
}

void GIMBAL_SetPanSpeed(GimbalControl_t *handle, float speed)
{
    if (!handle) return;
    handle->state.pan_speed = clamp_speed(speed, handle->limits.max_pan_speed);
    STEPPER_SetSpeed(handle->pan_stepper, fabs(handle->state.pan_speed));
    STEPPER_SetDirection(handle->pan_stepper, handle->state.pan_speed >= 0);
}

void GIMBAL_SetTiltSpeed(GimbalControl_t *handle, float speed)
{
    if (!handle) return;
    handle->state.tilt_speed = clamp_speed(speed, handle->limits.max_tilt_speed);
    STEPPER_SetSpeed(handle->tilt_stepper, fabs(handle->state.tilt_speed));
    STEPPER_SetDirection(handle->tilt_stepper, handle->state.tilt_speed >= 0);
}

void GIMBAL_SetSpeeds(GimbalControl_t *handle, float pan_speed, float tilt_speed)
{
    if (!handle) return;
    GIMBAL_SetPanSpeed(handle, pan_speed);
    GIMBAL_SetTiltSpeed(handle, tilt_speed);
}

const GimbalState_t* GIMBAL_GetState(const GimbalControl_t *handle)
{
    return handle ? &handle->state : NULL;
}

void GIMBAL_SetTargetPoint(GimbalControl_t *handle, const CartesianPoint_t *point)
{
    if (!handle || !point) return;
    
    // TODO: 实现笛卡尔坐标到云台角度的转换
    // 这部分可以根据实际的机械结构和需求来实现
    // float pan_angle = atan2(point->y, point->x) * 180.0f / M_PI;
    // float tilt_angle = atan2(point->z, sqrt(point->x*point->x + point->y*point->y)) * 180.0f / M_PI;
    // GIMBAL_SetAngles(handle, pan_angle, tilt_angle);
}

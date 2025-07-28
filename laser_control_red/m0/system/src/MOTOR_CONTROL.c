#include "MOTOR_CONTROL.h"

void MotorControl_Init(MotorControl_Handle_t *handle, 
                      Motor_Handle *motor,
                      Encoder_Handle_t *encoder,
                      const MotorControl_Params_t *params)
{
    if (handle == NULL || motor == NULL || encoder == NULL || params == NULL) {
        return;
    }

    handle->motor = motor;
    handle->encoder = encoder;
    handle->params = *params;
    
    handle->target_position = 0.0f;
    handle->target_velocity = 0.0f;
    handle->position_error = 0.0f;
    handle->velocity_error = 0.0f;
    handle->integral = 0.0f;
    handle->last_error = 0.0f;
}

void MotorControl_SetTargetPosition(MotorControl_Handle_t *handle, float position)
{
    if (handle != NULL) {
        handle->target_position = position;
    }
}

void MotorControl_SetTargetVelocity(MotorControl_Handle_t *handle, float velocity)
{
    if (handle != NULL) {
        handle->target_velocity = velocity;
    }
}

void MotorControl_PositionUpdate(MotorControl_Handle_t *handle)
{
    if (handle == NULL) {
        return;
    }

    // 计算位置误差
    float current_position = Encoder_GetPosition(handle->encoder);
    handle->position_error = handle->target_position - current_position;
    
    // PID控制
    handle->integral += handle->position_error;
    
    // 积分限幅
    if (handle->integral > handle->params.max_integral) {
        handle->integral = handle->params.max_integral;
    } else if (handle->integral < -handle->params.max_integral) {
        handle->integral = -handle->params.max_integral;
    }
    
    float derivative = handle->position_error - handle->last_error;
    
    // 计算输出
    float output = handle->params.kp * handle->position_error +
                  handle->params.ki * handle->integral +
                  handle->params.kd * derivative;
    
    // 输出限幅
    if (output > handle->params.max_output) {
        output = handle->params.max_output;
    } else if (output < -handle->params.max_output) {
        output = -handle->params.max_output;
    }
    
    // 更新电机输出
    if (output >= 0) {
        Motor_SetDirection(handle->motor, MOTOR_DIR_FORWARD);
        Motor_SetSpeed(handle->motor, (uint16_t)output);
    } else {
        Motor_SetDirection(handle->motor, MOTOR_DIR_BACKWARD);
        Motor_SetSpeed(handle->motor, (uint16_t)(-output));
    }
    
    handle->last_error = handle->position_error;
}

void MotorControl_VelocityUpdate(MotorControl_Handle_t *handle)
{
    if (handle == NULL) {
        return;
    }

    // 计算速度误差
    float current_velocity = Encoder_GetVelocity(handle->encoder);
    handle->velocity_error = handle->target_velocity - current_velocity;
    
    // PI控制
    handle->integral += handle->velocity_error;
    
    // 积分限幅
    if (handle->integral > handle->params.max_integral) {
        handle->integral = handle->params.max_integral;
    } else if (handle->integral < -handle->params.max_integral) {
        handle->integral = -handle->params.max_integral;
    }
    
    // 计算输出
    float output = handle->params.velocity_kp * handle->velocity_error +
                  handle->params.velocity_ki * handle->integral;
    
    // 输出限幅
    if (output > handle->params.max_output) {
        output = handle->params.max_output;
    } else if (output < -handle->params.max_output) {
        output = -handle->params.max_output;
    }
    
    // 更新电机输出
    if (output >= 0) {
        Motor_SetDirection(handle->motor, MOTOR_DIR_FORWARD);
        Motor_SetSpeed(handle->motor, (uint16_t)output);
    } else {
        Motor_SetDirection(handle->motor, MOTOR_DIR_BACKWARD);
        Motor_SetSpeed(handle->motor, (uint16_t)(-output));
    }
}

float MotorControl_GetPositionError(const MotorControl_Handle_t *handle)
{
    return handle ? handle->position_error : 0.0f;
}

float MotorControl_GetVelocityError(const MotorControl_Handle_t *handle)
{
    return handle ? handle->velocity_error : 0.0f;
}

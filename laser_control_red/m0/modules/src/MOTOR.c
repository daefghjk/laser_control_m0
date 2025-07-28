#include "ti_msp_dl_config.h"
#include "MOTOR.h"
#include "PWM.h"

// 内部函数：更新电机硬件状态
static void Motor_UpdateHardwareState(Motor_Handle *motor)
{
    if (motor == NULL) {
        return;
    }

    // 计算实际PWM占空比
    uint32_t effective_percent = 0;
    switch (motor->state.direction)
    {
        case MOTOR_DIR_FORWARD:
            DL_GPIO_clearPins(motor->config.in2_port, motor->config.in2_pin);
            effective_percent = motor->current_speed;
            break;
        case MOTOR_DIR_BACKWARD:
            DL_GPIO_setPins(motor->config.in2_port, motor->config.in2_pin);
            effective_percent = 100 - motor->current_speed;
            break;
        case MOTOR_DIR_STOP:
        default:
            DL_GPIO_clearPins(motor->config.in2_port, motor->config.in2_pin);
            effective_percent = 0;
            break;
    }
    
    // 更新PWM占空比
    PWM_SetDuty(motor->config.pwm_tim, motor->config.pwm_channel, effective_percent);
    if((motor->state.direction == MOTOR_DIR_STOP) && (motor->state.enabled == 0)) {
        // 如果是停止状态，停止PWM
        PWM_Stop(motor->config.pwm_tim, motor->config.pwm_channel);
    } else {
        // 启动PWM
        PWM_Start(motor->config.pwm_tim);
    }
}


void Motor_Init(Motor_Handle *motor, const MOTOR_CONFIG_T *config)
{
    if (motor == NULL || config == NULL) {
        return;
    }

    // 初始化配置
    motor->config = *config;
    motor->state.direction = MOTOR_DIR_STOP;
    motor->current_speed = 0;

    // 初始化PWM
    PWM_Init();
    PWM_SetFre(motor->config.pwm_tim, motor->config.pwm_channel, 1000);
    
    // 更新初始状态
    Motor_UpdateHardwareState(motor);
}

void Motor_SetSpeed(Motor_Handle *motor, int16_t percent)
{
    if (motor == NULL) {
        return;
    }
    // 限幅
    if (percent > 100) percent = 100;
    if (percent < 0) percent = 0;
    motor->current_speed = percent;
    
    Motor_UpdateHardwareState(motor);
}

void Motor_SetDirection(Motor_Handle *motor, Motor_DirectionType dir)
{
    if (motor == NULL) {
        return;
    }
    motor->state.direction = dir;
    
    Motor_UpdateHardwareState(motor);
}


void Motor_Enable(Motor_Handle *motor)
{
    if (motor == NULL) {
        return;
    }

    // 设置使能状态
    motor->state.enabled = 1;
    
    // 更新硬件状态
    Motor_UpdateHardwareState(motor);
}
void Motor_Disable(Motor_Handle *motor)
{
    if (motor == NULL) {
        return;
    }

    // 设置失能状态
    motor->state.enabled = 0;
    
    // 更新硬件状态
    Motor_UpdateHardwareState(motor);
}
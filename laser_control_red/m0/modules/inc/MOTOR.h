#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "ti_msp_dl_config.h"
#include "PWM.h"

typedef enum {
    MOTOR_DIR_FORWARD,
    MOTOR_DIR_BACKWARD,
    MOTOR_DIR_STOP
} Motor_DirectionType;

typedef enum {
    MOTOR_TIMER_TIMG,
    MOTOR_TIMER_TIMA
} Motor_TimerType;

typedef struct {
    // 硬件资源
    GPIO_Regs *in1_port;
    uint32_t in1_pin;
    GPIO_Regs *in2_port;
    uint32_t in2_pin;
    GPTIMER_Regs * pwm_tim;
    DL_TIMER_CC_INDEX pwm_channel;
    Motor_TimerType timer_type;
}MOTOR_CONFIG_T;

typedef struct {
    // 硬件资源
    MOTOR_CONFIG_T config;
    // 运行状态
    struct {
        volatile uint8_t enabled  : 1;    // 使能状态 (1位)
        volatile Motor_DirectionType direction;    // 当前方向 (1位)
    } state;
    uint32_t current_speed;
} Motor_Handle;

void Motor_Init(Motor_Handle *motor, const MOTOR_CONFIG_T *config);
void Motor_SetDirection(Motor_Handle *motor, Motor_DirectionType dir);
void Motor_SetSpeed(Motor_Handle *motor, int16_t percent);
//使能失能
void Motor_Enable(Motor_Handle *motor);
void Motor_Disable(Motor_Handle *motor);

#endif
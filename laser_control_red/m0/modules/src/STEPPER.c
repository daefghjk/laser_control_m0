#include "ti_msp_dl_config.h"
#include "STEPPER.h"
#include "PWM.h"

//内部
static void set_pwm_freq(STEPPER_HANDLE_T *handle, float freq)
{
    // 限幅
    if (freq < handle->config.min_freq)
    {
        freq = handle->config.min_freq;
    }
    else if (freq > handle->config.max_freq)
    {
        freq = handle->config.max_freq;
    }

    // 设置频率
    PWM_SetFre(handle->config.pwm_tim, handle->config.pwm_channel, (uint32_t)freq);
    handle->current_freq = freq;
}
static void update_hardware_state(STEPPER_HANDLE_T *handle)
{
    // 更新方向引脚状态
    if (handle->state.direction)
    {
        DL_GPIO_setPins(handle->config.dir_port, handle->config.dir_pin);
    }
    else
    {
        DL_GPIO_clearPins(handle->config.dir_port, handle->config.dir_pin);
    }


    // 更新使能引脚状态
    if (handle->state.enabled)
    {
        DL_GPIO_setPins(handle->config.en_port, handle->config.en_pin);
    }
    else
    {
        DL_GPIO_clearPins(handle->config.en_port, handle->config.en_pin);
    }

    // 更新PWM频率
    if ((handle->state.enabled) && (handle->state.locked == 0) && (handle->state.moving)) 
    {
        set_pwm_freq(handle, handle->current_freq);
        PWM_Start(handle->config.pwm_tim);
    }
    else
    {
        PWM_Stop(handle->config.pwm_tim, handle->config.pwm_channel);
    }
}

//外部
void STEPPER_Init(STEPPER_HANDLE_T *handle, const STEPPER_CONFIG_T *config)
{
    // 检查参数有效性
    if (handle == NULL || config == NULL) {
        return;
    }

    // 初始化配置
    handle->config = *config;
    handle->state.enabled = 0;
    handle->state.locked = 0;
    handle->state.moving = 0;
    handle->state.direction = 0;
    handle->current_freq = 0.0f;
    handle->target_steps = 0;

    // 初始化PWM
    PWM_Init();
    PWM_Start(handle->config.pwm_tim);
    
    // 更新硬件状态
    update_hardware_state(handle);
}
//设置使能、失能、运动方向、锁定模式、运动速度、相对移动步数、相对移动角度
void STEPPER_Enable(STEPPER_HANDLE_T *handle)
{
    if (handle == NULL) {
        return;
    }

    // 设置使能状态
    handle->state.enabled = 1;
    update_hardware_state(handle);
}
void STEPPER_Disable(STEPPER_HANDLE_T *handle)
{
    if (handle == NULL) {
        return;
    }

    // 设置失能状态
    handle->state.enabled = 0;
    update_hardware_state(handle);
}

void STEPPER_SetDirection(STEPPER_HANDLE_T *handle, uint8_t direction)
{
    if (handle == NULL) {
        return;
    }

    // 设置方向
    handle->state.direction = direction;
    update_hardware_state(handle);
}

void STEPPER_Lock(STEPPER_HANDLE_T *handle)
{
    if (handle == NULL) {
        return;
    }

    // 设置锁定状态
    handle->state.locked = 1;
    update_hardware_state(handle);
}
void STEPPER_Unlock(STEPPER_HANDLE_T *handle)
{
    if (handle == NULL) {
        return;
    }

    // 设置解锁状态
    handle->state.locked = 0;
    update_hardware_state(handle);
}
void STEPPER_SetSpeed(STEPPER_HANDLE_T *handle, float speed_hz)
{
    if (handle == NULL) {
        return;
    }

    // 设置当前频率
    handle->current_freq = speed_hz;
    update_hardware_state(handle);
}
void STEPPER_MoveSteps(STEPPER_HANDLE_T *handle, int32_t steps)
{
    if (handle == NULL || steps == 0) {
        return;
    }

    // 设置目标步数
    handle->target_steps = steps;
    
    // 设置方向
    handle->state.direction = (steps > 0) ? 1 : 0;

    // 如果使能且未锁定，则开始移动
    if (handle->state.enabled && !handle->state.locked)
    {
        handle->state.moving = 1;
        update_hardware_state(handle);
    }
    // 在定时器中断回调里面，每触发一次就递减target_steps，到0时停止运动state.moving = 0;
}
void STEPPER_MoveAngle(STEPPER_HANDLE_T *handle, float angle_deg)
{
    if (handle == NULL) {
        return;
    }

    // 计算相对步数
    int32_t steps = (int32_t)(angle_deg / handle->config.step_angle);
    
    // 调用相对步数移动函数
    STEPPER_MoveSteps(handle, steps);
}

// 定时器中断回调函数
void STEPPER_TimerCallback(STEPPER_HANDLE_T *handle)
{
    if (handle == NULL || !handle->state.moving) {
        return;
    }

    // 步数递减
    if (handle->target_steps > 0) {
        handle->target_steps--;
    } else if (handle->target_steps < 0) {
        handle->target_steps++;
    }

    // 检查是否到达目标
    if (handle->target_steps == 0) {
        handle->state.moving = 0;
        update_hardware_state(handle);
    }
}


#ifndef __STEPPER_H__
#define __STEPPER_H__

// 定义配置结构体类型
typedef struct {
    // 硬件资源
    GPTIMER_Regs *pwm_tim;      // PWM定时器句柄
    DL_TIMER_CC_INDEX pwm_channel;    // PWM通道
    GPIO_Regs *dir_port;        // 方向GPIO端口
    uint32_t dir_pin;           // 方向引脚
    GPIO_Regs *en_port;         // 使能GPIO端口
    uint32_t en_pin;            // 使能引脚
    
    // 电机参数
    float step_angle;           // 每步进角度(度)
    float min_freq;             // 最小工作频率(Hz)
    float max_freq;             // 最大工作频率(Hz)
} STEPPER_CONFIG_T;

typedef struct {
    // 配置信息
    STEPPER_CONFIG_T config;

    // 运行状态
    struct {
        volatile uint8_t enabled  : 1;    // 使能状态 (1位)
        volatile uint8_t locked   : 1;    // 锁定模式 (1位)
        volatile uint8_t moving   : 1;    // 运动状态 (1位)
        volatile uint8_t direction: 1;    // 当前方向 (1位)
    } state;
    
    volatile float current_freq;           // 当前PWM频率
    volatile int32_t target_steps;         // 目标步数(相对)
} STEPPER_HANDLE_T;

//内部
static void set_pwm_freq(STEPPER_HANDLE_T *handle, float freq);
static void update_hardware_state(STEPPER_HANDLE_T *handle);

//外部
void STEPPER_Init(STEPPER_HANDLE_T *handle, const STEPPER_CONFIG_T *config);
//设置使能、失能、运动方向、锁定模式、运动速度、相对移动步数、相对移动角度
void STEPPER_Enable(STEPPER_HANDLE_T *handle);
void STEPPER_Disable(STEPPER_HANDLE_T *handle);
void STEPPER_SetDirection(STEPPER_HANDLE_T *handle, uint8_t direction);
void STEPPER_Lock(STEPPER_HANDLE_T *handle);
void STEPPER_Unlock(STEPPER_HANDLE_T *handle);
void STEPPER_SetSpeed(STEPPER_HANDLE_T *handle, float speed_hz);
void STEPPER_MoveSteps(STEPPER_HANDLE_T *handle, int32_t steps);
void STEPPER_MoveAngle(STEPPER_HANDLE_T *handle, float angle_deg);
void STEPPER_TimerCallback(STEPPER_HANDLE_T *handle);
void STEPPER_CommonIRQHandler(STEPPER_HANDLE_T *handle);

#endif
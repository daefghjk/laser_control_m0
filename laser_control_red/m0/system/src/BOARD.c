#include "ti_msp_dl_config.h"
// 系统级别
#include "BOARD.h"
#include "DELAY.h"
#include "PWM.h"
// 硬件层
#include "OLED.h"
#include "JOYSTICK.h"
#include "BUTTON.h"
#include "BEEP.h"
#include "K230.h"
// 硬件层（可能不单独使用）
#include "ENCODER.h"
#include "LASER.h"
#include "GRAY.h"
#include "MOTOR.h"
#include "STEPPER.h"
// 控制层
#include "GIMBAL_CONTROL.h"
#include "MOTOR_CONTROL.h"

volatile uint64_t Systick_Count = 0;    // 系统滴答计数,1ms累加

JOYSTICK_HANDLE_T joystick_handle;
OLED_Handle_t oled_handle;
STEPPER_HANDLE_T stepper1_handle, stepper2_handle;
BTN_GROUP_HANDLE_T btn_group;
LASER_HANDLE_T laser_handle;

/*
系统初始化BOARD_Init
 * 进行SYSCFG_DL_init
 * 定义并初始化变量：oled_handle, joystick_handle, stepper1_handle, stepper2_handle
 * 开启中断：stepper1_handle, stepper2_handle
*/

void BOARD_Init(void)
{
    SYSCFG_DL_init();
    SysTick_Config(CPUCLK_FREQ / 1000);
    
    OLED_Config_t oled_config = OLED_DEFAULT_CONFIG(OLED_INST);
    OLED_Init(&oled_handle, &oled_config);

    JOYSTICK_CONFIG_T joystick_config = JOYSTICK_DEFAULT_CONFIG(
        ADC12_JOYSTICK_INST, 
        ADC12_JOYSTICK_INST_INT_IRQN, 
        DMA_CH_ADC_JOYSTICK_CHAN_ID, 
        0.3f
    );
    JOYSTICK_Init(&joystick_handle, &joystick_config);

    STEPPER_CONFIG_T STEPPER_1_CONFIG = {
        .pwm_tim = STEPPER_1_INST,
        .pwm_channel = GPIO_STEPPER_1_C0_IDX,
        .dir_port = GPIO_STEPPER_MOTOR_PORT,
        .dir_pin = GPIO_STEPPER_MOTOR_PIN_DIR_H_PIN,
        .en_port = GPIO_STEPPER_MOTOR_PORT,
        .en_pin = GPIO_STEPPER_MOTOR_PIN_EN_H_PIN,
        .step_angle = 1.8f / 32, // 每步进角度
        .min_freq = 10.0f, // 最小工作频率
        .max_freq = 1000.0f // 最大工作频率
    };
    STEPPER_Init(&stepper1_handle, &STEPPER_1_CONFIG);
    NVIC_EnableIRQ(STEPPER_1_INST_INT_IRQN);

    STEPPER_CONFIG_T STEPPER_2_CONFIG = {
        .pwm_tim = STEPPER_2_INST,
        .pwm_channel = GPIO_STEPPER_2_C1_IDX,
        .dir_port = GPIO_STEPPER_MOTOR_PORT,
        .dir_pin = GPIO_STEPPER_MOTOR_PIN_DIR_L_PIN,
        .en_port = GPIO_STEPPER_MOTOR_PORT,
        .en_pin = GPIO_STEPPER_MOTOR_PIN_EN_L_PIN,
        .step_angle = 1.8f / 32,
        .min_freq = 10.0f,
        .max_freq = 1000.0f
    };
    STEPPER_Init(&stepper2_handle, &STEPPER_2_CONFIG);
    NVIC_EnableIRQ(STEPPER_2_INST_INT_IRQN);
   
    BTN_GROUP_CONFIG_T btn_group_config = {
        .num_buttons = 6,                    // 使用6个按键
        .support_mask = BTN_SUPPORT_ALL,     // 所有按键都支持全部功能
        .long_time = 1000,                   // 长按1秒
        .double_time = 300,                  // 双击间隔300ms
        .buttons = {
            // 配置按步进电机拓展板
            [0] = {.port = GPIOB, .pin = DL_GPIO_PIN_0}, // 按键0配置
            [1] = {.port = GPIOB, .pin = DL_GPIO_PIN_1}, // 按键1配置
            [2] = {.port = GPIOB, .pin = DL_GPIO_PIN_2}, // 按键2配置
            [3] = {.port = GPIOB, .pin = DL_GPIO_PIN_3}, // 按键3配置
            [4] = {.port = GPIOB, .pin = DL_GPIO_PIN_6}, // 按键4配置
            [5] = {.port = GPIOB, .pin = DL_GPIO_PIN_7}  // 按键5配置

            // 配置按四驱小车拓展板
            // [0] = {.port = GPIOA, .pin = DL_GPIO_PIN_13}, // 按键0配置
            // [1] = {.port = GPIOA, .pin = DL_GPIO_PIN_14}, // 按键1配置
            // [2] = {.port = GPIOA, .pin = DL_GPIO_PIN_23}, // 按键2配置
            // [3] = {.port = GPIOA, .pin = DL_GPIO_PIN_25}, // 按键3配置
            // [4] = {.port = GPIOA, .pin = DL_GPIO_PIN_27}, // 按键4配置
            // [5] = {.port = GPIOA, .pin = DL_GPIO_PIN_31}  // 按键5配置
        } 
    }; 
    BTN_Init(&btn_group, &btn_group_config);

    // 激光笔配置
    // LASER_CONFIG_T laser_config = {
    //     .ctrl_port = GPIO_LASER_PORT,
    //     .ctrl_pin = GPIO_LASER_PIN
    // };
    // LASER_Init(&laser_handle, &laser_config);
}

void SysTick_Handler(void)
{
    Systick_Count++;
}

void ADC12_JOYSTICK_INST_IRQHandler(void)
{
    JOYSTICK_CommonIRQHandler(&joystick_handle);
}
void STEPPER_1_INST_IRQHandler(void)
{
    STEPPER_CommonIRQHandler(&stepper1_handle);  // 修正函数名
}
void STEPPER_2_INST_IRQHandler(void)
{
    STEPPER_CommonIRQHandler(&stepper2_handle);  // 修正函数名
}
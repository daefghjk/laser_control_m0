#include "ti_msp_dl_config.h"
#include "BOARD.h"
#include "JOYSTICK.h"
#include "OLED.h"
#include "PWM.h"
#include "STEPPER.h"

volatile uint64_t Systick_Count = 0;    // 系统滴答计数,1ms累加

JOYSTICK_HANDLE_T joystick_handle;
OLED_Handle_t oled_handle;
STEPPER_HANDLE_T stepper1_handle, stepper2_handle;

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
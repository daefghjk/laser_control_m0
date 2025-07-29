#ifndef __BUTTON_H__
#define __BUTTON_H__

#include "ti_msp_dl_config.h"

#define BTN_MAX_BUTTONS 8    // 最大按键数量

// 按键功能配置掩码
#define BTN_SUPPORT_SINGLE_CLICK    0x01    // 支持单击
#define BTN_SUPPORT_DOUBLE_CLICK    0x02    // 支持双击
#define BTN_SUPPORT_LONG_PRESS      0x04    // 支持长按
#define BTN_SUPPORT_ALL             0x07    // 支持所有功能

// 单个按键配置结构体
typedef struct {
    GPIO_Regs *port;          // GPIO端口
    uint32_t pin;            // GPIO引脚
} BTN_PIN_CONFIG_T;

// 按键组配置结构体
typedef struct {
    uint8_t num_buttons;                          // 实际使用的按键数量
    uint8_t support_mask;                         // 所有按键共用的功能支持掩码
    uint16_t long_time;                          // 长按时间(ms)
    uint16_t double_time;                        // 双击间隔时间(ms)
    BTN_PIN_CONFIG_T buttons[BTN_MAX_BUTTONS];   // 按键配置数组
} BTN_GROUP_CONFIG_T;

// 按键状态枚举
typedef enum {
    BTN_STATE_IDLE = 0,     // 空闲状态
    BTN_STATE_DEBOUNCE,     // 消抖状态
    BTN_STATE_WAIT_RELEASE, // 等待释放
    BTN_STATE_WAIT_DOUBLE   // 等待双击
} BTN_STATE_E;


// 按键组句柄结构体
typedef struct {
    BTN_GROUP_CONFIG_T config;                   // 配置信息
    struct {
        BTN_STATE_E state[BTN_MAX_BUTTONS];      // 每个按键的当前状态
        uint32_t press_time[BTN_MAX_BUTTONS];    // 每个按键的按下时间点
        uint32_t release_time[BTN_MAX_BUTTONS];  // 每个按键的释放时间点
        uint8_t current_states[BTN_MAX_BUTTONS]; // 当前按键状态：1-按下，0-释放
    } state;
} BTN_GROUP_HANDLE_T;

// 函数声明
void BTN_Init(BTN_GROUP_HANDLE_T *handle, const BTN_GROUP_CONFIG_T *config);
void BTN_Update(BTN_GROUP_HANDLE_T *handle, uint32_t current_tick);

// 获取按键状态的辅助函数
const uint8_t* BTN_GetStates(BTN_GROUP_HANDLE_T *handle);
uint8_t BTN_IsPressed(BTN_GROUP_HANDLE_T *handle, uint8_t button_index);

// 弱定义回调函数声明
SYSCONFIG_WEAK void BTN_OnClick(BTN_GROUP_HANDLE_T *handle, uint8_t button_index);
SYSCONFIG_WEAK void BTN_OnDoubleClick(BTN_GROUP_HANDLE_T *handle, uint8_t button_index);
SYSCONFIG_WEAK void BTN_OnLongPress(BTN_GROUP_HANDLE_T *handle, uint8_t button_index);

/*
使用示例:
==========

1. 基本配置示例
---------------
// 配置按键组
BTN_GROUP_CONFIG_T btn_group_config = {
    .num_buttons = 3,                    // 使用3个按键
    .support_mask = BTN_SUPPORT_ALL,     // 所有按键都支持全部功能
    .long_time = 1000,                   // 长按1秒
    .double_time = 300,                  // 双击间隔300ms
    .buttons = {
        // 按键1配置
        [0] = {
            .port = GPIOA,
            .pin = DL_GPIO_PIN_1
        },
        // 按键2配置
        [1] = {
            .port = GPIOA,
            .pin = DL_GPIO_PIN_2
        },
        // 按键3配置
        [2] = {
            .port = GPIOA,
            .pin = DL_GPIO_PIN_3
        }
    }
};

// 创建按键组句柄
BTN_GROUP_HANDLE_T btn_group;

// 初始化按键组
BTN_Init(&btn_group, &btn_group_config);

2. 在主循环中使用
----------------
void main(void) {
    while(1) {
        BTN_Update(&btn_group, get_system_tick());
        
        // 获取所有按键当前状态
        const uint8_t* states = BTN_GetStates(&btn_group);
        
        // 检查特定按键是否按下
        if(BTN_IsPressed(&btn_group, 0)) {
            // 按键0当前正在按下
        }
    }
}

3. 实现回调函数
--------------
void BTN_OnClick(BTN_GROUP_HANDLE_T *handle, uint8_t button_index)
{
    switch(button_index) {
        case 0:
            // 处理按键0的单击
            break;
        case 1:
            // 处理按键1的单击
            break;
        case 2:
            // 处理按键2的单击
            break;
    }
}

void BTN_OnDoubleClick(BTN_GROUP_HANDLE_T *handle, uint8_t button_index)
{
    // 类似单击的处理方式
}

void BTN_OnLongPress(BTN_GROUP_HANDLE_T *handle, uint8_t button_index)
{
    // 类似单击的处理方式
}
*/

/*
按键配置
//步进电机板子按键b0-3，b6，b7
BTN_GROUP_CONFIG_T btn_group_config = {
    .num_buttons = 6,                    // 使用6个按键
    .support_mask = BTN_SUPPORT_ALL,     // 所有按键都支持全部功能
    .long_time = 1000,                   // 长按1秒
    .double_time = 300,                  // 双击间隔300ms
    .buttons = {
        // 按键0配置
        [0] = {.port = GPIOB, .pin = DL_GPIO_PIN_0}, // 按键0配置
        [1] = {.port = GPIOB, .pin = DL_GPIO_PIN_1}, // 按键1配置
        [2] = {.port = GPIOB, .pin = DL_GPIO_PIN_2}, // 按键2配置
        [3] = {.port = GPIOB, .pin = DL_GPIO_PIN_3}, // 按键3配置
        [4] = {.port = GPIOB, .pin = DL_GPIO_PIN_6}, // 按键4配置
        [5] = {.port = GPIOB, .pin = DL_GPIO_PIN_7}  // 按键5配置
    } 
}; 
 
        [0] = {.port = GPIOA, .pin = DL_GPIO_PIN_13}, // 按键0配置
        [1] = {.port = GPIOA, .pin = DL_GPIO_PIN_14}, // 按键1配置
        [2] = {.port = GPIOA, .pin = DL_GPIO_PIN_23}, // 按键2配置
        [3] = {.port = GPIOA, .pin = DL_GPIO_PIN_25}, // 按键3配置
        [4] = {.port = GPIOA, .pin = DL_GPIO_PIN_27}, // 按键4配置
        [5] = {.port = GPIOA, .pin = DL_GPIO_PIN_31}  // 按键5配置

*/

#endif

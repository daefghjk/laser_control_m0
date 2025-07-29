#include "BUTTON.h"

// 默认参数
#define BTN_DEBOUNCE_TIME   20      // 消抖时间(ms)
#define BTN_DEFAULT_LONG    1000    // 默认长按时间(ms)
#define BTN_DEFAULT_DOUBLE  300     // 默认双击间隔(ms)

// 读取按键状态(低电平为按下)
static inline uint8_t BTN_ReadPin(const BTN_PIN_CONFIG_T *config)
{
    return !DL_GPIO_readPins(config->port, config->pin);
}

static void update_button_state(BTN_GROUP_HANDLE_T *handle, uint8_t index, uint32_t current_tick)
{
    uint8_t is_pressed = BTN_ReadPin(&handle->config.buttons[index]);
    uint32_t elapsed;
    
    switch(handle->state.state[index]) {
        case BTN_STATE_IDLE:
            if(is_pressed) {
                handle->state.state[index] = BTN_STATE_DEBOUNCE;
                handle->state.press_time[index] = current_tick;
            }
            break;
            
        case BTN_STATE_DEBOUNCE:
            elapsed = current_tick - handle->state.press_time[index];
            if(!is_pressed) {
                handle->state.state[index] = BTN_STATE_IDLE;
            } else if(elapsed >= BTN_DEBOUNCE_TIME) {
                handle->state.state[index] = BTN_STATE_WAIT_RELEASE;
            }
            break;
            
        case BTN_STATE_WAIT_RELEASE:
            if(!is_pressed) {
                handle->state.release_time[index] = current_tick;
                elapsed = handle->state.release_time[index] - handle->state.press_time[index];
                
                if((handle->config.support_mask & BTN_SUPPORT_LONG_PRESS) && 
                   elapsed >= handle->config.long_time) {
                    BTN_OnLongPress(handle, index);
                    handle->state.state[index] = BTN_STATE_IDLE;
                }
                else if(handle->config.support_mask & BTN_SUPPORT_DOUBLE_CLICK) {
                    handle->state.state[index] = BTN_STATE_WAIT_DOUBLE;
                }
                else if(handle->config.support_mask & BTN_SUPPORT_SINGLE_CLICK) {
                    BTN_OnClick(handle, index);
                    handle->state.state[index] = BTN_STATE_IDLE;
                }
            }
            break;
            
        case BTN_STATE_WAIT_DOUBLE:
            elapsed = current_tick - handle->state.release_time[index];
            if(is_pressed) {
                if(elapsed <= handle->config.double_time) {
                    BTN_OnDoubleClick(handle, index);
                }
                handle->state.state[index] = BTN_STATE_DEBOUNCE;
                handle->state.press_time[index] = current_tick;
            }
            else if(elapsed > handle->config.double_time) {
                if(handle->config.support_mask & BTN_SUPPORT_SINGLE_CLICK) {
                    BTN_OnClick(handle, index);
                }
                handle->state.state[index] = BTN_STATE_IDLE;
            }
            break;
    }
    
    // 更新当前按键状态
    handle->state.current_states[index] = is_pressed;
}

void BTN_Init(BTN_GROUP_HANDLE_T *handle, const BTN_GROUP_CONFIG_T *config)
{
    if(handle == NULL || config == NULL || config->num_buttons > BTN_MAX_BUTTONS) return;
    
    // 复制配置
    handle->config = *config;
    
    // 初始化状态
    for(uint8_t i = 0; i < BTN_MAX_BUTTONS; i++) {
        handle->state.state[i] = BTN_STATE_IDLE;
        handle->state.press_time[i] = 0;
        handle->state.release_time[i] = 0;
        handle->state.current_states[i] = 0;
    }
}

void BTN_Update(BTN_GROUP_HANDLE_T *handle, uint32_t current_tick)
{
    if(handle == NULL) return;
    
    for(uint8_t i = 0; i < handle->config.num_buttons; i++) {
        update_button_state(handle, i, current_tick);
    }
}

const uint8_t* BTN_GetStates(BTN_GROUP_HANDLE_T *handle)
{
    return handle->state.current_states;
}

uint8_t BTN_IsPressed(BTN_GROUP_HANDLE_T *handle, uint8_t button_index)
{
    if(handle == NULL || button_index >= handle->config.num_buttons) return 0;
    return handle->state.current_states[button_index];
}

// 弱定义回调函数
SYSCONFIG_WEAK void BTN_OnClick(BTN_GROUP_HANDLE_T *handle, uint8_t button_index) {}
SYSCONFIG_WEAK void BTN_OnDoubleClick(BTN_GROUP_HANDLE_T *handle, uint8_t button_index) {}
SYSCONFIG_WEAK void BTN_OnLongPress(BTN_GROUP_HANDLE_T *handle, uint8_t button_index) {}

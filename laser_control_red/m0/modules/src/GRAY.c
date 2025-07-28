#include "ti_msp_dl_config.h"
#include "GRAY.h"

static void update_hardware_state(GRAY_Handle *handle)
{
    if (handle == NULL) return;

    // 初始化统计变量
    uint8_t detected = 0;        // 是否检测到线的标志
    float sum = 0;              // 位置加权和
    float count = 0;            // 检测到的传感器数量
    int8_t leftmost = GRAY_MAX_SENSORS;    // 最左侧位置（初始值设为最大以便后续比较）
    int8_t rightmost = -GRAY_MAX_SENSORS;  // 最右侧位置（初始值设为最小以便后续比较）

    // 遍历所有传感器
    for (int i = 0; i < handle->config.num_sensors; i++) {
        // 读取GPIO引脚状态
        uint8_t pin_state = DL_GPIO_readPins(handle->config.sensors[i].port, 
                                           handle->config.sensors[i].pin) ? 1 : 0;
        // 根据配置的极性（高电平有效/低电平有效）确定实际状态
        handle->state.sensor_states[i] = (pin_state == handle->config.active_high) ? 1 : 0;
        
        // 如果该传感器检测到线
        if (handle->state.sensor_states[i]) {
            detected = 1;  // 标记检测到线
            // 计算该传感器的位置（相对于中心的偏移）
            // 例如8个传感器时：-3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5
            float pos = (float)i - (handle->config.num_sensors - 1) / 2.0f;
            sum += pos;    // 累加位置
            count += 1;    // 计数器加1
            
            // 计算相对于中心的整数位置（例如8个传感器时：-3到4）
            int8_t current_pos = (int8_t)(i - (handle->config.num_sensors / 2));
            // 更新最左和最右位置
            if (current_pos < leftmost) leftmost = current_pos;
            if (current_pos > rightmost) rightmost = current_pos;
        }
    }

    // 更新状态结构体
    handle->state.line_detected = detected;  // 更新是否检测到线
    handle->state.average_pos = (count > 0) ? (sum / count) : 0;  // 计算平均位置
    handle->state.leftmost_pos = (detected) ? leftmost : 0;  // 更新最左位置
    handle->state.rightmost_pos = (detected) ? rightmost : 0;  // 更新最右位置
}

void GRAY_Init(GRAY_Handle *handle, const GRAY_CONFIG_T *config)
{
    // 参数有效性检查
    if (handle == NULL || config == NULL) return;
    if (config->num_sensors > GRAY_MAX_SENSORS) return;  // 确保传感器数量不超过最大值

    // 保存配置信息
    handle->config = *config;

    // 初始化所有传感器状态为0
    for (int i = 0; i < GRAY_MAX_SENSORS; i++) {
        handle->state.sensor_states[i] = 0;
    }
    
    // 初始化状态变量
    handle->state.leftmost_pos = 0;    // 最左位置初始化为0
    handle->state.rightmost_pos = 0;   // 最右位置初始化为0
    handle->state.average_pos = 0;     // 平均位置初始化为0
    handle->state.line_detected = 0;   // 初始状态设为未检测到线

    // 执行一次状态更新以获取初始值
    update_hardware_state(handle);
}

void GRAY_Update(GRAY_Handle *handle)
{
    if (handle == NULL) return;
    update_hardware_state(handle);
}

const uint8_t* GRAY_GetStates(GRAY_Handle *handle)
{
    if (handle == NULL) return NULL;
    return handle->state.sensor_states;
}

float GRAY_GetAveragePosition(GRAY_Handle *handle)
{
    if (handle == NULL) return 0;
    return handle->state.average_pos;
}

int8_t GRAY_GetLeftmostPosition(GRAY_Handle *handle)
{
    if (handle == NULL) return 0;
    return handle->state.leftmost_pos;
}

int8_t GRAY_GetRightmostPosition(GRAY_Handle *handle)
{
    if (handle == NULL) return 0;
    return handle->state.rightmost_pos;
}

uint8_t GRAY_IsLineDetected(GRAY_Handle *handle)
{
    if (handle == NULL) return 0;
    return handle->state.line_detected;
}

#ifndef __GRAY_H__
#define __GRAY_H__

#define GRAY_MAX_SENSORS 8    // 最大传感器数量
#define GRAY_CENTER_POS 0     // 中心位置值

// 传感器配置
typedef struct {
    GPIO_Regs *port;          // GPIO端口
    uint32_t pin;             // GPIO引脚
} GRAY_SENSOR_CONFIG_T;

typedef struct {
    uint8_t num_sensors;                              // 实际使用的传感器数量
    uint8_t active_high;                              // 1:高电平有效, 0:低电平有效
    GRAY_SENSOR_CONFIG_T sensors[GRAY_MAX_SENSORS];   // 传感器配置数组，左边序号小
} GRAY_CONFIG_T;

typedef struct {
    GRAY_CONFIG_T config;                 // 配置信息
    struct {
        uint8_t sensor_states[GRAY_MAX_SENSORS];  // 传感器状态数组: 从左到右8个传感器的状态
        int8_t leftmost_pos;      // 最左边检测到的位置: 相对于中心的偏移量(-3到4)
        int8_t rightmost_pos;     // 最右边检测到的位置: 相对于中心的偏移量(-3到4)
        float average_pos;        // 平均位置: 所有检测点的加权平均值(-3.5到3.5)
        uint8_t line_detected;    // 是否检测到线: 1表示至少有一个传感器检测到，0表示全部未检测到
    } state;
} GRAY_Handle;

// 初始化
void GRAY_Init(GRAY_Handle *handle, const GRAY_CONFIG_T *config);

// 更新所有传感器状态 (建议在主循环中以固定频率调用)
void GRAY_Update(GRAY_Handle *handle);

// 获取最近一次更新的传感器状态数组
const uint8_t* GRAY_GetStates(GRAY_Handle *handle);

// 获取最近一次更新的平均位置
float GRAY_GetAveragePosition(GRAY_Handle *handle);

// 获取最近一次更新的最左侧检测位置
int8_t GRAY_GetLeftmostPosition(GRAY_Handle *handle);

// 获取最近一次更新的最右侧检测位置
int8_t GRAY_GetRightmostPosition(GRAY_Handle *handle);

// 获取最近一次更新的线检测状态
uint8_t GRAY_IsLineDetected(GRAY_Handle *handle);

#endif

#ifndef __OLED_H__
#define __OLED_H__

#include <stdint.h>

/**
 * @brief OLED配置结构体
 */
typedef struct {
    void* i2c_inst;         // I2C实例指针
    uint8_t address;        // OLED设备地址
    uint8_t width;          // 屏幕宽度（像素）
    uint8_t height;         // 屏幕高度（像素）
    uint8_t pages;          // 页数（通常为8）
} OLED_Config_t;

/**
 * @brief OLED句柄结构体
 */
typedef struct {
    OLED_Config_t config;   // OLED配置
    uint8_t initialized;    // 初始化状态标志
} OLED_Handle_t;

// 函数声明
int OLED_Init(OLED_Handle_t* handle, const OLED_Config_t* config);
int OLED_Clear(OLED_Handle_t* handle);
int OLED_ShowChar(OLED_Handle_t* handle, uint8_t Line, uint8_t Column, char Char);
int OLED_ShowString(OLED_Handle_t* handle, uint8_t Line, uint8_t Column, char *String);
int OLED_ShowNum(OLED_Handle_t* handle, uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
int OLED_ShowSignedNum(OLED_Handle_t* handle, uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
int OLED_ShowHexNum(OLED_Handle_t* handle, uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
int OLED_ShowBinNum(OLED_Handle_t* handle, uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
int OLED_ShowFloat(OLED_Handle_t* handle, uint8_t Line, uint8_t Column, float Num, uint8_t Length1, uint8_t Length2);

// 辅助宏定义，用于创建默认配置
#define OLED_DEFAULT_CONFIG(i2c_instance) { \
    .i2c_inst = i2c_instance, \
    .address = 0x3C, \
    .width = 128, \
    .height = 64, \
    .pages = 8 \
}

// 错误代码定义
#define OLED_OK             0
#define OLED_ERROR          -1
#define OLED_ERROR_PARAM    -2
#define OLED_ERROR_INIT     -3

#endif
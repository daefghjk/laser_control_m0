#include <ti/driverlib/driverlib.h>
#include "DELAY.h"
#include "OLEDFont.h"
#include "OLED.h"

/**
 * @brief  参数检查宏
 */
#define CHECK_HANDLE(handle) \
    if ((handle) == NULL || !(handle)->initialized) { \
        return OLED_ERROR_PARAM; \
    }

/**
 * @brief  OLED写命令
 * @param  handle OLED句柄
 * @param  Command 要写入的命令
 * @retval 0: 成功, -1: 失败
 */
static int OLED_WriteCommand(OLED_Handle_t* handle, uint8_t Command)
{
    if (handle == NULL)
        return OLED_ERROR_PARAM;
    
    uint8_t buffer[2] = {0x00, Command};
    DL_I2C_fillControllerTXFIFO(handle->config.i2c_inst, buffer, 2);
    while (!(DL_I2C_getControllerStatus(handle->config.i2c_inst) & DL_I2C_CONTROLLER_STATUS_IDLE));
    DL_I2C_startControllerTransfer(handle->config.i2c_inst, handle->config.address, DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    while (DL_I2C_getControllerStatus(handle->config.i2c_inst) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    while (!(DL_I2C_getControllerStatus(handle->config.i2c_inst) & DL_I2C_CONTROLLER_STATUS_IDLE));
    
    return OLED_OK;
}

/**
 * @brief  OLED写数据
 * @param  handle OLED句柄
 * @param  Data 要写入的数据
 * @retval 0: 成功, -1: 失败
 */
static int OLED_WriteData(OLED_Handle_t* handle, uint8_t Data)
{
    if (handle == NULL)
        return OLED_ERROR_PARAM;
    
    uint8_t buffer[2] = { 0x40, Data};
    DL_I2C_fillControllerTXFIFO(handle->config.i2c_inst, buffer, 2);
    while (!(DL_I2C_getControllerStatus(handle->config.i2c_inst) & DL_I2C_CONTROLLER_STATUS_IDLE));
    DL_I2C_startControllerTransfer(handle->config.i2c_inst, handle->config.address, DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    while (DL_I2C_getControllerStatus(handle->config.i2c_inst) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    while (!(DL_I2C_getControllerStatus(handle->config.i2c_inst) & DL_I2C_CONTROLLER_STATUS_IDLE));
    
    return OLED_OK;
}

/**
 * @brief  OLED设置光标位置
 * @param  handle OLED句柄
 * @param  Y 以左上角为原点，向下方向的坐标，范围：0~7
 * @param  X 以左上角为原点，向右方向的坐标，范围：0~127
 * @retval 0: 成功, -1: 失败
 */
static int OLED_SetCursor(OLED_Handle_t* handle, uint8_t Y, uint8_t X)
{
    CHECK_HANDLE(handle);
    
    OLED_WriteCommand(handle, 0xB0 | Y);					//设置Y位置
    OLED_WriteCommand(handle, 0x10 | ((X & 0xF0) >> 4));	//设置X位置高4位
    OLED_WriteCommand(handle, 0x00 | (X & 0x0F));			//设置X位置低4位
    
    return OLED_OK;
}

/**
 * @brief  OLED清屏
 * @param  handle OLED句柄
 * @retval 0: 成功, -1: 失败
 */
int OLED_Clear(OLED_Handle_t* handle)
{  
    CHECK_HANDLE(handle);
    
    uint8_t i, j;
    for (j = 0; j < handle->config.pages; j++)
    {
        OLED_SetCursor(handle, j, 0);
        for(i = 0; i < handle->config.width; i++)
        {
            OLED_WriteData(handle, 0x00);
        }
    }
    
    return OLED_OK;
}

/**
 * @brief  OLED显示一个字符
 * @param  handle OLED句柄
 * @param  Line 行位置，范围：1~4
 * @param  Column 列位置，范围：1~16
 * @param  Char 要显示的一个字符，范围：ASCII可见字符
 * @retval 0: 成功, -1: 失败
 */
int OLED_ShowChar(OLED_Handle_t* handle, uint8_t Line, uint8_t Column, char Char)
{      	
    CHECK_HANDLE(handle);
    
    uint8_t i;
    OLED_SetCursor(handle, (Line - 1) * 2, (Column - 1) * 8);		//设置光标位置在上半部分
    for (i = 0; i < 8; i++)
    {
        OLED_WriteData(handle, OLED_F8x16[Char - ' '][i]);			//显示上半部分内容
    }
    OLED_SetCursor(handle, (Line - 1) * 2 + 1, (Column - 1) * 8);	//设置光标位置在下半部分
    for (i = 0; i < 8; i++)
    {
        OLED_WriteData(handle, OLED_F8x16[Char - ' '][i + 8]);		//显示下半部分内容
    }
    
    return OLED_OK;
}

/**
 * @brief  OLED显示字符串
 * @param  handle OLED句柄
 * @param  Line 起始行位置，范围：1~4
 * @param  Column 起始列位置，范围：1~16
 * @param  String 要显示的字符串，范围：ASCII可见字符
 * @retval 0: 成功, -1: 失败
 */
int OLED_ShowString(OLED_Handle_t* handle, uint8_t Line, uint8_t Column, char *String)
{
    CHECK_HANDLE(handle);
    
    uint8_t i;
    for (i = 0; String[i] != '\0'; i++)
    {
        OLED_ShowChar(handle, Line, Column + i, String[i]);
    }
    
    return OLED_OK;
}

/**
 * @brief  OLED次方函数
 * @retval 返回值等于X的Y次方
 */
static uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--)
    {
        Result *= X;
    }
    return Result;
}

/**
 * @brief  OLED显示数字（十进制，正数）
 * @param  handle OLED句柄
 * @param  Line 起始行位置，范围：1~4
 * @param  Column 起始列位置，范围：1~16
 * @param  Number 要显示的数字，范围：0~4294967295
 * @param  Length 要显示数字的长度，范围：1~10
 * @retval 0: 成功, -1: 失败
 */
int OLED_ShowNum(OLED_Handle_t* handle, uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    CHECK_HANDLE(handle);
    
    uint8_t i;
    for (i = 0; i < Length; i++)							
    {
        OLED_ShowChar(handle, Line, Column + i, Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
    }
    
    return OLED_OK;
}

/**
 * @brief  OLED显示数字（十进制，带符号数）
 * @param  handle OLED句柄
 * @param  Line 起始行位置，范围：1~4
 * @param  Column 起始列位置，范围：1~16
 * @param  Number 要显示的数字，范围：-2147483648~2147483647
 * @param  Length 要显示数字的长度，范围：1~10
 * @retval 0: 成功, -1: 失败
 */
int OLED_ShowSignedNum(OLED_Handle_t* handle, uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
    CHECK_HANDLE(handle);
    
    uint8_t i;
    uint32_t Number1;
    if (Number >= 0)
    {
        OLED_ShowChar(handle, Line, Column, '+');
        Number1 = Number;
    }
    else
    {
        OLED_ShowChar(handle, Line, Column, '-');
        Number1 = -Number;
    }
    for (i = 0; i < Length; i++)							
    {
        OLED_ShowChar(handle, Line, Column + i + 1, Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0');
    }
    
    return OLED_OK;
}

/**
 * @brief  OLED显示数字（十六进制，正数）
 * @param  handle OLED句柄
 * @param  Line 起始行位置，范围：1~4
 * @param  Column 起始列位置，范围：1~16
 * @param  Number 要显示的数字，范围：0~0xFFFFFFFF
 * @param  Length 要显示数字的长度，范围：1~8
 * @retval 0: 成功, -1: 失败
 */
int OLED_ShowHexNum(OLED_Handle_t* handle, uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    CHECK_HANDLE(handle);
    
    uint8_t i, SingleNumber;
    for (i = 0; i < Length; i++)							
    {
        SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;
        if (SingleNumber < 10)
        {
            OLED_ShowChar(handle, Line, Column + i, SingleNumber + '0');
        }
        else
        {
            OLED_ShowChar(handle, Line, Column + i, SingleNumber - 10 + 'A');
        }
    }
    
    return OLED_OK;
}

/**
 * @brief  OLED显示数字（二进制，正数）
 * @param  handle OLED句柄
 * @param  Line 起始行位置，范围：1~4
 * @param  Column 起始列位置，范围：1~16
 * @param  Number 要显示的数字，范围：0~1111 1111 1111 1111
 * @param  Length 要显示数字的长度，范围：1~16
 * @retval 0: 成功, -1: 失败
 */
int OLED_ShowBinNum(OLED_Handle_t* handle, uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    CHECK_HANDLE(handle);
    
    uint8_t i;
    for (i = 0; i < Length; i++)							
    {
        OLED_ShowChar(handle, Line, Column + i, Number / OLED_Pow(2, Length - i - 1) % 2 + '0');
    }
    
    return OLED_OK;
}

/**
 * @brief  OLED显示浮点数
 * @param  handle OLED句柄
 * @param  Line 起始行位置，范围：1~4
 * @param  Column 起始列位置，范围：1~16
 * @param  Num 要显示的浮点数
 * @param  Length1 整数部分个数
 * @param  Length2 小数部分个数
 * @retval 0: 成功, -1: 失败
 */
int OLED_ShowFloat(OLED_Handle_t* handle, uint8_t Line, uint8_t Column, float Num, uint8_t Length1, uint8_t Length2)
{
    CHECK_HANDLE(handle);
    
    if (Num < 0)
    {
        OLED_ShowChar(handle, Line, Column, '-');
        Num = -Num;
    }
    else {
        OLED_ShowChar(handle, Line, Column, '+');
    }
    OLED_ShowNum(handle, Line, Column + 1, (int)Num, Length1);
    OLED_ShowChar(handle, Line, Column + Length1 + 1, '.');
    OLED_ShowNum(handle, Line, Column + Length1 + 2, (int)(Num * OLED_Pow(10, Length2)) % OLED_Pow(10, Length2), Length2);
    
    return OLED_OK;
}

/**
 * @brief  OLED初始化
 * @param  handle OLED句柄
 * @param  config OLED配置参数
 * @retval 0: 成功, -1: 失败
 */
int OLED_Init(OLED_Handle_t* handle, const OLED_Config_t* config)
{
    if (handle == NULL || config == NULL) {
        return OLED_ERROR_PARAM;
    }
    
    handle->config = *config;
    handle->initialized = 0;
    
    Delay_ms(500);
    
    OLED_WriteCommand(handle, 0xAE);	//关闭显示
    
    OLED_WriteCommand(handle, 0xD5);	//设置显示时钟分频比/振荡器频率
    OLED_WriteCommand(handle, 0x80);
    
    OLED_WriteCommand(handle, 0xA8);	//设置多路复用率
    OLED_WriteCommand(handle, 0x3F);
    
    OLED_WriteCommand(handle, 0xD3);	//设置显示偏移
    OLED_WriteCommand(handle, 0x00);
    
    OLED_WriteCommand(handle, 0x40);	//设置显示开始行
    
    OLED_WriteCommand(handle, 0xA1);	//设置左右方向，0xA1正常 0xA0左右反置
    
    OLED_WriteCommand(handle, 0xC8);	//设置上下方向，0xC8正常 0xC0上下反置

    OLED_WriteCommand(handle, 0xDA);	//设置COM引脚硬件配置
    OLED_WriteCommand(handle, 0x12);
    
    OLED_WriteCommand(handle, 0x81);	//设置对比度控制
    OLED_WriteCommand(handle, 0xCF);

    OLED_WriteCommand(handle, 0xD9);	//设置预充电周期
    OLED_WriteCommand(handle, 0xF1);

    OLED_WriteCommand(handle, 0xDB);	//设置VCOMH取消选择级别
    OLED_WriteCommand(handle, 0x30);

    OLED_WriteCommand(handle, 0xA4);	//设置整个显示打开/关闭

    OLED_WriteCommand(handle, 0xA6);	//设置正常/倒转显示

    OLED_WriteCommand(handle, 0x8D);	//设置充电泵
    OLED_WriteCommand(handle, 0x14);

    OLED_WriteCommand(handle, 0xAF);	//开启显示
    
    // 标记初始化完成
    handle->initialized = 1;
    
    // OLED清屏
    OLED_Clear(handle);
    
    return OLED_OK;
}

/**
  ******************************************************************************
  * @file    oled.c
  * @brief   0.96 寸 OLED(SSD1306, 128x64) 硬件驱动（硬件 I2C1 + HAL 库）
  ******************************************************************************
  */
#include "oled.h"

#include "oled_font.h"

OLED_TypeDef g_oled;
/* 当前单屏使用：句柄和失败门控属于驱动内部，不放入页面状态。 */
static I2C_HandleTypeDef *oledI2c;
static HAL_StatusTypeDef oledStatus = HAL_ERROR;

/**
  * @brief  写命令（硬件 I2C）：[0x00, Command]
  */
static void OLED_WriteCommand(OLED_TypeDef *oled, uint8_t command)
{
    uint8_t buf[2];
    if (oled == NULL || oledI2c == NULL) return;
    buf[0] = 0x00;
    buf[1] = command;
    if (oledStatus != HAL_OK) return;
    oledStatus = HAL_I2C_Master_Transmit(oledI2c, (uint16_t)(OLED_ADDR << 1),
                                               buf, 2, OLED_I2C_TIMEOUT_MS);
}

/**
  * @brief  批量写显示数据（硬件 I2C）：[0x40, Data0, Data1, ...]
  *         自动按 OLED_I2C_BUF_SIZE 分包发送
  */
static void OLED_WriteDataBuf(OLED_TypeDef *oled, const uint8_t *data, uint16_t len)
{
    uint8_t buf[OLED_I2C_BUF_SIZE];
    uint16_t i;
    uint16_t sent = 0;
    if (oled == NULL || oledI2c == NULL) return;

    buf[0] = 0x40;
    while (sent < len)
    {
        uint8_t n = 0;
        for (i = 0; i < (OLED_I2C_BUF_SIZE - 1U) && sent < len; i++)
        {
            buf[1U + i] = data[sent++];
            n++;
        }
        if (oledStatus != HAL_OK) return;
        oledStatus = HAL_I2C_Master_Transmit(oledI2c, (uint16_t)(OLED_ADDR << 1),
                                                   buf, (uint16_t)(n + 1U), OLED_I2C_TIMEOUT_MS);
        if (oledStatus != HAL_OK)
        {
            return;
        }
    }
}

/**
  * @brief  设置光标位置
  * @param  page : 页（0~7），每页 8 个像素高
  * @param  x    : 列（0~127）
  */
void OLED_SetCursor(OLED_TypeDef *oled, uint8_t page, uint8_t x)
{
    OLED_WriteCommand(oled, (uint8_t)(0xB0U | page));                 /* 页地址 */
    OLED_WriteCommand(oled, (uint8_t)(0x10U | ((x & 0xF0U) >> 4)));   /* 列高 4 位 */
    OLED_WriteCommand(oled, (uint8_t)(0x00U | (x & 0x0FU)));          /* 列低 4 位 */
}

/**
  * @brief  清屏
  */
void OLED_Clear(OLED_TypeDef *oled)
{
    uint8_t empty[OLED_I2C_BUF_SIZE - 1U];
    uint8_t page;
    uint8_t i;

    for (i = 0; i < (OLED_I2C_BUF_SIZE - 1U); i++)
    {
        empty[i] = 0x00;
    }

    for (page = 0; page < 8U; page++)
    {
        OLED_SetCursor(oled, page, 0);
        for (i = 0; i < 128U; i += (OLED_I2C_BUF_SIZE - 1U))
        {
            uint8_t count = (uint8_t)(128U - i);
            if (count > OLED_I2C_BUF_SIZE - 1U) count = OLED_I2C_BUF_SIZE - 1U;
            OLED_WriteDataBuf(oled, empty, count);
        }
    }
}

/**
  * @brief  显示一个 8x16 ASCII 字符
  * @param  line   : 行 1~4
  * @param  column : 列 1~16
  */
void OLED_ShowChar(OLED_TypeDef *oled, uint8_t line, uint8_t column, char ch)
{
    uint8_t x;
    uint8_t idx;

    if (line < 1U || line > 4U || column < 1U || column > 16U)
    {
        return;
    }

    if (ch < ' ' || ch > '~') ch = '?';
    idx = (uint8_t)ch - ' ';
    x = (uint8_t)((column - 1U) * 8U);

    /* 上半部分 8 行，一次硬件 I2C 突发发送 */
    OLED_SetCursor(oled, (uint8_t)((line - 1U) * 2U), x);
    OLED_WriteDataBuf(oled, &OLED_F8x16[idx][0], 8);

    /* 下半部分 8 行 */
    OLED_SetCursor(oled, (uint8_t)((line - 1U) * 2U + 1U), x);
    OLED_WriteDataBuf(oled, &OLED_F8x16[idx][8], 8);
}

/**
  * @brief  显示字符串
  */
void OLED_ShowString(OLED_TypeDef *oled, uint8_t line, uint8_t column, const char *str)
{
    uint8_t i = 0;
    while (str[i] != '\0')
    {
        OLED_ShowChar(oled, line, (uint8_t)(column + i), str[i]);
        i++;
    }
}

/**
  * @brief  10 的 n 次方
  */
static uint32_t OLED_Pow(uint32_t x, uint32_t n)
{
    uint32_t result = 1;
    while (n--)
    {
        result *= x;
    }
    return result;
}

/**
  * @brief  显示无符号十进制数字
  * @param  len : 数字位数
  */
void OLED_ShowNum(OLED_TypeDef *oled, uint8_t line, uint8_t column, uint32_t num, uint8_t len)
{
    uint8_t i;
    for (i = 0; i < len; i++)
    {
        uint8_t d = (uint8_t)(num / OLED_Pow(10, (uint32_t)(len - i - 1U)) % 10U);
        OLED_ShowChar(oled, line, (uint8_t)(column + i), (char)('0' + d));
    }
}

/**
  * @brief  显示带符号十进制数字
  */
void OLED_ShowSignedNum(OLED_TypeDef *oled, uint8_t line, uint8_t column, int32_t num, uint8_t len)
{
    uint8_t i;
    uint32_t numAbs;

    if (num >= 0)
    {
        OLED_ShowChar(oled, line, column, '+');
        numAbs = (uint32_t)num;
    }
    else
    {
        OLED_ShowChar(oled, line, column, '-');
        numAbs = (uint32_t)(-num);
    }

    for (i = 0; i < len; i++)
    {
        uint8_t d = (uint8_t)(numAbs / OLED_Pow(10, (uint32_t)(len - i - 1U)) % 10U);
        OLED_ShowChar(oled, line, (uint8_t)(column + i + 1U), (char)('0' + d));
    }
}

/**
  * @brief  显示十六进制数字
  */
void OLED_ShowHexNum(OLED_TypeDef *oled, uint8_t line, uint8_t column, uint32_t num, uint8_t len)
{
    uint8_t i;
    for (i = 0; i < len; i++)
    {
        uint8_t d = (uint8_t)(num / OLED_Pow(16, (uint32_t)(len - i - 1U)) % 16U);
        if (d < 10U)
        {
            OLED_ShowChar(oled, line, (uint8_t)(column + i), (char)('0' + d));
        }
        else
        {
            OLED_ShowChar(oled, line, (uint8_t)(column + i), (char)('A' + d - 10U));
        }
    }
}

/**
  * @brief  显示浮点数
  * @param  intLen : 整数部分位数（不含符号）
  * @param  decLen : 小数部分位数
  * 例如 OLED_ShowFloat(oled, 1, 1, -12.34, 2, 2) 显示 "-12.34"
  */
void OLED_ShowFloat(OLED_TypeDef *oled, uint8_t line, uint8_t column,
                    float num, uint8_t intLen, uint8_t decLen)
{
    uint8_t pos = 0;
    uint32_t scale;
    uint32_t rounded;
    uint32_t intPart;
    uint32_t fracPart;
    uint8_t i;
    uint8_t neg = 0;

    if (num < 0.0f)
    {
        neg = 1;
        num = -num;
    }

    scale = OLED_Pow(10, decLen);

    /* 四舍五入到指定小数位后转成定点整数，避免直接对浮点取模 */
    rounded = (uint32_t)(num * (float)scale + 0.5f);
    intPart = rounded / scale;
    fracPart = rounded % scale;

    if (neg)
    {
        OLED_ShowChar(oled, line, (uint8_t)(column + pos), '-');
        pos++;
    }

    for (i = 0; i < intLen; i++)
    {
        uint8_t d = (uint8_t)(intPart / OLED_Pow(10, (uint32_t)(intLen - i - 1U)) % 10U);
        OLED_ShowChar(oled, line, (uint8_t)(column + pos), (char)('0' + d));
        pos++;
    }

    if (decLen > 0U)
    {
        OLED_ShowChar(oled, line, (uint8_t)(column + pos), '.');
        pos++;
        for (i = 0; i < decLen; i++)
        {
            uint8_t d = (uint8_t)(fracPart / OLED_Pow(10, (uint32_t)(decLen - i - 1U)) % 10U);
            OLED_ShowChar(oled, line, (uint8_t)(column + pos), (char)('0' + d));
            pos++;
        }
    }
}

/**
  * @brief  初始化 OLED（SSD1306 标准上电序列）
  */
void OLED_Init(OLED_TypeDef *oled, I2C_HandleTypeDef *hi2c)
{
    if (oled == NULL || hi2c == NULL) return;
    oledI2c = hi2c;
    oled->displayMode = 0U;
    oledStatus = HAL_OK;

    HAL_Delay(200);   /* 上电等待电源稳定 */

    OLED_WriteCommand(oled, 0xAE);   /* 关闭显示 */

    OLED_WriteCommand(oled, 0xD5);   /* 设置显示时钟分频/振荡频率 */
		//最高刷新率：0xF0
    OLED_WriteCommand(oled, 0xF0);

    OLED_WriteCommand(oled, 0xA8);   /* 多路复用率 1/64 */
    OLED_WriteCommand(oled, 0x3F);

    OLED_WriteCommand(oled, 0xD3);   /* 显示偏移 0 */
    OLED_WriteCommand(oled, 0x00);

    OLED_WriteCommand(oled, 0x40);   /* 显示起始行 0 */

    OLED_WriteCommand(oled, 0xA1);   /* 段重映射：A1 正常 / A0 镜像 */

    OLED_WriteCommand(oled, 0xC8);   /* COM 扫描方向：C8 正常 / C0 镜像 */

    OLED_WriteCommand(oled, 0xDA);   /* COM 引脚硬件配置 */
    OLED_WriteCommand(oled, 0x12);

    OLED_WriteCommand(oled, 0x81);   /* 对比度 */
    OLED_WriteCommand(oled, 0xCF);

    OLED_WriteCommand(oled, 0xD9);   /* 预充电周期 */
    OLED_WriteCommand(oled, 0xF1);

    OLED_WriteCommand(oled, 0xDB);   /* VCOMH 电平 */
    OLED_WriteCommand(oled, 0x30);

    OLED_WriteCommand(oled, 0xA4);   /* GDDRAM 内容正常输出 */
    OLED_WriteCommand(oled, 0xA6);   /* 正常显示（非反色） */

    OLED_WriteCommand(oled, 0x8D);   /* 电荷泵使能 */
    OLED_WriteCommand(oled, 0x14);

    OLED_WriteCommand(oled, 0xAF);   /* 开启显示 */

    OLED_Clear(oled);
}

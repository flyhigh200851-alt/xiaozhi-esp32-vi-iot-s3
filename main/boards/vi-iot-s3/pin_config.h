#pragma once

#include <driver/gpio.h>

/* ============ XL9555 IO 扩展器 I2C ============ */
#define XL9555_I2C_SDA_PIN          GPIO_NUM_10
#define XL9555_I2C_SCL_PIN          GPIO_NUM_11

/* ============ ST7789 i80 接口 8-bit ============ */
#define ST7789_LCD_CS               GPIO_NUM_2
#define ST7789_LCD_RS               GPIO_NUM_1   /* D/C: 0=命令,1=数据 */
#define ST7789_LCD_WR               GPIO_NUM_41  /* 写使能 */

/* 8位并行数据总线 */
#define ST7789_LCD_D0               GPIO_NUM_40
#define ST7789_LCD_D1               GPIO_NUM_38
#define ST7789_LCD_D2               GPIO_NUM_39
#define ST7789_LCD_D3               GPIO_NUM_48
#define ST7789_LCD_D4               GPIO_NUM_45
#define ST7789_LCD_D5               GPIO_NUM_21
#define ST7789_LCD_D6               GPIO_NUM_47
#define ST7789_LCD_D7               GPIO_NUM_14

/* 背光/复位 → 通过 XL9555 IO 扩展器控制 */
#define ST7789_LCD_BL_PIN           GPIO_NUM_NC
#define ST7789_LCD_RST_PIN          GPIO_NUM_NC

/* XL9555 IO 扩展器引脚定义 */
#define XL9555_LCD_RST              IO1_3
#define XL9555_LCD_BL               IO1_2
#define XL9555_TP_RST               IO1_0

/* ============ 音频 I2S ============ */
#define AUDIO_I2S_BCLK              GPIO_NUM_46
#define AUDIO_I2S_WS                GPIO_NUM_9
#define AUDIO_I2S_DOUT              GPIO_NUM_8  /* I2S 数据输出(到codec) */
#define AUDIO_I2S_DIN               GPIO_NUM_15 /* I2S 数据输入(从codec) */

/* ============ WS2812 LED ============ */
#define WS2812_GPIO_NUM             GPIO_NUM_18

/* ============ 按键 ============ */
#define BOOT_BUTTON_GPIO            GPIO_NUM_0
#define XL9555_BTN_1                IO0_1
#define XL9555_BTN_2                IO0_2
#define XL9555_BTN_3                IO0_3
#define XL9555_BTN_4                IO0_4

#define XL9555_PA_PIN               IO0_0

#define PDM_CLK                     GPIO_NUM_3
#define PDM_DATA                    GPIO_NUM_42

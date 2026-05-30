#ifndef LCD_H
#define LCD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define LCD_WIDTH  800U
#define LCD_HEIGHT 480U

#define LCD_COLOR_BLACK   0x0000U
#define LCD_COLOR_WHITE   0xFFFFU
#define LCD_COLOR_RED     0xF800U
#define LCD_COLOR_GREEN   0x07E0U
#define LCD_COLOR_BLUE    0x001FU
#define LCD_COLOR_YELLOW  0xFFE0U
#define LCD_COLOR_CYAN    0x07FFU
#define LCD_COLOR_MAGENTA 0xF81FU

extern LTDC_HandleTypeDef hltdc;

HAL_StatusTypeDef LCD_Init(void);
void LCD_BacklightSet(uint8_t enabled);
void LCD_BacklightSetDuty(uint8_t dutyPercent);
uint8_t LCD_BacklightGetDuty(void);
void LCD_BacklightSelfTest(void);
GPIO_PinState LCD_BacklightRead(void);
uint8_t LCD_FramebufferTest(void);
void LCD_Fill(uint16_t color);
uint16_t LCD_ReadPixel(uint16_t x, uint16_t y);
void LCD_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void LCD_DrawFilledRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void LCD_DrawTestPattern(void);

#ifdef __cplusplus
}
#endif

#endif

#include "lcd.h"

#include <stddef.h>

#define LCD_FRAMEBUFFER_ADDR 0xC0000000UL

#define LCD_HSYNC            1U
#define LCD_HBP              46U
#define LCD_HFP              22U
#define LCD_VSYNC            1U
#define LCD_VBP              23U
#define LCD_VFP              22U

static const uint32_t g_framebufferTestOffsets[] = {
    0U,
    1U,
    LCD_WIDTH - 1U,
    LCD_WIDTH,
    (LCD_WIDTH * LCD_HEIGHT) / 2U,
    (LCD_WIDTH * LCD_HEIGHT) - 2U,
    (LCD_WIDTH * LCD_HEIGHT) - 1U
};

LTDC_HandleTypeDef hltdc;

static volatile uint16_t * const g_framebuffer = (volatile uint16_t *)LCD_FRAMEBUFFER_ADDR;

HAL_StatusTypeDef LCD_Init(void)
{
    LTDC_LayerCfgTypeDef layer = {0};
    RCC_PeriphCLKInitTypeDef periphClock = {0};

    periphClock.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
    periphClock.PLL3.PLL3M = 25;
    periphClock.PLL3.PLL3N = 270;
    periphClock.PLL3.PLL3P = 2;
    periphClock.PLL3.PLL3Q = 2;
    periphClock.PLL3.PLL3R = 10;
    periphClock.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_0;
    periphClock.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
    periphClock.PLL3.PLL3FRACN = 0;
    if (HAL_RCCEx_PeriphCLKConfig(&periphClock) != HAL_OK)
    {
        return HAL_ERROR;
    }

    hltdc.Instance = LTDC;
    if (!LCD_FramebufferTest())
    {
        return HAL_ERROR;
    }

    hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
    hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
    hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
    hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
    hltdc.Init.HorizontalSync = LCD_HSYNC - 1U;
    hltdc.Init.VerticalSync = LCD_VSYNC - 1U;
    hltdc.Init.AccumulatedHBP = LCD_HSYNC + LCD_HBP - 1U;
    hltdc.Init.AccumulatedVBP = LCD_VSYNC + LCD_VBP - 1U;
    hltdc.Init.AccumulatedActiveW = LCD_HSYNC + LCD_HBP + LCD_WIDTH - 1U;
    hltdc.Init.AccumulatedActiveH = LCD_VSYNC + LCD_VBP + LCD_HEIGHT - 1U;
    hltdc.Init.TotalWidth = LCD_HSYNC + LCD_HBP + LCD_WIDTH + LCD_HFP - 1U;
    hltdc.Init.TotalHeigh = LCD_VSYNC + LCD_VBP + LCD_HEIGHT + LCD_VFP - 1U;
    hltdc.Init.Backcolor.Blue = 0;
    hltdc.Init.Backcolor.Green = 0;
    hltdc.Init.Backcolor.Red = 0;

    if (HAL_LTDC_Init(&hltdc) != HAL_OK)
    {
        return HAL_ERROR;
    }

    //LCD_Fill(LCD_COLOR_BLACK);
    LCD_DrawTestPattern();

    layer.WindowX0 = 0;
    layer.WindowX1 = LCD_WIDTH;
    layer.WindowY0 = 0;
    layer.WindowY1 = LCD_HEIGHT;
    layer.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
    layer.FBStartAdress = LCD_FRAMEBUFFER_ADDR;
    layer.Alpha = 255;
    layer.Alpha0 = 0;
    layer.Backcolor.Blue = 0;
    layer.Backcolor.Green = 0;
    layer.Backcolor.Red = 0;
    layer.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
    layer.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
    layer.ImageWidth = LCD_WIDTH;
    layer.ImageHeight = LCD_HEIGHT;

    if (HAL_LTDC_ConfigLayer(&hltdc, &layer, 0) != HAL_OK)
    {
        return HAL_ERROR;
    }

    LCD_BacklightSet(1U);

    return HAL_OK;
}

void LCD_BacklightSet(uint8_t enabled)
{
    LCD_BacklightSetDuty(enabled ? 100U : 0U);
}

void LCD_BacklightSetDuty(uint8_t dutyPercent)
{
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_6, dutyPercent ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void LCD_BacklightSelfTest(void)
{
    LCD_BacklightSetDuty(0U);
    HAL_Delay(200);
    LCD_BacklightSetDuty(100U);
    HAL_Delay(200);
    LCD_BacklightSetDuty(10U);
    HAL_Delay(200);
    LCD_BacklightSetDuty(100U);
}

GPIO_PinState LCD_BacklightRead(void)
{
    return HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_6);
}

uint8_t LCD_FramebufferTest(void)
{
    static const uint16_t patterns[] = {0x0000U, 0xFFFFU, 0xA55AU, 0x5AA5U};

    for (uint32_t p = 0; p < (sizeof(patterns) / sizeof(patterns[0])); ++p)
    {
        for (uint32_t i = 0; i < (sizeof(g_framebufferTestOffsets) / sizeof(g_framebufferTestOffsets[0])); ++i)
        {
            g_framebuffer[g_framebufferTestOffsets[i]] = patterns[p];
        }

        for (uint32_t i = 0; i < (sizeof(g_framebufferTestOffsets) / sizeof(g_framebufferTestOffsets[0])); ++i)
        {
            if (g_framebuffer[g_framebufferTestOffsets[i]] != patterns[p])
            {
                return 0U;
            }
        }
    }

    return 1U;
}

void LCD_Fill(uint16_t color)
{
    for (uint32_t i = 0; i < (LCD_WIDTH * LCD_HEIGHT); ++i)
    {
        g_framebuffer[i] = color;
    }
}

uint16_t LCD_ReadPixel(uint16_t x, uint16_t y)
{
    if (x >= LCD_WIDTH || y >= LCD_HEIGHT)
    {
        return 0U;
    }

    return g_framebuffer[((uint32_t)y * LCD_WIDTH) + x];
}

void LCD_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
    if (x >= LCD_WIDTH || y >= LCD_HEIGHT)
    {
        return;
    }

    g_framebuffer[((uint32_t)y * LCD_WIDTH) + x] = color;
}

void LCD_DrawFilledRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    if (x >= LCD_WIDTH || y >= LCD_HEIGHT)
    {
        return;
    }

    if ((uint32_t)x + w > LCD_WIDTH)
    {
        w = (uint16_t)(LCD_WIDTH - x);
    }

    if ((uint32_t)y + h > LCD_HEIGHT)
    {
        h = (uint16_t)(LCD_HEIGHT - y);
    }

    for (uint16_t row = 0; row < h; ++row)
    {
        volatile uint16_t *dst = &g_framebuffer[((uint32_t)(y + row) * LCD_WIDTH) + x];
        for (uint16_t col = 0; col < w; ++col)
        {
            dst[col] = color;
        }
    }
}

void LCD_DrawTestPattern(void)
{
    static const uint16_t colors[] = {
        LCD_COLOR_RED,
        LCD_COLOR_GREEN,
        LCD_COLOR_BLUE,
        LCD_COLOR_YELLOW,
        LCD_COLOR_CYAN,
        LCD_COLOR_MAGENTA,
        LCD_COLOR_WHITE,
        LCD_COLOR_BLACK
    };
    const uint16_t stripeWidth = LCD_WIDTH / (uint16_t)(sizeof(colors) / sizeof(colors[0]));

    for (uint16_t y = 0; y < LCD_HEIGHT; ++y)
    {
        for (uint16_t x = 0; x < LCD_WIDTH; ++x)
        {
            uint16_t stripe = x / stripeWidth;
            if (stripe >= (sizeof(colors) / sizeof(colors[0])))
            {
                stripe = (uint16_t)((sizeof(colors) / sizeof(colors[0])) - 1U);
            }
            g_framebuffer[((uint32_t)y * LCD_WIDTH) + x] = colors[stripe];
        }
    }
}

void HAL_LTDC_MspInit(LTDC_HandleTypeDef *hltdc_arg)
{
    GPIO_InitTypeDef gpio = {0};

    if (hltdc_arg->Instance != LTDC)
    {
        return;
    }

    __HAL_RCC_LTDC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOI_CLK_ENABLE();
    HAL_EnableCompensationCell();

    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    gpio.Alternate = GPIO_AF14_LTDC;
    gpio.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Alternate = GPIO_AF13_LTDC;
    gpio.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Alternate = GPIO_AF14_LTDC;
    gpio.Pin = GPIO_PIN_6;
    HAL_GPIO_Init(GPIOD, &gpio);

    gpio.Pin = GPIO_PIN_5 | GPIO_PIN_6;
    HAL_GPIO_Init(GPIOE, &gpio);

    gpio.Pin = GPIO_PIN_10;
    HAL_GPIO_Init(GPIOF, &gpio);

    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14;
    HAL_GPIO_Init(GPIOG, &gpio);

    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 |
               GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOH, &gpio);

    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_4 |
               GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_9 | GPIO_PIN_10;
    HAL_GPIO_Init(GPIOI, &gpio);

    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pin = GPIO_PIN_6;
    HAL_GPIO_Init(GPIOH, &gpio);
    LCD_BacklightSet(0U);
}

void HAL_LTDC_MspDeInit(LTDC_HandleTypeDef *hltdc_arg)
{
    if (hltdc_arg->Instance != LTDC)
    {
        return;
    }

    __HAL_RCC_LTDC_CLK_DISABLE();

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2 | GPIO_PIN_8);
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_6);
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_5 | GPIO_PIN_6);
    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_10);
    HAL_GPIO_DeInit(GPIOG, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14);
    HAL_GPIO_DeInit(GPIOH, GPIO_PIN_6 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |
                           GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 |
                           GPIO_PIN_15);
    HAL_GPIO_DeInit(GPIOI, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_4 |
                           GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_9 |
                           GPIO_PIN_10);
}

#include "touch.h"

#define TOUCH_SCL_GPIO_Port GPIOI
#define TOUCH_SCL_Pin       GPIO_PIN_11
#define TOUCH_SDA_GPIO_Port GPIOI
#define TOUCH_SDA_Pin       GPIO_PIN_8
#define TOUCH_RST_GPIO_Port GPIOH
#define TOUCH_RST_Pin       GPIO_PIN_4
#define TOUCH_INT_GPIO_Port GPIOG
#define TOUCH_INT_Pin       GPIO_PIN_3

#define GT9XX_ADDR_5D       0x5DU
#define GT9XX_ADDR_14       0x14U
#define GT9XX_REG_PRODUCT   0x8140U
#define GT9XX_REG_STATUS    0x814EU
#define GT9XX_REG_POINT1    0x8150U

static uint8_t g_touchAddress = GT9XX_ADDR_5D;

static void TouchGpioInit(void);

static void TouchDelay(void)
{
    for (volatile uint32_t i = 0; i < 5000U; ++i)
    {
        __NOP();
    }
}

static void SclWrite(GPIO_PinState state)
{
    HAL_GPIO_WritePin(TOUCH_SCL_GPIO_Port, TOUCH_SCL_Pin, state);
    TouchDelay();
}

static void SdaWrite(GPIO_PinState state)
{
    HAL_GPIO_WritePin(TOUCH_SDA_GPIO_Port, TOUCH_SDA_Pin, state);
    TouchDelay();
}

static GPIO_PinState SdaRead(void)
{
    TouchDelay();
    return HAL_GPIO_ReadPin(TOUCH_SDA_GPIO_Port, TOUCH_SDA_Pin);
}

static void I2cStart(void)
{
    SdaWrite(GPIO_PIN_SET);
    SclWrite(GPIO_PIN_SET);
    SdaWrite(GPIO_PIN_RESET);
    SclWrite(GPIO_PIN_RESET);
}

static void I2cStop(void)
{
    SdaWrite(GPIO_PIN_RESET);
    SclWrite(GPIO_PIN_SET);
    SdaWrite(GPIO_PIN_SET);
}

static uint8_t I2cWriteByte(uint8_t data)
{
    for (uint8_t bit = 0; bit < 8U; ++bit)
    {
        SdaWrite((data & 0x80U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        SclWrite(GPIO_PIN_SET);
        SclWrite(GPIO_PIN_RESET);
        data <<= 1;
    }

    SdaWrite(GPIO_PIN_SET);
    SclWrite(GPIO_PIN_SET);
    uint8_t ack = (SdaRead() == GPIO_PIN_RESET) ? 1U : 0U;
    SclWrite(GPIO_PIN_RESET);
    return ack;
}

static uint8_t I2cReadByte(uint8_t ack)
{
    uint8_t data = 0;

    SdaWrite(GPIO_PIN_SET);
    for (uint8_t bit = 0; bit < 8U; ++bit)
    {
        data <<= 1;
        SclWrite(GPIO_PIN_SET);
        if (SdaRead() == GPIO_PIN_SET)
        {
            data |= 1U;
        }
        SclWrite(GPIO_PIN_RESET);
    }

    SdaWrite(ack ? GPIO_PIN_RESET : GPIO_PIN_SET);
    SclWrite(GPIO_PIN_SET);
    SclWrite(GPIO_PIN_RESET);
    SdaWrite(GPIO_PIN_SET);

    return data;
}

static HAL_StatusTypeDef TouchWriteReg(uint8_t address, uint16_t reg, const uint8_t *data, uint16_t len)
{
    I2cStart();
    if (!I2cWriteByte((uint8_t)(address << 1)))
    {
        I2cStop();
        return HAL_ERROR;
    }
    if (!I2cWriteByte((uint8_t)(reg >> 8)) || !I2cWriteByte((uint8_t)reg))
    {
        I2cStop();
        return HAL_ERROR;
    }
    for (uint16_t i = 0; i < len; ++i)
    {
        if (!I2cWriteByte(data[i]))
        {
            I2cStop();
            return HAL_ERROR;
        }
    }
    I2cStop();
    return HAL_OK;
}

static HAL_StatusTypeDef TouchReadReg(uint8_t address, uint16_t reg, uint8_t *data, uint16_t len)
{
    I2cStart();
    if (!I2cWriteByte((uint8_t)(address << 1)))
    {
        I2cStop();
        return HAL_ERROR;
    }
    if (!I2cWriteByte((uint8_t)(reg >> 8)) || !I2cWriteByte((uint8_t)reg))
    {
        I2cStop();
        return HAL_ERROR;
    }

    I2cStart();
    if (!I2cWriteByte((uint8_t)((address << 1) | 1U)))
    {
        I2cStop();
        return HAL_ERROR;
    }

    for (uint16_t i = 0; i < len; ++i)
    {
        data[i] = I2cReadByte((i + 1U) < len);
    }
    I2cStop();
    return HAL_OK;
}

static void TouchGpioInit(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOI_CLK_ENABLE();

    gpio.Pin = TOUCH_SCL_Pin | TOUCH_SDA_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOI, &gpio);

    gpio.Pin = TOUCH_RST_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(TOUCH_RST_GPIO_Port, &gpio);

    gpio.Pin = TOUCH_INT_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(TOUCH_INT_GPIO_Port, &gpio);

    SdaWrite(GPIO_PIN_SET);
    SclWrite(GPIO_PIN_SET);
    HAL_GPIO_WritePin(TOUCH_RST_GPIO_Port, TOUCH_RST_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TOUCH_INT_GPIO_Port, TOUCH_INT_Pin, GPIO_PIN_SET);
}

static void TouchIntAsInput(void)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Pin = TOUCH_INT_Pin;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(TOUCH_INT_GPIO_Port, &gpio);
}

static void TouchIntAsOutput(GPIO_PinState state)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Pin = TOUCH_INT_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(TOUCH_INT_GPIO_Port, &gpio);
    HAL_GPIO_WritePin(TOUCH_INT_GPIO_Port, TOUCH_INT_Pin, state);
}

static HAL_StatusTypeDef TouchProbeAddress(uint8_t address, GPIO_PinState intState)
{
    uint8_t product[4];

    SdaWrite(GPIO_PIN_SET);
    SclWrite(GPIO_PIN_SET);
    HAL_Delay(2);

    HAL_GPIO_WritePin(TOUCH_RST_GPIO_Port, TOUCH_RST_Pin, GPIO_PIN_RESET);
    TouchIntAsOutput(intState);
    HAL_Delay(20);
    HAL_GPIO_WritePin(TOUCH_RST_GPIO_Port, TOUCH_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(20);
    TouchIntAsInput();
    HAL_Delay(100);

    for (uint8_t attempt = 0; attempt < 10U; ++attempt)
    {
        if (TouchReadReg(address, GT9XX_REG_PRODUCT, product, sizeof(product)) == HAL_OK)
        {
            g_touchAddress = address;
            return HAL_OK;
        }

        HAL_Delay(20);
    }

    return HAL_ERROR;
}

static HAL_StatusTypeDef TouchProbeAddressNoReset(uint8_t address)
{
    uint8_t product[4];

    for (uint8_t attempt = 0; attempt < 10U; ++attempt)
    {
        if (TouchReadReg(address, GT9XX_REG_PRODUCT, product, sizeof(product)) == HAL_OK)
        {
            g_touchAddress = address;
            return HAL_OK;
        }

        HAL_Delay(20);
    }

    return HAL_ERROR;
}

HAL_StatusTypeDef Touch_Init(void)
{
    TouchGpioInit();
    HAL_Delay(20);

    if (TouchProbeAddressNoReset(GT9XX_ADDR_5D) == HAL_OK)
    {
        return HAL_OK;
    }

    if (TouchProbeAddressNoReset(GT9XX_ADDR_14) == HAL_OK)
    {
        return HAL_OK;
    }

    if (TouchProbeAddress(GT9XX_ADDR_5D, GPIO_PIN_SET) == HAL_OK)
    {
        return HAL_OK;
    }

    return TouchProbeAddress(GT9XX_ADDR_14, GPIO_PIN_RESET);
}

HAL_StatusTypeDef Touch_Read(TouchState *state)
{
    uint8_t status = 0;
    uint8_t point[8] = {0};
    uint8_t clear = 0;

    if (state == NULL)
    {
        return HAL_ERROR;
    }

    state->pressed = 0;
    state->x = 0;
    state->y = 0;

    if (TouchReadReg(g_touchAddress, GT9XX_REG_STATUS, &status, sizeof(status)) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if ((status & 0x80U) == 0U)
    {
        return HAL_OK;
    }

    if ((status & 0x0FU) != 0U)
    {
        if (TouchReadReg(g_touchAddress, GT9XX_REG_POINT1, point, sizeof(point)) == HAL_OK)
        {
            state->pressed = 1U;
            state->x = (uint16_t)point[1] | ((uint16_t)point[2] << 8);
            state->y = (uint16_t)point[3] | ((uint16_t)point[4] << 8);
        }
    }

    (void)TouchWriteReg(g_touchAddress, GT9XX_REG_STATUS, &clear, sizeof(clear));
    return HAL_OK;
}

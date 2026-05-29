#ifndef TOUCH_H
#define TOUCH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef struct
{
    uint8_t pressed;
    uint16_t x;
    uint16_t y;
} TouchState;

HAL_StatusTypeDef Touch_Init(void);
HAL_StatusTypeDef Touch_Read(TouchState *state);

#ifdef __cplusplus
}
#endif

#endif

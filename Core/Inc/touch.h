#ifndef TOUCH_H
#define TOUCH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define TOUCH_MAX_POINTS 5U

typedef struct
{
    uint8_t id;
    uint16_t x;
    uint16_t y;
    uint16_t size;
} TouchPoint;

typedef struct
{
    uint8_t count;
    TouchPoint points[TOUCH_MAX_POINTS];
} TouchState;

HAL_StatusTypeDef Touch_Init(void);
HAL_StatusTypeDef Touch_Read(TouchState *state);

#ifdef __cplusplus
}
#endif

#endif

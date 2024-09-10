#ifndef __GAME_CONTRLLER_H
#define __GAME_CONTRLLER_H

#include "stm32f4xx_hal.h"

/* joystick report data structure */
typedef struct
{
    int16_t X;
    int16_t Y;
    int16_t Z;
    int16_t Rz;
    uint16_t Rx;
    uint16_t Ry;
    uint16_t slider;
    uint16_t dial;
    uint8_t HAT;
    uint32_t buttons;
} __attribute__((__packed__)) JoyData_t;

void gameControllerLoop(void);

#endif /* __GAME_CONTRLLER_H */
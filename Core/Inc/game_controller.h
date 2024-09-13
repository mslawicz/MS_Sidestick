#ifndef __GAME_CONTRLLER_H
#define __GAME_CONTRLLER_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

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

typedef struct
{
    int16_t X;
    int16_t Y;
    int16_t Z;
} int16_XYZ_t;

typedef struct
{
    float X;
    float Y;
    float Z;
} float_XYZ_t;

typedef struct
{
    float pitch;
    float roll;
    float yaw;
} float_3D_t;


void gameControllerLoop(void);

#endif /* __GAME_CONTRLLER_H */
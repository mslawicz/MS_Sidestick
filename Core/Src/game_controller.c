#include "game_controller.h"
#include "usbd_customhid.h"
#include "usbd_custom_hid_if.h"
#include "main.h"
#include "IMU.h"

osEventFlagsId_t* pGameCtrlEventsHandle;
JoyData_t joyReport = {0};

/* game controller loop
    upon reception of an event, the function prepares the controll data
    for the host and sends the appriopriate report */
void gameControllerLoop(void)
{
    uint16_t step = 0;   // XXX test

    /* IMU timer will call the first IMU readout */
    start_IMU_timer();

    while(1)
    {
        osEventFlagsWait(gameCtrlEventsHandle, IMU_DATA_READY_EVENT, osFlagsWaitAny, osWaitForever);

        /* restart IMU timer to prevent additional readouts if IMU interrupts come on time */
        start_IMU_timer();        

        HAL_GPIO_WritePin(TEST1_GPIO_Port, TEST1_Pin, GPIO_PIN_SET);
        int16_t i16 = -32767 + (step % 100) * 655;
        joyReport.X = i16;
        joyReport.Y = i16;
        joyReport.Z = i16;
        joyReport.Rz = i16;
        uint16_t u16 = (step % 100) * 327;
        joyReport.Rx = u16;
        joyReport.Ry = u16;
        joyReport.slider = u16;
        joyReport.dial = u16;
        joyReport.HAT = ((step >> 4) % 8) + 1;
        joyReport.buttons = 1 << ((step >> 4) % 32);
        step++;
        USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&joyReport, sizeof(joyReport));
        if((step % 60) == 0)
        {
            /* it should be executed roughly every half a second */
            HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
        }
        HAL_GPIO_WritePin(TEST1_GPIO_Port, TEST1_Pin, GPIO_PIN_RESET);
    }
}  
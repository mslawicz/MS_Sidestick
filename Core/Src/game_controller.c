#include "game_controller.h"
#include "cmsis_os.h"
#include "usbd_customhid.h"
#include "usbd_custom_hid_if.h"

JoyData_t joyReport = {0};

/* game controller loop
    upon reception of an event, the function prepares the controll data
    for the host and sends the appriopriate report */
void gameControllerLoop(void)
{
    uint16_t step = 0;   // XXX test

    while(1)
    {
        osDelay(100);   //XXX test
        int16_t i16 = -32767 + (step % 10) * 6553;
        joyReport.X = i16;
        joyReport.Y = i16;
        joyReport.Z = i16;
        joyReport.HAT = (step % 8) + 1;
        joyReport.buttons = 1 << (step % 32);
        step++;
        USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&joyReport, sizeof(joyReport));
    }
}  
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
    uint16_t step = 0;
    int16_XYZ_t IMU_G_rawData;  // IMU gyroscope raw data
    int16_XYZ_t IMU_A_rawData;  // IMU accelerometer raw data
    int16_XYZ_t IMU_M_rawData;  // IMU magnetometer raw data
    static const float IMU_G_toRadPerSec = 2.6632423658e-4f;
    static const float IMU_A_toG = 3.0518509476e-5f;
    static const float IMU_M_toGauss = 4.882961516e-4f;
    float_XYZ_t stickAngularRate;   //stick angular rate in rad/s
    float_XYZ_t stickAcceleration;  // stick acceleration in G */
    float_XYZ_t stickMagnFluxDens;  // stick magnetic flux density in gauss */

    /* IMU timer will call the first IMU readout */
    start_IMU_timer();

    while(1)
    {
        osEventFlagsWait(gameCtrlEventsHandle, IMU_DATA_READY_EVENT, osFlagsWaitAny, osWaitForever);

        /* restart IMU timer to prevent additional readouts if IMU interrupts come on time */
        start_IMU_timer();

        /* read IMU data from its buffers */
        /* gyroscope full scale +- 500 deg/s */
        IMU_G_rawData.X = *(int16_t*)(IMU_AG_rxBuf);
        IMU_G_rawData.Y = *(int16_t*)(IMU_AG_rxBuf + 2);
        IMU_G_rawData.Z = *(int16_t*)(IMU_AG_rxBuf + 4);

        /* accelerometer full scale +- 2 G */
        IMU_A_rawData.X = *(int16_t*)(IMU_AG_rxBuf + 6);
        IMU_A_rawData.Y = *(int16_t*)(IMU_AG_rxBuf + 8);
        IMU_A_rawData.Z = *(int16_t*)(IMU_AG_rxBuf + 10);

        /* magnetometer full scale +- 16 gauss */
        IMU_M_rawData.X = *(int16_t*)(IMU_M_rxBuf);
        IMU_M_rawData.Y = *(int16_t*)(IMU_M_rxBuf + 2);
        IMU_M_rawData.Z = *(int16_t*)(IMU_M_rxBuf + 4); 

        /* convert IMU data to real values */
        /* stick angular rate [rad/s] = val / 0x7FFF * 500 deg/s / 360 deg * 2*PI */
        /* stick angular rate [rad/s] = val *  2.6632423658e-4 */
        stickAngularRate.X = IMU_G_rawData.X * IMU_G_toRadPerSec;
        stickAngularRate.Y = IMU_G_rawData.Y * IMU_G_toRadPerSec;
        stickAngularRate.Z = IMU_G_rawData.Z * IMU_G_toRadPerSec;

        /* stick acceleration [G] = val / 0x7FFF * 2 G */
        /* stick acceleration [G] = val * 3.0518509476e-5 */
        stickAcceleration.X = IMU_A_rawData.X * IMU_A_toG;
        stickAcceleration.Y = IMU_A_rawData.Y * IMU_A_toG;
        stickAcceleration.Z = IMU_A_rawData.Z * IMU_A_toG;

        /* stick magnetic flux density [gauss] = val / 0x7FFF * 16 gauss */
        /* stick magnetic flux density [gauss] = val * 4.882961516e-4 */
        stickMagnFluxDens.X = IMU_M_rawData.X * IMU_M_toGauss;
        stickMagnFluxDens.Y = IMU_M_rawData.Y * IMU_M_toGauss;
        stickMagnFluxDens.Z = IMU_M_rawData.Z * IMU_M_toGauss;

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
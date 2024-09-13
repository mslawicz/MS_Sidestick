#include "game_controller.h"
#include "usbd_customhid.h"
#include "usbd_custom_hid_if.h"
#include "main.h"
#include "IMU.h"
#include "convert.h"
#include <math.h>

osEventFlagsId_t* pGameCtrlEventsHandle;
JoyData_t joyReport = {0};

/* game controller loop
    upon reception of an event, the function prepares the controll data
    for the host and sends the appriopriate report */
void gameControllerLoop(void)
{
    static const float IMU_G_sensitivity = 2.6632423658e-4f;     //rad/s/1 bit
    static const float IMU_A_sensitivity = 6.1037018952e-5f;     //G/1 bit
    static const float IMU_M_sensitivity = 4.882961516e-4f;      //gauss/1 bit
    static const float PI_3 = 1.04719755f;      // PI/3 (60 deg)
    static const float PI_2 = 1.5707963268f;  // PI/2
    static const float Max15bit = 32767.0f;    //max 15-bit value in float type
    uint16_t step = 0;
    int16_XYZ_t IMU_G_rawData;  // IMU gyroscope raw data
    int16_XYZ_t IMU_A_rawData;  // IMU accelerometer raw data
    int16_XYZ_t IMU_M_rawData;  // IMU magnetometer raw data
    float_XYZ_t sensorAngularRate;   //sensor angular rate in rad/s
    float_XYZ_t sensorAcceleration;  // sensor acceleration in G */
    float_XYZ_t sensorMagnFluxDens;  // sensor magnetic flux density in gauss */
    float_3D_t sensorPosition;   /* 3D sensor position calculated from accelerometer and magnetometer (no gyroscope) */

    /* IMU timer will call the first IMU readout */
    start_IMU_timer();

    while(1)
    {
        osEventFlagsWait(gameCtrlEventsHandle, IMU_DATA_READY_EVENT, osFlagsWaitAny, osWaitForever);

        /* restart IMU timer to prevent additional readouts if IMU interrupts come on time */
        start_IMU_timer();
        HAL_GPIO_WritePin(TEST1_GPIO_Port, TEST1_Pin, GPIO_PIN_SET);    //XXX test

        /* read raw IMU data from its buffers; range -32767 ... 32767 */
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

        /* convert raw IMU data to real values */
        /* sensor angular rate [rad/s] = raw / 0x7FFF * 500 deg/s / 360 deg * 2*PI */
        /* sensor angular rate [rad/s] = raw *  2.6632423658e-4 */
        sensorAngularRate.X = IMU_G_rawData.X * IMU_G_sensitivity;
        sensorAngularRate.Y = IMU_G_rawData.Y * IMU_G_sensitivity;
        sensorAngularRate.Z = IMU_G_rawData.Z * IMU_G_sensitivity;

        /* sensor acceleration [G] = raw / 0x7FFF * 2 G */
        /* sensor acceleration [G] = raw * 6.1037018952e-5 */
        sensorAcceleration.X = IMU_A_rawData.X * IMU_A_sensitivity;
        sensorAcceleration.Y = IMU_A_rawData.Y * IMU_A_sensitivity;
        sensorAcceleration.Z = IMU_A_rawData.Z * IMU_A_sensitivity;

        /* sensor magnetic flux density [gauss] = raw / 0x7FFF * 16 gauss */
        /* sensor magnetic flux density [gauss] = raw * 4.882961516e-4 */
        sensorMagnFluxDens.X = IMU_M_rawData.X * IMU_M_sensitivity;
        sensorMagnFluxDens.Y = IMU_M_rawData.Y * IMU_M_sensitivity;
        sensorMagnFluxDens.Z = IMU_M_rawData.Z * IMU_M_sensitivity;

        /* acceleration vector precalculations */
        float accelerationZ2 = sensorAcceleration.Z * sensorAcceleration.Z;
        float accelerationXZ = sqrt(sensorAcceleration.X * sensorAcceleration.X + accelerationZ2);
        float accelerationYZ = sqrt(sensorAcceleration.Y * sensorAcceleration.Y + accelerationZ2);

        /* calculate sensor pitch from sensor acceleration [rad] */
        sensorPosition.pitch = atan2f(-sensorAcceleration.Y, accelerationXZ);

        /* calculate sensor roll from sensor acceleration [rad] */
        sensorPosition.roll = atan2f(sensorAcceleration.X, accelerationYZ);        

        /* calculate sensor yaw from sensor magnetic flux density [rad] */
        static const float magnetometerYawGain = 0.42f;    //experimentally adjusted for real angles
        sensorPosition.yaw = magnetometerYawGain * atan2f(sensorMagnFluxDens.Z, -sensorMagnFluxDens.X);



        int16_t i16 = -32767 + (step % 100) * 655;
        joyReport.X = (int16_t)scale(-PI_3, PI_3, sensorPosition.roll, -Max15bit, Max15bit);   //TODO it should be replaced with calibrated stick roll
        joyReport.Y = (int16_t)scale(-PI_3, PI_3, sensorPosition.pitch, -Max15bit, Max15bit);   //TODO it should be replaced with calibrated stick pitch
        joyReport.Z = i16;
        /* sensor yaw in -PI/3 ... PI/3 range */
        joyReport.Rz = (int16_t)scale(-PI_3, PI_3, sensorPosition.yaw, -Max15bit, Max15bit);   //TODO it should be replaced with calibrated stick yaw
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
        HAL_GPIO_WritePin(TEST1_GPIO_Port, TEST1_Pin, GPIO_PIN_RESET);  //XXX test
    }
}  
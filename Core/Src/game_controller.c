#include "game_controller.h"
#include "usbd_customhid.h"
#include "usbd_custom_hid_if.h"
#include "main.h"
#include "IMU.h"
#include "convert.h"
#include <math.h>

#define USE_COMPL_FLT_YAW   0

osEventFlagsId_t* pGameCtrlEventsHandle;
JoyData_t joyReport = {0};
TIM_HandleTypeDef* pStopWatch = NULL;

//XXX test - monitor variables
float global_x;
float global_y;
float global_z;

/* game controller loop
    upon reception of an event, the function prepares the controll data
    for the host and sends the appriopriate report */
void gameControllerLoop(void)
{
    static const float IMU_G_sensitivity = 2.6632423658e-4f;     //rad/s/1 bit
    static const float IMU_A_sensitivity = 6.1037018952e-5f;     //G/1 bit
    static const float IMU_M_sensitivity = 4.882961516e-4f;      //gauss/1 bit
    static const float PI_3 = 1.04719755f;      // PI/3 (60 deg)
    static const float Max15bit = 32767.0f;    //max 15-bit value in float type
    uint16_t loopCounter = 0;
    int16_XYZ_t IMU_G_rawData;  // IMU gyroscope raw data
    int16_XYZ_t IMU_A_rawData;  // IMU accelerometer raw data
    int16_XYZ_t IMU_M_rawData;  // IMU magnetometer raw data
    float_XYZ_t sensorAngularRate;   //sensor angular rate in rad/s
    float_XYZ_t sensorAcceleration;  // sensor acceleration in G */
    float_XYZ_t sensorMagnFluxDens;  // sensor magnetic flux density in gauss */
    float_3D_t sensorPosition;   /* 3D sensor position calculated from accelerometer or magnetometer (no gyroscope) [rad] */
    float_3D_t sensorPositionFused = {0, 0, 0}; /* 3D sensor position calculated from accelerometer or magnetometer, and gyroscope (sensor fusion) [rad] */
    uint32_t previousTimerValue;     // used for loop pass interval calculation
    float_3D_t prevSensorPositionFused;     // used for stick calibration
    float_3D_t sensorVariability ={0};   // sensor variability used for stick calibration
    float_3D_t sensorReference = {0, 0, 0};     // measured sensor reference values for calibration [rad]
    float_3D_t sensorPositionCalibrated;    // calibrated sensor position [rad]
    float_3D_t stickPosition;   // stick position independent on yaw

    /* IMU timer will call the first IMU readout */
    start_IMU_timer();
    HAL_TIM_Base_Start(pStopWatch);
    previousTimerValue = pStopWatch->Instance->CNT;

    while(1)
    {
        osEventFlagsWait(gameCtrlEventsHandle, IMU_DATA_READY_EVENT, osFlagsWaitAny, osWaitForever);
        HAL_GPIO_WritePin(TEST1_GPIO_Port, TEST1_Pin, GPIO_PIN_SET);    //XXX test
        loopCounter++;

        /* restart IMU timer to prevent additional readouts if IMU interrupts come on time */
        start_IMU_timer();

        /* calculate time from the previous call */
        uint32_t currentTimerValue = pStopWatch->Instance->CNT;
        float deltaT = 1e-6f * (float)(currentTimerValue - previousTimerValue);     // time elapsed since last pass [s]
        previousTimerValue = currentTimerValue;

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

        /* calculate sensor roll from sensor acceleration [rad] */
        sensorPosition.roll = atan2f(sensorAcceleration.X, accelerationYZ);        

        /* calculate sensor pitch from sensor acceleration [rad] */
        sensorPosition.pitch = atan2f(-sensorAcceleration.Y, accelerationXZ);

        /* calculate sensor yaw from sensor magnetic flux density [rad] */
        static const float magnetometerYawGain = 0.27f;    //experimentally adjusted for real angles
        sensorPosition.yaw = magnetometerYawGain * atan2f(sensorMagnFluxDens.Z, -sensorMagnFluxDens.X);

        /* store previous value of sensor position fused for calibration calculations */
        prevSensorPositionFused.roll = sensorPositionFused.roll;
        prevSensorPositionFused.pitch = sensorPositionFused.pitch;
        prevSensorPositionFused.yaw = sensorPositionFused.roll;

        /* sensor fusion with gyroscope data; complementary filter used */
        static const float ComplementaryFilterAlpha = 0.02f;

        /* calculate sensor roll from sensor A+G fusion with complementary filter [rad] */
        sensorPositionFused.roll = (1.0f - ComplementaryFilterAlpha) * (sensorPositionFused.roll - sensorAngularRate.Y * deltaT) +
                                    ComplementaryFilterAlpha * sensorPosition.roll;

        /* calculate sensor pitch from sensor A+G fusion with complementary filter [rad] */
        sensorPositionFused.pitch = (1.0f - ComplementaryFilterAlpha) * (sensorPositionFused.pitch - sensorAngularRate.X * deltaT) +
                                    ComplementaryFilterAlpha * sensorPosition.pitch;

#if(USE_COMPL_FLT_YAW)
        /* calculate sensor pitch from sensor M+G fusion with complementary filter [rad] */
        sensorPositionFused.yaw = (1.0f - ComplementaryFilterAlpha) * (sensorPositionFused.yaw - sensorAngularRate.Z * deltaT) +
                                    ComplementaryFilterAlpha * sensorPosition.yaw;        
#else
        /* senor fusion is not applied to yaw, because magnetometer calculations are precise and reliable */
        sensorPositionFused.yaw = sensorPosition.yaw;
#endif //USE_COMPL_FLT_YAW        

        /* calculate sensor position variability for roll and pitch axes */
        const float VariabilityFilterAlpha = 0.01f;
        float reciprocalDeltaT = (deltaT > 0.0f) ? (1.0f / deltaT) : 1.0f;
        sensorVariability.roll = (1.0f - VariabilityFilterAlpha) * sensorVariability.roll + 
            VariabilityFilterAlpha * fabsf(sensorPositionFused.roll - prevSensorPositionFused.roll) * reciprocalDeltaT;
        sensorVariability.pitch = (1.0f - VariabilityFilterAlpha) * sensorVariability.pitch + 
            VariabilityFilterAlpha * fabsf(sensorPositionFused.pitch - prevSensorPositionFused.pitch) * reciprocalDeltaT;

        /* continuous sensor calibration */
        static const float SensorVariabilityTreshold = 0.0025f;
        if((sensorVariability.roll < SensorVariabilityTreshold) &&
            (sensorVariability.pitch < SensorVariabilityTreshold))
        {
            /* sensor steady - calibrate */
            static const float ReferenceFilterAlpha = 0.01f;
            sensorReference.roll = (1.0f - ReferenceFilterAlpha) * sensorReference.roll + ReferenceFilterAlpha * sensorPositionFused.roll;
            sensorReference.pitch = (1.0f - ReferenceFilterAlpha) * sensorReference.pitch + ReferenceFilterAlpha * sensorPositionFused.pitch;
            sensorReference.yaw = (1.0f - ReferenceFilterAlpha) * sensorReference.yaw + ReferenceFilterAlpha * sensorPositionFused.yaw;
            HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
        }
        else
        {
            /* sensor not steady */
            HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
        }

        /* calculate calibrated sensor position [rad] */
        sensorPositionCalibrated.roll = sensorPositionFused.roll - sensorReference.roll;
        sensorPositionCalibrated.pitch = sensorPositionFused.pitch - sensorReference.pitch;
        sensorPositionCalibrated.yaw = sensorPositionFused.yaw - sensorReference.yaw;

        /* calculate stick roll and pitch independent on yaw [rad] */
        float cosYaw = cosf(sensorPositionCalibrated.yaw);
        float sinYaw = sinf(sensorPositionCalibrated.yaw);
        stickPosition.roll = sensorPositionCalibrated.roll * cosYaw - sensorPositionCalibrated.pitch * sinYaw;
        stickPosition.pitch = sensorPositionCalibrated.pitch * cosYaw + sensorPositionCalibrated.roll * sinYaw;                             
        stickPosition.yaw = sensorPositionCalibrated.yaw;                                                     

                                                             


        //XXX test
        global_x = 10000.0f * stickPosition.roll;
        global_y = 10000.0f * stickPosition.pitch;
        global_z = 10000.0f * stickPosition.yaw;
        


        int16_t i16 = -32767 + (loopCounter % 100) * 655;
        joyReport.X = (int16_t)scale(-PI_3, PI_3, stickPosition.roll, -Max15bit, Max15bit);
        joyReport.Y = (int16_t)scale(-PI_3, PI_3, stickPosition.pitch, -Max15bit, Max15bit);
        joyReport.Z = i16;
        joyReport.Rz = (int16_t)scale(-PI_3, PI_3, stickPosition.yaw, -Max15bit, Max15bit);
        uint16_t u16 = (loopCounter % 100) * 327;
        joyReport.Rx = u16;
        joyReport.Ry = u16;
        joyReport.slider = u16;
        joyReport.dial = u16;
        joyReport.HAT = ((loopCounter >> 4) % 8) + 1;
        joyReport.buttons = 1 << ((loopCounter >> 4) % 32);
        USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&joyReport, sizeof(joyReport));
        if((loopCounter % 60) == 0)
        {
            /* it should be executed roughly every half a second */
            HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
        }
        HAL_GPIO_WritePin(TEST1_GPIO_Port, TEST1_Pin, GPIO_PIN_RESET);  //XXX test
    }
}  
#include "game_controller.h"
#include "usbd_customhid.h"
#include "usbd_custom_hid_if.h"
#include "main.h"
#include "IMU.h"
#include "convert.h"
#include <math.h>

#define USE_COMPL_FLT_YAW   0

enum Buttons_t
{
    BT_HAT_MID,
    BT_HAT_UP,
    BT_HAT_DOWN,
    BT_HAT_RIGHT,
    BT_HAT_LEFT
};

osEventFlagsId_t* pGameCtrlEventsHandle;
JoyData_t joyReport = {0};
TIM_HandleTypeDef* pStopWatch = NULL;

//XXX test - monitor variables
float global_x;
float global_y;
float global_z;

static uint8_t getHAT(bool HAT_active);
static uint32_t getButtons(bool HAT_active);

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
    float brakeLeft;
    float brakeRight;
    int16_t lastJoyReportY = 0;   // store last joystick report Y value when brakes are not active

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

        /* calculate brakes */
        if(HAL_GPIO_ReadPin(HAT_RESET_GPIO_Port, HAT_RESET_Pin) == GPIO_PIN_RESET)
        {
            // brakes active
            // both brakes activated with stick pushed forward
            brakeLeft = brakeRight = 2.0f * ( stickPosition.pitch < 0 ? -stickPosition.pitch : 0.0f);
            // left and right brakes activated with joystick deflected sideways
            brakeLeft += (stickPosition.roll < 0 ? -stickPosition.roll : 0.0f);
            brakeRight += (stickPosition.roll > 0 ? stickPosition.roll : 0.0f);
            // joystick Y value is hold steady
            joyReport.Y = lastJoyReportY;
        }
        else
        {
            //joystick Y value is uptaded when brakes are not active only
            joyReport.Y = (int16_t)scale(-PI_3, PI_3, stickPosition.pitch, -Max15bit, Max15bit);
            lastJoyReportY = joyReport.Y;
            //release brakes
            brakeLeft = brakeRight = 0.0f;
        }                                         


        joyReport.X = (int16_t)scale(-PI_3, PI_3, stickPosition.roll, -Max15bit, Max15bit);
        joyReport.Z = 0;
        joyReport.Rz = (int16_t)scale(-PI_3, PI_3, stickPosition.yaw, -Max15bit, Max15bit);
        joyReport.Rx = (uint16_t)scale(0, 1.0f, brakeLeft, 0, Max15bit);
        joyReport.Ry = (uint16_t)scale(0, 1.0f, brakeRight, 0, Max15bit);
        joyReport.slider = 0;
        joyReport.dial = 0;
        joyReport.HAT = getHAT(true);
        joyReport.buttons = getButtons(false);
        USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&joyReport, sizeof(joyReport));


        //XXX test
        global_x = 10000.0f * stickPosition.roll;
        global_y = 10000.0f * stickPosition.pitch;
        global_z = 10000.0f * stickPosition.yaw;

        if((loopCounter % 60) == 0)
        {
            /* it should be executed roughly every half a second */
            HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
        }
        HAL_GPIO_WritePin(TEST1_GPIO_Port, TEST1_Pin, GPIO_PIN_RESET);  //XXX test
    }
}  

/* get HAT switch value */
/* if HAT is not active, returns 0 */
uint8_t getHAT(bool HAT_active)
{
    static const uint8_t HAT_value[] =
    {
        0,  //0x00: combination not possible
        0,  //0x01: combination not possible
        0,  //0x02: combination not possible
        6,  //0x03: down+left
        0,  //0x04: combination not possible
        0,  //0x05: combination not possible
        8,  //0x06: up+left
        7,  //0x07: left
        0,  //0x08: combination not possible
        4,  //0x09: down+right
        0,  //0x0A: combination not possible
        5,  //0x0B: down
        2,  //0x0C: up+right
        3,  //0x0D: right
        1,  //0x0E: up
        0   //0x0F: no button pressed
    };

    if(!HAT_active)
    {
        return 0;
    }

    uint8_t HAT_buttons =
        HAL_GPIO_ReadPin(HAT_UP_GPIO_Port, HAT_UP_Pin) |
        (HAL_GPIO_ReadPin(HAT_RIGHT_GPIO_Port, HAT_RIGHT_Pin) << 1) |
        (HAL_GPIO_ReadPin(HAT_DOWN_GPIO_Port, HAT_DOWN_Pin) << 2) |
        (HAL_GPIO_ReadPin(HAT_LEFT_GPIO_Port, HAT_LEFT_Pin) << 3);

    return HAT_value[HAT_buttons];
}

uint32_t getButtons(bool HAT_active)
{
    uint32_t buttons = 0;

    buttons |= (HAL_GPIO_ReadPin(HAT_MID_GPIO_Port, HAT_MID_Pin) ^ 1) << BT_HAT_MID;

    if(!HAT_active)
    {
        //these buttons are handled when HAT is not active
        buttons |= (HAL_GPIO_ReadPin(HAT_UP_GPIO_Port, HAT_UP_Pin) ^ 1) << BT_HAT_UP;
        buttons |= (HAL_GPIO_ReadPin(HAT_DOWN_GPIO_Port, HAT_DOWN_Pin) ^ 1) << BT_HAT_DOWN;
        buttons |= (HAL_GPIO_ReadPin(HAT_RIGHT_GPIO_Port, HAT_RIGHT_Pin) ^ 1) << BT_HAT_RIGHT;
        buttons |= (HAL_GPIO_ReadPin(HAT_LEFT_GPIO_Port, HAT_LEFT_Pin) ^ 1) << BT_HAT_LEFT;        
    }

    return buttons;
}
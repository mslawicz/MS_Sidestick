#include "IMU.h"
#include "logger.h"
#include "main.h"
#include "stdint.h"

#define IMU_RX_BUF_SIZE 32
#define IMU_RX_TIMEOUT  10
#define IMU_TX_TIMEOUT  20

I2C_HandleTypeDef* pIMU_I2C;
static uint8_t rxBuf[IMU_RX_BUF_SIZE];

void IMU_init(void)
{
    HAL_StatusTypeDef status;
    const uint8_t Cfg_INT1_CTRL[] =
    {
        0x02   //INT1_CTRL: Gyroscope data ready on INT 1_A/G pin
    };

    const uint8_t Cfg_CTRL_REG1_G[] =
    {
        0x69,   //CTRL_REG1_G: ODR 119 Hz, 500 dps, LPF 31 Hz 
        0x00,
        0x46    //CTRL_REG3_G: HPF 0.1 Hz
    };

    const uint8_t Cfg_CTRL_REG6_XL[] =
    {
        0x63    //CTRL_REG6_XL: ODR 119 Hz, +-2g, badwidth default 50 Hz, anti-aliasing 50 Hz
    };

    const uint8_t Cfg_CTRL_REG1_M[] =
    {
        0x5C,   //CTRL_REG1_M: High-performance mode, ODR 80 Hz
        0x00,   //CTRL_REG2_M: +-4 gauss
        0x00,   //CTRL_REG3_M: continuous-conversion mode
        0x08    //CTRL_REG4_M: Z axis high performance mode, little endian
    };      

    /* check WHO_AM_I register of IMU A/G */
    status = HAL_I2C_Mem_Read(pIMU_I2C, IMU_AG_addr, WHO_AM_I_XG, I2C_MEMADD_SIZE_8BIT, rxBuf, 1, IMU_RX_TIMEOUT);
    if(status)
    {
        LOG_ERROR("failed to read IMU A/G register with status=%u", status);
        Error_Handler();
    }
    else
    {
        if(rxBuf[0] == WHO_AM_I_AG_RSP)
        {
            LOG_INFO("IMU AG response OK");
        }
        else
        {
            LOG_WARNING("IMU A/G invalid WHO_AM_I response=0x%02X", rxBuf[0]);
        }
    }

    /* check WHO_AM_I register of IMU M */
    status = HAL_I2C_Mem_Read(pIMU_I2C, IMU_M_addr, WHO_AM_I_M, I2C_MEMADD_SIZE_8BIT, rxBuf, 1, IMU_RX_TIMEOUT);
    if(status)
    {
        LOG_ERROR("failed to read IMU M register with status=%u", status);
        Error_Handler();
    }
    else
    {
        if(rxBuf[0] == WHO_AM_I_M_RSP)
        {
            LOG_INFO("IMU M response OK");
        }
        else
        {
            LOG_WARNING("IMU M invalid WHO_AM_I response=0x%02X", rxBuf[0]);
        }
    }    

    /* configure IMU A/G registers */
    status = HAL_OK;
    status |= HAL_I2C_Mem_Write(pIMU_I2C, IMU_AG_addr, INT1_CTRL, I2C_MEMADD_SIZE_8BIT, (uint8_t*)Cfg_INT1_CTRL, sizeof(Cfg_INT1_CTRL), IMU_TX_TIMEOUT);
    status |= HAL_I2C_Mem_Write(pIMU_I2C, IMU_AG_addr, CTRL_REG1_G, I2C_MEMADD_SIZE_8BIT, (uint8_t*)Cfg_CTRL_REG1_G, sizeof(Cfg_CTRL_REG1_G), IMU_TX_TIMEOUT);
    status |= HAL_I2C_Mem_Write(pIMU_I2C, IMU_AG_addr, CTRL_REG6_XL, I2C_MEMADD_SIZE_8BIT, (uint8_t*)Cfg_CTRL_REG6_XL, sizeof(Cfg_CTRL_REG6_XL), IMU_TX_TIMEOUT);

    /* configure IMU M registers */
    status |= HAL_I2C_Mem_Write(pIMU_I2C, IMU_M_addr, CTRL_REG1_M, I2C_MEMADD_SIZE_8BIT, (uint8_t*)Cfg_CTRL_REG1_M, sizeof(Cfg_CTRL_REG1_M), IMU_TX_TIMEOUT);

    if(status)
    {
        LOG_ERROR("failed to configure IMU registers with status=%u", status);
        Error_Handler();        
    }
    else
    {
        LOG_INFO("IMU setup OK");
    }
}
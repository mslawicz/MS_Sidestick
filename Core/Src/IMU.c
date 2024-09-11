#include "IMU.h"
#include "logger.h"
#include "main.h"
#include "stdint.h"

#define IMU_AG_BUF_SIZE 12
#define IMU_M_BUF_SIZE  6
#define IMU_RX_TIMEOUT  10
#define IMU_TX_TIMEOUT  20

I2C_HandleTypeDef* pIMU_I2C;
static uint8_t IMU_AG_rxBuf[IMU_AG_BUF_SIZE];   //rx buffer for IMU A/G raw data
static uint8_t IMU_M_rxBuf[IMU_M_BUF_SIZE];     //rx buffer for IMU M raw data
static volatile bool IMU_AG_transferActive = false;     //flag indicating IMU A/G transfer phase

void IMU_AG_readRequest(void);
void IMU_M_readRequest(void);

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
    status = HAL_I2C_Mem_Read(pIMU_I2C, IMU_AG_addr, WHO_AM_I_XG, I2C_MEMADD_SIZE_8BIT, IMU_AG_rxBuf, 1, IMU_RX_TIMEOUT);
    if(status)
    {
        LOG_ERROR("failed to read IMU A/G register with status=%u", status);
        Error_Handler();
    }
    else
    {
        if(IMU_AG_rxBuf[0] == WHO_AM_I_AG_RSP)
        {
            LOG_INFO("IMU AG response OK");
        }
        else
        {
            LOG_WARNING("IMU A/G invalid WHO_AM_I response=0x%02X", IMU_AG_rxBuf[0]);
        }
    }

    /* check WHO_AM_I register of IMU M */
    status = HAL_I2C_Mem_Read(pIMU_I2C, IMU_M_addr, WHO_AM_I_M, I2C_MEMADD_SIZE_8BIT, IMU_M_rxBuf, 1, IMU_RX_TIMEOUT);
    if(status)
    {
        LOG_ERROR("failed to read IMU M register with status=%u", status);
        Error_Handler();
    }
    else
    {
        if(IMU_M_rxBuf[0] == WHO_AM_I_M_RSP)
        {
            LOG_INFO("IMU M response OK");
        }
        else
        {
            LOG_WARNING("IMU M invalid WHO_AM_I response=0x%02X", IMU_M_rxBuf[0]);
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
        /* init the first IMU A/G readout */
        IMU_AG_readRequest();
    }
}

/* request of DMA readout of IMU G (6 bytes) and A (6 bytes)
    on transfer complete the HAL_I2C_MemRxCpltCallback is called */
void IMU_AG_readRequest(void)
{
    HAL_I2C_Mem_Read_DMA(pIMU_I2C, IMU_AG_addr, OUT_X_L_G, I2C_MEMADD_SIZE_8BIT, IMU_AG_rxBuf, IMU_AG_BUF_SIZE);
    IMU_AG_transferActive = true;
}

/* request of DMA readout of IMU M (6 bytes)
    on transfer complete the HAL_I2C_MemRxCpltCallback is called */
void IMU_M_readRequest(void)
{
    HAL_I2C_Mem_Read_DMA(pIMU_I2C, IMU_M_addr, OUT_X_L_M, I2C_MEMADD_SIZE_8BIT, IMU_M_rxBuf, IMU_M_BUF_SIZE);
}

/**
  * @brief  Memory Rx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if(pIMU_I2C == hi2c)
  {
    HAL_GPIO_WritePin(TEST1_GPIO_Port, TEST1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TEST1_GPIO_Port, TEST1_Pin, GPIO_PIN_RESET);

    if(IMU_AG_transferActive)
    {
        /* callback on IMU A/G transfer complete - continue with IMU M transfer */
        IMU_M_readRequest();
        IMU_AG_transferActive = false;
    }
    else
    {
        /* set IMU new data ready */
    }
  }
}
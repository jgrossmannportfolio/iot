/**
******************************************************************************
* @file    LPS25H_Driver.c
* @author  HESA Application Team
* @version 1.0.0
* @date    07/04/2014
* @brief   LPS25H driver file

* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOURCE CODE IS PROTECTED BY A LICENSE.
* FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
* IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
*
* <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>

******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "lps25h_driver.h"
#include "cfa.h"

#ifdef  USE_FULL_ASSERT_LPS25H
#include <stdio.h>
#endif

// Read operation to the lower level driver is 0.
#define I2C_SLAVE_OPERATION_READ                    (0)

// Write operation to the lower level driver is 1.
#define I2C_SLAVE_OPERATION_WRITE                   (1)

/** @addtogroup Environmental_Sensor
* @{
*/

/** @defgroup LPS25H_DRIVER
* @brief LPS25H DRIVER
* @{
*/


/** @defgroup LPS25H_Public_Functions
* @{
*/

/**
* @brief  Read LPS25H Registers
* @param  uint32_t RegAddr:      address of the first register to read
* @param  uint8_t *Data:         pointer to the destination data buffer
* @param  uint8_t NumByteToRea:  number of bytes to read
* @retval LPS25H_ERROR or LPS25H_OK 
*/
LPS25H_Error_et LPS25H_ReadReg(uint8_t RegAddr, uint8_t NumByteToRead, uint8_t* Data)
{
	CFA_BSC_STATUS read_status;

	if(NumByteToRead > 1)
		RegAddr |= 0x80;

	read_status = cfa_bsc_OpExtended(Data, NumByteToRead, &RegAddr, sizeof(uint8_t), LPS25H_ADDRESS, I2C_SLAVE_OPERATION_READ);

    switch(read_status)
    {
        case CFA_BSC_STATUS_INCOMPLETE:
            // Transaction did not go through. ERROR. Handle this case.
        	return LPS25H_ERROR;
            break;
        case CFA_BSC_STATUS_SUCCESS:
            // The read was successful.
        	return LPS25H_OK;
            break;
        case CFA_BSC_STATUS_NO_ACK:
            // No slave device with this address exists on the I2C bus. ERROR. Handle this.
        default:
            break;
    }

    return LPS25H_ERROR;
}

/**
* @brief  Write DEV Registers
* @param  uint32_t RegAddr:      address of the register to write
* @param  uint8_t *Data:         pointer to the source data buffer
* @retval LPS25H_ERROR or LPS25H_OK 
*/
LPS25H_Error_et LPS25H_WriteReg(uint8_t RegAddr, uint8_t NumByteToWrite, uint8_t* Data)
{  
	CFA_BSC_STATUS read_status;
	UINT8 i;
    UINT8 reg_data_bytes[16];

    if(NumByteToWrite >= sizeof(reg_data_bytes))
    	return LPS25H_ERROR;

    reg_data_bytes[0]= RegAddr;

    for(i = 0; i < NumByteToWrite; i++)
    {
    	reg_data_bytes[1 + i] = *Data++;
    }

    read_status = cfa_bsc_OpExtended(reg_data_bytes, NumByteToWrite + 1, NULL, 0, LPS25H_ADDRESS, I2C_SLAVE_OPERATION_WRITE);

    switch(read_status)
    {
        case CFA_BSC_STATUS_INCOMPLETE:
            // Transaction did not go through. ERROR. Handle this case.
        	return LPS25H_ERROR;
            break;
        case CFA_BSC_STATUS_SUCCESS:
            // The read was successful.
        	return LPS25H_OK;
            break;
        case CFA_BSC_STATUS_NO_ACK:
            // No slave device with this address exists on the I2C bus. ERROR. Handle this.
        default:
            break;
    }

    return LPS25H_ERROR;
}

/**
* @brief  Read identification code by WHO_AM_I register
* @param  Buffer to empty by Device identification Value.
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_DeviceID(uint8_t* deviceid)
{
  if(LPS25H_ReadReg(LPS25H_WHO_AM_I_REG, 1, deviceid))
    return LPS25H_ERROR;
  
  return LPS25H_OK;
}


/**
* @brief  Get the LPS25H driver version.
* @param  None
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_DriverVersion(LPS25H_DriverVersion_st *Version)
{  
  Version->Major = LPS25H_DriverVersion_Major;
  Version->Minor = LPS25H_DriverVersion_Minor;
  Version->Point = LPS25H_DriverVersion_Point;
  
  return LPS25H_OK;
}

/**
* @brief  Set LPS25H Pressure and Temperature Resolution Mode
* @param  Pressure Resolution
* @param  Temperature Resolution
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_Avg(LPS25H_Avgp_et avgp,LPS25H_Avgt_et avgt )
{  
  uint8_t tmp;
  
  LPS25H_assert_param(IS_LPS25H_AVGT(avgt));
  LPS25H_assert_param(IS_LPS25H_AVGP(avgp));
  
  if(LPS25H_ReadReg(LPS25H_RES_CONF_REG, 1, &tmp))
    return LPS25H_ERROR;
  
  tmp &= ~(LPS25H_AVGT_MASK | LPS25H_AVGP_MASK);
  tmp |= (uint8_t)avgp;
  tmp |= (uint8_t)avgt;
  
  if(LPS25H_WriteReg(LPS25H_RES_CONF_REG, 1, &tmp) )
    return LPS25H_ERROR;
  
  return LPS25H_OK;
}

/**
* @brief  Exit the shutdown mode for LPS25H.
* @param  None
* @retval None
*/

LPS25H_Error_et LPS25H_Activate(void)
{  
  if(LPS25H_Set_PowerDownMode(LPS25H_SET))
    return LPS25H_ERROR;
  
  return LPS25H_OK;
}

/**
* @brief  Enter the shutdown mode for LPS25H.
* @param  None
* @retval None
*/
LPS25H_Error_et LPS25H_DeActivate(void)
{  
  if(LPS25H_Set_PowerDownMode(LPS25H_RESET))
    return LPS25H_ERROR;
  
  return LPS25H_OK;
}

/**
* @brief  Set LPS25H Pressure Resolution Mode
* @param  Pressure Resolution
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_AvgP(LPS25H_Avgp_et avgp)
{  
  uint8_t tmp;  
  
  LPS25H_assert_param(IS_LPS25H_AVGP(avgp));
  
  if(LPS25H_ReadReg(LPS25H_RES_CONF_REG, 1, &tmp))
    return LPS25H_ERROR;
  
  tmp &= ~LPS25H_AVGP_MASK;
  tmp |= (uint8_t)avgp;
  
  if(LPS25H_WriteReg(LPS25H_RES_CONF_REG, 1, &tmp))
    return LPS25H_ERROR;
  
  return LPS25H_OK;
}

/**
* @brief  Set LPS25H Temperature Resolution Mode
* @param  Temperature Resolution
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_AvgT(LPS25H_Avgt_et avgt)
{  
  uint8_t tmp;
  
  LPS25H_assert_param(IS_LPS25H_AVGT(avgt));
  
  if(LPS25H_ReadReg(LPS25H_RES_CONF_REG, 1, &tmp))
    return LPS25H_ERROR;
  
  tmp &= ~LPS25H_AVGT_MASK;
  tmp |= (uint8_t)avgt;
  
  if(LPS25H_WriteReg(LPS25H_RES_CONF_REG, 1, &tmp))
    return LPS25H_ERROR;
  
  return LPS25H_OK;
}

/**
* @brief  Get LPS25H Pressure Resolution Mode
* @param  Pressure Resolution
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_AvgP(LPS25H_Avgp_et* avgp)
{
  uint8_t tmp;
  
  if(LPS25H_ReadReg(LPS25H_RES_CONF_REG, 1, &tmp))
    return LPS25H_ERROR;
  
  *avgp = (LPS25H_Avgp_et)(tmp & LPS25H_AVGP_MASK);
  
  return LPS25H_OK;
}


/**
* @brief  Get LPS25H Temperature Resolution Mode
* @param  Temperature Resolution
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_AvgT(LPS25H_Avgt_et* avgt)
{  
  uint8_t tmp;
  
  if(LPS25H_ReadReg(LPS25H_RES_CONF_REG, 1, &tmp) )
    return LPS25H_ERROR;
  
  *avgt = (LPS25H_Avgt_et)(tmp & LPS25H_AVGT_MASK);
  
  return LPS25H_OK;
}


/**
* @brief  Set LPS25H Output Data Rate
* @param  Output Data Rate
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_Odr(LPS25H_Odr_et odr)
{  
  uint8_t tmp;
  
  LPS25H_assert_param(IS_LPS25H_ODR(odr));
  
  if(LPS25H_ReadReg(LPS25H_CTRL_REG1, 1, &tmp))
    return LPS25H_ERROR;
  
  tmp &= ~LPS25H_ODR_MASK;
  tmp |= (uint8_t)odr;
  
  if(LPS25H_WriteReg(LPS25H_CTRL_REG1, 1, &tmp))
    return LPS25H_ERROR;
  
  return LPS25H_OK;
}


/**
* @brief  Get LPS25H Output Data Rate
* @param  Buffer to empty with Output Data Rate
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_Odr(LPS25H_Odr_et* odr)
{
  uint8_t tmp;
  
  if(LPS25H_ReadReg(LPS25H_CTRL_REG1, 1, &tmp))
    return LPS25H_ERROR;
  
  *odr = (LPS25H_Odr_et)(tmp & LPS25H_ODR_MASK);
  
  return LPS25H_OK;
}

/**
* @brief  SET/RESET LPS25H Power Down Mode bit
* @param  LPS25H_SET (Active Mode)/LPS25H_RESET (Power Down Mode)
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_PowerDownMode(LPS25H_BitStatus_et pd)
{
  uint8_t tmp;
  
  LPS25H_assert_param(IS_LPS25H_BitStatus(pd));
  
  if(LPS25H_ReadReg(LPS25H_CTRL_REG1, 1, &tmp))
    return LPS25H_ERROR;
  
  tmp &= ~LPS25H_PD_MASK;
  tmp |= ((uint8_t)pd)<<LPS25H_PD_BIT;
  
  if(LPS25H_WriteReg(LPS25H_CTRL_REG1, 1, &tmp))
    return LPS25H_ERROR;
  
  return LPS25H_OK;
}

/**
* @brief  Set Block Data Mode
* @detail It is recommended to set BDU bit to ‘1’.
* @detail This feature avoids reading LSB and MSB related to different samples.
* @param 	LPS25H_BDU_CONTINUOS_UPDATE, LPS25H_BDU_NO_UPDATE
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/

LPS25H_Error_et LPS25H_Set_Bdu(LPS25H_Bdu_et bdu)
{  
  uint8_t tmp;
  
  LPS25H_assert_param(IS_LPS25H_BDUMode(bdu));
  
  if(LPS25H_ReadReg(LPS25H_CTRL_REG1, 1, &tmp))
    return LPS25H_ERROR;
  
  tmp &= ~LPS25H_BDU_MASK;
  tmp |= ((uint8_t)bdu);
  
  if(LPS25H_WriteReg(LPS25H_CTRL_REG1, 1, &tmp))
    return LPS25H_ERROR;
  
  return LPS25H_OK;
}

/**
* @brief  Get Block Data Mode
* @param 	Buffer to empty whit thw bdu mode read from sensor
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_Bdu(LPS25H_Bdu_et* bdu)
{  
  uint8_t tmp;
  
  if(LPS25H_ReadReg(LPS25H_CTRL_REG1, 1, &tmp))
    return LPS25H_ERROR;
  
  *bdu = (LPS25H_Bdu_et)(tmp & LPS25H_BDU_MASK);
  
  return LPS25H_OK;
}

/**
* @brief  Enable/Disable Interrupt Circuit.
* @param 	LPS25H_ENABLE/LPS25H_DISABLE
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_InterruptCircuitEnable(LPS25H_State_et diff_en) 
{  
  uint8_t tmp;
  
  LPS25H_assert_param(IS_LPS25H_State(diff_en));
  
  if(LPS25H_ReadReg(LPS25H_CTRL_REG1, 1, &tmp))
    return LPS25H_ERROR;
  
  tmp &= ~LPS25H_DIFF_EN_MASK;
  tmp |= ((uint8_t)diff_en)<<LPS25H_DIFF_EN_BIT;
  
  if(LPS25H_WriteReg(LPS25H_CTRL_REG1, 1, &tmp))
    return LPS25H_ERROR;
  
  return LPS25H_OK;
}

/**
* @brief  Get the Interrupt Circuit bit.
* @param 	Buffer to empty whit thw diff_en mode read from sensor
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_InterruptCircuitEnable(LPS25H_State_et* diff_en)
{  
  uint8_t tmp;
  
  if(LPS25H_ReadReg(LPS25H_CTRL_REG1, 1, &tmp))
    return LPS25H_ERROR;
  
  *diff_en= (LPS25H_State_et)((tmp & LPS25H_DIFF_EN_MASK)>>LPS25H_DIFF_EN_BIT);
  
  return LPS25H_OK;
}


/**
* @brief  Set ResetAutoZero Function bit
* @details REF_P reg (@0x08..0A) set pressure reference to default tmp RPDS reg (0x39/3A).
* @param 	None
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_ResetAZ(void)
{  
  uint8_t tmp;
  
  if(LPS25H_ReadReg(LPS25H_CTRL_REG1, 1, &tmp))
    return LPS25H_ERROR;
  
  /* Set the RESET_AZ bit*/
  /* RESET_AZ is self cleared*/
  tmp |= LPS25H_RESET_AZ_MASK;
  
  if(LPS25H_WriteReg(LPS25H_CTRL_REG1, 1, &tmp))
    return LPS25H_ERROR;
  
  return LPS25H_OK;
}

/**
* @brief  Set SPI mode: 3 Wire Interface or 4 Wire Interface
* @param 	LPS25H_SPI_3_WIRE, LPS25H_SPI_4_WIRE
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_SpiInterface(LPS25H_SPIMode_et spimode)
{  
  uint8_t tmp;
  
  LPS25H_assert_param(IS_LPS25H_SPIMode(spimode));
  
  if(LPS25H_ReadReg(LPS25H_CTRL_REG1, 1, &tmp))
    return LPS25H_ERROR;
  
  tmp &= ~LPS25H_SIM_MASK;
  tmp |= (uint8_t)spimode;
  
  if(LPS25H_WriteReg(LPS25H_CTRL_REG1, 1, &tmp))
    return LPS25H_ERROR;
  
  return LPS25H_OK;
}


/**
* @brief  Get SPI mode: 3 Wire Interface or 4 Wire Interface
* @param 	Buffet to empty with spi mode read from Sensor
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_SpiInterface(LPS25H_SPIMode_et* spimode)
{  
  uint8_t tmp;
  
  if(LPS25H_ReadReg(LPS25H_CTRL_REG1, 1, &tmp))
    return LPS25H_ERROR;
  
  *spimode = (LPS25H_SPIMode_et)(tmp & LPS25H_SIM_MASK);
  
  return LPS25H_OK;
}

/**
* @brief  Enable/Disable I2C Mode 
* @param 	State: Enable (reset bit)/ Disable (set bit)
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_I2C(LPS25H_State_et statei2c)
{  
  uint8_t tmp;
  
  LPS25H_assert_param(IS_LPS25H_State(statei2c));
  
  if(LPS25H_ReadReg(LPS25H_CTRL_REG2, 1, &tmp))
    return LPS25H_ERROR;
  
  tmp &= ~LPS25H_I2C_MASK;
  if(statei2c == LPS25H_DISABLE) {
    tmp |= LPS25H_I2C_MASK;
  }
  
  if(LPS25H_WriteReg(LPS25H_CTRL_REG2, 1, &tmp))
    return LPS25H_ERROR;
  
  return LPS25H_OK;
  
}

/**
* @brief  Set the one-shot bit in order to start acquisition when the ONE SHOT mode
has been selected by the ODR configuration.
* @detail 	Once the measurement is done, ONE_SHOT bit will self-clear.
* @param 	None
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_StartOneShotMeasurement(void)
{
  uint8_t tmp;
  
  if(LPS25H_ReadReg(LPS25H_CTRL_REG2, 1, &tmp))
    return LPS25H_ERROR;
  
  /* Set the one shot bit */
  /* Once the measurement is done, one shot bit will self-clear*/
  tmp |= LPS25H_ONE_SHOT_MASK;
  
  if(LPS25H_WriteReg(LPS25H_CTRL_REG2, 1, &tmp))
    return LPS25H_ERROR;
  
  return LPS25H_OK;
  
}

/**
* @brief  Set AutoZero Function bit
* @detail When set to ‘1’, the actual pressure output is copied in the REF_P reg (@0x08..0A) 
* @param 	LPS25H_SET/LPS25H_RESET
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_AutoZeroFunction(LPS25H_BitStatus_et autozero) 
{  
  uint8_t tmp;
  
  LPS25H_assert_param(IS_LPS25H_BitStatus(autozero));
  
  if(LPS25H_ReadReg(LPS25H_CTRL_REG2, 1, &tmp))
    return LPS25H_ERROR;
  
  tmp &=  ~LPS25H_AUTO_ZERO_MASK;
  tmp |= ((uint8_t)autozero)<<LPS25H_AUTO_ZERO_BIT;
  
  if(LPS25H_WriteReg(LPS25H_CTRL_REG2, 1, &tmp))
    return LPS25H_ERROR;
  
  return LPS25H_OK;
}

/**
* @brief   Software Reset. Self-clearing upon completion
* @param 	 None
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_SwReset(void) 
{  
  uint8_t tmp;
  
  if(LPS25H_ReadReg(LPS25H_CTRL_REG2, 1, &tmp))
    return LPS25H_ERROR;
  
  tmp |= (0x01<<LPS25H_SW_RESET_BIT);
  
  if(LPS25H_WriteReg(LPS25H_CTRL_REG2, 1, &tmp))
    return LPS25H_ERROR;
  
  return LPS25H_OK;
}

/**
* @brief  Reboot Memory Content
* @param 	 None
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/

LPS25H_Error_et LPS25H_MemoryBoot(void) 
{  
  uint8_t tmp;
  
  if(LPS25H_ReadReg(LPS25H_CTRL_REG2, 1, &tmp))
    return LPS25H_ERROR;
  
  tmp |= (0x01<<LPS25H_BOOT_BIT);
  
  if(LPS25H_WriteReg(LPS25H_CTRL_REG2, 1, &tmp))
    return LPS25H_ERROR;
  
  return LPS25H_OK;
}

/**
* @brief   Software Reset ann Reboot Memory Content. 
* @detail  The device is reset to the power on configuration if the SWRESET bit is set to ‘1’ 
and BOOT is set to ‘1’; Self-clearing upon completion.
* @param 	 None
* @retval  Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_SwResetAndMemoryBoot(void) 
{
  uint8_t tmp;
  
  if(LPS25H_ReadReg(LPS25H_CTRL_REG2, 1, &tmp))
    return LPS25H_ERROR;
  
  tmp |= ((0x01<<LPS25H_SW_RESET_BIT) | (0x01<<LPS25H_BOOT_BIT));
  
  if(LPS25H_WriteReg(LPS25H_CTRL_REG2, 1, &tmp))
    return LPS25H_ERROR;
  
  return LPS25H_OK;
}

/**
* @brief   Enable/Disable FIFO Mode 
* @param 	 LPS25H_ENABLE/LPS25H_DISABLE
* @retval  Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_FifoModeUse(LPS25H_State_et status) 
{  
  uint8_t tmp;
  
  LPS25H_assert_param(IS_LPS25H_State(status));
  
  if(LPS25H_ReadReg(LPS25H_CTRL_REG2, 1, &tmp))
    return LPS25H_ERROR;
  
  tmp &= ~LPS25H_FIFO_EN_MASK;
  tmp |= ((uint8_t)status)<<LPS25H_FIFO_EN_BIT;
  
  if(LPS25H_WriteReg(LPS25H_CTRL_REG2, 1, &tmp))
    return LPS25H_ERROR;
  
  return LPS25H_OK;
}
/**
* @brief    Enable/Disable FIFO Watermark Level Use 
* @param 	 LPS25H_ENABLE/LPS25H_DISABLE
* @retval  Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_FifoWatermarkLevelUse(LPS25H_State_et status) 
{  
  uint8_t tmp;
  
  LPS25H_assert_param(IS_LPS25H_State(status));
  
  if(LPS25H_ReadReg(LPS25H_CTRL_REG2, 1, &tmp))
    return LPS25H_ERROR;
  
  tmp &= ~LPS25H_WTM_EN_MASK;
  tmp |= ((uint8_t)status)<<LPS25H_WTM_EN_BIT;
  
  if(LPS25H_WriteReg(LPS25H_CTRL_REG2, 1, &tmp))
    return LPS25H_ERROR;
  
  return LPS25H_OK;
}

/**
* @brief   Enable/Disable 1 HZ ODR Decimation
* @param 	 LPS25H_ENABLE/LPS25H_DISABLE
* @retval  Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_FifoMeanDecUse(LPS25H_State_et status) 
{  
  uint8_t tmp;
  
  LPS25H_assert_param(IS_LPS25H_State(status));
  
  if(LPS25H_ReadReg(LPS25H_CTRL_REG2, 1, &tmp))
    return LPS25H_ERROR;
  
  tmp &= ~LPS25H_FIFO_MEAN_MASK;
  tmp |= ((uint8_t)status)<<LPS25H_FIFO_MEAN_BIT;
  
  if(LPS25H_WriteReg(LPS25H_CTRL_REG2, 1, &tmp))
    return LPS25H_ERROR;
  
  return LPS25H_OK;	
  
}

/**
* @brief   Enable/Disable Interrupt Active High (default tmp 0) or Low(tmp 1)
* @param 	 LPS25H_ENABLE/LPS25H_DISABLE
* @retval  Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_InterruptActiveLevel(LPS25H_State_et status)
{  
  uint8_t tmp;
  
  LPS25H_assert_param(IS_LPS25H_State(status));
  
  if(LPS25H_ReadReg(LPS25H_CTRL_REG3, 1, &tmp))
    return LPS25H_ERROR;
  
  tmp &= ~LPS25H_INT_H_L_MASK;
  tmp |= ((uint8_t)status)<<LPS25H_INT_H_L_BIT;
  
  if(LPS25H_WriteReg(LPS25H_CTRL_REG3, 1, &tmp))
    return LPS25H_ERROR;
  
  return LPS25H_OK;	
}

/**
* @brief   Push-pull/open drain selection on interrupt pads. Default tmp: 0
* @param 	 LPS25H_PushPull/LPS25H_OpenDrain
* @retval  Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_InterruptOutputType(LPS25H_OutputType_et output)
{
  uint8_t tmp;
  
  LPS25H_assert_param(IS_LPS25H_OutputType(output));
  
  if(LPS25H_ReadReg(LPS25H_CTRL_REG3, 1, &tmp))
    return LPS25H_ERROR;
  
  tmp &= ~LPS25H_PP_OD_MASK;
  tmp |= (uint8_t)output;
  
  if(LPS25H_WriteReg(LPS25H_CTRL_REG3, 1, &tmp))
    return LPS25H_ERROR;
  
  return LPS25H_OK;	
}

/**
* @brief   Set Data signal on INT1 pad control bits. Default tmp: 00
* @param 	 PS25H_DATA,LPS25H_P_HIGH_LPS25H_P_LOW,LPS25H_P_LOW_HIGH
* @retval  Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_InterruptControlConfig(LPS25H_OutputSignalConfig_et config)
{  
  uint8_t tmp;
  
  LPS25H_assert_param(IS_LPS25H_OutputSignal(config));
  
  if(LPS25H_ReadReg(LPS25H_CTRL_REG3, 1, &tmp))
    return LPS25H_ERROR;
  
  tmp &= ~(LPS25H_INT1_S2_MASK | LPS25H_INT1_S1_MASK);
  tmp |= (uint8_t)config;
  
  if(LPS25H_WriteReg(LPS25H_CTRL_REG3, 1, &tmp))
    return LPS25H_ERROR;
  
  return LPS25H_OK;
}

/**
* @brief   Set INT1 interrupt pins configuration
* @param 	 LPS25H_EMPTY,LPS25H_WTM,LPS25H_OVR,LPS25H_DATA_READY
* @retval  Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_InterruptDataConfig(LPS25H_DataSignalType_et signal)
{  
  uint8_t tmp;
  
  LPS25H_assert_param(IS_LPS25H_DataSignal(signal));
  
  if(LPS25H_ReadReg(LPS25H_CTRL_REG4, 1, &tmp))
    return LPS25H_ERROR;
  
  tmp &= ~(LPS25H_P1_EMPTY_MASK | LPS25H_P1_WTM_MASK \
    | LPS25H_P1_OVERRUN_MASK  | LPS25H_P1_DRDY_MASK );
  tmp |= (uint8_t)signal;
  
  if(LPS25H_WriteReg(LPS25H_CTRL_REG4, 1, &tmp))
    return LPS25H_ERROR;
  
  return LPS25H_OK;
}

/**
* @brief   Enable\Disable Interrupt Generation on differential pressure low and/or high event
* @param 	 LPS25H_DISABLE_INT, LPS25H_PHE,LPS25H_PLE,LPS25H_PLE_PHE
* @retval  Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_InterruptDifferentialConfig(LPS25H_InterruptDiffConfig_et config)
{  
  uint8_t tmp;
  
  LPS25H_assert_param(IS_LPS25H_InterruptDiff(config));
  
  if(LPS25H_ReadReg(LPS25H_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS25H_ERROR;
  
  tmp &= ~(LPS25H_PL_E_MASK | LPS25H_PH_E_MASK);
  tmp |= (uint8_t)config;
  if(config!=LPS25H_DISABLE_INT){
    /* Enable DIFF_EN bit in CTRL_REG1 */
    if(LPS25H_Set_InterruptCircuitEnable(LPS25H_ENABLE))
      return LPS25H_ERROR;
  }
  
  if(LPS25H_WriteReg(LPS25H_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS25H_ERROR;
  
  return LPS25H_OK;
}

/**
* @brief   Enable/Disable Latch Interrupt request into INT_SOURCE register.
* @param 	 LPS25H_ENABLE/LPS25H_DISABLE
* @retval  Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_LatchInterruptRequest(LPS25H_State_et status)
{  
  uint8_t tmp;
  
  LPS25H_assert_param(IS_LPS25H_State(status));
  
  if(LPS25H_ReadReg(LPS25H_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS25H_ERROR;
  
  tmp &= ~LPS25H_LIR_MASK;
  tmp |= (((uint8_t)status)<<LPS25H_LIR_BIT);
  
  if(LPS25H_WriteReg(LPS25H_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS25H_ERROR;
  
  return LPS25H_OK;
}

/**
* @brief   Get the Interrupt Generation on differential pressure status event.
* @detail  The INT_SOURCE register is cleared by reading it.
* @param 	 Status Event Flag: PH,PL,IA
* @retval  Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_InterruptDifferentialEventStatus(LPS25H_InterruptDiffStatus_st* interruptsource)
{  
  uint8_t tmp;
  
  if(LPS25H_ReadReg(LPS25H_INTERRUPT_SOURCE_REG, 1, &tmp))
    return LPS25H_ERROR;
  
  interruptsource->PH = (uint8_t)(tmp & LPS25H_PH_MASK); 
  interruptsource->PL = (uint8_t)((tmp & LPS25H_PL_MASK)>>LPS25H_PL_BIT);
  interruptsource->IA = (uint8_t)((tmp & LPS25H_IA_MASK)>>LPS25H_IA_BIT);
  
  return LPS25H_OK;
}

/**
* @brief   Get the status of Pressure and Temperature data 
* @param 	 Data Status Flag:  TempDataAvailable, TempDataOverrun, PressDataAvailable, PressDataOverrun
* @retval  Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_DataStatus(LPS25H_DataStatus_st* datastatus)
{
  uint8_t tmp;
  
  if(LPS25H_ReadReg(LPS25H_STATUS_REG, 1, &tmp))
    return LPS25H_ERROR;
  
  datastatus->TempDataAvailable = (tmp & LPS25H_TDA_MASK);
  datastatus->PressDataAvailable = (uint8_t)((tmp & LPS25H_PDA_MASK)>>LPS25H_PDA_BIT);  
  datastatus->TempDataOverrun = (uint8_t)((tmp & LPS25H_TOR_MASK)>>LPS25H_TOR_BIT);   
  datastatus->PressDataOverrun = (uint8_t)((tmp & LPS25H_POR_MASK)>>LPS25H_POR_BIT);  
  
  return LPS25H_OK;
}


/**
* @brief    Get the raw pressure tmp
* @detail   The data are expressed as PRESS_OUT_H/_L/_XL in 2’s complement.
Pout(hPA)=PRESS_OUT / 4096
* @param 	 The buffer to empty with the pressure raw tmp
* @retval  Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_RawPressure(int32_t *raw_press)
{
  uint8_t buffer[3];
  uint32_t tmp = 0;  
  uint8_t i;
  
  if(LPS25H_ReadReg(LPS25H_PRESS_OUT_XL_REG, 3, buffer))
    return LPS25H_ERROR;
  
  /* Build the raw data */
  for(i=0; i<3; i++)
    tmp |= (((uint32_t)buffer[i]) << (8*i));
  
  /* convert the 2's complement 24 bit to 2's complement 32 bit */
  if(tmp & 0x00800000)
    tmp |= 0xFF000000;
  
  *raw_press = ((int32_t)tmp);
  
  return LPS25H_OK;   
}

/**
* @brief    Get the Pressure value in hPA.
* @detail   The data are expressed as PRESS_OUT_H/_L/_XL in 2’s complement.
Pout(hPA)=PRESS_OUT / 4096
* @param 	  The buffer to empty with the pressure value that must be divided by 100 to get the value in hPA
* @retval   Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_Pressure(int32_t* Pout)
{  
  int32_t raw_press;
  
  if(LPS25H_Get_RawPressure(&raw_press))
    return LPS25H_ERROR;
  
  *Pout = (raw_press*100)/4096;
  
  return LPS25H_OK;
}

/**
* @brief    Get the Raw Temperature tmp.
* @detail   Temperature data are expressed as TEMP_OUT_H&TEMP_OUT_L as 2’s complement number.
Tout(degC)=42.5+ (TEMP_OUT/480)
* @param 	  Buffer to empty with the temperature raw tmp.
* @retval   Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_RawTemperature(int16_t* raw_data)
{
  uint8_t buffer[2];
  uint16_t tmp;
  
  if(LPS25H_ReadReg(0x80 | LPS25H_TEMP_OUT_L_REG, 2, buffer))
    return LPS25H_ERROR;
  
  /* Build the raw tmp */
  tmp = (((uint16_t)buffer[1]) << 8) + (uint16_t)buffer[0];
  
  *raw_data = ((int16_t)tmp);
  
  return LPS25H_OK;    
}


/**
* @brief    Get the Temperature value in °C.
* @detail   Temperature data are expressed as TEMP_OUT_H&TEMP_OUT_L as 2’s complement number.
* Tout(degC)=42.5+ (TEMP_OUT/480)
* @param 	  Buffer to empty with the temperature value that must be divided by 10 to get the value in °C
* @retval   Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_Temperature(int16_t* Tout)
{  
  int16_t raw_data;
  
  if(LPS25H_Get_RawTemperature(&raw_data))
    return LPS25H_ERROR;
  
  *Tout = raw_data/48 + 425;
  
  return LPS25H_OK;
}

/**
* @brief    Get the threshold tmp used for pressure interrupt generation (hPA).
* @detail   THS_P=THS_P_H&THS_P_L and is expressed as unsigned number.
P_ths(hPA)=(THS_P)/16.							
* @param 	  Buffer to empty with the pressure threshold in hPA
* @retval   Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_PressureThreshold(int16_t* P_ths)
{
  uint8_t tempReg[2];
  
  if(LPS25H_ReadReg(LPS25H_THS_P_LOW_REG, 2, tempReg))
    return LPS25H_ERROR;
  
  *P_ths= (((((uint16_t)tempReg[1])<<8) + tempReg[0])/16);
  
  return LPS25H_OK;	
}

/**
* @brief    Set the threshold tmp used for pressure interrupt generation (hPA).
* @detail   THS_P=THS_P_H&THS_P_L and is expressed as unsigned number.
P_ths(hPA)=(THS_P)/16.							
* @param 	  Pressure threshold in hPA
* @retval   Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_PressureThreshold(int16_t P_ths)
{  
  uint8_t buffer[2];
  
  buffer[0] = (uint8_t)(16 * P_ths);
  buffer[1] = (uint8_t)(((uint16_t)(16 * P_ths))>>8);   
  
  if(LPS25H_WriteReg(LPS25H_THS_P_LOW_REG, 2, buffer))
    return LPS25H_ERROR;
  
  return LPS25H_OK;	
}


/**
* @brief    Set Fifo Mode					
* @param 	  LPS25H_FIFO_BYPASS_MODE, LPS25H_FIFO_MODE, LPS25H_FIFO_STREAM_MODE, LPS25H_FIFO_TRIGGER_STREAMTOFIFO_MODE, 
LPS25H_FIFO_TRIGGER_BYPASSTOSTREAM_MODE,LPS25H_FIFO_MEAN_MODE, LPS25H_FIFO_TRIGGER_BYPASSTOFIFO_MODE
* @retval   Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_FifoMode(LPS25H_FifoMode_et fifomode) 
{  
  uint8_t tmp;  
  
  LPS25H_assert_param(IS_LPS25H_FifoMode(fifomode));
  
  if(LPS25H_ReadReg(LPS25H_CTRL_FIFO_REG, 1, &tmp))
    return LPS25H_ERROR;
  
  tmp &= ~LPS25H_FMODE_MASK;
  tmp |= (uint8_t)fifomode;
  
  if(LPS25H_WriteReg(LPS25H_CTRL_FIFO_REG, 1, &tmp))           
    return LPS25H_ERROR;  
  
  return LPS25H_OK;
}

/**
* @brief    Get Fifo Mode					
* @param 	  buffer to empty with fifo mode tmp
* @retval   Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_FifoMode(LPS25H_FifoMode_et* fifomode) 
{  
  uint8_t tmp;  
  
  if(LPS25H_ReadReg(LPS25H_CTRL_FIFO_REG, 1, &tmp))
    return LPS25H_ERROR;
  
  tmp &= ~LPS25H_WTM_POINT_MASK;
  *fifomode = (LPS25H_FifoMode_et)tmp;
  
  return LPS25H_OK;
}

/**
* @brief    Get the Fifo Status			
* @param 	  Status Flag: FIFO_WTM,FIFO_EMPTY,FIFO_FULL,FIFO_LEVEL
* @retval   Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_FifoStatus(LPS25H_FifoStatus_st* status)
{  
  uint8_t tmp;
  
  if(LPS25H_ReadReg(LPS25H_STATUS_FIFO_REG, 1, &tmp))
    return LPS25H_ERROR;
  
  status->FIFO_WTM = (uint8_t)((tmp & LPS25H_WTM_FIFO_MASK)>>LPS25H_WTM_FIFO_BIT);
  status->FIFO_FULL = (uint8_t)((tmp & LPS25H_FULL_FIFO_MASK)>>LPS25H_FULL_FIFO_BIT);
  status->FIFO_EMPTY = (uint8_t)((tmp & LPS25H_EMPTY_FIFO_MASK)>>LPS25H_EMPTY_FIFO_BIT);
  status->FIFO_LEVEL = (uint8_t)(tmp & LPS25H_DIFF_POINT_MASK);
  
  return LPS25H_OK;
}

/**
* @brief    Set Watermark Value
* @param 	  wtmlevel = [0,31]
* @retval   Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_FifoWatermarkLevel(uint8_t wtmlevel)
{  
  uint8_t tmp; 
  
  LPS25H_assert_param(IS_LPS25H_WtmLevel(wtmlevel));
  
  if(LPS25H_ReadReg(LPS25H_CTRL_FIFO_REG, 1, &tmp))
    return LPS25H_ERROR;
  
  tmp &= ~LPS25H_WTM_POINT_MASK;
  tmp |= wtmlevel;
  
  if(LPS25H_WriteReg(LPS25H_CTRL_FIFO_REG, 1, &tmp))           
    return LPS25H_ERROR;
  
  return LPS25H_OK;
}

/**
* @brief    Get Watermark Value
* @param 	  wtmlevel tmp read from sensor
* @retval   Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_FifoWatermarkLevel(uint8_t *wtmlevel)
{  
  if(LPS25H_ReadReg(LPS25H_CTRL_FIFO_REG, 1, wtmlevel))
    return LPS25H_ERROR;
  
  *wtmlevel &= LPS25H_WTM_POINT_MASK;
  
  return LPS25H_OK;
}

/**
* @brief   Set the number of sample to perform moving average when FIFO_MEAN_MODE is used
* @param 	 LPS25H_FIFO_SAMPLE_2,LPS25H_FIFO_SAMPLE_4,LPS25H_FIFO_SAMPLE_8,LPS25H_FIFO_SAMPLE_16,LPS25H_FIFO_SAMPLE_32
* @retval  Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_FifoSampleSize(LPS25H_FifoMeanModeSample_et samplesize)
{  
  uint8_t tmp; 
  
  LPS25H_assert_param(IS_LPS25H_FifoMeanModeSample(samplesize));
  
  if(LPS25H_ReadReg(LPS25H_CTRL_FIFO_REG, 1, &tmp))
    return LPS25H_ERROR;
  
  tmp &= ~LPS25H_WTM_POINT_MASK;
  tmp |= (uint8_t)samplesize;
  
  
  if(LPS25H_WriteReg(LPS25H_CTRL_FIFO_REG, 1, &tmp))           
    return LPS25H_ERROR;
  
  return LPS25H_OK;	
}

/**
* @brief   Get the number of sample to perform moving average when FIFO_MEAN_MODE is used
* @param 	 buffer to empty with sample size tmp
* @retval  Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_FifoSampleSize(LPS25H_FifoMeanModeSample_et* samplesize)
{  
  uint8_t tmp; 
  
  if(LPS25H_ReadReg(LPS25H_CTRL_FIFO_REG, 1, &tmp))
    return LPS25H_ERROR;
  
  tmp &= LPS25H_WTM_POINT_MASK;
  *samplesize = (LPS25H_FifoMeanModeSample_et)tmp;
  
  return LPS25H_OK;	
}

/**
* @brief   Get the reference pressure after soldering for computing differential pressure (hPA)
* @param 	 buffer to empty with the he pressure tmp (hPA)
* @retval  Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_PressureOffsetValue(int16_t *pressoffset)
{  
  uint8_t buffer[2];
  int16_t raw_press;  
  
  if(LPS25H_ReadReg(LPS25H_RPDS_L_REG, 2, buffer))
    return LPS25H_ERROR;
  
  raw_press = (int16_t)((((uint16_t)buffer[1]) << 8) + (uint16_t)buffer[0]);
  
  *pressoffset = (raw_press*100)/4096;	
  
  return LPS25H_OK;	
}

/**
* @brief   Set Generic Configuration
* @param 	 Struct to empty with the chosen tmp
* @retval  Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_GenericConfig(LPS25H_ConfigTypeDef_st* pxLPS25HInit)
{  
  /* Step 1. Init REF_P register*/
  /* The REF_P is the Reference Pressure. Its reset tmp is 0x00*/
  /* The REF_P will be set to the defualt RPDS (0x39h) tmp  if Reset_AZ is enabled.*/
  /* The REF_P will be set the actual pressure output if AutoZero is enabled*/
  
  if((pxLPS25HInit->Reset_AZ)==LPS25H_ENABLE){
    if(LPS25H_ResetAZ())
      return LPS25H_ERROR;
  }
  else if((pxLPS25HInit->AutoZero)==LPS25H_ENABLE){
    if(LPS25H_Set_AutoZeroFunction(LPS25H_SET))
      return LPS25H_ERROR;
  }
  
  /* Step 2. Init the Pressure and Temperature Resolution*/
  if(LPS25H_Set_Avg(pxLPS25HInit->PressResolution,pxLPS25HInit->TempResolution))
    return LPS25H_ERROR;
  
  /* Step 3. Init the Output Data Rate*/
  if(LPS25H_Set_Odr(pxLPS25HInit->OutputDataRate))
    return LPS25H_ERROR;
  
  /*Step 4. BDU bit is used to inhibit the output registers update between the reading of upper and
  lower register parts. In default mode (BDU = ‘0’), the lower and upper register parts are
  updated continuously. If it is not sure to read faster than output data rate, it is recommended
  to set BDU bit to ‘1’. In this way, after the reading of the lower (upper) register part, the
  content of that output registers is not updated until the upper (lower) part is read too.
  This feature avoids reading LSB and MSB related to different samples.*/
  
  if(LPS25H_Set_Bdu(pxLPS25HInit->BDU))
    return LPS25H_ERROR;
  
  /*Step 5. SIM bit selects the SPI serial interface mode.*/
  /* This feature has effect only if SPI interface is used*/
  
  if(LPS25H_Set_SpiInterface(pxLPS25HInit->Sim))
    return LPS25H_ERROR;
  
  return LPS25H_OK;
}


/**
* @brief   Get Generic Configuration
* @param 	 Struct to empty with the  tmp read from sensor
* @retval  Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_GenericConfig(LPS25H_ConfigTypeDef_st* pxLPS25HInit)
{  
  uint8_t tmp;
  
  if(LPS25H_ReadReg(LPS25H_RES_CONF_REG, 1, &tmp))
    return LPS25H_ERROR;
  
  pxLPS25HInit->PressResolution=(LPS25H_Avgp_et)(tmp&LPS25H_AVGP_MASK);
  pxLPS25HInit->TempResolution=(LPS25H_Avgt_et)(tmp& LPS25H_AVGT_MASK);
  
  if(LPS25H_ReadReg(LPS25H_CTRL_REG1, 1, &tmp))
    return LPS25H_ERROR;
  
  
  
  pxLPS25HInit->OutputDataRate= (LPS25H_Odr_et)(tmp & LPS25H_ODR_MASK);
  
  pxLPS25HInit->BDU=(LPS25H_Bdu_et)(tmp & LPS25H_BDU_MASK);
  pxLPS25HInit->Sim=(LPS25H_SPIMode_et)(tmp& LPS25H_SIM_MASK);
  
  if(LPS25H_ReadReg(LPS25H_CTRL_REG2, 1, &tmp))
    return LPS25H_ERROR;
  
  pxLPS25HInit->AutoZero=(LPS25H_State_et)((tmp&LPS25H_RESET_AZ_MASK)>>LPS25H_AUTO_ZERO_BIT);
  
  return LPS25H_OK;
  
}

/**
* @brief   Set Interrupt Configuration
* @param 	 Struct to empty with the chosen tmp
* @retval  Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_InterruptConfig(LPS25H_InterruptTypeDef_st* pLPS25HInt)
{  
  if(LPS25H_Set_InterruptActiveLevel(pLPS25HInt->INT_H_L))
    return LPS25H_ERROR;
  
  if(LPS25H_Set_InterruptOutputType(pLPS25HInt->PP_OD))
    return LPS25H_ERROR;
  
  if(LPS25H_Set_InterruptControlConfig(pLPS25HInt->OutputSignal_INT1))
    return LPS25H_ERROR;
  
  if(pLPS25HInt->OutputSignal_INT1==LPS25H_DATA){
    
    if(LPS25H_Set_InterruptDataConfig(pLPS25HInt->DataInterrupt_INT1))
      return LPS25H_ERROR;
  }
  
  if(LPS25H_LatchInterruptRequest(pLPS25HInt->LatchIRQ))
    return LPS25H_ERROR;
  
  if(LPS25H_Set_PressureThreshold(pLPS25HInt->fP_threshold))
    return LPS25H_ERROR;
  
  /*DIFF_EN bit is used to enable the circuitry for the computing of differential pressure output.*/
  /*It is suggested to turn on the circuitry only after the configuration of REF_P_x and THS_P_x.*/
  
  if(LPS25H_Set_InterruptDifferentialConfig(pLPS25HInt->PressureInterrupt))
    return LPS25H_ERROR;
  
  return LPS25H_OK;
}

/**
* @brief   Get Interrupt Configuration
* @param 	 Struct to empty with the tmp read from sensor
* @retval  Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_InterruptConfig(LPS25H_InterruptTypeDef_st* pLPS25HInt)
{  
  uint8_t tmp;
  
  if(LPS25H_ReadReg(LPS25H_CTRL_REG3, 1, &tmp))
    return LPS25H_ERROR;
  
  pLPS25HInt->INT_H_L=(LPS25H_State_et)((tmp&LPS25H_INT_H_L_MASK)>>LPS25H_INT_H_L_BIT);
  
  pLPS25HInt->PP_OD =(LPS25H_OutputType_et)(tmp&LPS25H_PP_OD_MASK);
  pLPS25HInt->OutputSignal_INT1=(LPS25H_OutputSignalConfig_et)((tmp&0x03));
  
  if(pLPS25HInt->OutputSignal_INT1==LPS25H_DATA){
    if(LPS25H_ReadReg(LPS25H_CTRL_REG4, 1, &tmp))
      return LPS25H_ERROR;
    
    pLPS25HInt->DataInterrupt_INT1=(LPS25H_DataSignalType_et)(tmp &=0x0F);
    
  }
  if(LPS25H_ReadReg(LPS25H_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS25H_ERROR;
  
  pLPS25HInt->LatchIRQ=(LPS25H_State_et)((tmp &LPS25H_LIR_MASK)>>LPS25H_LIR_BIT);
  pLPS25HInt->PressureInterrupt=(LPS25H_InterruptDiffConfig_et)(tmp &LPS25H_PE_MASK);
  if(LPS25H_Get_PressureThreshold(&pLPS25HInt->fP_threshold))
    return LPS25H_ERROR;
  
  return LPS25H_OK;
}
/**
* @brief   Set Fifo Configuration
* @param 	 Struct to empty with the chosen tmp 
* @retval  Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_FifoConfig(LPS25H_FIFOTypeDef_st* pLPS25HFIFO)
{
  if(pLPS25HFIFO->FIFO_MODE == LPS25H_FIFO_BYPASS_MODE) { 			
    /* FIFO Disable-> FIFO_EN bit=0 in CTRL_REG2*/
    if(LPS25H_Set_FifoModeUse(LPS25H_DISABLE))
      return LPS25H_ERROR;
  } 
  else {
    /* FIFO Enable-> FIFO_EN bit=1 in CTRL_REG2*/
    if(LPS25H_Set_FifoModeUse(LPS25H_ENABLE))
      return LPS25H_ERROR;
    
    if(pLPS25HFIFO->FIFO_MODE==LPS25H_FIFO_MEAN_MODE){
      if(LPS25H_Set_FifoSampleSize(pLPS25HFIFO->MEAN_MODE_SAMPLE))
        return LPS25H_ERROR;
      if(pLPS25HFIFO->FIFO_MEAN_DEC)
        if(LPS25H_Set_FifoMeanDecUse(LPS25H_ENABLE))
          return LPS25H_ERROR;
    }
    else{
      if (pLPS25HFIFO->WTM_INT){
        if(LPS25H_Set_FifoWatermarkLevelUse(LPS25H_ENABLE))
          return LPS25H_ERROR;
        if(LPS25H_Set_FifoWatermarkLevel(pLPS25HFIFO->WTM_LEVEL))
          return LPS25H_ERROR;	
      }
    }
  }
  
  if(LPS25H_Set_FifoMode(pLPS25HFIFO->FIFO_MODE))
    return LPS25H_ERROR;
  
  return LPS25H_OK;
  
}

/**
* @brief   Get Fifo Configuration
* @param 	 Struct to empty with the  tmp read from sensor
* @retval  Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_FifoConfig(LPS25H_FIFOTypeDef_st* pLPS25HFIFO)
{  
  uint8_t tmp;
  
  if(LPS25H_ReadReg(LPS25H_CTRL_FIFO_REG, 1, &tmp))
    return LPS25H_ERROR;
  
  
  pLPS25HFIFO->FIFO_MODE=(LPS25H_FifoMode_et)(tmp&LPS25H_FMODE_MASK);
  
  if(pLPS25HFIFO->FIFO_MODE==LPS25H_FIFO_MEAN_MODE){
    
    pLPS25HFIFO->MEAN_MODE_SAMPLE=(LPS25H_FifoMeanModeSample_et)(tmp&LPS25H_WTM_POINT_MASK);
    
    if(LPS25H_ReadReg(LPS25H_CTRL_REG2, 1, &tmp))
      return LPS25H_ERROR;
    
    pLPS25HFIFO->FIFO_MEAN_DEC=(LPS25H_State_et)((tmp&LPS25H_FIFO_MEAN_MASK)>>LPS25H_FIFO_MEAN_BIT);
    
  }
  else{
    if(pLPS25HFIFO->FIFO_MODE != LPS25H_FIFO_BYPASS_MODE) { 
      if(LPS25H_ReadReg(LPS25H_CTRL_REG2, 1, &tmp))
        return LPS25H_ERROR;
      
      pLPS25HFIFO->WTM_INT=(LPS25H_State_et)((tmp&LPS25H_WTM_EN_MASK)>>LPS25H_WTM_EN_BIT);
      
      if (pLPS25HFIFO->WTM_INT){
        if(LPS25H_ReadReg(LPS25H_CTRL_FIFO_REG, 1, &tmp))
          return LPS25H_ERROR;
        pLPS25HFIFO->WTM_LEVEL=(uint8_t)(tmp&LPS25H_WTM_POINT_MASK);
        
      }
    }
  }
  
  return LPS25H_OK;
}


/**
* @brief   Get the Reference Pressure tmp that is sum to the sensor output pressure
* @param 	 Buffer to empty with reference pressure tmp
* @retval  Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_ReferencePressure(int32_t* RefP)
{  
  uint8_t buffer[3];
  uint32_t tempVal=0;
  int32_t raw_press;  
  uint8_t i;
  
  if(LPS25H_ReadReg(LPS25H_REF_P_XL_REG, 3, buffer))
    return LPS25H_ERROR;
  
  /* Build the raw data */
  for(i=0; i<3; i++)
    tempVal |= (((uint32_t)buffer[i]) << (8*i));
  
  /* convert the 2's complement 24 bit to 2's complement 32 bit */
  if(tempVal & 0x00800000)
    tempVal |= 0xFF000000;
  
  raw_press =((int32_t)tempVal);
  *RefP = (raw_press*100)/4096;
  
  return LPS25H_OK;
}


/**
* @brief  Check if the single measurement has completed.
* @param  tmp is set to 1, when the measure is completed
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/  
LPS25H_Error_et LPS25H_IsMeasurementCompleted(uint8_t* Is_Measurement_Completed)
{  
  uint8_t tmp;
  LPS25H_DataStatus_st datastatus;
  
  if(LPS25H_ReadReg(LPS25H_STATUS_REG, 1, &tmp))
    return LPS25H_ERROR;
  
  datastatus.TempDataAvailable=(uint8_t)(tmp&0x01);
  datastatus.PressDataAvailable= (uint8_t)((tmp&0x02)>>LPS25H_PDA_BIT);  
  
  *Is_Measurement_Completed=(uint8_t)((datastatus.PressDataAvailable) & (datastatus.TempDataAvailable));
  
  return LPS25H_OK;
}

/**
* @brief  Get the values of the last single measurement.
* @param  Pressure and temperature tmp
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_Measurement(LPS25H_MeasureTypeDef_st *Measurement_Value)
{  
  int16_t Tout;
  int32_t Pout;
  
  if(LPS25H_Get_Temperature(&Tout))
    return LPS25H_ERROR;
  
  Measurement_Value->Tout=Tout;
  
  if(LPS25H_Get_Pressure(&Pout))
    return LPS25H_ERROR;
  
  Measurement_Value->Pout=Pout;
  
  return LPS25H_OK;
  
}


/**
* @brief  De initialization function for LPS25H.
*         This function put the LPS25H in power down, make a memory boot and clear the data output flags.
* @param  None.
* @retval Error code [LPS25H_OK, LPS25H_ERROR].
*/
LPS25H_Error_et LPS25H_DeInit(void)
{    
  LPS25H_MeasureTypeDef_st Measurement_Value;    
  
  /* LPS25H in power down */
  if(LPS25H_Set_PowerDownMode(LPS25H_RESET))
    return LPS25H_ERROR;
  
  /* Make LPS25H Reset and Reboot */ 
  if(LPS25H_SwResetAndMemoryBoot())
    return LPS25H_ERROR;
  
  /* Dump of data output */
  if(LPS25H_Get_Measurement(& Measurement_Value))
    return LPS25H_ERROR;
  
  return LPS25H_OK;
}



#ifdef  USE_FULL_ASSERT_LPS25H
/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param file: pointer to the source file name
* @param line: assert_param error line source number
* @retval : None
*/
void LPS25H_assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number */
  printf("Wrong parameters tmp: file %s on line %d\r\n", file, line);
  
  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
* @}
*/

/**
* @}
*/

/**
* @}
*/

/******************* (C) COPYRIGHT 2013 STMicroelectronics *****END OF FILE****/

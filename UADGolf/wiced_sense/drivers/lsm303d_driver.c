/******************** (C) COPYRIGHT 2012 STMicroelectronics ********************
* File Name          : lsm303d_driver.c
* Author             : MSH Application Team
* Author             : Abhishek Anand
* Version            : $Revision:$
* Date               : $Date:$
* Description        : LSM303D driver file
*                      
* HISTORY:
* Date               |	Modification                    |	Author
* 24/05/2012         |	Initial Revision                |	Abhishek Anand

********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
*
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "lsm303d_driver.h"
#include "cfa.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

// Read operation to the lower level driver is 0.
#define I2C_SLAVE_OPERATION_READ                    (0)

// Write operation to the lower level driver is 1.
#define I2C_SLAVE_OPERATION_WRITE                   (1)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/*******************************************************************************
* Function Name		: ReadReg
* Description		: Generic Reading function. It must be fullfilled with either
*			: I2C or SPI reading functions					
* Input			: Register Address
* Output		: Data Read
* Return		: None
*******************************************************************************/
u8_t LSM303D_ReadReg(u8_t deviceAddr, u8_t Reg, u8_t* Data) {
	CFA_BSC_STATUS read_status;

	read_status = cfa_bsc_OpExtended(Data, sizeof(u8_t), &Reg, sizeof(u8_t), deviceAddr, I2C_SLAVE_OPERATION_READ);

    switch(read_status)
    {
        case CFA_BSC_STATUS_INCOMPLETE:
            // Transaction did not go through. ERROR. Handle this case.
        	return MEMS_ERROR;
            break;
        case CFA_BSC_STATUS_SUCCESS:
            // The read was successful.
        	return MEMS_SUCCESS;
            break;
        case CFA_BSC_STATUS_NO_ACK:
            // No slave device with this address exists on the I2C bus. ERROR. Handle this.
        default:
            break;
    }

    return MEMS_ERROR;
}


/*******************************************************************************
* Function Name		: WriteReg
* Description		: Generic Writing function. It must be fullfilled with either
*			: I2C or SPI writing function
* Input			: Register Address, Data to be written
* Output		: None
* Return		: None
*******************************************************************************/
u8_t LSM303D_WriteReg(u8_t deviceAddress, u8_t WriteAddr, u8_t Data) {
	CFA_BSC_STATUS read_status;
    UINT8 reg_data_bytes[2];

    reg_data_bytes[0]= WriteAddr;
    reg_data_bytes[1] = Data;

    read_status = cfa_bsc_OpExtended(reg_data_bytes, sizeof(reg_data_bytes), NULL, 0, deviceAddress, I2C_SLAVE_OPERATION_WRITE);

    switch(read_status)
    {
        case CFA_BSC_STATUS_INCOMPLETE:
            // Transaction did not go through. ERROR. Handle this case.
        	return MEMS_ERROR;
            break;
        case CFA_BSC_STATUS_SUCCESS:
            // The read was successful.
        	return MEMS_SUCCESS;
            break;
        case CFA_BSC_STATUS_NO_ACK:
            // No slave device with this address exists on the I2C bus. ERROR. Handle this.
        default:
            break;
    }

    return MEMS_ERROR;
}


/* Private functions ---------------------------------------------------------*/

/***************CTRL0***************/
/*******************************************************************************
* Function Name  : LSM303D_BootEnable
* Description    : Enable/Disable Boot
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_BootEnable(State_t boot) {
  u8_t value;
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL0, &value) )
    return MEMS_ERROR;
                  
  value &= 0x67; //bit<4,3> must be =0 for correct working 
  value |= (boot<<LSM303D_BOOT);
  
  if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL0, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303D_FIFOWaterMarkEnable
* Description    : Enable/Disable Boot
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_FIFOWaterMarkEnable(State_t boot) {
  u8_t value;
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL0, &value) )
    return MEMS_ERROR;
                  
  value &= 0x27; //bit<4,3> must be =0 for correct working 
  value |= (boot<<LSM303D_FTH_EN);
  
  if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL0, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303D_HPFClickEnable
* Description    : Enable/Disable High Pass Filter for click
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_HPFClickEnable(State_t hpfe) {
  u8_t value;
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL0, &value) )
    return MEMS_ERROR;
                  
  value &= 0xE3; //bit<4,3> must be =0 for correct working
  value |= (hpfe<<LSM303D_HPCLICK);
  
  if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL0, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM303D_HPFAOI1Enable
* Description    : Enable/Disable High Pass Filter for AOI on INT_1
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_HPFAOI1Enable(State_t hpfe) {
  u8_t value;
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL0, &value) )
    return MEMS_ERROR;
                  
  value &= 0xE5; //bit<4,3> must be =0 for correct working
  value |= (hpfe<<LSM303D_HPIS1);
  
  if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL0, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM303D_HPFAOI2Enable
* Description    : Enable/Disable High Pass Filter for AOI on INT_2
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_HPFAOI2Enable(State_t hpfe) {
  u8_t value;
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL0, &value) )
    return MEMS_ERROR;
                  
  value &= 0xE6; //bit<4,3> must be =0 for correct working
  value |= (hpfe<<LSM303D_HPIS2);
  
  if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL0, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/***************CTRL1***************/
/*******************************************************************************
* Function Name  : LSM303D_SetAccODR
* Description    : Sets LSM303D Output Data Rate Accelerometer
* Input          : Output Data Rate
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_SetAccODR(LSM303D_ODR_A_t ov){
  u8_t value;

  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL1, &value) )
    return MEMS_ERROR;

  value &= 0x0F;
  value |= ov<<LSM303D_ODR_BIT_A;

  if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL1, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM303D_SetAccMode
* Description    : Put the Accelerometer in Power Down Mode
* Input          : ENABLE/DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_SetAccMode(LSM303D_Mode_A_t md){
  u8_t value;
  if(md != LSM303D_POWER_DOWN_MODE_A)
    return MEMS_ERROR;
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL1, &value) )
    return MEMS_ERROR;
  
  value &= 0x0F;
  value |= (md<<LSM303D_ODR_BIT_A);
  
  if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL1, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}	


/*******************************************************************************
* Function Name  : LSM303D_SetBDU
* Description    : Enable/Disable Block Data Update Functionality
* Input          : ENABLE/DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_SetBDU(State_t bdu) {
  u8_t value;
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL1, &value) )
    return MEMS_ERROR;
 
  value &= 0xF7;
  value |= (bdu<<LSM303D_BDU);

  if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL1, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM303D_SetAxis
* Description    : Enable/Disable LSM303D Axis
* Input          : LSM303D_X_ENABLE/LSM303D_X_DISABLE | LSM303D_Y_ENABLE/LSM303D_Y_DISABLE 
				   | LSM303D_Z_ENABLE/LSM303D_Z_DISABLE
* Output         : None
* Note           : You MUST use ALL input variable in the argument, as example
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_SetAxis(LSM303D_Axis_t axis) {
  u8_t value;
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL1, &value) )
    return MEMS_ERROR;
  value &= 0xF8;
  value |= (0x07 & axis);
   
  if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL1, value) )
    return MEMS_ERROR;   
  
  return MEMS_SUCCESS;
}

/***************CTRL2***************/
/*******************************************************************************
* Function Name  : LSM303D_SetAccABW
* Description    : Sets LSM303D Accelerometer anti-alias filter bandwidth
* Input          : LSM303D_ABW_773Hz_A/LSM303D_ABW_362Hz_A/LSM303D_ABW_194Hz_A
					/LSM303D_ABW_50Hz_A
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_SetAccABW(LSM303D_ABW_A_t abw){
  u8_t value;

  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL2, &value) )
    return MEMS_ERROR;

  value &= 0x1B; //bit<2> must be =0 for correct working
  value |= abw<<LSM303D_ABW_A;

  if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL2, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303D_SetAccFullScale
* Description    : Sets the LSM303D Accelerometer FullScale
* Input          : LSM303D_FULLSCALE_2_A/LSM303D_FULLSCALE_4_A/LSM303D_FULLSCALE_6_A
                   LSM303D_FULLSCALE_8_A/LSM303D_FULLSCALE_16_A
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_SetAccFullScale(LSM303D_Fullscale_A_t fs) {
  u8_t value;
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL2, &value) )
    return MEMS_ERROR;
                  
  value &= 0xC3; //bit<2> must be =0 for correct working	
  value |= (fs<<LSM303D_FS_A);
  
  if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL2, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303D_SetAccSelfTest
* Description    : Enables/Disables Self Test on the Accelerometer
* Input          : MEMS_SET, MEMS_RESET
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_SetAccSelfTest(State_t st) {
  u8_t value;
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL2, &value) )
    return MEMS_ERROR;
                  
  value &= 0xF9; //bit<2> must be =0 for correct working
  value |= (st<<LSM303D_ST_A);
  
  if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL2, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303D_SetSPIMode
* Description    : Sets LSM303D SPI Mode
* Input          : LSM303D_SPI_4_WIRE/LSM303D_SPI_3_WIRE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_SetSPIMode(LSM303D_SPIMode_t spimode){
  u8_t value;
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL2, &value) )
    return MEMS_ERROR;
                  
  value &= 0xFA; //bit<2> must be =0 for correct working
  value |= spimode;
  
  if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL2, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/***************CTRL3***************/
/*******************************************************************************
* Function Name  : LSM303D_SetInt1Pin
* Description    : Set Interrupt1 pin Function
* Input          :  LSM303D_BOOT_ON_PIN_INT1_ENABLE/DISABLE | LSM303D_CLICK_ON_PIN_INT1_ENABLE/DISABLE |              
                    LSM303D_IG1_ON_PIN_INT1_ENABLE/DISABLE	| LSM303D_IG2_ON_PIN_INT1_ENABLE/DISABLE|              
                    LSM303D_IGM_ON_PIN_INT1_ENABLE/DISABLE | LSM303D_DRDYA_ON_PIN_INT1_ENABLE/DISABLE|           
                    LSM303D_DRDYM_ON_PIN_INT1_ENABLE/DISABLE| LSM303D_EMPTY_ON_PIN_INT1_ENABLE/DISABLE
* example        : SetInt1Pin(LSM303D_BOOT_ON_PIN_INT1_ENABLE | LSM303D_CLICK_ON_PIN_INT1_ENABLE |              
                    LSM303D_IG1_ON_PIN_INT1_DISABLE | LSM303D_IG2_ON_PIN_INT1_ENABLE | LSM303D_IGM_ON_PIN_INT1_ENABLE |
                    LSM303D_DRDYA_ON_PIN_INT1_ENABLE | LSM303D_DRDYM_ON_PIN_INT1_ENABLE | LSM303D_EMPTY_ON_PIN_INT1_ENABLE) 
* Note           : To enable Interrupt signals on INT1 Pad (You MUST use ALL input variable in the argument, as example)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_SetInt1Pin(LSM303D_IntPinConf_t pinConf) {
  u8_t value;
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL3, &value) )
    return MEMS_ERROR;
                  
  value &= 0x00;
  value |= pinConf;
  
  if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL3, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/***************CTRL4***************/
/*******************************************************************************
* Function Name  : LSM303D_SetInt2Pin
* Description    : Set Interrupt2 pin Function
* Input          :  LSM303D_CLICK_ON_PIN_INT2_ENABLE/DISABLE | LSM303D_IG1_ON_PIN_INT2_ENABLE/DISABLE |              
                    LSM303D_IG2_ON_PIN_INT2_ENABLE/DISABLE	| LSM303D_IGM_ON_PIN_INT2_ENABLE/DISABLE|              
                    LSM303D_DRDYA_ON_PIN_INT2_ENABLE/DISABLE | LSM303D_WTM_ON_PIN_INT2_ENABLE/DISABLE|           
                    LSM303D_OVERRUN_ON_PIN_INT2_ENABLE/DISABLE| LSM303D_EMPTY_ON_PIN_INT1_ENABLE/DISABLE
* example        : SetInt1Pin(LSM303D_CLICK_ON_PIN_INT2_ENABLE | LSM303D_IG1_ON_PIN_INT2_ENABLE |              
                    LSM303D_INT2_ON_PIN_INT2_DISABLE | LSM303D_IGM_ON_PIN_INT2_ENABLE | LSM303D_DRDYA_ON_PIN_INT2_ENABLE |
                    LSM303D_DRDYM_ON_PIN_INT2_ENABLE | LSM303D_OVERRUN_ON_PIN_INT2_ENABLE | LSM303D_WTM_ON_PIN_INT2_ENABLE) 
* Note           : To enable Interrupt signals on INT2 Pad (You MUST use ALL input variable in the argument, as example)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_SetInt2Pin(LSM303D_IntPinConf_t pinConf) {
  u8_t value;
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL4, &value) )
    return MEMS_ERROR;
                  
  value &= 0x00;
  value |= pinConf;
  
  if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL4, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/***************CTRL5***************/
/*******************************************************************************
* Function Name  : LSM303D_EnableTemperature
* Description    : Sets LSM303D Temperature Sensor
* Input          : MEMS_ENABLE, MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_EnableTemperature(State_t state){
  u8_t value;

  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL5, &value) )
    return MEMS_ERROR;

  value &= 0x1F; //bit<6,5> must be =0 for correct working
  value |= state<<LSM303D_TEMP_EN;

  if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL5, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303D_SetMagResolution
* Description    : Sets LSM303D Magnetometer Resolution
* Input          : LSM303D_LOW_RES_M, LSM303D_HIGH_RES_M
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_SetMagResolution(LSM303D_RES_M_t magres){
  u8_t value;

  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL5, &value) )
    return MEMS_ERROR;

  value &= 0x5F;
  value |= magres<<LSM303D_MRES;

  if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL5, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303D_SetMagODR
* Description    : Sets LSM303 Output Data Rate Magnetometer
* Input          : Output Data Rate
*				   LSM303D_ODR_3_125Hz_M/LSM303D_ODR_6_25Hz_M......
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_SetMagODR(LSM303D_ODR_M_t ov){
  u8_t value;

  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL5, &value) )
    return MEMS_ERROR;

  value &= 0x83; //bit<6,5> must be =0 for correct working
  value |= ov<<LSM303D_ODR_BIT_M;

  if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL5, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303D_EnableInt1Latch
* Description    : Enable Interrupt 1 Latching function
* Input          : ENABLE/DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_EnableInt1Latch(State_t latch) {
  u8_t value;
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL5, &value) )
    return MEMS_ERROR;
                  
  value &= 0x9E; //bit<6,5> must be =0 for correct working
  value |= latch;
  
  if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL5, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303D_EnableInt2Latch
* Description    : Enable Interrupt 2 Latching function
* Input          : ENABLE/DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_EnableInt2Latch(State_t latch) {
  u8_t value;
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL5, &value) )
    return MEMS_ERROR;
                  
  value &= 0x9D; //bit<6,5> must be =0 for correct working
  value |= latch<<LSM303D_LIR2;
  
  if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL5, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303D_ResetInt1Latch
* Description    : Reset Interrupt 1 Latching function
* Input          : None
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_ResetInt1Latch(void) {
  u8_t value;
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_IG_SRC1, &value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303D_ResetInt2Latch
* Description    : Reset Interrupt 2 Latching function
* Input          : None
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_ResetInt2Latch(void) {
  u8_t value;
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_IG_SRC2, &value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/***************CTRL6***************/

/*******************************************************************************
* Function Name  : LSM303D_SetMagFullScale
* Description    : Sets the LSM303D Magnetometer FullScale
* Input          : LSM303D_FULLSCALE_2_M/LSM303D_FULLSCALE_4_M/LSM303D_FULLSCALE_6_M
					/LSM303D_FULLSCALE_12_M
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_SetMagFullScale(LSM303D_Fullscale_M_t fs) {
  u8_t value;
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL6, &value) )
    return MEMS_ERROR;
                  
  value &= 0x00;	//bit<7,4,3,2,1,0> must be =0 for correct working
  value |= (fs<<LSM303D_FS_M);
  
  if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL6, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/***************CTRL7***************/
/*******************************************************************************
* Function Name  : LSM303D_SetHPFMode
* Description    : Set High Pass Filter Modality
* Input          : LSM303D_HPM_NORMAL_MODE_RES_A/LSM303D_HPM_REF_SIGNAL_A
				   /LSM303D_HPM_NORMAL_MODE_A/LSM303D_HPM_AUTORESET_INT_A
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_SetHPFMode(LSM303D_HPFMode_A_t hpm) {
  u8_t value;
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL7, &value) )
    return MEMS_ERROR;
                  
  value &= 0x37;	//bit<3> must be =0 for correct working
  value |= (hpm<<LSM303D_AHPM);
  
  if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL7, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303D_SetAccFilterDataSel
* Description    : Set Filter Data Selection bypassed or sent to FIFO OUT register
* Input          : MEMS_SET, MEMS_RESET
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_SetAccFilterDataSel(State_t state) {
  u8_t value;
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL7, &value) )
    return MEMS_ERROR;
                  
  value &= 0xD7;	//bit<3> must be =0 for correct working
  value |= (state<<LSM303D_AFDS);
  
  if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL7, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
  
}

/*******************************************************************************
* Function Name  : LSM303D_SetTemperatureOnly
* Description    : Enable/Disable the Temperature Only mode 
* Input          : MEMS_SET, MEMS_RESET
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_SetTemperatureOnly(State_t state) {
  u8_t value;
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL7, &value) )
    return MEMS_ERROR;
                  
  value &= 0xE7;	//bit<3> must be =0 for correct working
  value |= (state<<LSM303D_T_ONLY);
  
  if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL7, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303D_SetMagLPMode
* Description    : Enable/Disable the Magnetometer Low Power Mode 
* Input          : MEMS_SET, MEMS_RESET
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_SetMagLPMode(State_t state) {
  u8_t value;
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL7, &value) )
    return MEMS_ERROR;
                  
  value &= 0xF3;	//bit<3> must be =0 for correct working
  value |= (state<<LSM303D_MLP);
  
  if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL7, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303D_SetMagMode
* Description    : Sets LSM303D Magnetometer Modality
* Input          : Modality (LSM303D_CONTINUOUS_MODE_M/LSM303D_SINGLE_MODE_M/LSM303D_POWER_DOWN_MODE_M)	
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_SetMagMode(LSM303D_Mode_M_t Mode){
  u8_t value;

  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL7, &value) )
    return MEMS_ERROR;

  value &= 0xF4; //bit<3> must be =0 for correct working
  value |= Mode<<LSM303D_MD;

  if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL7, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/***************OFFSET REGISTERS******************/

/*******OFFSET_X_L_M, OFFSET_X_H_M********/
/*******************************************************************************
* Function Name  : LSM303D_SetMagOffsetX
* Description    : Sets Magnetic Offset on the X Axis
* Input          : Offset value (-2048 to 2047)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//AA: Confirm with design
status_t LSM303D_SetMagOffsetX(i16_t offset){
  u8_t offsetL;
  u8_t offsetH;
  if((offset > 2047) || (offset < -2048)){
	return MEMS_ERROR;
  }
  else{
	offset = offset*16;
	offsetL = (u8_t)(offset | 0xFF);
	offsetH = (u8_t)((offset >> 8)|0xFF);

	if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_OFFSET_X_L_M, offsetL) )
		return MEMS_ERROR;
	if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_OFFSET_X_H_M, offsetH) )
		return MEMS_ERROR;

	return MEMS_SUCCESS;
  }
}

/*******OFFSET_Y_L_M, OFFSET_Y_H_M********/
/*******************************************************************************
* Function Name  : LSM303D_SetMagOffsetY
* Description    : Sets Magnetic Offset on the Y Axis
* Input          : Offset value (-2048 to 2047)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//AA: Confirm with design
status_t LSM303D_SetMagOffsetY(i16_t offset){
  u8_t offsetL;
  u8_t offsetH;
  if((offset > 2047) || (offset < -2048)){
	return MEMS_ERROR;
  }
  else{
	offset = offset*16;
	offsetL = (u8_t)(offset | 0xFF);
	offsetH = (u8_t)((offset >> 8)|0xFF);

	if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_OFFSET_Y_L_M, offsetL) )
		return MEMS_ERROR;
	if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_OFFSET_Y_H_M, offsetH) )
		return MEMS_ERROR;

	return MEMS_SUCCESS;
  }
}

/*******OFFSET_Z_L_M, OFFSET_Z_H_M********/
/*******************************************************************************
* Function Name  : LSM303D_SetMagOffsetZ
* Description    : Sets Magnetic Offset on the Z Axis
* Input          : Offset value (-2048 to 2047)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//AA: Confirm with design
status_t LSM303D_SetMagOffsetZ(i16_t offset){
  u8_t offsetL;
  u8_t offsetH;
  if((offset > 2047) || (offset < -2048)){
	return MEMS_ERROR;
  }
  else{
	offset = offset*16;
	offsetL = (u8_t)(offset | 0xFF);
	offsetH = (u8_t)((offset >> 8)|0xFF);

	if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_OFFSET_Z_L_M, offsetL) )
		return MEMS_ERROR;
	if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_OFFSET_Z_H_M, offsetH) )
		return MEMS_ERROR;

	return MEMS_SUCCESS;
  }
}

/**************REFERENCE REGISTERS****************/
/**************REFERENCE_X****************/
/*******************************************************************************
* Function Name  : LSM303D_SetAccRefX
* Description    : Sets Reference value for the high-pass filter on X-axis Acc Data
* Input          : Offset value (0-255)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_SetAccRefX(u8_t ref){
   if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_REFERENCE_X, ref) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/**************REFERENCE_Y****************/
/*******************************************************************************
* Function Name  : LSM303D_SetAccRefY
* Description    : Sets Reference value for the high-pass filter on Y-axis Acc Data
* Input          : Offset value (0-255)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_SetAccRefY(u8_t ref){
   if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_REFERENCE_Y, ref) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/**************REFERENCE_Z****************/
/*******************************************************************************
* Function Name  : LSM303D_SetAccRefZ
* Description    : Sets Reference value for the high-pass filter on Z-axis Acc Data
* Input          : Offset value (0-255)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_SetAccRefZ(u8_t ref){
   if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_REFERENCE_Z, ref) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/****************FIFO REGISTERS******************/
/**************FIFO_CNTRL_REG****************/
/*******************************************************************************
* Function Name  : LSM303D_FIFOModeEnable
* Description    : Sets Fifo Modality
* Input          : LSM303D_FIFO_BYPASS_MODE/LSM303D_FIFO_MODE/LSM303D_FIFO_STREAM_MODE/
				   LSM303D_FIFO_STREAM_TO_FIFO_MODE/LSM303D_FIFO_BYPASS_TO_STREAM_MODE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_FIFOModeEnable(LSM303D_FifoMode_t fm) {
  u8_t value;
  
  //FIFO_DISABLE  
  if(fm == LSM303D_FIFO_DISABLE_MODE) { 
    //FIFO mode to BYPASS
    if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_FIFO_CNTRL_REG, &value) )
      return MEMS_ERROR;  
    value &= 0x1f;
    value |= (LSM303D_FIFO_BYPASS_MODE<<LSM303D_FM);                     
    if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_FIFO_CNTRL_REG, value) )	
      return MEMS_ERROR;   
    
    //Disable FIFO
    if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL0, &value) )
      return MEMS_ERROR;                 
    value &= 0xBF;    
    if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL0, value) )
      return MEMS_ERROR;   
  }
  
  //Other Modes
  else if((fm == LSM303D_FIFO_BYPASS_MODE) || 
          (fm == LSM303D_FIFO_MODE) || 
            (fm == LSM303D_FIFO_STREAM_MODE) ||
              (fm == LSM303D_FIFO_STREAM_TO_FIFO_MODE) || 
                (fm == LSM303D_FIFO_BYPASS_TO_STREAM_MODE)) {  
               //Enable FIFO
                  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL0, &value) )
                    return MEMS_ERROR;               
                  value &= 0xBF;
                  value |= MEMS_SET<<LSM303D_FIFO_EN;
                  if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CTRL0, value) )
                    return MEMS_ERROR;  
                  
                  //Set Mode
                  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_FIFO_CNTRL_REG, &value) )
                    return MEMS_ERROR;   
                  value &= 0x1f;
                  value |= (fm<<LSM303D_FM);                        
                  if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_FIFO_CNTRL_REG, value) )
                    return MEMS_ERROR;
                }
  
  //Invalid input
  else{
    return MEMS_ERROR;
  }
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303D_FIFOSetWaterMark
* Description    : Sets Watermark Value
* Input          : Watermark = [0,31]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_FIFOSetWaterMark(u8_t wtm) {
  u8_t value;
  
  if(wtm > 31)
    return MEMS_ERROR;  

  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_FIFO_CNTRL_REG, &value) )
    return MEMS_ERROR;
                  
  value &= 0xE0;
  value |= wtm; 
  
  if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_FIFO_CNTRL_REG, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/**************FIFO_SRC_REG****************/
/*******************************************************************************
* Function Name  : LSM303D_GetFifoSourceReg
* Description    : Read Fifo source Register
* Input          : Byte to empity by FIFO source register value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_GetFifoSourceReg(u8_t* val) {
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_FIFO_SRC_REG, val) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303D_GetFifoSourceBit
* Description    : Read Fifo WaterMark source bit
* Input          : statusBIT: LSM303D_FIFO_SRC_FTH, LSM303D_FIFO_SRC_OVRUN, LSM303D_FIFO_SRC_EMPTY
*	           val: Byte to fill  with the bit value
* Output         : None
* Return         : Status of BIT [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_GetFifoSourceBit(u8_t statusBIT, u8_t *val){
  u8_t value;  
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_FIFO_SRC_REG, &value) )
    return MEMS_ERROR;
  
  if(statusBIT == LSM303D_FIFO_SRC_FTH){
    if(value &= LSM303D_FIFO_SRC_FTH){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == LSM303D_FIFO_SRC_OVRUN){
    if(value &= LSM303D_FIFO_SRC_OVRUN){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  if(statusBIT == LSM303D_FIFO_SRC_EMPTY){
    if(value &= statusBIT == LSM303D_FIFO_SRC_EMPTY){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  return MEMS_ERROR;
}

/**************INT_CTRL_M****************/
/*******************************************************************************
* Function Name  : LSM303D_SetMagIntConfiguration
* Description    : Magnetometer Interrupt Configuration (EXCEPT 4D)
* Input          : LSM303D_INT_XMIEN_ENABLE/DISABLE | LSM303D_INT_YMIEN_ENABLE/DISABLE  
                   | LSM303D_INT_ZMIEN_ENABLE/DISABLE | LSM303D_INT_PPOD_ENABLE/DISABLE
                   | LSM303D_INT_MIEA_ENABLE/DISABLE | LSM303D_INT_MIEL_ENABLE/DISABLE
                   | LSM303D_INT_MIEN_ENABLE/DISABLE
* Output         : None
* Note           : You MUST use ALL input variable in the argument, as in example above
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_SetMagIntConfiguration(LSM303D_IntConf_t ic) {
  u8_t value;
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_INT_CTRL_M, &value) )
    return MEMS_ERROR;
  
  value &= 0x02; //4D bit not changed, modified using LSM303D_SetInt6D4DConfiguration 
  value |= ic;
  
  if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_INT_CTRL_M, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/**************INT_SRC_M****************/
/*******************************************************************************
* Function Name  : LSM303D_GetMagIntSrcReg
* Description    : Read INT_SRC_M Register
* Input          : Byte to empty by INT_SRC_M register value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_GetMagIntSrcReg(u8_t* val) {
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_INT_SRC_M, val) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303D_GetMagIntSrcBit
* Description    : Read INT_SRC_M source bit
* Input          : statusBIT: LSM303D_INT_SRC_M_PTH_X, LSM303D_INT_SRC_M_PTH_X...
	           val: Byte to fill  with the bit value
* Output         : None
* Return         : Status of BIT [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_GetMagIntSrcBit(u8_t statusBIT, u8_t *val){
  u8_t value;  
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_INT_SRC_M, &value) )
    return MEMS_ERROR;
  
  if(statusBIT == LSM303D_INT_SRC_M_PTH_X){
    if(value &= LSM303D_INT_SRC_M_PTH_X){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == LSM303D_INT_SRC_M_PTH_Y){
    if(value &= LSM303D_INT_SRC_M_PTH_Y){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == LSM303D_INT_SRC_M_PTH_Z){
    if(value &= LSM303D_INT_SRC_M_PTH_Z){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == LSM303D_INT_SRC_M_NTH_X){
    if(value &= LSM303D_INT_SRC_M_NTH_X){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == LSM303D_INT_SRC_M_NTH_Y){
    if(value &= LSM303D_INT_SRC_M_NTH_Y){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }  
  
  if(statusBIT == LSM303D_INT_SRC_M_NTH_Z){
    if(value &= LSM303D_INT_SRC_M_NTH_Z){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == LSM303D_INT_SRC_MROI){
    if(value &= LSM303D_INT_SRC_MROI){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == LSM303D_INT_SRC_MINT){
    if(value &= LSM303D_INT_SRC_MINT){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  return MEMS_ERROR;
}

/**********INT_THS_L_M; INT_THS_H_M**********/
/*******************************************************************************
* Function Name  : LSM303D_SetMagIntThreshold
* Description    : Sets the Threshold for Magnetic Interrupt
* Input          : Value of reference acceleration value (-2048 to 2047)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//AA: Confirm with design
status_t LSM303D_SetMagIntThreshold(i16_t ths){
  u8_t thsL;
  u8_t thsH;
  if((ths > 2047) || (ths < -2048)){
	return MEMS_ERROR;
  }
  else{
	ths = ths*16;
	thsL = (u8_t)(ths | 0xFF);
	thsH = (u8_t)((ths >> 8)|0xFF);

	if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_OFFSET_X_L_M, thsL) )
		return MEMS_ERROR;
	if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_OFFSET_X_H_M, thsH) )
		return MEMS_ERROR;

	return MEMS_SUCCESS;
  }
}

/**************ACCELEROMETER INTERRUPT REGISTERS***************/

/**************IG_CFG1***************/
/*******************************************************************************
* Function Name  : LSM303D_SetAccInt1Configuration
* Description    : Interrupt 1 Configuration (without 6D_INT)
* Input          : LSM303D_INT_AND/OR | LSM303D_INT_ZHIE_ENABLE/DISABLE | LSM303D_INT_ZLIE_ENABLE/DISABLE...
* Output         : None
* Note           : You MUST use all input variable in the argument, as example
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_SetAccInt1Configuration(LSM303D_IntConf_t ic) {
  u8_t value;
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_IG_CFG1, &value) )
    return MEMS_ERROR;
  
  value &= 0x40; 
  value |= ic; 
  
  if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_IG_CFG1, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303D_SetAccInt1Mode
* Description    : Interrupt 1 Configuration mode (OR, 6D Movement, AND, 6D Position)
* Input          : LSM303D_INT_MODE_OR, LSM303D_INT_MODE_6D_MOVEMENT, 
                   LSM303D_INT_MODE_AND, LSM303D_INT_MODE_6D_POSITION
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_SetAccInt1Mode(LSM303D_IntMode_t int_mode) {
  u8_t value;
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_IG_CFG1, &value) )
    return MEMS_ERROR;
  
  value &= 0x3F; 
  value |= (int_mode<<LSM303D_INT_6D);
  
  if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_IG_CFG1, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303D_SetInt6D4DConfiguration
* Description    : 6D, 4D Interrupt Configuration
* Input          : LSM303D_INT1_6D_ENABLE, LSM303D_INT1_4D_ENABLE, LSM303D_INT1_6D_4D_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_SetInt6D4DConfiguration(LSM303D_INT_6D_4D_t ic) {
  u8_t value;
  u8_t value2;
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_IG_CFG1, &value) )
    return MEMS_ERROR;
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_INT_CTRL_M, &value2) )
    return MEMS_ERROR;
  
  if(ic == LSM303D_INT1_6D_ENABLE){
      value &= 0xBF; 
      value |= (MEMS_ENABLE<<LSM303D_INT_6D);
      value2 &= 0xFD; 
      value2 |= (MEMS_DISABLE<<LSM303D_4D);
  }
  
    if(ic == LSM303D_INT1_4D_ENABLE){
      value &= 0xBF; 
      value |= (MEMS_ENABLE<<LSM303D_INT_6D);
      value2 &= 0xFD; 
      value2 |= (MEMS_ENABLE<<LSM303D_4D);
  }
  
    if(ic == LSM303D_INT1_6D_4D_DISABLE){
      value &= 0xBF; 
      value |= (MEMS_DISABLE<<LSM303D_INT_6D);
      value2 &= 0xFD; 
      value2 |= (MEMS_DISABLE<<LSM303D_4D);
  }
  
  if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_IG_CFG1, value) )
    return MEMS_ERROR;
  if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_INT_CTRL_M, value2) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/**************IG_SRC1***************/
/*******************************************************************************
* Function Name  : LSM303D_GetAccInt1Src
* Description    : Reset Interrupt 1 Latching function
* Input          : None
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_GetAccInt1Src(u8_t* val) {
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_IG_SRC1, val) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303D_GetAccInt1SrcBit
* Description    : Reset Interrupt 1 Latching function
* Input          : statusBIT: LSM303D_INT_SRC_IA, LSM303D_INT_SRC_ZH, LSM303D_INT_SRC_ZL.....
                   val: Byte to be filled with the status bit
* Output         : None
* Return         : Status of BIT [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_GetInt1SrcBit(u8_t statusBIT, u8_t* val) {
  u8_t value;  
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_IG_SRC1, &value) )
    return MEMS_ERROR;
  
  
  if(statusBIT == LSM303D_INT_SRC_IA){
    if(value &= LSM303D_INT_SRC_IA){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == LSM303D_INT_SRC_ZH){
    if(value &= LSM303D_INT_SRC_ZH){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == LSM303D_INT_SRC_ZL){
    if(value &= LSM303D_INT_SRC_ZL){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == LSM303D_INT_SRC_YH){
    if(value &= LSM303D_INT_SRC_YH){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == LSM303D_INT_SRC_YL){
    if(value &= LSM303D_INT_SRC_YL){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  if(statusBIT == LSM303D_INT_SRC_XH){
    if(value &= LSM303D_INT_SRC_XH){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == LSM303D_INT_SRC_XL){
    if(value &= LSM303D_INT_SRC_XL){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  return MEMS_ERROR;
}

/*******************************************************************************
* Function Name  : LSM303D_Get6DPosition
* Description    : 6D, 4D Interrupt Position Detect
* Input          : Byte to empity by LSM303D_POSITION_6D_t Typedef
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_Get6DPosition(u8_t* val){
  u8_t value;
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_IG_SRC1, &value) )
    return MEMS_ERROR;

  value &= 0x7F;
  
  switch (value){
  case LSM303D_UP_SX:   
    *val = LSM303D_UP_SX;    
    break;
  case LSM303D_UP_DX:   
    *val = LSM303D_UP_DX;    
    break;
  case LSM303D_DW_SX:   
    *val = LSM303D_DW_SX;    
    break;
  case LSM303D_DW_DX:   
    *val = LSM303D_DW_DX;    
    break;
  case LSM303D_TOP:     
    *val = LSM303D_TOP;      
    break;
  case LSM303D_BOTTOM:  
    *val = LSM303D_BOTTOM;   
    break;
  }
  
return MEMS_SUCCESS;  
}

/***************IG_THS1**************/
/*******************************************************************************
* Function Name  : LSM303D_SetAccInt1Threshold
* Description    : Sets Interrupt 1 Threshold
* Input          : Threshold = [0,127]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_SetAccInt1Threshold(u8_t ths) {
  if (ths > 127)
    return MEMS_ERROR;
  
      if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_IG_THS1, ths) )
        return MEMS_ERROR;    

  return MEMS_SUCCESS;
}

/************IG_DUR1************/
/*******************************************************************************
* Function Name  : LSM303D_SetAccInt1Duration
* Description    : Sets Interrupt 1 Duration
* Input          : Duration value = [0,127]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_SetAccInt1Duration(LSM303D_IntConf_t id) {
 
  if (id > 127)
    return MEMS_ERROR;

  if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_IG_DUR1, id) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/**************IG_CFG2***************/

/*******************************************************************************
* Function Name  : LSM303D_SetAccInt2Configuration
* Description    : Interrupt 2 Configuration (without 6D_INT)
* Input          : LSM303D_INT_AND/OR | LSM303D_INT_ZHIE_ENABLE/DISABLE | LSM303D_INT_ZLIE_ENABLE/DISABLE...
* Output         : None
* Note           : You MUST use all input variable in the argument, as example
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_SetAccInt2Configuration(LSM303D_IntConf_t ic) {
  u8_t value;
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_IG_CFG2, &value) )
    return MEMS_ERROR;
  
  value &= 0x40; 
  value |= ic; //AA: Doesn't set 6D to 0 if it is 1, is it ok?
  
  if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_IG_CFG2, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303D_SetAccInt2Mode
* Description    : Interrupt 2 Configuration mode (OR, 6D Movement, AND, 6D Position)
* Input          : LSM303D_INT_MODE_OR, LSM303D_INT_MODE_6D_MOVEMENT, 
                   LSM303D_INT_MODE_AND, LSM303D_INT_MODE_6D_POSITION
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_SetAccInt2Mode(LSM303D_IntMode_t int_mode) {
  u8_t value;
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_IG_CFG2, &value) )
    return MEMS_ERROR;
  
  value &= 0x3F; 
  value |= (int_mode<<LSM303D_INT_6D);
  
  if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_IG_CFG2, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/**************IG_SRC2***************/
/*******************************************************************************
* Function Name  : LSM303D_GetAccInt2Src
* Description    : Reset Interrupt 2 Latching function
* Input          : None
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_GetAccInt2Src(u8_t* val) {
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_IG_SRC2, val) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303D_GetAccInt2SrcBit
* Description    : Reset Interrupt 2 Latching function
* Input          : LSM303D_INT_SRC_IA, LSM303D_INT_SRC_ZH, LSM303D_INT_SRC_ZL.....
                   val: Byte to be filled with the status bit
* Output         : None
* Return         : Status of BIT [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t GetInt2SrcBit(u8_t statusBIT, u8_t* val) {
  u8_t value;  
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_IG_SRC2, &value) )
    return MEMS_ERROR;
  
  
  if(statusBIT == LSM303D_INT_SRC_IA){
    if(value &= LSM303D_INT_SRC_IA){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == LSM303D_INT_SRC_ZH){
    if(value &= LSM303D_INT_SRC_ZH){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == LSM303D_INT_SRC_ZL){
    if(value &= LSM303D_INT_SRC_ZL){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == LSM303D_INT_SRC_YH){
    if(value &= LSM303D_INT_SRC_YH){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == LSM303D_INT_SRC_YL){
    if(value &= LSM303D_INT_SRC_YL){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  if(statusBIT == LSM303D_INT_SRC_XH){
    if(value &= LSM303D_INT_SRC_XH){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == LSM303D_INT_SRC_XL){
    if(value &= LSM303D_INT_SRC_XL){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  return MEMS_ERROR;
}

/***************IG_THS2**************/
/*******************************************************************************
* Function Name  : LSM303D_SetAccInt2Threshold
* Description    : Sets Interrupt 2 Threshold
* Input          : Threshold = [0,127]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_SetAccInt2Threshold(u8_t ths) {
  if (ths > 127)
    return MEMS_ERROR;
  
      if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_IG_THS2, ths) )
        return MEMS_ERROR;    

  return MEMS_SUCCESS;
}

/************IG_DUR2************/
/*******************************************************************************
* Function Name  : LSM303D_SetAccInt2Duration
* Description    : Sets Interrupt 2 Duration
* Input          : Duration value = [0,127]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_SetAccInt2Duration(LSM303D_IntConf_t id) {
 
  if (id > 127)
    return MEMS_ERROR;

  if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_IG_DUR2, id) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*********************CLICK REGISTERS**********************/

/************CLICK_CFG************/
/*******************************************************************************
* Function Name  : LSM303D_SetClickCFG
* Description    : Set Click Interrupt config Function
* Input          : LSM303D_ZD_ENABLE/DISABLE | LSM303D_ZS_ENABLE/DISABLE  
                   | LSM303D_YD_ENABLE/DISABLE | LSM303D_YS_ENABLE/DISABLE 
                   | LSM303D_XD_ENABLE/DISABLE  | LSM303D_XS_ENABLE/DISABLE 
* example        : LSM303D_SetClickCFG( LSM303D_ZD_ENABLE | LSM303D_ZS_DISABLE | LSM303D_YD_ENABLE 
                                        | LSM303D_YS_DISABLE | LSM303D_XD_ENABLE | LSM303D_XS_ENABLE)
* Note           : You MUST use all input variable in the argument, as example
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_SetClickCFG(u8_t status) {
  u8_t value;
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CLICK_CFG, &value) )
    return MEMS_ERROR;
                  
  value &= 0xC0;
  value |= status;
  
  if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CLICK_CFG, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}  

/************CLICK_SRC************/
/*******************************************************************************
* Function Name  : LSM303_Get

* Description    : Get Click Interrupt Responce by LSM303D_CLICK_SRC REGISTER
* Input          : char to empty with the Click Responce Typedef
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303_GetClickResponse(u8_t* res) {
  u8_t value;
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CLICK_SRC, &value) ) 
    return MEMS_ERROR;
  
  value &= 0x7F;
  
  if((value & LSM303D_IA)==0) {        
    *res = LSM303D_NO_CLICK;     
    return MEMS_SUCCESS;
  }
  else {
    if (value & LSM303D_DCLICK){
      if (value & LSM303D_CLICK_SIGN){
        if (value & LSM303D_CLICK_Z) {
          *res = LSM303D_DCLICK_Z_N;   
          return MEMS_SUCCESS;
        }
        if (value & LSM303D_CLICK_Y) {
          *res = LSM303D_DCLICK_Y_N;   
          return MEMS_SUCCESS;
        }
        if (value & LSM303D_CLICK_X) {
          *res = LSM303D_DCLICK_X_N;   
          return MEMS_SUCCESS;
        }
      }
      else{
        if (value & LSM303D_CLICK_Z) {
          *res = LSM303D_DCLICK_Z_P;   
          return MEMS_SUCCESS;
        }
        if (value & LSM303D_CLICK_Y) {
          *res = LSM303D_DCLICK_Y_P;   
          return MEMS_SUCCESS;
        }
        if (value & LSM303D_CLICK_X) {
          *res = LSM303D_DCLICK_X_P;   
          return MEMS_SUCCESS;
        }
      }       
    }
    else{
      if (value & LSM303D_CLICK_SIGN){
        if (value & LSM303D_CLICK_Z) {
          *res = LSM303D_SCLICK_Z_N;   
          return MEMS_SUCCESS;
        }
        if (value & LSM303D_CLICK_Y) {
          *res = LSM303D_SCLICK_Y_N;   
          return MEMS_SUCCESS;
        }
        if (value & LSM303D_CLICK_X) {
          *res = LSM303D_SCLICK_X_N;   
          return MEMS_SUCCESS;
        }
      }
      else{
        if (value & LSM303D_CLICK_Z) {
          *res = LSM303D_SCLICK_Z_P;  
          return MEMS_SUCCESS;
        }
        if (value & LSM303D_CLICK_Y) {
          *res = LSM303D_SCLICK_Y_P;   
          return MEMS_SUCCESS;
        }
        if (value & LSM303D_CLICK_X) {
          *res = LSM303D_SCLICK_X_P;  
          return MEMS_SUCCESS;
        }
      }
    }
  }
  return MEMS_ERROR;
} 

/************CLICK_THS************/
status_t LSM303D_SetClickTHS(u8_t ths);
/*******************************************************************************
* Function Name  : LSM303D_SetClickThreshold
* Description    : Set Click Interrupt threshold
* Input          : Click-click Threshold value [0-127]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetClickTHS(u8_t ths) {
  
  if(ths>127)     
    return MEMS_ERROR;
  
  if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CLICK_THS, ths) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
} 

/************TIME_LIMIT***********/
/*******************************************************************************
* Function Name  : LSM303D_SetClickLIMIT
* Description    : Set Click Interrupt Time Limit
* Input          : Click-click Time Limit value [0-127]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_SetClickLIMIT(u8_t t_limit) {
  
  if(t_limit>127)     
    return MEMS_ERROR;
  
  if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CLICK_TIME_LIMIT, t_limit) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
} 

/**********TIME_LATENCY***********/
/*******************************************************************************
* Function Name  : LSM303D_SetClickLATENCY
* Description    : Set Click Interrupt Time Latency
* Input          : Click-click Time Latency value [0-255]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetClickLATENCY(u8_t t_latency) {
  
  if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CLICK_TIME_LATENCY, t_latency) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
} 

/***********TIME_WINDOW***********/
/*******************************************************************************
* Function Name  : LSM303D_SetClickWINDOW
* Description    : Set Click Interrupt Time Window
* Input          : Click-click Time Window value [0-255]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_SetClickWINDOW(u8_t t_window) {
 
  if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_CLICK_TIME_WINDOW, t_window) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/********************SLEEP TO WAKE REGISTERS******************/
/*******************************************************************************
* Function Name  : LSM303D_SetActTHS
* Description    : Set Sleep to Wake, Return to Sleep activation threshold
* Input          : Threshold [0-127];
* Output         : None
* Note           : 1LSb = 16mg
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_SetActTHS(u8_t act_ths) {
  
  if(act_ths>127)     
    return MEMS_ERROR;
  
  if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_Act_THS, act_ths) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
} 

/***********Act_DUR***********/
/*******************************************************************************
* Function Name  : LSM303D_SetActDUR
* Description    : Set Sleep to Wake, Return to Sleep activation duration
* Input          : Duration [0-255];
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_SetActDUR(u8_t act_dur) {
  
  if( !LSM303D_WriteReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_Act_DUR, act_dur) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/**********ACCELEROMETER: STATUS AND OUTPUT REGISTERS***********/

/***********STATUS_A***********/
/*******************************************************************************
* Function Name  : LSM303D_GetAccStatusReg
* Description    : Read the Accelerometer status register
* Input          : char to empity by Status Reg Value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_GetAccStatusReg(u8_t* val) {
  if( !ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_STATUS_A, val) )
      return MEMS_ERROR;
  
  return MEMS_SUCCESS;  
}

/*******************************************************************************
* Function Name  : LSM303D_GetAccStatusBit
* Description    : Read the Accelerometer status register BIT
* Input          : LSM303D_STATUS_ZYXAOR, LSM303D_STATUS_ZAOR, LSM303D_STATUS_YAOR, 
                   LSM303D_STATUS_XAOR, LSM303D_STATUS_ZYXADA, LSM303D_STATUS_ZADA
                   LSM303D_STATUS_YADA, LSM303D_STATUS_XADA, LSM303D_DATAREADY_BIT_M
                   val: Byte to be filled with the BIT value(0/1) 
* Output         : status register BIT
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_GetAccStatusBit(u8_t statusBIT, u8_t* val) {
  u8_t value;  
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_STATUS_A, &value) )
    return MEMS_ERROR;
  
  
  if(statusBIT == LSM303D_STATUS_ZYXAOR){
    if(value &= LSM303D_STATUS_ZYXAOR){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == LSM303D_STATUS_ZAOR){
    if(value &= LSM303D_STATUS_ZAOR){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == LSM303D_STATUS_YAOR){
    if(value &= LSM303D_STATUS_YAOR){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == LSM303D_STATUS_XAOR){
    if(value &= LSM303D_STATUS_XAOR){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == LSM303D_STATUS_ZYXADA){
    if(value &= LSM303D_STATUS_ZYXADA){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == LSM303D_STATUS_ZADA){
    if(value &= LSM303D_STATUS_ZADA){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == LSM303D_STATUS_YADA){
    if(value &= LSM303D_STATUS_YADA){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == LSM303D_STATUS_XADA){
    if(value &= LSM303D_STATUS_XADA){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  return MEMS_ERROR;
}

/***********OUT_*_*_A***********/
/*******************************************************************************
* Function Name  : LSM303D_GetAccAxesRaw
* Description    : Read the Acceleration Values Output Registers
* Input          : buffer to empity by AxesRaw_t Typedef
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_GetAccAxesRaw(AxesRaw_t* buff) {
  i16_t value;
  u8_t *valueL = (u8_t *)(&value);
  u8_t *valueH = ((u8_t *)(&value)+1);
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_OUT_X_L_A, valueL) )
    return MEMS_ERROR;
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_OUT_X_H_A, valueH) )
    return MEMS_ERROR;
  
  buff->AXIS_X = value;
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_OUT_Y_L_A, valueL) )
    return MEMS_ERROR;
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_OUT_Y_H_A, valueH) )
    return MEMS_ERROR;
  
  buff->AXIS_Y = value;
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_OUT_Z_L_A, valueL) )
    return MEMS_ERROR;
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_OUT_Z_H_A, valueH) )
    return MEMS_ERROR;
  
  buff->AXIS_Z = value;
  
  return MEMS_SUCCESS; 
}

/***********MAGNETOMETER: STATUS AND OUTPUT REGISTERS***********/

/***********STATUS_M***********/
/*******************************************************************************
* Function Name  : LSM303D_GetMagStatusReg
* Description    : Read the Magnetometer status register
* Input          : char to empity by Status Reg Value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_GetMagStatusReg(u8_t* val) {
  if( !ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_STATUS_M, val) )
      return MEMS_ERROR;
  
  return MEMS_SUCCESS;  
}

/*******************************************************************************
* Function Name  : LSM303D_GetMagStatusBit
* Description    : Read the Magnetometer status register BIT
* Input          : LSM303D_STATUS_ZYXMOR, LSM303D_STATUS_ZMOR, LSM303D_STATUS_YMOR, 
                   LSM303D_STATUS_XMOR, LSM303D_STATUS_ZYXMDA, LSM303D_STATUS_ZMDA
                   LSM303D_STATUS_YMDA, LSM303D_STATUS_XMDA, LSM303D_TEMPOR_BIT, 
                   LSM303D_TEMPDA, LSM303D_DATAREADY_BIT_M
                   val: Byte to be filled with the BIT value(0/1) 
* Output         : status register BIT
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_GetMagStatusBit(u8_t statusBIT, u8_t* val) {
  u8_t value;  
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_STATUS_M, &value) )
    return MEMS_ERROR;
  
  
  if(statusBIT == LSM303D_STATUS_ZYXMOR){
    if(value &= LSM303D_STATUS_ZYXMOR){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == LSM303D_STATUS_ZMOR){
    if(value &= LSM303D_STATUS_ZMOR){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == LSM303D_STATUS_YMOR){
    if(value &= LSM303D_STATUS_YMOR){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == LSM303D_STATUS_XMOR){
    if(value &= LSM303D_STATUS_XMOR){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == LSM303D_STATUS_ZYXMDA){
    if(value &= LSM303D_STATUS_ZYXMDA){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == LSM303D_STATUS_ZMDA){
    if(value &= LSM303D_STATUS_ZMDA){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == LSM303D_STATUS_YMDA){
    if(value &= LSM303D_STATUS_YMDA){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == LSM303D_STATUS_XMDA){
    if(value &= LSM303D_STATUS_XMDA){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  return MEMS_ERROR;
}


/***********OUT_*_*_M***********/
status_t LSM303D_GetMagAxesRaw(AxesRaw_t* buff);
/*******************************************************************************
* Function Name  : LSM303D_GetMagAxesRaw
* Description    : Read the Magnetometer Values Output Registers
* Input          : buffer to empity by AxesRaw_t Typedef
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_GetMagAxesRaw(AxesRaw_t* buff) {
  i16_t value;
  u8_t *valueL = (u8_t *)(&value);
  u8_t *valueH = ((u8_t *)(&value)+1);
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_OUT_X_L_M, valueL) )
    return MEMS_ERROR;
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_OUT_X_H_M, valueH) )
    return MEMS_ERROR;
  
  buff->AXIS_X = value;
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_OUT_Y_L_M, valueL) )
    return MEMS_ERROR;
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_OUT_Y_H_M, valueH) )
    return MEMS_ERROR;
  
  buff->AXIS_Y = value;
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_OUT_Z_L_M, valueL) )
    return MEMS_ERROR;
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_OUT_Z_H_M, valueH) )
    return MEMS_ERROR;
  
  buff->AXIS_Z = value;
  
  return MEMS_SUCCESS;  
}

/********************TEMPERATURE REGISTERS*******************/
/*******************************************************************************
* Function Name  : LSM303D_GetTempRaw
* Description    : Read the Temperature Values by TEMP_OUT Output Registers
* Input          : Buffer to empity (12 bits form left justified 16 Bit two's complement)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303D_GetTempRaw(i16_t* val) {
  i16_t value;
  u8_t *valueL = (u8_t *)(&value);
  u8_t *valueH = ((u8_t *)(&value)+1);  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_OUT_TEMP_L, valueL) )
    return MEMS_ERROR;
  
  if( !LSM303D_ReadReg(LSM303D_MEMS_I2C_ADDRESS, LSM303D_OUT_TEMP_H, valueH) )
    return MEMS_ERROR;
  
  *val = value; ///16;
  
  return MEMS_SUCCESS;  
}

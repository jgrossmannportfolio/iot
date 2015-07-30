/******************** (C) COPYRIGHT 2013 STMicroelectronics ********************
* File Name          : lis3dsh_driver.c
* Author             : MSH Application Team
* Author             : Fabio Tota
* Version            : $Revision:$
* Date               : $Date:$
* Description        : LIS3DSH driver file
*                      
* HISTORY:
* Date               |	Modification                    |	Author
* 21/01/2013         |	Initial Revision                |	Fabio Tota

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
#include "lis3dsh_driver.h"
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
u8_t ReadReg(u8_t deviceAddr, u8_t Reg, u8_t* Data) {
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
}


/*******************************************************************************
* Function Name		: WriteReg
* Description		: Generic Writing function. It must be fullfilled with either
*			: I2C or SPI writing function
* Input			: Register Address, Data to be written
* Output		: None
* Return		: None
*******************************************************************************/
u8_t WriteReg(u8_t deviceAddress, u8_t WriteAddr, u8_t Data) {
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

/*******************************************************************************
* Function Name  : GetWHO_AM_I
* Description    : Read identification code by WHO_AM_I register
* Input          : Char to empty by Device identification Value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t GetWHO_AM_I(u8_t* val){
  
  if( !ReadReg(MEMS_I2C_ADDRESS, WHO_AM_I, val) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : GetOUT_T
* Description    : Read temperature register 1LSB/deg (00h = 25degC)
* Input          : Char to empty by temperature value (8bit 2's complement)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t GetOUT_T(u8_t* val){
   if( !ReadReg(MEMS_I2C_ADDRESS, OUT_T, val) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetODR
* Description    : Sets Output Data Rate
* Input          : Output Data Rate typedef
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetODR(ODR_t ov){
  u8_t value;

  if( !ReadReg(MEMS_I2C_ADDRESS, CNTL4, &value) )
    return MEMS_ERROR;

  value &= 0x0f;
  value |= ov<<ODR_BIT;
    
    if( !WriteReg(MEMS_I2C_ADDRESS, CNTL4, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetAxis
* Description    : Enable/Disable Acc. Axis
* Input          : X_ENABLE/X_DISABLE | Y_ENABLE/Y_DISABLE | Z_ENABLE/Z_DISABLE
* Output         : None
* Note           : You MUST use all input variable in the argument
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetAxis(Axis_t axis) {
  u8_t value;
  
  if( !ReadReg(MEMS_I2C_ADDRESS, CNTL4, &value) )
    return MEMS_ERROR;
  value &= 0xF8;
  value |= (0x07 & axis);
   
  if( !WriteReg(MEMS_I2C_ADDRESS, CNTL4, value) )
    return MEMS_ERROR;   
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetFullScale
* Description    : Set FullScale by typedef definition
* Input          : FULLSCALE_2/FULLSCALE_4/FULLSCALE_8/FULLSCALE_16
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetFullScale(Fullscale_t fs) {
  u8_t value;
  
  if( !ReadReg(MEMS_I2C_ADDRESS, CNTL5, &value) )
    return MEMS_ERROR;
                  
  value &= 0xC7;	
  value |= (fs<<FSCALE);
  
  if( !WriteReg(MEMS_I2C_ADDRESS, CNTL5, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetBDU
* Description    : Enable/Disable Block Data Update Functionality
* Input          : ENABLE/DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetBDU(State_t bdu) {
  u8_t value;
  
  if( !ReadReg(MEMS_I2C_ADDRESS, CNTL4, &value) )
    return MEMS_ERROR;
 
  value &= 0xF7;
  value |= (bdu<<LIS3DSH_BDU);

  if( !WriteReg(MEMS_I2C_ADDRESS, CNTL4, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetSelfTest
* Description    : Set Self Test Modality
* Input          : SELF_TEST_NORMAL/SELF_TEST_POSITIVE/SELF_TEST_NEGATIVE...
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetSelfTest(SelfTest_t st) {
  u8_t value;
  
  if( !ReadReg(MEMS_I2C_ADDRESS, CNTL5, &value) )
    return MEMS_ERROR;
                  
  value &= 0xF9;
  value |= (st<<ST);
  
  if( !WriteReg(MEMS_I2C_ADDRESS, CNTL5, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : BandWidth
* Description    : Set BandWidth filter by typedef definition
* Input          : BANDWIDTH_1/BANDWIDTH_2/BANDWIDTH_3...
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t BandWidth(BandWidth_t bw) {
  u8_t value;
  
  bw &= 0x03; 
  if( !ReadReg(MEMS_I2C_ADDRESS, CNTL5, &value) )
    return MEMS_ERROR;
                  
  value &= 0x3F;
  value |= (bw<<BW);
  
  if( !WriteReg(MEMS_I2C_ADDRESS, CNTL5, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : Int1Enable
* Description    : Set Interrupt1 Enable-Disable
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t Int1Enable(State_t conf) {
  u8_t value;
  
  if( !ReadReg(MEMS_I2C_ADDRESS, CNTL3, &value) )
    return MEMS_ERROR;
                  
  value &= 0xF7;
  value |= (conf<<INT1_EN);
  
  if( !WriteReg(MEMS_I2C_ADDRESS, CNTL3, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : Int2Enable
* Description    : Set Interrupt2 Enable-Disable
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t Int2Enable(State_t conf) {
  u8_t value;
  
  if( !ReadReg(MEMS_I2C_ADDRESS, CNTL3, &value) )
    return MEMS_ERROR;
                  
  value &= 0xEF;
  value |= (conf<<INT2_EN);
  
  if( !WriteReg(MEMS_I2C_ADDRESS, CNTL3, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

                 
/*******************************************************************************
* Function Name  : IntLatchEnable
* Description    : Enable Interrupt Latching function
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t IntLatchEnable(State_t latch) {
  u8_t value;
  
  if( !ReadReg(MEMS_I2C_ADDRESS, CNTL3, &value) )
    return MEMS_ERROR;
                  
  value &= 0xDF;
  value |= (latch<<IEL);
  
  if( !WriteReg(MEMS_I2C_ADDRESS, CNTL3, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : IntSignPol
* Description    : Interrupt Polarity
* Input          : POL_HIGH/POL_LOW
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t IntSignPol(Polarity_t pol) {
  u8_t value;
  
  if( !ReadReg(MEMS_I2C_ADDRESS, CNTL3, &value) )
    return MEMS_ERROR;
                  
  value &= 0xDF;
  value |= (pol<<IEA);
  
  if( !WriteReg(MEMS_I2C_ADDRESS, CNTL3, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : DataReadyInt
* Description    : Data ready connect to interrupt 1 Enable
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t DataReadyInt(State_t drdy) {
  u8_t value;
  
  if( !ReadReg(MEMS_I2C_ADDRESS, CNTL3, &value) )
    return MEMS_ERROR;
                  
  value &= 0x7F;
  value |= (drdy<<DR_EN);
  
  if( !WriteReg(MEMS_I2C_ADDRESS, CNTL3, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : ReBootEnable
* Description    : Force Reboot
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t ReBootEnable(State_t boot) {
  u8_t value;
  
  if( !ReadReg(MEMS_I2C_ADDRESS, CNTL6, &value) )
    return MEMS_ERROR;
                  
  value &= 0x7F;
  value |= (boot<<BOOT);
  
  if( !WriteReg(MEMS_I2C_ADDRESS, CNTL6, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : FIFOEnable
* Description    : FIFO enable/disable
* Input          : MEMS_ENABLE/MEMS_DISABLE, n max sample in FIFO (must be < 30)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t FIFOEnable(State_t fifo, u8_t nMax) {
  u8_t value;
  
  //check n max of sample in FIFO
  if(nMax > 30) return MEMS_ERROR;
  
  //only stream mode fifo
  if(! FIFOMode(FIFO_STREAM_MODE))
    return MEMS_ERROR;  
 
  //set WTM > n sample in FIFO 
  if(! SetWaterMark(nMax))
    return MEMS_ERROR;
  
  if( !ReadReg(MEMS_I2C_ADDRESS, CNTL6, &value) )
    return MEMS_ERROR;
                  
  value &= 0xBF;
  value |= (fifo<<FIFO_EN);
  
  if( !WriteReg(MEMS_I2C_ADDRESS, CNTL6, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : FIFOMode
* Description    : Sets FIFO Modality
* Input          : FIFO_BYPASS_MODE, FIFO_MODE...
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t FIFOMode(FifoMode_t fm) {
  u8_t value;           
  
  if( !ReadReg(MEMS_I2C_ADDRESS, FIFO_CTRL, &value) )
      return MEMS_ERROR;
    
  value &= 0x1f;
  value |= (fm<<FMODE);                  
    
  if( !WriteReg(MEMS_I2C_ADDRESS, FIFO_CTRL, value) )
      return MEMS_ERROR;
    
  return MEMS_SUCCESS;
}       


/*******************************************************************************
* Function Name  : SetWaterMark
* Description    : Sets Watermark Value
* Input          : Watermark = [0,31]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetWaterMark(u8_t wtm) {
  u8_t value;
  
  if(wtm > 31)
    return MEMS_ERROR;  
  
  if( !ReadReg(MEMS_I2C_ADDRESS, FIFO_CTRL, &value) )
    return MEMS_ERROR;
                  
  value &= 0xE0;
  value |= wtm; 
  
  if( !WriteReg(MEMS_I2C_ADDRESS, FIFO_CTRL, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : AddIncEnable
* Description    : Register address increment (during multiple byte access) enable/disable
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AddIncEnable(State_t addinc) {
  u8_t value;
  
  if( !ReadReg(MEMS_I2C_ADDRESS, CNTL6, &value) )
    return MEMS_ERROR;
                  
  value &= 0xEF;
  value |= (addinc<<ADD_INC);
  
  if( !WriteReg(MEMS_I2C_ADDRESS, CNTL6, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : FifoEmptyInt1
* Description    : FIFO empty indication on INT1 enable/disable
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t FifoEmptyInt1(State_t empty) {
  u8_t value;
  
  if( !ReadReg(MEMS_I2C_ADDRESS, CNTL6, &value) )
    return MEMS_ERROR;
                  
  value &= 0xF7;
  value |= (empty<<I1_EMPTY);
  
  if( !WriteReg(MEMS_I2C_ADDRESS, CNTL6, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : FifoOvrInt1
* Description    : FIFO Overrun interrupt on INT1 enable/disable
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t FifoOvrInt1(State_t overrun) {
  u8_t value;
  
  if( !ReadReg(MEMS_I2C_ADDRESS, CNTL6, &value) )
    return MEMS_ERROR;
                  
  value &= 0xFD;
  value |= (overrun<<I1_OVERRUN);
  
  if( !WriteReg(MEMS_I2C_ADDRESS, CNTL6, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : BootInt2
* Description    : Boot Interrupt on INT2 enable/disable
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t BootInt2(State_t booti2) {
  u8_t value;
  
  if( !ReadReg(MEMS_I2C_ADDRESS, CNTL6, &value) )
    return MEMS_ERROR;
                  
  value &= 0xFE;
  value |= (booti2<<I2_BOOT);
  
  if( !WriteReg(MEMS_I2C_ADDRESS, CNTL6, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : VectFiltEnable
* Description    : Vector Filter Enable-Disable
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t VectFiltEnable(State_t vfe) {
  u8_t value;
  
  if( !ReadReg(MEMS_I2C_ADDRESS, CNTL3, &value) )
    return MEMS_ERROR;
                  
  value &= 0x7F;
  value |= (vfe<<VFILT);
  
  if( !WriteReg(MEMS_I2C_ADDRESS, CNTL3, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SoftReset
* Description    : Soft Reset BIT Enable-Disable
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SoftReset(State_t strt) {
  u8_t value;
  
  if( !ReadReg(MEMS_I2C_ADDRESS, CNTL3, &value) )
    return MEMS_ERROR;
                  
  value &= 0xFE;
  value |= (strt<<STRT);
  
  if( !WriteReg(MEMS_I2C_ADDRESS, CNTL3, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}    


/*******************************************************************************
* Function Name  : SetOFFSET
* Description    : Set offset Value
* Input          : AXIS: SET_AXIS_X/SET_AXSIS_Y/SET_AXIS_Z, Offest value = [0,255]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetOFFSET(SET_AXIS_t axis, u8_t val) {
  u8_t reg=0;
  
  if(!((axis==SET_AXIS_X)||(axis==SET_AXIS_Y)||(axis==SET_AXIS_Z)))
    return MEMS_ERROR;
  
  switch(axis){
  	case SET_AXIS_X: reg=OFF_X; break;
  	case SET_AXIS_Y: reg=OFF_Y; break;
  	case SET_AXIS_Z: reg=OFF_Z; break;
  }  
  
  if( !WriteReg(MEMS_I2C_ADDRESS, reg, val) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetCS
* Description    : Set Constant Shift Value
* Input          : AXIS: SET_AXIS_X/SET_AXSIS_Y/SET_AXIS_Z, Constant shift value = [0,255]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetCS(SET_AXIS_t axis, u8_t val) {
  u8_t reg=0;
  
  if(!((axis==SET_AXIS_X)||(axis==SET_AXIS_Y)||(axis==SET_AXIS_Z)))
    return MEMS_ERROR;
  
  switch(axis){
  	case SET_AXIS_X: reg=CS_X; break;
  	case SET_AXIS_Y: reg=CS_Y; break;
  	case SET_AXIS_Z: reg=CS_Z; break;
  }  
  
  if( !WriteReg(MEMS_I2C_ADDRESS, reg, val) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetLC
* Description    : Set Long Counter Register Value
* Input          : Long Counter 16Bit value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetLC(u16_t val) {
  u8_t val_L=0;
  u8_t val_H=0;
  
  val_L = (u8_t) (val & 0x00FF);
  val_H = (u8_t) ((val & 0xFF00)>>8);
  
  if( !WriteReg(MEMS_I2C_ADDRESS, LC_L, val_L) )
    return MEMS_ERROR;
    
  if( !WriteReg(MEMS_I2C_ADDRESS, LC_H, val_H) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : GetLC
* Description    : Get Long Counter Register Value
* Input          : 16Bit Variable to empty by Counter value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t GetLC(i16_t* val) {
  u8_t val_L=0;
  u8_t val_H=0;
 
  if( !ReadReg(MEMS_I2C_ADDRESS, LC_L, &val_L) )
    return MEMS_ERROR;
    
  if( !ReadReg(MEMS_I2C_ADDRESS, LC_H, &val_H) )
    return MEMS_ERROR;
 
  *val = (i16_t)((val_H<<8) + val_L);
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetVectorCoeff
* Description    : Set Vector Coefficient Value for Differential filter
* Input          : SET_VFC_1/SET_VFC_2/SET_VFC_3, Coefficient value = [0,255]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetVectorCoeff(SET_VFC_t vfc, u8_t val) {
  u8_t reg=0;
  
  if(!((vfc==SET_VFC_1)||(vfc==SET_VFC_2)||(vfc==SET_VFC_3)||(vfc==SET_VFC_4)))
    return MEMS_ERROR;
  
  switch(vfc){
  	case SET_VFC_1: reg=VFC_1; break;
  	case SET_VFC_2: reg=VFC_2; break;
 	case SET_VFC_3: reg=VFC_3; break;
  	case SET_VFC_4: reg=VFC_4; break;
  }  
  
  if( !WriteReg(MEMS_I2C_ADDRESS, reg, val) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : GetVectorCoeff
* Description    : Get Vector Coefficient Value for Differential filter
* Input          : SET_VFC_1/SET_VFC_2/SET_VFC_3, variable to empty
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t GetVectorCoeff(SET_VFC_t vfc, u8_t* val) {
  u8_t reg;
  
  if(!((vfc==SET_VFC_1)||(vfc==SET_VFC_2)||(vfc==SET_VFC_3)||(vfc==SET_VFC_4)))
    return MEMS_ERROR;
  
  switch(vfc){
  case SET_VFC_1: reg = VFC_1; break;
  case SET_VFC_2: reg = VFC_2; break;
  case SET_VFC_3: reg = VFC_3; break;
  case SET_VFC_4: reg = VFC_4; break;
  }  
  
  if( !ReadReg(MEMS_I2C_ADDRESS, reg, val) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

   
/*******************************************************************************
* Function Name  : SetDebugMode
* Description    : Set Debug Mode
* Input          : State of debug mode (MEMS_ENABLE/MEMS_DISABLE)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
****************************************************************************** */
status_t SetDebugMode(State_t state) {
	/*
  u8_t value;
  
  if( !ReadReg(MEMS_I2C_ADDRESS, CNTL1, &value) )
    return MEMS_ERROR;

  value &= 0xEF;
  value |= (state<<DEBUG_MODE); 
  
  if( !WriteReg(MEMS_I2C_ADDRESS, CNTL1, value) )
  */
    return MEMS_ERROR;
  /*
  return MEMS_SUCCESS;
  */
}                          


/*******************************************************************************
* Function Name  : SetDebugXYZ
* Description    : set x,y,z value for debug mode
* Input          : Value of x,y,z debug (8bit)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetDebugXYZ(u8_t x_debug, u8_t y_debug, u8_t z_debug) {
  
  if( !WriteReg(MEMS_I2C_ADDRESS, X_DEBUG, x_debug) )
    return MEMS_ERROR;
  
  if( !WriteReg(MEMS_I2C_ADDRESS, Y_DEBUG, y_debug) )
    return MEMS_ERROR;
    
  if( !WriteReg(MEMS_I2C_ADDRESS, Z_DEBUG, z_debug) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}                          


/*******************************************************************************
* Function Name  : SetThrs3
* Description    : set Threshold3 Coefficient Value
* Input          : Value of threshold [0,255]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetThrs3(u8_t val) {
  
  if( !WriteReg(MEMS_I2C_ADDRESS, THRS3, val) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : GetThrs3
* Description    : Get Threshold3 Coefficient Value
* Input          : Variable to empty
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t GetThrs3(u8_t* val) {
  
  if( !ReadReg(MEMS_I2C_ADDRESS, THRS3, val) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : GetThrsSM
* Description    : Get Threshold 1 or 2 by SM1 or SM2
* Input          : SM1/SM2, THRS1/THRS2, Variable to empty
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t GetThrsSM(SM_t sm, THRS_t thrs, u8_t* val) {
  u8_t reg=0;
  
  switch(thrs){
  case THRS_1:  if(sm==SM1) 	reg = THRS1_1; 
  		else 		reg = THRS1_2; 
		break;
  case THRS_2:  if(sm==SM1) 	reg = THRS2_1; 
  		else 		reg = THRS2_2; 
		break;
  }  
  
  if( !ReadReg(MEMS_I2C_ADDRESS, reg, val) )
    return MEMS_ERROR;
   
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetThrsSM
* Description    : Set Threshold 1 or 2 for SM1 or SM2
* Input          : SM1/SM2, THRS1/THRS2, Threshold Value [0,255]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetThrsSM(SM_t sm, THRS_t thrs, u8_t val) {
  u8_t reg=0;
  
  switch(thrs){
  case THRS_1:  if(sm==SM1) reg = THRS1_1; 
  		else reg = THRS1_2; 
		break;
  case THRS_2:  if(sm==SM1) reg = THRS2_1; 
  		else reg = THRS2_2; 
		break;
  }  
  
  if( !WriteReg(MEMS_I2C_ADDRESS, reg, val) )
    return MEMS_ERROR;
   
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetTimerSM
* Description    : Set Timer 1(16bit),2(16bit),3(8bit),4(8bit) for SM1 or SM2
* Input          : SM1/SM2, TIM1/TIM2..., Timer Value (8bit or 16bit)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetTimerSM(SM_t sm, TIM_t timer, u16_t val) {
  u8_t reg=0;
  u8_t val_L=0;
  u8_t val_H=0;
  
  switch(timer){
  case TIM_1:  if(sm==SM1) 	reg = TIM1_1_L; 
  		else 		reg = TIM1_2_L; 
		break;
  case TIM_2:  if(sm==SM1) 	reg = TIM2_1_L; 
  		else 		reg = TIM2_2_L; 
		break;
  case TIM_3:  if(sm==SM1) 	reg = TIM3_1; 
  		else 		reg = TIM3_2; 
		break;
  case TIM_4:  if(sm==SM1) 	reg = TIM4_1; 
  		else 		reg = TIM4_2; 
		break;		
  } 
  //for 8bit register
  if((timer==TIM_3)||(timer==TIM_4)){
    val_L = (u8_t) val;
    if( !WriteReg(MEMS_I2C_ADDRESS, reg, val_L) )
    	return MEMS_ERROR;
  }
  //for 16bit register
  else{
  val_L = (u8_t)  val;
  val_H = (u8_t) (val>>8);  
    
  if( !WriteReg(MEMS_I2C_ADDRESS, reg, val_L) )
    return MEMS_ERROR;
  if( !WriteReg(MEMS_I2C_ADDRESS, reg+1, val_H) )
    return MEMS_ERROR;   
  }
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetMaskSM
* Description    : Set Mask A or B for SM1 or SM2
* Input          : SM1/SM2, MASK_A/MASK_B, Mask Value [0,255]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetMaskSM(SM_t sm, MASK_t mask, u8_t val) {
  u8_t reg=0;
  
  switch(mask){
  case MASK_A:  if(sm==SM1) 	reg = MASKA_1; 
  		else 		reg = MASKA_2; 
		break;
  case MASK_B:  if(sm==SM1) 	reg = MASKB_1; 
  		else 		reg = MASKB_2; 
		break;
  }  
  
  if( !WriteReg(MEMS_I2C_ADDRESS, reg, val) )
    return MEMS_ERROR;
   
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : GetProgPointSM
* Description    : Get Program pointer for SM1 or SM2
* Input          : Byte to empty by Program pointer value (4bit)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t GetProgPointSM(SM_t sm, u8_t* val) {
  u8_t reg=0; 
  
  switch(sm){
  case SM1 : reg = PR1; break;
  case SM2 : reg = PR2; break;
  }
  
  if( !ReadReg(MEMS_I2C_ADDRESS, reg, val) )
    return MEMS_ERROR;
 
  *val = (*val & 0xF0) >> 4;
  
  return MEMS_SUCCESS;
} 


/*******************************************************************************
* Function Name  : GetResetPointSM
* Description    : Get Reset pointer for SM1 or SM2
* Input          : Byte to empty by Reset pointer value (4bit)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t GetResetPointSM(SM_t sm, u8_t* val) {
  u8_t reg=0; 
  
  switch(sm){
  case SM1 : reg = PR1; break;
  case SM2 : reg = PR2; break;
  }
  
  if( !ReadReg(MEMS_I2C_ADDRESS, reg, val) )
    return MEMS_ERROR;
 
  *val = (*val & 0x0F);
  
  return MEMS_SUCCESS;
} 


/*******************************************************************************
* Function Name  : GetTCSM
* Description    : Get 16bit general Timer Value for SM1 or SM2
* Input          : SM1/SM2, 16bit Variable to empty by timer value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t GetTCSM(SM_t sm, u16_t* val) {
  u8_t val_L=0;
  u8_t val_H=0;
  u8_t reg=0;
  
  switch(sm){
  case SM1: reg = TC1_L;  break;
  case SM2: reg = TC2_L;  break;  
  }
  
  if( !ReadReg(MEMS_I2C_ADDRESS, reg, &val_L) )
    return MEMS_ERROR;
    
  if( !ReadReg(MEMS_I2C_ADDRESS, reg+1, &val_H) )
    return MEMS_ERROR;
 
  *val = (u16_t)((val_H<<8) + val_L);
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : GetOutSBitSM
* Description    : Read the output flags for interrupt by SM1 or SM2
* Input          : Out interrupt Bit to read (P_X/P_Y/N_Z/N_V....)
* Output         : None
* Return         : Status of BIT [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t GetOutSBitSM(SM_t sm, u8_t FLAG_INT_OUT) {
  u8_t value; 
  u8_t reg;
 
  switch(sm){
  case SM1: reg = OUTS1;  break;
  case SM2: reg = OUTS2;  break;  
  }
  
  if( !ReadReg(MEMS_I2C_ADDRESS, reg, &value) )
      return MEMS_ERROR;
 
  switch (FLAG_INT_OUT){
  case F_P_X : if(value & F_P_X) return MEMS_SUCCESS;
             else  		return MEMS_ERROR;
  case F_N_X : if(value & F_N_X) return MEMS_SUCCESS;
             else  		return MEMS_ERROR;
  case F_P_Y : if(value & F_P_Y) return MEMS_SUCCESS;
             else  		return MEMS_ERROR;
  case F_N_Y : if(value & F_N_Y) return MEMS_SUCCESS;
             else  		return MEMS_ERROR;
  case F_P_Z : if(value & F_P_Z) return MEMS_SUCCESS;
             else  		return MEMS_ERROR;
  case F_N_Z : if(value & F_N_Z) return MEMS_SUCCESS;
             else  		return MEMS_ERROR;
  case F_P_V : if(value & F_P_V) return MEMS_SUCCESS;
             else  		return MEMS_ERROR;
  case F_N_V : if(value & F_N_V) return MEMS_SUCCESS;
             else  		return MEMS_ERROR;     
  }
  
return MEMS_ERROR;
}
  

/*******************************************************************************
* Function Name  : GetPeakSM
* Description    : Read the Peak detection Register value by SM1 or SM2
* Input          : SM1/SM2, Variable (8bit) to empty by Peak Register Value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t GetPeakSM(SM_t sm, u8_t* val) {
  u8_t reg;
  
  switch(sm){
  case SM1: reg = PEAK1; break;
  case SM2: reg = PEAK2; break;  
  }
  
  if( !ReadReg(MEMS_I2C_ADDRESS, reg, val) )
      return MEMS_ERROR;
  
  return MEMS_SUCCESS;  
}


/*******************************************************************************
* Function Name  : GetDecimSM2
* Description    : Read the Decimator counter Register value by SM2
* Input          : Variable (8bit) to empty by Decimator counter Value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t GetDecimSM2(u8_t* val) {
  
  if( !ReadReg(MEMS_I2C_ADDRESS, DES2, val) )
      return MEMS_ERROR;
  
  return MEMS_SUCCESS;  
}


/*******************************************************************************
* Function Name  : SetIntPinSM
* Description    : Set SMx Interrupt PIN routed to INT1 or INT2
* Input          : SMx, MEMS_DISABLE/ENABLE  (MEMS_DISABLE = routed INT1)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetIntPinSM(SM_t sm, State_t state) {
  u8_t reg=0;
  u8_t value=0;
  
  switch(sm){
  case SM1:  reg = CNTL1; break;
  case SM2:  reg = CNTL2; break;
  }  
  
  if( !ReadReg(MEMS_I2C_ADDRESS, reg, &value) )
    return MEMS_ERROR;
 
  value &= 0xF7;
  value |= (state<<SM_PIN);   
  
  if( !WriteReg(MEMS_I2C_ADDRESS, reg, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetIntEnaSM
* Description    : Set SMx Interrupt Enable for SM1 or SM2
* Input          : SMx, MEMS_DISABLE/ENABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetIntEnaSM(SM_t sm, State_t state) {
  u8_t reg=0;
  u8_t value=0;
  
  switch(sm){
  case SM1:  reg = CNTL1; break;
  case SM2:  reg = CNTL2; break;
  }  
  
  if( !ReadReg(MEMS_I2C_ADDRESS, reg, &value) )
    return MEMS_ERROR;  
  
  value &= 0xFE;
  value |= (state<<SM_EN);  
  
  if( !WriteReg(MEMS_I2C_ADDRESS, reg, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetPeakDetSM
* Description    : Set SMx Peak Detection Enable
* Input          : SMx, MEMS_DISABLE/ENABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetPeakDetSM(SM_t sm, State_t state) {
  u8_t reg=0;    
  u8_t value;
  
  switch(sm){
  case SM1:  reg = SETT1; break;
  case SM2:  reg = SETT2; break;
  }  
 
  if( !ReadReg(MEMS_I2C_ADDRESS, reg, &value) )
    return MEMS_ERROR;
                  
  value &= 0x7F;
  value |= (state<<P_DET);
  
  if( !WriteReg(MEMS_I2C_ADDRESS, reg, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetThr3SaSM
* Description    : Set SMx threshold3 limit value for axis and sign mask reset (MASKB_x)
* Input          : SMx, MEMS_DISABLE/ENABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetThr3SaSM(SM_t sm, State_t state) {
  u8_t reg=0;    
  u8_t value;
  
  switch(sm){
  case SM1:  reg = SETT1; break;
  case SM2:  reg = SETT2; break;
  }  
 
  if( !ReadReg(MEMS_I2C_ADDRESS, reg, &value) )
    return MEMS_ERROR;
                  
  value &= 0xBF;
  value |= (state<<THR3_SA);
  
  if( !WriteReg(MEMS_I2C_ADDRESS, reg, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetThr3MaSM
* Description    : Set SMx threshold3 limit value for axis and sign mask reset (MASKA_x)
* Input          : SMx, MEMS_DISABLE/ENABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetThr3MaSM(SM_t sm, State_t state) {
  u8_t reg=0;    
  u8_t value;
  
  switch(sm){
  case SM1:  reg = SETT1; break;
  case SM2:  reg = SETT2; break;
  }  
 
  if( !ReadReg(MEMS_I2C_ADDRESS, reg, &value) )
    return MEMS_ERROR;
                  
  value &= 0xFB;
  value |= (state<<THR3_MA);
  
  if( !WriteReg(MEMS_I2C_ADDRESS, reg, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetAbsSM
* Description    : Set SMx absolute value enable
* Input          : SMx, MEMS_DISABLE/ENABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetAbsSM(SM_t sm, State_t state) {
  u8_t reg=0;    
  u8_t value;
  
  switch(sm){
  case SM1:  reg = SETT1; break;
  case SM2:  reg = SETT2; break;
  }  
 
  if( !ReadReg(MEMS_I2C_ADDRESS, reg, &value) )
    return MEMS_ERROR;
                  
  value &= 0xDF;
  value |= (state<<ABS);
  
  if( !WriteReg(MEMS_I2C_ADDRESS, reg, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetRTamSM
* Description    : Set SMx next condition validation flag
* Input          : SMx, MEMS_DISABLE/ENABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetRTamSM(SM_t sm, State_t state) {
  u8_t reg=0;    
  u8_t value;
  
  switch(sm){
  case SM1:  reg = SETT1; break;
  case SM2:  reg = SETT2; break;
  }  
 
  if( !ReadReg(MEMS_I2C_ADDRESS, reg, &value) )
    return MEMS_ERROR;
                  
  value &= 0xFD;
  value |= (state<<R_TAM);
  
  if( !WriteReg(MEMS_I2C_ADDRESS, reg, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetSitrSM
* Description    : Set SMx program flow can be modified by STOP and COUNT
* Input          : SMx, MEMS_DISABLE/ENABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetSitrSM(SM_t sm, State_t state) {
  u8_t reg=0;    
  u8_t value;
  
  switch(sm){
  case SM1:  reg = SETT1; break;
  case SM2:  reg = SETT2; break;
  }  
 
  if( !ReadReg(MEMS_I2C_ADDRESS, reg, &value) )
    return MEMS_ERROR;
                  
  value &= 0xFE;
  value |= (state<<SITR);
  
  if( !WriteReg(MEMS_I2C_ADDRESS, reg, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;

}


/*******************************************************************************
* Function Name  : SetHystSM 
* Description    : Set Hysteresis for SM1 or SM2
* Input          : SM1/SM2, Hysteresis Value [0,7] (3bit)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetHystSM(SM_t  sm, u8_t val) {
  u8_t reg=0;
  u8_t read=0;
  
  switch(sm){
  case SM1:  reg = CNTL1; break;
  case SM2:  reg = CNTL2; break;
  }  
  
  if( !ReadReg(MEMS_I2C_ADDRESS, reg, &read) )
    return MEMS_ERROR;
  
  read &= 0x1F;
  read |= (val<<HYST);
  
  if( !WriteReg(MEMS_I2C_ADDRESS, reg, read) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : GetSatusReg
* Description    : Read the status register
* Input          : char to empty by Status Reg Value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t GetSatusReg(u8_t* val) {
 
  if( !ReadReg(MEMS_I2C_ADDRESS, STATUS, val) )
      return MEMS_ERROR;
  
  return MEMS_SUCCESS;  
}

      
/*******************************************************************************
* Function Name  : GetSatusBIT
* Description    : Read the status register BIT
* Input          : STATUS_REG_ZYXOR, STATUS_REG_ZOR, STATUS_REG_YOR, STATUS_REG_XOR,
                   STATUS_REG_ZYXDA, STATUS_REG_ZDA, STATUS_REG_YDA, STATUS_REG_XDA, DATAREADY_BIT
* Output         : status register BIT
* Return         : Status of BIT [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t GetSatusBit(u8_t statusBIT) {
  u8_t value;  
  
  if( !ReadReg(MEMS_I2C_ADDRESS, STATUS, &value) )
      return MEMS_ERROR;
 
  switch (statusBIT){
    case STATUS_REG_ZYXOR:     if(value & STATUS_REG_ZYXOR) return MEMS_SUCCESS;
                               else  return MEMS_ERROR; 
    case STATUS_REG_ZOR:       if(value & STATUS_REG_ZOR) return MEMS_SUCCESS;
                               else  return MEMS_ERROR;
    case STATUS_REG_YOR:       if(value & STATUS_REG_YOR) return MEMS_SUCCESS;
                               else  return MEMS_ERROR;                               
    case STATUS_REG_XOR:       if(value & STATUS_REG_XOR) return MEMS_SUCCESS;
                               else  return MEMS_ERROR;   
    case STATUS_REG_ZYXDA:     if(value & STATUS_REG_ZYXDA) return MEMS_SUCCESS;
                               else  return MEMS_ERROR; 
    case STATUS_REG_ZDA:       if(value & STATUS_REG_ZDA) return MEMS_SUCCESS;
                               else  return MEMS_ERROR; 
    case STATUS_REG_YDA:       if(value & STATUS_REG_YDA) return MEMS_SUCCESS;
                               else  return MEMS_ERROR; 
    case STATUS_REG_XDA:       if(value & STATUS_REG_XDA) return MEMS_SUCCESS;
                               else  return MEMS_ERROR;                                
    
  }
return MEMS_ERROR;
}

   
/*******************************************************************************
* Function Name  : GetAccAxesRaw
* Description    : Read the Acceleration Values Output Registers
* Input          : buffer to empty by AccAxesRaw_t Typedef
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t GetAccAxesRaw(AxesRaw_t* buff) {
  u8_t valueL;
  u8_t valueH;
  
  if( !ReadReg(MEMS_I2C_ADDRESS, OUT_X_L, &valueL) )
      return MEMS_ERROR;
  
  if( !ReadReg(MEMS_I2C_ADDRESS, OUT_X_H, &valueH) )
      return MEMS_ERROR;
  
  buff->AXIS_X = (i16_t)( (valueH << 8) | valueL );//16;
  
  if( !ReadReg(MEMS_I2C_ADDRESS, OUT_Y_L, &valueL) )
      return MEMS_ERROR;
  
  if( !ReadReg(MEMS_I2C_ADDRESS, OUT_Y_H, &valueH) )
      return MEMS_ERROR;
  
  buff->AXIS_Y = (i16_t)( (valueH << 8) | valueL );//16;
  
   if( !ReadReg(MEMS_I2C_ADDRESS, OUT_Z_L, &valueL) )
      return MEMS_ERROR;
  
  if( !ReadReg(MEMS_I2C_ADDRESS, OUT_Z_H, &valueH) )
      return MEMS_ERROR;
  
  buff->AXIS_Z = (i16_t)( (valueH << 8) | valueL );//16;
  
  return MEMS_SUCCESS;  
}


/*******************************************************************************
* Function Name  : GetStatBIT
* Description    : Read single BIT status of STAT register
* Input          : Stat BIT Mask Flag (F_LONG,F_SYNC1...)
* Output         : None
* Return         : Status of BIT [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t GetStatBIT(u8_t StatBITMask) {
  u8_t value;
  
  if( !ReadReg(MEMS_I2C_ADDRESS, STAT, &value) )
    return MEMS_ERROR;
 
  if(value & StatBITMask)    return MEMS_SUCCESS;
    return MEMS_ERROR; 
}  


/*******************************************************************************
* Function Name  : GetFifoSourceReg
* Description    : Read Fifo source Register
* Input          : Byte to empty by FIFO source register value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t GetFifoSourceReg(u8_t* val) {
  
  if( !ReadReg(MEMS_I2C_ADDRESS, FIFO_SRC, val) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : GetFifoSourceBit
* Description    : Read Fifo WaterMark source bit
* Input          : FIFO_WTM_S, FIFO_EMPTY_S, FIFO_EMPTY_S...
* Output         : None
* Return         : Status of BIT [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t GetFifoSourceBit(u8_t statusBIT){
  u8_t value;  
  
  if( !ReadReg(MEMS_I2C_ADDRESS, FIFO_SRC, &value) )
      return MEMS_ERROR;
 
  if(statusBIT == FIFO_WTM_S){
    if(value & FIFO_WTM_S)     return MEMS_SUCCESS;
    else  return MEMS_ERROR;  
  }
  
  if(statusBIT == FIFO_OVRN_S){
    if(value & FIFO_OVRN_S)   return MEMS_SUCCESS;
    else  return MEMS_ERROR;  
  }

  if(statusBIT == FIFO_EMPTY_S){
    if(value & FIFO_EMPTY_S)   return MEMS_SUCCESS;
    else  return MEMS_ERROR;  
  }
  
return MEMS_ERROR;
}


/*******************************************************************************
* Function Name  : GetFifoSourceFSS
* Description    : Read Fifo source Data Stored
* Input          : Byte to empty by FIFO source Data Stored value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t GetFifoSourceFSS(u8_t* val) {
  
  if( !ReadReg(MEMS_I2C_ADDRESS, FIFO_SRC, val) )
    return MEMS_ERROR;
 
  *val &= 0x1F;
  
  return MEMS_SUCCESS;
}     


/*******************************************************************************
* Function Name  : ReadFifoData
* Description    : Read all Fifo Data stored
* Input          : AccAxesRaw_t Buffer to empty by FIFO Data Stored value, Byte to empty by depth of FIFO
* Note		 : Must call this function every [nMax sample * ODR] seconds max (or more fastly)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t ReadFifoData(AxesRaw_t* FifoBuff, u8_t* depth) {
  u8_t val=0;
  u8_t i=0;
  i8_t j=0;
  AxesRaw_t data;
  
  
  if(! GetFifoSourceFSS(&val))  //read FSS fifo value
  return MEMS_ERROR;

  if(val<1) return MEMS_ERROR; //there aren't fifo value

  //read n data from FIFO
  for(j=val;j>=0;j--){
        GetAccAxesRaw(&data);
        FifoBuff[i].AXIS_X = data.AXIS_X;
	FifoBuff[i].AXIS_Y = data.AXIS_Y;
	FifoBuff[i].AXIS_Z = data.AXIS_Z;
	i++;
	}
  
  *depth = val;
  
  return MEMS_SUCCESS;
}   


/*******************************************************************************
* Function Name  : SetSPIInterface
* Description    : Set SPI mode: 3 Wire Interface OR 4 Wire Interface
* Input          : SPI_3_WIRE, SPI_4_WIRE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetSPIInterface(SPIMode_t spi) {
  u8_t value;
  
  if( !ReadReg(MEMS_I2C_ADDRESS, CNTL5, &value) )
    return MEMS_ERROR;
                  
  value &= 0xFE;
  value |= spi<<SIM;
  
  if( !WriteReg(MEMS_I2C_ADDRESS, CNTL5, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetSMCodeReg
* Description    : Set single SMx Code Register byte
* Input          : Code Address, Code (Byte)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetSMCodeReg(u8_t CodeADD, u8_t CodeByte) {
 
  //check correct address
  if(! (((CodeADD >= 0x40)&&(CodeADD <= 0x4F)) || ((CodeADD >= 0x60)&&(CodeADD <= 0x6F))) )
	return MEMS_ERROR;
  
    if( !WriteReg(MEMS_I2C_ADDRESS, CodeADD, CodeByte) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetSMBufferCodeReg
* Description    : Set All SMx Code Registers by Buffer input
* Input          : SMx, Code Buffer[16]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetSMBufferCodeReg(SM_t sm, u8_t* CodeBuff) {
  u8_t reg=0;
  u8_t i=0;
  
  switch(sm){
  case SM1:  reg = ST1_1; break;
  case SM2:  reg = ST1_2; break;
  }  
   
  for(i=0;i<16;i++){
    if( !WriteReg(MEMS_I2C_ADDRESS, reg+i, CodeBuff[i]) )
    return MEMS_ERROR;
  }
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : GetSMCodeRegister
* Description    : Get single code register number of SMx
* Input          : SMx, Register Number [1,16], variable to empty
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t GetSMCodeRegister(SM_t sm, u8_t RegNumber, u8_t* val) {
  u8_t reg=0;
 
  if((RegNumber==0)||(RegNumber>16))    return MEMS_ERROR;
  
  switch(sm){
  case SM1:  reg = ST1_1; break;
  case SM2:  reg = ST1_2; break;
  }  
   
    if( !ReadReg(MEMS_I2C_ADDRESS, reg + RegNumber-1, val) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

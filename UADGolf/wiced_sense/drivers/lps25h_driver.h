/**
******************************************************************************
* @author  HESA Application Team
* @version 1.0.0
* @date    07/04/2014
* @brief   LPS25H driver header file

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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LPS25H_DRIVER__H
#define __LPS25H_DRIVER__H

#include <stdint.h>

/* Uncomment the line below to expanse the "assert_param" macro in the  drivers code */
// #define USE_FULL_ASSERT_LPS25H

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT_LPS25H

/**
* @brief  The assert_param macro is used for function's parameters check.
* @param  expr: If expr is false, it calls assert_failed function which reports 
*         the name of the source file and the source line number of the call 
*         that failed. If expr is true, it returns no value.
* @retval None
*/
#define LPS25H_assert_param(expr) ((expr) ? (void)0 : LPS25H_assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
void LPS25H_assert_failed(uint8_t* file, uint32_t line);
#else
#define LPS25H_assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT_LPS25H */


#ifdef __cplusplus
extern "C" {
#endif
  
  /** @addtogroup Environmental_Sensor
  * @{
  */
  
  /** @addtogroup LPS25H_DRIVER
  * @{
  */
  
  /* Exported Types -------------------------------------------------------------*/
  /** @defgroup LPS25H_Exported_Types
  * @{
  */
  
  /**
  * @brief  Error type. 
  */
  typedef enum {LPS25H_OK = (uint8_t)0, LPS25H_ERROR = !LPS25H_OK} LPS25H_Error_et;
  
  /**
  * @brief  Enable/Disable type. 
  */
  typedef enum {LPS25H_DISABLE = (uint8_t)0, LPS25H_ENABLE = !LPS25H_DISABLE} LPS25H_State_et; 
#define IS_LPS25H_State(MODE) ((MODE == LPS25H_ENABLE) || (MODE == LPS25H_DISABLE) ) 
  
  /**
  * @brief  Bit status type. 
  */
  typedef enum {LPS25H_RESET = (uint8_t)0, LPS25H_SET = !LPS25H_RESET} LPS25H_BitStatus_et; 
#define IS_LPS25H_BitStatus(MODE) ((MODE == LPS25H_RESET) || (MODE == LPS25H_SET))
  
  /**
  * @brief  Pressure average. 
  */
  typedef enum {
    LPS25H_AVGP_8          = (uint8_t)0x00,         /*!< Internal average on 8 samples */
    LPS25H_AVGP_32         = (uint8_t)0x01,         /*!< Internal average on 32 samples */
    LPS25H_AVGP_128        = (uint8_t)0x02,         /*!< Internal average on 128 samples */
    LPS25H_AVGP_512        = (uint8_t)0x03          /*!< Internal average on 512 sample */
  }LPS25H_Avgp_et;
#define IS_LPS25H_AVGP(AVGP) ((AVGP == LPS25H_AVGP_8) || (AVGP == LPS25H_AVGP_32) || \
  (AVGP == LPS25H_AVGP_128) || (AVGP == LPS25H_AVGP_512))

/**
* @brief  Temperature average. 
*/
typedef enum {
  LPS25H_AVGT_8          = (uint8_t)0x00,        /*!< Internal average on 8 samples */
  LPS25H_AVGT_16         = (uint8_t)0x04,        /*!< Internal average on 16 samples */
  LPS25H_AVGT_32         = (uint8_t)0x08,        /*!< Internal average on 32 samples */
  LPS25H_AVGT_64         = (uint8_t)0x0C         /*!< Internal average on 64 samples */
}LPS25H_Avgt_et;

#define IS_LPS25H_AVGT(AVGT) ((AVGT == LPS25H_AVGT_8) || (AVGT == LPS25H_AVGT_16) || \
(AVGT == LPS25H_AVGT_32) || (AVGT == LPS25H_AVGT_64))

/**
* @brief  Output data rate configuration.
*/
typedef enum {  
  
  LPS25H_ODR_ONE_SHOT  = (uint8_t)0x00,         /*!< Output Data Rate: one shot */
  LPS25H_ODR_1HZ       = (uint8_t)0x10,         /*!< Output Data Rate: 1Hz */
  LPS25H_ODR_7HZ       = (uint8_t)0x20,         /*!< Output Data Rate: 7Hz */
  LPS25H_ODR_12_5HZ    = (uint8_t)0x30,         /*!< Output Data Rate: 12.5Hz */
  LPS25H_ODR_25HZ      = (uint8_t)0x40          /*!< Output Data Rate: 25Hz */  
} LPS25H_Odr_et;

#define IS_LPS25H_ODR(ODR) ((ODR == LPS25H_ODR_ONE_SHOT) || (ODR == LPS25H_ODR_1HZ) || \
(ODR == LPS25H_ODR_7HZ) || (ODR == LPS25H_ODR_12_5HZ)|| (ODR == LPS25H_ODR_25HZ))
/**
* @brief  LPS25H Spi Mode configuration.
*/
typedef enum {
  LPS25H_SPI_4_WIRE   =  (uint8_t)0x00,
  LPS25H_SPI_3_WIRE   =  (uint8_t)0x01
} LPS25H_SPIMode_et;

#define IS_LPS25H_SPIMode(MODE) ((MODE == LPS25H_SPI_4_WIRE) || (MODE == LPS25H_SPI_3_WIRE))

/**
* @brief  Block data update.
*/

typedef enum {
  LPS25H_BDU_CONTINUOUS_UPDATE     =  (uint8_t)0x00,  /*!< Data updated continuously */
  LPS25H_BDU_NO_UPDATE             =  (uint8_t)0x04   /*!< Data updated after a read operation */
} LPS25H_Bdu_et;
#define IS_LPS25H_BDUMode(MODE) ((MODE == LPS25H_BDU_CONTINUOUS_UPDATE) || (MODE == LPS25H_BDU_NO_UPDATE))

/**
* @brief  Data Signal on INT1 pad control bits.
*/
typedef enum
{
  LPS25H_DATA = (uint8_t)0x00,
  LPS25H_P_HIGH = (uint8_t)0x01,
  LPS25H_P_LOW = (uint8_t)0x02,
  LPS25H_P_LOW_HIGH = (uint8_t)0x03
}LPS25H_OutputSignalConfig_et;
#define IS_LPS25H_OutputSignal(MODE) ((MODE == LPS25H_DATA) || (MODE == LPS25H_P_HIGH)||\
(MODE == LPS25H_P_LOW) || (MODE == LPS25H_P_LOW_HIGH))

/**
* @brief  LPS25H INT1 Interrupt pins configuration.
*/
typedef enum
{
  LPS25H_DISABLE_DATA_INT = (uint8_t)0x00,
  LPS25H_DATA_READY = (uint8_t)0x01,
  LPS25H_OVR = (uint8_t)0x02,
  LPS25H_WTM = (uint8_t)0x04,
  LPS25H_EMPTY      = (uint8_t)0x08
}LPS25H_DataSignalType_et;
#define IS_LPS25H_DataSignal(MODE) ((MODE == LPS25H_DISABLE_DATA_INT) ||(MODE == LPS25H_EMPTY) || (MODE ==LPS25H_WTM)||\
(MODE == LPS25H_OVR) || (MODE == LPS25H_DATA_READY))

/**
* @brief  LPS25H Push-pull/Open Drain selection on Interrupt pads.
*/
typedef enum
{
  LPS25H_PushPull = (uint8_t)0x00, 
  LPS25H_OpenDrain  = (uint8_t)0x40 
}LPS25H_OutputType_et;
#define IS_LPS25H_OutputType(MODE) ((MODE == LPS25H_PushPull) || (MODE == LPS25H_OpenDrain))

/**
* @brief  LPS25H Interrupt Differential Configuration.
*/
typedef enum
{
  LPS25H_DISABLE_INT = (uint8_t)0x00,
  LPS25H_PHE = (uint8_t)0x01,
  LPS25H_PLE = (uint8_t)0x02,
  LPS25H_PLE_PHE = (uint8_t)0x03 
}LPS25H_InterruptDiffConfig_et;
#define IS_LPS25H_InterruptDiff(MODE) ((MODE == LPS25H_DISABLE_INT) || (MODE ==LPS25H_PHE)||\
(MODE == LPS25H_PLE) || (MODE == LPS25H_PLE_PHE))

/**
* @brief  LPS25H Interrupt Differential Status.
*/

typedef struct
{
  uint8_t PH;          /*!< High Differential Pressure event occured */ 
  uint8_t PL;          /*!< Low Differential Pressure event occured */  
  uint8_t IA;          /*!< One or more interrupt events have been  generated.Interrupt Active */ 
}LPS25H_InterruptDiffStatus_st;


/**
* @brief  LPS25H Pressure and Temperature data status.
*/
typedef struct
{
  uint8_t TempDataAvailable;           /*!< Temperature data available bit */
  uint8_t PressDataAvailable;          /*!< Pressure data available bit */  
  uint8_t TempDataOverrun;             /*!< Temperature data over-run bit */ 
  uint8_t PressDataOverrun;            /*!< Pressure data over-run bit */  
}LPS25H_DataStatus_st;


/**
* @brief  LPS25H Fifo Mode.
*/

typedef enum {
  LPS25H_FIFO_BYPASS_MODE             			 = (uint8_t)0x00,	  /*!< The FIFO is disabled and empty. The pressure is read directly*/
  LPS25H_FIFO_MODE                           = (uint8_t)0x20,    /*!< Stops collecting data when full */
  LPS25H_FIFO_STREAM_MODE                    = (uint8_t)0x40,    /*!< Keep the newest measurements in the FIFO*/
  LPS25H_FIFO_TRIGGER_STREAMTOFIFO_MODE      = (uint8_t)0x60,    /*!< STREAM MODE until trigger deasserted, then change to FIFO MODE*/
  LPS25H_FIFO_TRIGGER_BYPASSTOSTREAM_MODE    = (uint8_t)0x80,    /*!< BYPASS MODE until trigger deasserted, then STREAM MODE*/
  LPS25H_FIFO_MEAN_MODE                      = (uint8_t)0xC0,    /*!< FIFO is used to generate a running average filtered pressure */
  LPS25H_FIFO_TRIGGER_BYPASSTOFIFO_MODE      = (uint8_t)0xE0     /*!< BYPASS mode until trigger deasserted, then FIFO MODE*/
} LPS25H_FifoMode_et;

#define IS_LPS25H_FifoMode(MODE) ((MODE == LPS25H_FIFO_BYPASS_MODE) || (MODE ==LPS25H_FIFO_MODE)||\
(MODE == LPS25H_FIFO_STREAM_MODE) || (MODE == LPS25H_FIFO_TRIGGER_STREAMTOFIFO_MODE)||\
  (MODE == LPS25H_FIFO_TRIGGER_BYPASSTOSTREAM_MODE) || (MODE == LPS25H_FIFO_MEAN_MODE)||\
    (MODE == LPS25H_FIFO_TRIGGER_BYPASSTOFIFO_MODE))


/**
* @brief  LPS25H Fifo Mean Mode Sample Size.
*/

typedef enum {
  LPS25H_FIFO_SAMPLE_2    = (uint8_t)0x01,	    /*!< 2 Fifo Mean Mode samples */
  LPS25H_FIFO_SAMPLE_4    = (uint8_t)0x03,  		/*!< 4 Fifo Mean Mode samples */
  LPS25H_FIFO_SAMPLE_8    = (uint8_t)0x07,     /*!< 8 Fifo Mean Mode samples */
  LPS25H_FIFO_SAMPLE_16   = (uint8_t)0x0F,     /*!< 16 Fifo Mean Mode samples */
  LPS25H_FIFO_SAMPLE_32   = (uint8_t)0x1F  	  /*!< 32 Fifo Mean Mode samples */
} LPS25H_FifoMeanModeSample_et;

#define IS_LPS25H_FifoMeanModeSample(MODE) ((MODE == LPS25H_FIFO_SAMPLE_2) || (MODE ==LPS25H_FIFO_SAMPLE_4)||\
(MODE == LPS25H_FIFO_SAMPLE_8) || (MODE == LPS25H_FIFO_SAMPLE_16)||\
  (MODE == LPS25H_FIFO_SAMPLE_32))
/**
* @brief  LPS25H Fifo Satus.
*/
typedef struct {
  uint8_t FIFO_LEVEL;          /*!< FIFO Stored data level t */ 
  uint8_t FIFO_EMPTY;          /*!< Empy FIFO bit.1 FIFO is empty	*/  
  uint8_t FIFO_FULL;           /*!< Overrun bit status. 1 FIFO is full */ 
  uint8_t FIFO_WTM;            /*!< Watermark Status. 1 FIFO is equal or higher then wtm level.*/ 
}LPS25H_FifoStatus_st;


/**
* @brief  LPS25H Configuration structure definition.
*/
typedef struct
{
  LPS25H_Avgp_et 				PressResolution;         	/*!< Pressure sensor resolution */
  LPS25H_Avgt_et 				TempResolution;     			/*!< Temperature sensor resolution */
  LPS25H_Odr_et  				OutputDataRate;      			/*!< Output Data Rate */
  LPS25H_Bdu_et  				BDU;                			/*!< Enable to inhibit the output registers update between the reading of upper and lower register parts.*/
  LPS25H_State_et   		AutoZero;									/* < Auto zero feature enabled.the actual pressure output is copied in the REF_P*/
  LPS25H_State_et 			Reset_AZ;   							/*!< Reset the Auto ZeroFunc. The daefualt RPDS value is copied in REF_P   */
  LPS25H_SPIMode_et 		Sim;  			 							/*!< SPI Serial Interface Mode selection */
}LPS25H_ConfigTypeDef_st;



/**
* @brief  LPS25H Interrupt structure definition .
*/
typedef struct {
  LPS25H_State_et 				INT_H_L;                /*!< Interrupt active high, low. Default value: 0 */
  LPS25H_OutputType_et 				PP_OD; 		        /*!< Push-pull/open drain selection on interrupt pads. Default value: 0 */
  LPS25H_OutputSignalConfig_et 			OutputSignal_INT1;	/*!< Data signal on INT1 Pad: Data,Pressure High, Preessure Low,P High or Low*/
  LPS25H_DataSignalType_et       		DataInterrupt_INT1;	/*!< Interrupt Pin Config:DRDY, WTM, OVERRUN, EMPTY*/
  LPS25H_InterruptDiffConfig_et 		PressureInterrupt;      /*!< Interrupt differential Configuration: PL_E and/or PH_E*/
  LPS25H_State_et				LatchIRQ;		/*!< Latch Interrupt request in to INT_SOURCE reg*/
  int16_t 					fP_threshold;		/*!< Threshold value for pressure interrupt generation*/
}LPS25H_InterruptTypeDef_st;

/**
* @brief  LPS25H FIFO structure definition.
*/
typedef struct {
  LPS25H_FifoMode_et 			FIFO_MODE;               /*!< Fifo Mode Selection */
  LPS25H_State_et			WTM_INT; 		/*!< Enable/Disable the watermark interrupt*/
  LPS25H_State_et			FIFO_MEAN_DEC;		/*!< Enable/Disable 1 Hz ODR decimation */
  LPS25H_FifoMeanModeSample_et 		MEAN_MODE_SAMPLE;	/*!< FIFO Mean Mode Sample Size*/
  uint8_t 				WTM_LEVEL;		/*!< FIFO threshold.Watermark level setting*/
}LPS25H_FIFOTypeDef_st;

#define IS_LPS25H_WtmLevel(LEVEL) ((LEVEL > 0) && (LEVEL <=31))

/**
* @brief  LPS25H Measure Type definition. 
*/  
typedef struct {
  int16_t Tout;
  int32_t Pout;
}LPS25H_MeasureTypeDef_st;


/**
* @brief  LPS25H Driver Version Info structure definition. 
*/   
typedef struct {
  uint8_t   Major;
  uint8_t   Minor;
  uint8_t Point; 
}LPS25H_DriverVersion_st; 


/**
* @brief  Bitfield positioning.
*/
#define LPS25H_BIT(x) ((uint8_t)x)

/**
* @brief  I2C address.
*/
#define LPS25H_ADDRESS  (uint8_t)0xB8

/**
* @brief  Set the LPS25H driver version.
*/
#define LPS25H_DriverVersion_Major (uint8_t)1
#define LPS25H_DriverVersion_Minor (uint8_t)1
#define LPS25H_DriverVersion_Point (uint8_t)5

/**
* @}
*/


/* Exported Constants ---------------------------------------------------------*/	 
/** @defgroup LPS25H_Exported_Constants
* @{
*/


/**
* @addtogroup LPS25H_Register
* @{
*/



/**
* @brief Device Identification register.
* \code
* Read
* Default value: 0xBD
* 7:0 This read-only register contains the device identifier that, for LPS25H, is set to BDh.
* \endcode
*/

#define LPS25H_WHO_AM_I_REG         (uint8_t)0x0F

/**
* @brief Device Identification value.
*/
#define LPS25H_WHO_AM_I_VAL         (uint8_t)0xBD


/**
* @brief Reference Pressure  Register(LSB data)
* \code
* Read/write
* Default value: 0x00
* 7:0 REFL7-0: Lower part of the reference pressure value that
*      is sum to the sensor output pressure.
* \endcode
*/
#define LPS25H_REF_P_XL_REG         (uint8_t)0x08


/**
* @brief Reference Pressure Register (Middle data)
* \code
* Read/write
* Default value: 0x00
* 7:0 REFL15-8: Middle part of the reference pressure value that
*      is sum to the sensor output pressure.
* \endcode
*/
#define LPS25H_REF_P_L_REG          (uint8_t)0x09

/**
* @brief Reference Pressure Register (MSB data)
* \code
* Read/write
* Default value: 0x00
* 7:0 REFL23-16 Higest part of the reference pressure value that
*      is sum to the sensor output pressure.
* \endcode
*/
#define LPS25H_REF_P_H_REG          (uint8_t)0x0A


/**
* @brief Pressure and temperature resolution mode Register
* \code
* Read/write
* Default value: 0x05
* 7:4 RFU
* 3:2 AVGT1-AVGT0: Select the temperature internal average.
*      AVGT1 | AVGT0 | Nr. Internal Average
*   ------------------------------------------------------
*       0    |  0    |     8 
*       0    |  1    |     16  
*       1    |  0    |     32    
*       1    |  1    |     64 

*
* 1:0 AVGP1-AVGP0: Select pressure internal average.
*      AVGP1 |  AVGP0 | Nr. Internal Average
*   ------------------------------------------------------
*      0    |   0    |      8
*      0    |   1    |      32  
*      1    |   0    |      128   
*      1    |   1    |      512
* \endcode
*/
#define LPS25H_RES_CONF_REG     (uint8_t)0x10 

#define LPS25H_AVGT_BIT         LPS25H_BIT(2)
#define LPS25H_AVGP_BIT         LPS25H_BIT(0)

#define LPS25H_AVGT_MASK        (uint8_t)0x0C
#define LPS25H_AVGP_MASK        (uint8_t)0x03

/**
* @brief Control Register 1
* \code
* Read/write
* Default value: 0x00
* 7 PD: power down control. 0 - power down mode; 1 - active mode
* 6:4 ODR2, ODR1, ODR0: output data rate selection.
*     ODR2  | ODR1  | ODR0  | Pressure output data-rate(Hz)  | Pressure output data-rate(Hz)
*   ----------------------------------------------------------------------------------
*      0    |  0    |  0    |         one shot               |         one shot 
*      0    |  0    |  1    |            1                   |            1 
*      0    |  1    |  0    |            7                   |            7     
*      0    |  1    |  1    |            12.5                |            12.5  
*      1    |  0    |  0    |            25                  |            25    
*      1    |  0    |  1    |         Reserved               |         Reserved 
*      1    |  1    |  0    |         Reserved               |         Reserved 
*      1    |  1    |  1    |         Reserved               |         Reserved
*
* 3 DIFF_EN: Interrupt circuit Enable. 0 - interrupt generation disable; 1 - interrupt generation enable
* 2 BDU: block data update. 0 - continuous update; 1 - output registers not updated until MSB and LSB reading.
* 1 RESET_AZ: Reset AutoZero Function. Reset Ref_P reg, set pressure to default value in RDPS reg.  0 - disable;1 - Reset
* 0 SIM: SPI Serial Interface Mode selection. 0 - SPI 4-wire; 1 - SPI 3-wire 
* \endcode
*/
#define LPS25H_CTRL_REG1      (uint8_t)0x20
#define LPS25H_PD_BIT         LPS25H_BIT(7)
#define LPS25H_ODR_BIT        LPS25H_BIT(4)
#define LPS25H_DIFF_EN_BIT    LPS25H_BIT(3)
#define LPS25H_BDU_BIT        LPS25H_BIT(2)
#define LPS25H_RESET_AZ_BIT   LPS25H_BIT(1)
#define LPS25H_SIM_BIT        LPS25H_BIT(0)

#define LPS25H_PD_MASK        (uint8_t)0x80
#define LPS25H_ODR_MASK       (uint8_t)0x70
#define LPS25H_DIFF_EN_MASK   (uint8_t)0x08
#define LPS25H_BDU_MASK       (uint8_t)0x04
#define LPS25H_RESET_AZ_MASK  (uint8_t)0x02
#define LPS25H_SIM_MASK       (uint8_t)0x01


/**
* @brief Control  Register 2
* \code
* Read/write
* Default value: 0x00
* 7 BOOT:  Reboot memory content. 0: normal mode; 1: reboot memory content. Self-clearing upon completation
* 6 FIFO_EN: FIFO Enable. 0: disable; 1:  enable
* 5 WTM_EN:  FIFO Watermark level use. 0: disable; 1: enable
* 4 FIFO_MEAN_DEC: Enable 1 HZ decimation 0: enable; 1: disable
* 3 I2C Enable:  0: I2C Enable; 1: SPI disable????????
* 2 SWRESET: Software reset. 0: normal mode; 1: SW reset. Self-clearing upon completation
* 1 AUTO_ZERO: Autozero enable. 0: normal mode; 1: autozero enable.
* 0 ONE_SHOT: One shot enable. 0: waiting for start of conversion; 1: start for a new dataset
* \endcode
*/
#define LPS25H_CTRL_REG2      (uint8_t)0x21
#define LPS25H_BOOT_BIT       LPS25H_BIT(7)
#define LPS25H_FIFO_EN_BIT    LPS25H_BIT(6)
#define LPS25H_WTM_EN_BIT     LPS25H_BIT(5)
#define LPS25H_FIFO_MEAN_BIT  LPS25H_BIT(4)
#define LPS25H_I2C_BIT        LPS25H_BIT(3)
#define LPS25H_SW_RESET_BIT   LPS25H_BIT(2)
#define LPS25H_AUTO_ZERO_BIT  LPS25H_BIT(1)
#define LPS25H_ONE_SHOT_BIT   LPS25H_BIT(0)

#define LPS25H_BOOT_MASK      (uint8_t)0x80
#define LPS25H_FIFO_EN_MASK   (uint8_t)0x40
#define LPS25H_WTM_EN_MASK    (uint8_t)0x20
#define LPS25H_FIFO_MEAN_MASK (uint8_t)0x10
#define LPS25H_I2C_MASK       (uint8_t)0x08
#define LPS25H_SW_RESET_MASK  (uint8_t)0x04
#define LPS25H_AUTO_ZERO_MASK (uint8_t)0x02
#define LPS25H_ONE_SHOT_MASK  (uint8_t)0x01


/**
* @brief CTRL Reg3 Interrupt Control Register
* \code
* Read/write
* Default value: 0x00
* 7 INT_H_L: Interrupt active high, low. 0:active high; 1: active low.
* 6 PP_OD: Push-Pull/OpenDrain selection on interrupt pads. 0: Push-pull; 1: open drain.
* 5:2 Reserved
* 1:0 INT1_S2, INT1_S1: data signal on INT1 pad control bits.
*    INT1_S2  | INT1_S1  | INT1 pin
*   ------------------------------------------------------
*        0       |      0      |     Data signal 
*        0       |      1      |     Pressure high (P_high) 
*        1       |      0      |     Pressure low (P_low)     
*        1       |      1      |     P_low OR P_high 
* \endcode
*/
#define LPS25H_CTRL_REG3      (uint8_t)0x22 
#define LPS25H_INT_H_L_BIT    LPS25H_BIT(7)
#define LPS25H_PP_OD_BIT      LPS25H_BIT(6)
#define LPS25H_INT1_S2_BIT    LPS25H_BIT(1)
#define LPS25H_INT1_S1_BIT    LPS25H_BIT(0)

#define LPS25H_INT_H_L_MASK   (uint8_t)0x80
#define LPS25H_PP_OD_MASK     (uint8_t)0x40
#define LPS25H_INT1_S2_MASK   (uint8_t)0x02
#define LPS25H_INT1_S1_MASK   (uint8_t)0x01

/**
* @brief CTRL Reg4 Interrupt Configuration Register 
* \code
* Read/write
* Default value: 0x00
* 7:4 Reserved: Keep these bits at 0
* 3 P1_EMPTY: Empty Signal on INT1 pin.
* 2 P1_WTM: Watermark Signal on INT1 pin.
* 1 P1_Overrunn:Overrun Signal on INT1 pin.
* 0 P1_DRDY: Data Ready Signal on INT1 pin.
* \endcode
*/
#define LPS25H_CTRL_REG4       (uint8_t)0x23  
#define LPS25H_P1_EMPTY_BIT    LPS25H_BIT(3)
#define LPS25H_P1_WTM_BIT      LPS25H_BIT(2)
#define LPS25H_P1_OVERRUN_BIT  LPS25H_BIT(1)
#define LPS25H_P1_DRDY_BIT     LPS25H_BIT(0)

#define LPS25H_P1_EMPTY_MASK    (uint8_t)0x08
#define LPS25H_P1_WTM_MASK      (uint8_t)0x04
#define LPS25H_P1_OVERRUN_MASK  (uint8_t)0x02
#define LPS25H_P1_DRDY_MASK     (uint8_t)0x01

/**
* @brief Interrupt Differential configuration Register
* \code
* Read/write
* Default value: 0x00.
* 7:3 Reserved.
* 2 LIR: Latch Interrupt request into INT_SOURCE register. 0 - interrupt request not latched; 1 - interrupt request latched
* 1 PL_E: Enable interrupt generation on differential pressure low event. 0 - disable; 1 - enable
* 0 PH_E: Enable interrupt generation on differential pressure high event. 0 - disable; 1 - enable
* \endcode
*/
#define LPS25H_INTERRUPT_CFG_REG  (uint8_t)0x24 
#define LPS25H_LIR_BIT            LPS25H_BIT(2)
#define LPS25H_PL_E_BIT           LPS25H_BIT(1)
#define LPS25H_PH_E_BIT           LPS25H_BIT(0)

#define LPS25H_LIR_MASK           (uint8_t)0x04
#define LPS25H_PL_E_MASK          (uint8_t)0x02
#define LPS25H_PH_E_MASK          (uint8_t)0x01
#define LPS25H_PE_MASK          (uint8_t)0x03


/**
* @brief Interrupt source Register (It is cleared by reading it)
* \code
* Read
* Default value: 0x00.
* 7:3 Reserved: Keep these bits at 0
* 2 IA: Interrupt Active.0: no interrupt has been generated; 1: one or more interrupt events have been generated.
* 1 PL: Differential pressure Low. 0: no interrupt has been generated; 1: Low differential pressure event has occurred.
* 0 PH: Differential pressure High. 0: no interrupt has been generated; 1: High differential pressure event has occurred.
* \endcode
*/
#define LPS25H_INTERRUPT_SOURCE_REG   (uint8_t)0x25
#define LPS25H_IA_BIT                 LPS25H_BIT(2)
#define LPS25H_PL_BIT                 LPS25H_BIT(1)
#define LPS25H_PH_BIT                 LPS25H_BIT(0)

#define LPS25H_IA_MASK               (uint8_t)0x04
#define LPS25H_PL_MASK               (uint8_t)0x02
#define LPS25H_PH_MASK               (uint8_t)0x01

/**
* @brief  Status Register
* \code
* Read
* Default value: 0x00
* 7:6 Reserved: 0
* 5 P_OR: Pressure data overrun. 0: no overrun has occurred; 1: new data for pressure has overwritten the previous one.
* 4 T_OR: Temperature data overrun. 0: no overrun has occurred; 1: a new data for temperature has overwritten the previous one.
* 3:2 Reserved: 0
* 1 P_DA: Pressure data available. 0: new data for pressure is not yet available; 1: new data for pressure is available.
* 0 T_DA: Temperature data available. 0: new data for temperature is not yet available; 1: new data for temperature is available.
* \endcode
*/
#define LPS25H_STATUS_REG         (uint8_t)0x27
#define LPS25H_POR_BIT            LPS25H_BIT(5)
#define LPS25H_TOR_BIT            LPS25H_BIT(4)
#define LPS25H_PDA_BIT            LPS25H_BIT(1)
#define LPS25H_TDA_BIT            LPS25H_BIT(0)

#define LPS25H_POR_MASK           (uint8_t)0x20
#define LPS25H_TOR_MASK           (uint8_t)0x10
#define LPS25H_PDA_MASK           (uint8_t)0x02
#define LPS25H_TDA_MASK           (uint8_t)0x01

/**
* @brief  Pressure data (LSB) register.
* \code
* Read
* Default value: 0x00.(To be verified)
* POUT7 - POUT0: Pressure data LSB (2's complement).     
* Pressure output data: Pout(hPA)=(PRESS_OUT_H & PRESS_OUT_L &
* PRESS_OUT_XL)[dec]/4096.
* \endcode
*/

#define LPS25H_PRESS_OUT_XL_REG        (uint8_t)0x28
/**
* @brief  Pressure data (Middle part) register.
* \code
* Read
* Default value: 0x80.
* POUT15 - POUT8: Pressure data middle part (2's complement).    
* Pressure output data: Pout(hPA)=(PRESS_OUT_H & PRESS_OUT_L &
* PRESS_OUT_XL)[dec]/4096.
* \endcode
*/ 
#define LPS25H_PRESS_OUT_L_REG        (uint8_t)0x29

/**
* @brief  Pressure data (MSB) register.
* \code
* Read
* Default value: 0x2F.
* POUT23 - POUT16: Pressure data MSB (2's complement).
* Pressure output data: Pout(hPA)=(PRESS_OUT_H & PRESS_OUT_L &
* PRESS_OUT_XL)[dec]/4096.
* \endcode
*/
#define LPS25H_PRESS_OUT_H_REG        (uint8_t)0x2A

/**
* @brief  Temperature data (LSB) register.
* \code
* Read
* Default value: 0x00.
* TOUT7 - TOUT0: temperature data LSB. 
* T(degC) = 42.5 + (Temp_OUTH & TEMP_OUT_L)[dec]/480.
* \endcode
*/
#define LPS25H_TEMP_OUT_L_REG         (uint8_t)0x2B

/**
* @brief  Temperature data (MSB) register.
* \code
* Read
* Default value: 0x00.
* TOUT15 - TOUT8: temperature data MSB. 
* T(degC) = 42.5 + (Temp_OUTH & TEMP_OUT_L)[dec]/480.
* \endcode
*/
#define LPS25H_TEMP_OUT_H_REG         (uint8_t)0x2C

/**
* @brief Threshold pressure (LSB) register.
* \code
* Read/write
* Default value: 0x00.
* 7:0 THS7-THS0: LSB Threshold pressure Low part of threshold value for pressure interrupt
* generation. The complete threshold value is given by THS_P_H & THS_P_L and is
* expressed as unsigned number. P_ths(hPA)=(THS_P_H & THS_P_L)[dec]/16.
* \endcode
*/
#define LPS25H_THS_P_LOW_REG           (uint8_t)0x30

/**
* @brief Threshold pressure (MSB)
* \code
* Read/write
* Default value: 0x00.
* 7:0 THS15-THS8: MSB Threshold pressure. High part of threshold value for pressure interrupt
* generation. The complete threshold value is given by THS_P_H & THS_P_L and is
* expressed as unsigned number. P_ths(mbar)=(THS_P_H & THS_P_L)[dec]/16.
* \endcode
*/
#define LPS25H_THS_P_HIGH_REG         (uint8_t)0x31

/**
* @brief FIFO control register 
* \code
* Read/write
* Default value: 0x00
* 7:5 F_MODE2, F_MODE1, F_MODE0: FIFO mode selection.
*     FM2   | FM1   | FM0   |    FIFO MODE
*   ---------------------------------------------------
*      0    |  0    |  0    | BYPASS MODE               
*      0    |  0    |  1    | FIFO MODE. Stops collecting data when full               
*      0    |  1    |  0    | STREAM MODE: Keep the newest measurements in the FIFO                  
*      0    |  1    |  1    | STREAM MODE until trigger deasserted, then change to FIFO MODE              
*      1    |  0    |  0    | BYPASS MODE until trigger deasserted, then STREAM MODE                 
*      1    |  0    |  1    | Reserved for future use          
*      1    |  1    |  0    | FIFO_MEAN MODE: Fifo is used to generate a running average filtered pressure              
*      1    |  1    |  1    | BYPASS mode until trigger deasserted, then FIFO MODE              
*
* 4:0 WTM_POINT4-0 : FIFO threshold.Watermark level setting (0-31)
* 4:0 WTM_POINT4-0 is FIFO Mean Mode Sample size if FIFO_MEAN MODE is used
*     WTM_POINT4 | WTM_POINT4 | WTM_POINT4 |  WTM_POINT4 | WTM_POINT4 | Sample Size
*   ----------------------------------------------------------------------------------
*      0         |    0       |    0       |      0      |     1      |       2  
*      0         |    0       |    0       |      1      |     1      |       4
*      0         |    0       |    1       |      1      |     1      |       8 
*      0         |    1       |    1       |      1      |     1      |       16
*      1         |    1       |    1       |      1      |     1      |       32
* other values operation not guaranteed       
* \endcode
*/
#define LPS25H_CTRL_FIFO_REG          (uint8_t)0x2E
#define LPS25H_FMODE_BIT              LPS25H_BIT(5)
#define LPS25H_WTM_POINT_BIT          LPS25H_BIT(0)

#define LPS25H_FMODE_MASK             (uint8_t)0xE0
#define LPS25H_WTM_POINT_MASK         (uint8_t)0x1F

/**
* @brief FIFO Status register 
* \code
* Read
* Default value: 0x00
* 7 WTM_FIFO: Watermark status. 0:FIFO filling is lower than watermark level; 1: FIFO is equal or higher than watermark level.           
* 6 FULL_FIFO: Overrun bit status. 0 - FIFO not full; 1 -FIFO is full.
* 5 EMPTY_FIFO: Empty FIFO bit. 0 - FIFO not empty; 1 -FIFO is empty.
* 4:0 DIFF_POINT4-0: FIFOsStored data level.
* \endcode
*/
#define LPS25H_STATUS_FIFO_REG        (uint8_t)0x2F
#define LPS25H_WTM_FIFO_BIT           LPS25H_BIT(7)
#define LPS25H_FULL_FIFO_BIT          LPS25H_BIT(6)
#define LPS25H_EMPTY_FIFO_BIT         LPS25H_BIT(5)
#define LPS25H_DIFF_POINT_BIT         LPS25H_BIT(0)

#define LPS25H_WTM_FIFO_MASK          (uint8_t)0x80
#define LPS25H_FULL_FIFO_MASK         (uint8_t)0x40
#define LPS25H_EMPTY_FIFO_MASK        (uint8_t)0x20
#define LPS25H_DIFF_POINT_MASK        (uint8_t)0x1F

/**
* @brief Pressure offset register  (LSB)
* \code
* Read/write
* Default value: 0x00
* 7:0 RPDS7-0:Pressure Offset for 1 point calibration after soldering. 
* This register contains the low part of the pressure offset value after soldering,for
* differential pressure computing. The complete value is given by RPDS_L & RPDS_H
* and is expressed as signed 2 complement value. 
* \endcode
*/
#define LPS25H_RPDS_L_REG        (uint8_t)0x39

/**
* @brief Pressure offset register (MSB)
* \code
* Read/write
* Default value: 0x00
* 7:0 RPDS15-8:Pressure Offset for 1 point calibration after soldering.      
* This register contains the high part of the pressure offset value after soldering (see description RPDS_L) 
* \endcode
*/
#define LPS25H_RPDS_H_REG        (uint8_t)0x3A

/**
* @}
*/


/**
* @}
*/



/* Exported Functions -------------------------------------------------------------*/
/** @defgroup LPS25H_Exported_Functions
* @{
*/

/**
* @brief  Get the LPS25H driver version.
* @param  None
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_DriverVersion(LPS25H_DriverVersion_st *Version);

/**
* @brief  Init the HAL layer.
* @param  None
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_HalInit(void);

/**
* @brief  DeInit the HAL layer.
* @param  None
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_HalDeInit(void);

/**
* @brief  DeInit the LPS25H driver.
* @param  None
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/

LPS25H_Error_et LPS25H_DeInit(void);

/**
* @brief  Exit the shutdown mode for LPS25H.
* @param  None
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/

LPS25H_Error_et LPS25H_Activate(void);

/**
* @brief  Enter the shutdown mode for LPS25H.
* @param  None
* @retval  Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_DeActivate(void);


/**
* @brief  Check if the single measurement has completed.
* @param  value is set to 1, when the measure is completed
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_IsMeasurementCompleted(uint8_t *Is_Measurement_Completed);

/**
* @brief  Get the values of the last single measurement.
* @param  Pressure and temperature value
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/ 
LPS25H_Error_et LPS25H_Get_Measurement(LPS25H_MeasureTypeDef_st *Measurement_Value);


/**
* @brief  Get the Reference Pressure value that is sum to the sensor output pressure
* @param  buffer to empty with reference pressure value
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_ReferencePressure(int32_t *RefP);

/*Sensor Configuration Functions*/
/*CTRL_REG1*/
/**
* @brief  Set LPS25H Power Down Mode
* @param  BSET/BRESET bit in order to SET/RESET LPS25H Power Down Mode bit
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/

LPS25H_Error_et LPS25H_Set_PowerDownMode(LPS25H_BitStatus_et powerdown);
/**
* @brief  Sets LPS25H Pressure and Temperature Resolution Mode
* @param  Pressure and Temperature Resolution Congiguration
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_Avg(LPS25H_Avgp_et avgp,LPS25H_Avgt_et avgt);

/**
* @brief  Sets LPS25H Pressure Resolution Mode
* @param  Pressure Resolution Congiguration
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_AvgP(LPS25H_Avgp_et avgp);
/**
* @brief  Sets LPS25H  Temperature Resolution Mode
* @param  Temperature Resolution Congiguration
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_AvgT(LPS25H_Avgt_et avgt);

/**
* @brief  Gets LPS25H Pressure Resolution Mode
* @param  value to empty
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_AvgP(LPS25H_Avgp_et* avgpress);

/**
* @brief  Gets LPS25H Temperature Resolution Mode
* @param  value to empty
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_AvgT(LPS25H_Avgt_et* avgtemp);

/**
* @brief  Set the Output Data Rate
* @param  Output Data Rate Value
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_Odr(LPS25H_Odr_et odr);
/**
* @brief  Get the Output Data Rate
* @param  Output Data Rate Value
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_Odr(LPS25H_Odr_et* odr);

/**
* @brief  Enable/ Disable the Interrupt Circuit 
* @param  LPS25H_ENABLE,LPS25H_DISABLE
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_InterruptCircuitEnable(LPS25H_State_et diff_en) ;

/**
* @brief  Get the DIFF_EN bit value
* @param  buffer
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_InterruptCircuitEnable(LPS25H_State_et* diff_en);

/**
* @brief  Set Block Data Update mode
* @param  LPS25H_BDU_CONTINUOS_UPDATE/ LPS25H_BDU_NO_UPDATE
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_Bdu(LPS25H_Bdu_et bdu);


/**
* @brief  Get Block Data Update mode
* @param  
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_Bdu(LPS25H_Bdu_et* bdu);

/**
* @brief  Reset the Auto ZeroFunc. The defualt RPDS value is copied in REF_P   
* @param  void 
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_ResetAZ(void);

/**
* @brief  Set SPI mode: 3 Wire Interface OR 4 Wire Interface
* @param  LPS25H_SPI_4_WIRE/LPS25H_SPI_3_WIRE
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_SpiInterface(LPS25H_SPIMode_et spimode);

/**
* @brief  Get SPI mode: 3 Wire Interface OR 4 Wire Interface
* @param  buffer to empty with SPI mode
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_SpiInterface(LPS25H_SPIMode_et* spimode);


/**
* @brief  Enable/Disable I2C 
* @param  State. Enable (reset bit)/ Disable (set bit)
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_I2C(LPS25H_State_et i2cstate);
/**
* @brief  Set One Shot bit to start a new conversion (ODR mode has to be 000)
* @param  void
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_StartOneShotMeasurement(void);

/**
* @brief Set AutoZero Function bit
* @param  Bit action: LPS25H_BitRESET/LPS25H_BitSET
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_AutoZeroFunction(LPS25H_BitStatus_et autozero);

/**
* @brief Software Reset
* @param  void
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_SwReset(void);

/**
* @brief Reboot Memory Content.
* @param  void
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_MemoryBoot(void);

/**
* @brief Software Reset ann BOOT
* @param  void
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_SwResetAndMemoryBoot(void);

/**
* @brief  Enable or Disable FIFO
* @param  LPS25H_ENABLE/LPS25H_DISABLE
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_FifoModeUse(LPS25H_State_et status);

/**
* @brief  Enable or Disable FIFO Watermark level use
* @param  LPS25H_ENABLE/LPS25H_DISABLE
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_FifoWatermarkLevelUse(LPS25H_State_et status);

/**
* @brief  Enable or Disable 1 HZ ODR Decimation
* @param  LPS25H_ENABLE/LPS25H_DISABLE
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_FifoMeanDecUse(LPS25H_State_et status);

/*CTRL_REG3 Interrupt Control*/
/**
* @brief  Set Interrupt Active on High or Low Level
* @param  LPS25H_ENABLE/LPS25H_DISABLE
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_InterruptActiveLevel(LPS25H_State_et status);

/**
* @brief  Set Push-pull/open drain selection on interrupt pads.
* @param  LPS25H_PushPull/LPS25H_OpenDrain
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_InterruptOutputType(LPS25H_OutputType_et output);

/**
* @brief  Set Data signal on INT1 pad control bits.
* @param  LPS25H_DATA,LPS25H_P_HIGH_LPS25H_P_LOW,LPS25H_P_LOW_HIGH
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_InterruptControlConfig(LPS25H_OutputSignalConfig_et config);

/**
* @brief  Set INT1 interrupt pins configuration.
* @param  LPS25H_EMPTY,LPS25H_WTM,LPS25H_OVR,LPS25H_DATA_READY
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_InterruptDataConfig(LPS25H_DataSignalType_et signal);

/**
* @brief  Enable\Disable Interrupt Generation on differential pressure low and/or high event
* @param  LPS25H_DISABLE_INT, LPS25H_PHE,LPS25H_PLE,LPS25H_PLE_PHE
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_InterruptDifferentialConfig(LPS25H_InterruptDiffConfig_et config);

/**
* @brief  Latch Interrupt request into INT_SOURCE register. 
* @param  LPS25H_ENABLE/LPS25H_DISABLE
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_LatchInterruptRequest(LPS25H_State_et status);

/**
* @brief  Get the Status of  low-high differential pressure event
* @param  Status Event Flag: PH,PL,IA
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_InterruptDifferentialEventStatus(LPS25H_InterruptDiffStatus_st* interruptsource);

/**
* @brief  Get the status of Pressure and Temperature data
* @param  Data Status Flag:  TempDataAvailable, TempDataOverrun, PressDataAvailable, PressDataOverrun
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_DataStatus(LPS25H_DataStatus_st* datastatus);

/**
* @brief  Read LPS25H output register, and calculate the raw  pressure
* @param  The buffer to empty with the pressure raw value
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_RawPressure(int32_t *raw_press);

/**
* @brief  Read the Pressure value in mbar.
* @param  The buffer to empty with the pressure value that must be divided by 100 to get the value in hPA
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_Pressure(int32_t* Pout);

/**
* @brief  Read LPS25H output register, and calculate the raw temperature.
* @param  The buffer to empty with the temperature raw value
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_RawTemperature(int16_t *raw_data);

/**
* @brief  Read the Temperature value in °C.
* @param  The buffer to empty with the temperature value that must be divided by 10 to get the value in ['C]
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_Temperature(int16_t* Tout);

/**
* @brief  Get the threshold value used for pressure interrupt generation.
* @param  The buffer to empty with the temperature value
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_PressureThreshold(int16_t *P_ths);

/**
* @brief  Set the threshold value used for pressure interrupt generation.
* @param  The buffer to empty with the temperature value
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_PressureThreshold(int16_t P_ths);

/**
* @brief  Set Fifo Mode.
* @param  FifoModee struct
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_FifoMode(LPS25H_FifoMode_et fifomode);
/**
* @brief  Get Fifo Mode.
* @param  Buffer to empty with fifo mode value
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_FifoMode(LPS25H_FifoMode_et* fifomode);

/**
* @brief  Get Fifo Status.
* @param  Buffer to empty with fifo mode value
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_FifoStatus(LPS25H_FifoStatus_st* status);

/**
* @brief  Set Fifo Watermark Level.
* @param  Wtm level value [0 31]
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_FifoWatermarkLevel(uint8_t wtmlevel);

/**
* @brief  Get Fifo Watermark Level
* @param  Buffer to empty with wtm value
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_FifoWatermarkLevel(uint8_t *wtmlevel);

/**
* @brief  Set the number of sample to perform moving average when FIFO_MEAN_MODE is used
* @param  2,4,8,16 32 samples moving average value
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_FifoSampleSize(LPS25H_FifoMeanModeSample_et samplesize);

/**
* @brief  Get FiFo Sample size
* @param  2,4,8,16 32 samples moving average value
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_FifoSampleSize(LPS25H_FifoMeanModeSample_et* samplesize);

/**
* @brief  Get Device ID
* @param  value to empty with Identification Device
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_DeviceID(uint8_t* deviceid);

/**
* @brief  Get the reference pressure after soldering for computing differential pressure
* @param  buffer to empty with pressure value
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_PressureOffsetValue(int16_t *pressoffset);

/**
* @brief  Set Generic configuration 
* @param  Struct holding the configuration values
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_GenericConfig(LPS25H_ConfigTypeDef_st* pxLPS25HInit);

/**
* @brief  Get Generic configuration 
* @param  Struct to empty with configuration values
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_GenericConfig(LPS25H_ConfigTypeDef_st* pxLPS25HInit);

/**
* @brief  Set Interrupt configuration 
* @param  Struct holding the configuration values
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_InterruptConfig(LPS25H_InterruptTypeDef_st* pLPS25HInt);

/**
* @brief 	LPS25H_Get_InterruptConfig 
* @param  Struct to empty with configuration values
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_InterruptConfig(LPS25H_InterruptTypeDef_st* pLPS25HInt);

/**
* @brief  Set Fifo configuration 
* @param  Struct holding the configuration values
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Set_FifoConfig(LPS25H_FIFOTypeDef_st* pLPS25HFIFO);

/**
* @brief  Get Fifo configuration 
* @param  Struct to empty with the configuration values
* @retval Status [LPS25H_ERROR, LPS25H_OK]
*/
LPS25H_Error_et LPS25H_Get_FifoConfig(LPS25H_FIFOTypeDef_st* pLPS25HFIFO);


/**
* @brief  Read LPS25H Registers
* @param  uint32_t RegAddr:      	address of the first register to read
* @param  uint32_t NumByteToRead: 	number of registers to read
* @param  uint8_t *Data:         	pointer to the destination data buffer
* @retval LPS25H_ERROR or LPS25H_OK 
*/
LPS25H_Error_et LPS25H_ReadReg(uint8_t RegAddr, uint8_t NumByteToRead, uint8_t* Data);

/**
* @brief  Write LPS25H Registers
* @param  uint32_t RegAddr:      address of the register to write
* @param  uint8_t *Data:         pointer to the source data buffer
* @param  uint32_t NumByteToWrite:           Number of bytes to write
* @retval LPS25H_ERROR or LPS25H_OK 
*/
LPS25H_Error_et LPS25H_WriteReg(uint8_t RegAddr, uint8_t NumByteToWrite, uint8_t* Data);



#ifdef __cplusplus
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

#endif /* __LPS25H_DRIVER__H */

/******************* (C) COPYRIGHT 2013 STMicroelectronics *****END OF FILE****/

/******************** (C) COPYRIGHT 2013 STMicroelectronics ********************
* File Name          : lis3dsh_driver.h
* Author             : MSH Application Team
* Author             : Fabio Tota
* Version            : $Revision:$
* Date               : $Date:$
* Description        : Descriptor Header for lis3dsh_driver.c driver file
*
* HISTORY:
* Date        | Modification                                | Author
* 21/01/2013  | Initial Revision                            | Fabio Tota

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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LIS3DSH_DRIVER__H
#define __LIS3DSH_DRIVER__H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

//these could change accordingly with the architecture
typedef unsigned char u8_t;
typedef unsigned short int u16_t;
typedef signed char i8_t;
typedef short int i16_t;
typedef u8_t Axis_t;
typedef u8_t Int1Conf_t;

//define structure
#ifndef __SHARED__TYPES
#define __SHARED__TYPES
//structure typedef
typedef enum {
  MEMS_SUCCESS          =		0x01,
  MEMS_ERROR			=		0x00	
} status_t;

typedef enum {
  MEMS_ENABLE			=		0x01,
  MEMS_DISABLE			=		0x00	
} State_t;

typedef struct {
  i16_t AXIS_X;
  i16_t AXIS_Y;
  i16_t AXIS_Z;
} AxesRaw_t;

#endif /*__SHARED__TYPES*/

typedef enum {
  POL_HIGH			=		0x01,
  POL_LOW			=		0x00	
} Polarity_t;

typedef enum { 
  POWER_DOWN			=		0x00,
  ODR_3_125			=		0x01,		
  ODR_6_25Hz			=		0x02,
  ODR_12_5Hz		        =		0x03,
  ODR_25Hz		        =		0x04,
  ODR_50Hz		        =		0x05,	
  ODR_100Hz		        =		0x06,
  ODR_400Hz		        =		0x07,
  ODR_800Hz		        =		0x08,
  ODR_1600Hz			=		0x09	
} ODR_t;

typedef enum { 
  BANDWIDTH_800Hz		=		0x00,
  BANDWIDTH_400Hz		=		0x01,		
  BANDWIDTH_200Hz		=		0x02,
  BANDWIDTH_50Hz	        =		0x03
} BandWidth_t;

typedef enum {
  SET_AXIS_X			=		0x01,
  SET_AXIS_Y			=		0x02,
  SET_AXIS_Z			=		0x03
} SET_AXIS_t;

typedef enum {
  SET_VFC_1			=		0x01,
  SET_VFC_2			=		0x02,
  SET_VFC_3			=		0x03,
  SET_VFC_4			=		0x04
} SET_VFC_t;  

typedef enum {
  THRS_1			=		0x01,
  THRS_2			=		0x02
} THRS_t;  

typedef enum {
  SM1				=		0x01,
  SM2				=		0x02
} SM_t; 

typedef enum {
  TIM_1				=		0x01,
  TIM_2				=		0x02,
  TIM_3				=		0x03,
  TIM_4				=		0x04   
} TIM_t; 

typedef enum {
  MASK_A			=		0x01,
  MASK_B			=		0x02 
} MASK_t;

typedef enum {
  FULLSCALE_2                   =               0x00,
  FULLSCALE_4                   =               0x01,
  FULLSCALE_6                   =               0x02,
  FULLSCALE_8                   =               0x03,
  FULLSCALE_16                  =               0x04
} Fullscale_t;

typedef enum {
  BLE_LSB			=		0x00,
  BLE_MSB			=		0x01
} Endianess_t;


typedef enum {
  SELF_TEST_NORMAL		=               0x00,
  SELF_TEST_POSITIVE		=               0x01,
  SELF_TEST_NEGATIVE		=               0x02
} SelfTest_t;


typedef enum {
  FIFO_BYPASS_MODE              =               0x00,
  FIFO_MODE                     =               0x01,
  FIFO_STREAM_MODE              =               0x02,
  FIFO_STREAM_TRIGGER_MODE	=               0x03,
  FIFO_BYPASS_THAN_STREAM	=		0x04,
  FIFO_BYPASS_THAN_FIFO		=		0x07
} FifoMode_t;

typedef enum {
  TRIG_INT1                     =		0x00,
  TRIG_INT2 			=		0x01
} TrigInt_t;

typedef enum {
  SPI_4_WIRE                    =               0x00,
  SPI_3_WIRE                    =               0x01
} SPIMode_t;

typedef enum {
  X_ENABLE                      =               0x01,
  X_DISABLE                     =               0x00,
  Y_ENABLE                      =               0x02,
  Y_DISABLE                     =               0x00,
  Z_ENABLE                      =               0x04,
  Z_DISABLE                     =               0x00    
} AXISenable_t;


/* Exported constants --------------------------------------------------------*/

#define MEMS_SET                                0x01
#define MEMS_RESET                              0x00
#define MEMS_I2C_ADDRESS			0x3C  	//SEL=1

//Register info
#define OUT_T					0x0C
#define INFO_1					0x0D
#define INFO_2					0x0E
#define WHO_AM_I				0x0F	// device identification register

#define I_AM_LIS3DSH			        0x3F

//offset corrction register
#define OFF_X					0x10
#define OFF_Y					0x11
#define OFF_Z					0x12

//Constant shift register
#define CS_X					0x13
#define CS_Y					0x14
#define CS_Z					0x15

//Long counter register
#define LC_L					0x16
#define LC_H					0x17

//STAT_REG
#define STAT					0x18
#define LONG					BIT(7)
#define SYNCW					BIT(6)
#define SYNC1					BIT(5)
#define SYNC2					BIT(4)
#define INT_SM1					BIT(3)
#define INT_SM2					BIT(2)
#define DOR					BIT(1)
#define DRDY					BIT(0)

//Peack detection value for SM1/SM2
#define PEAK1					0x19
#define PEAK2					0x1A

//Vector coefficient Register
#define VFC_1					0x1B
#define VFC_2					0x1C
#define VFC_3					0x1D
#define VFC_4					0x1E

//threshold register
#define THRS3					0X1F

//CONTROL REGISTER 1
#define CNTL4					0x20
#define ODR_BIT				        BIT(4)
#define LIS3DSH_BDU					BIT(3)
#define ZEN					BIT(2)
#define YEN					BIT(1)
#define XEN					BIT(0)


//CONTROL REGISTER for SM1 / SM2
#define CNTL1					0x21
#define CNTL2					0x22
#define HYST     				BIT(7)
#define SM_PIN					BIT(3)
#define SM_EN					BIT(0)

//CONTROL REGISTER 3
#define CNTL3					0x23
#define DR_EN					BIT(7)
#define IEA					BIT(6)
#define IEL				        BIT(5)
#define INT2_EN					BIT(4)
#define INT1_EN					BIT(3)
#define VFILT					BIT(2)
#define STRT					BIT(0)

//CONTROL REGISTER 5
#define CNTL5					0x24
#define BW					BIT(6)
#define FSCALE					BIT(3)
#define ST					BIT(1)
#define SIM					BIT(0)

//CONTROL REGISTER 6
#define CNTL6					0x25
#define BOOT					BIT(7)
#define FIFO_EN					BIT(6)
#define WTM_EN					BIT(5)
#define ADD_INC					BIT(4)
#define I1_EMPTY				BIT(3)
#define I1_WTM					BIT(2)
#define I1_OVERRUN				BIT(1)
#define I2_BOOT					BIT(0)

//STATUS_REG_AXIES
#define STATUS					0x27
#define ZYXOR                                   BIT(7)
#define ZOR                                     BIT(6)
#define YOR                                     BIT(5)
#define XOR                                     BIT(4)
#define ZYXDA                                   BIT(3)
#define ZDA                                     BIT(2)
#define YDA                                     BIT(1)
#define XDA                                     BIT(0)

//OUTPUT ACCELERATION REGISTER
#define OUT_X_L					0x28
#define OUT_X_H					0x29
#define OUT_Y_L					0x2A
#define OUT_Y_H					0x2B
#define OUT_Z_L					0x2C
#define OUT_Z_H					0x2D

//FIFO CONTROL REGISTER
#define FIFO_CTRL				0x2E
#define FMODE					BIT(5)
#define WTMP					BIT(0)

//FIFO REGISTERS
#define FIFO_SRC			        0x2F

// State machine code register value for SM1
#define ST1_1					0x40
#define ST2_1					0x41
#define ST3_1					0x42
#define ST4_1					0x43
#define ST5_1					0x44
#define ST6_1					0x45
#define ST7_1					0x46
#define ST8_1					0x47
#define ST9_1					0x48
#define ST10_1					0x49
#define ST11_1					0x4A
#define ST12_1					0x4B
#define ST13_1					0x4C
#define ST14_1					0x4D
#define ST15_1					0x4E
#define ST16_1					0x4F
  
// State machine code register value for SM2
#define ST1_2					0x60
#define ST2_2					0x61
#define ST3_2					0x62
#define ST4_2					0x63
#define ST5_2					0x64
#define ST6_2					0x65
#define ST7_2					0x66
#define ST8_2					0x67
#define ST9_2					0x68
#define ST10_2					0x69
#define ST11_2					0x6A
#define ST12_2					0x6B
#define ST13_2					0x6C
#define ST14_2					0x6D
#define ST15_2					0x6E
#define ST16_2					0x6F

//General timer for SM1
#define TIM4_1					0x50
#define TIM3_1					0x51

//16bit unsigned value timer for SM1
#define TIM2_1_L				0x52
#define TIM2_1_H				0x53
#define TIM1_1_L				0x54
#define TIM1_1_H				0x55

//threshold signed value for SM1
#define THRS2_1					0X56
#define THRS1_1					0X57

//axis and sign mask for SM1 motion direction operation
#define MASKB_1					0x59
//axis and sign mask for SM2 motion direction operation
#define MASKA_1					0x5A
#define P_X					BIT(7)
#define N_X					BIT(6)
#define P_Y					BIT(5)
#define N_Y					BIT(4)
#define P_Z					BIT(3)
#define N_Z					BIT(2)
#define P_V					BIT(1)
#define N_V					BIT(0)

//threshold, peak settings for SM1 / SM2
#define SETT1					0x5B
#define SETT2					0x7B
#define P_DET					BIT(7)
#define THR3_SA					BIT(6)
#define	ABS					BIT(5)
#define RADI					BIT(4) //only SETT2
#define D_CS					BIT(3) //only SETT2
#define THR3_MA					BIT(2)
#define R_TAM					BIT(1)
#define SITR					BIT(0)

//program and reset pointer for SM1/SM2
#define PR1					0x5C
#define PR2					0x7C
#define PP					BIT(4)      
#define RP					BIT(0)     

//General timer for SM2
#define TIM4_2					0x70
#define TIM3_2					0x71
//16bit unsigned value timer for SM2
#define TIM2_2_L				0x72
#define TIM2_2_H				0x73
#define TIM1_2_L				0x74
#define TIM1_2_H				0x75

//threshold signed value for SM1
#define THRS2_2					0x76
#define THRS1_2					0x77

//decimation counter value for SM2
#define DES2					0x78

//axis and sign mask for SM1 motion direction operation
#define MASKB_2					0x79
//axis and sign mask for SM1 motion direction operation
#define MASKA_2					0x7A
#define P_X					BIT(7)
#define N_X					BIT(6)
#define P_Y					BIT(5)
#define N_Y					BIT(4)
#define P_Z					BIT(3)
#define N_Z					BIT(2)
#define P_V					BIT(1)
#define N_V					BIT(0)

//General Timer 16 bit for SM1/SM2
#define TC1_L					0x5D
#define TC1_H					0x5E
#define TC2_L					0x7D
#define TC2_H					0x7E

// output flags on axis for interrupt SM1/SM2
#define OUTS1					0x5F
#define OUTS2					0x7F

//mask Stat register flag
#define F_LONG					0x80
#define F_SYNCW					0x40
#define F_SYNC1					0x20
#define F_SYNC2					0x10
#define F_INT_SM1				0x08
#define F_INT_SM2				0x04
#define F_DOR					0x02
#define F_DRDY					0x01

//INPUT 8Bit ACCELERATION Register (debug mode)
#define X_DEBUG					0x28
#define Y_DEBUG					0x2A
#define Z_DEBUG					0x2C

//STATUS REGISTER bit mask
#define STATUS_REG_ZYXOR                        0x80    // 1	:	new data set has over written the previous one
							// 0	:	no overrun has occurred (default)	
#define STATUS_REG_ZOR                          0x40    // 0	:	no overrun has occurred (default)
							// 1	:	new Z-axis data has over written the previous one
#define STATUS_REG_YOR                          0x20    // 0	:	no overrun has occurred (default)
							// 1	:	new Y-axis data has over written the previous one
#define STATUS_REG_XOR                          0x10    // 0	:	no overrun has occurred (default)
							// 1	:	new X-axis data has over written the previous one
#define STATUS_REG_ZYXDA                        0x08    // 0	:	a new set of data is not yet avvious one
                                                        // 1	:	a new set of data is available 
#define STATUS_REG_ZDA                          0x04    // 0	:	a new data for the Z-Axis is not availvious one
                                                        // 1	:	a new data for the Z-Axis is available
#define STATUS_REG_YDA                          0x02    // 0	:	a new data for the Y-Axis is not available
                                                        // 1	:	a new data for the Y-Axis is available
#define STATUS_REG_XDA                          0x01    // 0	:	a new data for the X-Axis is not available

#define DATAREADY_BIT                           STATUS_REG_ZYXDA

//bit mask for out flag for interrupt
#define F_P_X					0x80
#define F_N_X					0x40
#define F_P_Y					0x20
#define F_N_Y					0x10
#define F_P_Z					0x08
#define F_N_Z					0x04
#define F_P_V					0x02
#define F_N_V					0x01

//bit mask of FIFO SOURCE
#define FIFO_WTM_S				0x80
#define FIFO_OVRN_S				0x40
#define FIFO_EMPTY_S				0x20
#define	FIFO_STORED_S				0x1F



/* Exported macro ------------------------------------------------------------*/
#define ValBit(VAR,Place)         (VAR & (1<<Place))
#define BIT(x) ( (x) )

/* Exported functions --------------------------------------------------------*/
//Sensor Configuration Functions
status_t SetODR(ODR_t ov);
status_t SetAxis(Axis_t axis);
status_t SetFullScale(Fullscale_t fs);
status_t ReBootEnable(State_t boot);
status_t AddIncEnable(State_t addinc);
status_t BootInt2(State_t booti2);
status_t SetBDU(State_t bdu);
status_t SetSelfTest(SelfTest_t st);
status_t SoftReset(State_t strt);
status_t SetOFFSET(SET_AXIS_t axis, u8_t val); 
status_t SetCS(SET_AXIS_t axis, u8_t val);
status_t GetStatBIT(u8_t StatBITMask);

//threshold
status_t GetThrs3(u8_t* val);
status_t SetThrs3(u8_t val);
status_t SetThrsSM(SM_t sm, THRS_t thrs, u8_t val);
status_t GetThrsSM(SM_t sm, THRS_t thrs, u8_t* val); 

//debug function
status_t SetDebugMode(State_t state);                         
status_t SetDebugXYZ(u8_t x_debug, u8_t y_debug, u8_t z_debug);

//state machine
status_t SetSMCodeReg(u8_t CodeADD, u8_t CodeByte);
status_t SetSMBufferCodeReg(SM_t sm, u8_t* CodeBuff);
status_t GetSMCodeRegister(SM_t sm, u8_t RegNumber, u8_t* val);
status_t SetLC(u16_t val); 
status_t GetLC(i16_t* val);
status_t GetResetPointSM(SM_t sm, u8_t* val);
status_t GetProgramPointSM(SM_t sm, u8_t* val);
status_t GetDecimSM2(u8_t* val);
status_t SetTimerSM(SM_t sm, TIM_t timer, u16_t val);
status_t SetMaskSM(SM_t sm, MASK_t mask, u8_t val);
status_t GetTCSM(SM_t sm, u16_t* val);
status_t GetPeakSM(SM_t sm, u8_t* val);
status_t SetHystSM(SM_t  sm, u8_t val);
status_t SetIntPinSM(SM_t sm, State_t state);
status_t SetSitrSM(SM_t sm, State_t state);
status_t SetIntEnaSM(SM_t sm, State_t state);

//Filtering Functions
status_t BandWidth(BandWidth_t bw);
status_t VectFiltEnable(State_t vfe);
status_t SetVectorCoeff(SET_VFC_t vfc, u8_t val);
status_t GetVectorCoeff(SET_VFC_t vfc, u8_t* val);

//Interrupt Functions
status_t DataReadyInt(State_t drdy);
status_t Int1Enable(State_t conf);
status_t Int2Enable(State_t conf);
status_t IntLatchEnable(State_t latch);
status_t IntSignPol(Polarity_t pol);
status_t ResetInt1Latch(void);
status_t SetIntConfiguration(Int1Conf_t ic);
status_t SetInt1Threshold(u8_t ths);
status_t SetInt1Duration(Int1Conf_t id);
status_t GetInt1Src(u8_t* val);
status_t GetInt1SrcBit(u8_t statusBIT);
status_t GetOutSBitSM(SM_t sm, u8_t FLAG_INT_OUT);
status_t SetPeakDetSM(SM_t sm, State_t state);
status_t SetThr3SaSM(SM_t sm, State_t state);
status_t SetAbsSM(SM_t sm, State_t state);
status_t SetRTamSM(SM_t sm, State_t state);
status_t SetThr3MaSM(SM_t sm, State_t state);

//FIFO Functions
status_t FIFOMode(FifoMode_t fm);
status_t SetWaterMark(u8_t wtm);
status_t SetTriggerInt(TrigInt_t tr);
status_t GetFifoSourceReg(u8_t* val);
status_t GetFifoSourceBit(u8_t statusBIT);
status_t GetFifoSourceFSS(u8_t* val);
status_t FIFOEnable(State_t fifo, u8_t nMax);
status_t FifoEmptyInt1(State_t empty);
status_t FifoOvrInt1(State_t overrun);
status_t ReadFifoData(AxesRaw_t* FifoBuff, u8_t* depth);

//Other Reading Functions
status_t GetSatusReg(u8_t* val);
status_t GetSatusBit(u8_t statusBIT);
status_t GetAccAxesRaw(AxesRaw_t* buff);
status_t GetWHO_AM_I(u8_t* val);
status_t GetOUT_T(u8_t* val);

//Generic
u8_t ReadReg(u8_t deviceAddr, u8_t Reg, u8_t* Data);
u8_t WriteReg(u8_t deviceAddress, u8_t WriteAddr, u8_t Data);


#endif /* __LIS3DSH_DRIVER__H */

/******************* (C) COPYRIGHT 2013 STMicroelectronics *****END OF FILE****/




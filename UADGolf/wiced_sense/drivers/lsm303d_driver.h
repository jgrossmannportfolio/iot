/******************** (C) COPYRIGHT 2012 STMicroelectronics ********************
* File Name          : lsm303d_driver.h
* Author             : MSH Application Team
* Author             : Abhishek Anand
* Version            : $Revision:$
* Date               : $Date:$
* Description        : Descriptor Header for lsm303d driver file
*
* HISTORY:
* Date        | Modification                                | Author
* 21/05/2012  | Initial Revision                            | Abhishek Anand

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
#ifndef __LSM303D_DRIVER__H
#define __LSM303D_DRIVER__H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

//these could change accordingly with the architecture

#ifndef __ARCHDEP__TYPES
#define __ARCHDEP__TYPES
typedef unsigned char u8_t;
typedef unsigned short int u16_t;
typedef short int i16_t;
typedef signed char i8_t;
#endif /*__ARCHDEP__TYPES*/

typedef u8_t LSM303D_IntPinConf_t;
typedef u8_t LSM303D_Axis_t;
typedef u8_t LSM303D_IntConf_t;

//define structure
#ifndef __SHARED__TYPES
#define __SHARED__TYPES

typedef enum {
  MEMS_SUCCESS				=		0x01,
  MEMS_ERROR				=		0x00	
} status_t;

typedef enum {
  MEMS_ENABLE				=		0x01,
  MEMS_DISABLE				=		0x00	
} State_t;

typedef struct {
  i16_t AXIS_X;
  i16_t AXIS_Y;
  i16_t AXIS_Z;
} AxesRaw_t;

#endif /*__SHARED__TYPES*/

typedef enum {  
  LSM303D_ODR_3_125Hz_A			=		0x01,		
  LSM303D_ODR_6_25Hz_A                  =		0x02,
  LSM303D_ODR_12_5Hz_A			=		0x03,
  LSM303D_ODR_25Hz_A			=		0x04,
  LSM303D_ODR_50Hz_A			=		0x05,	
  LSM303D_ODR_100Hz_A			=		0x06,
  LSM303D_ODR_200Hz_A			=		0x07,
  LSM303D_ODR_400Hz_A			=		0x08,
  LSM303D_ODR_800Hz_A			=		0x09,
  LSM303D_ODR_1600Hz_A			=		0x0A
} LSM303D_ODR_A_t;

typedef enum {
  LSM303D_X_ENABLE                      =               0x01,
  LSM303D_X_DISABLE                     =               0x00,
  LSM303D_Y_ENABLE                      =               0x02,
  LSM303D_Y_DISABLE                     =               0x00,
  LSM303D_Z_ENABLE                      =               0x04,
  LSM303D_Z_DISABLE                     =               0x00    
} LSM303D_AXISenable_t;

typedef enum {
  LSM303D_POWER_DOWN_MODE_A             =		0x00
} LSM303D_Mode_A_t;

typedef enum {  
  LSM303D_ABW_773Hz_A			=		0x00,		
  LSM303D_ABW_362Hz_A			=		0x01,
  LSM303D_ABW_194Hz_A			=		0x02,
  LSM303D_ABW_50Hz_A			=		0x03
} LSM303D_ABW_A_t;

typedef enum {
  LSM303D_FULLSCALE_2_A                 =		0x00,
  LSM303D_FULLSCALE_4_A                 =		0x01,
  LSM303D_FULLSCALE_6_A                 =		0x02,
  LSM303D_FULLSCALE_8_A                 =		0x03,
  LSM303D_FULLSCALE_16_A                =		0x04
} LSM303D_Fullscale_A_t;

typedef enum {
  LSM303D_SPI_4_WIRE			=               0x00,
  LSM303D_SPI_3_WIRE                    =               0x01
} LSM303D_SPIMode_t;

typedef enum {  
  LSM303D_ODR_3_125Hz_M			=		0x00,		
  LSM303D_ODR_6_25Hz_M                  =		0x01,
  LSM303D_ODR_12_5Hz_M			=		0x02,
  LSM303D_ODR_25Hz_M			=		0x03,
  LSM303D_ODR_50Hz_M			=		0x04,	
  LSM303D_ODR_100Hz_M			=		0x05
} LSM303D_ODR_M_t;

typedef enum {  
  LSM303D_LOW_RES_M			=		0x00,		
  LSM303D_HIGH_RES_M			=		0x03,
} LSM303D_RES_M_t;

typedef enum {
  LSM303D_FULLSCALE_2_M			=               0x00,
  LSM303D_FULLSCALE_4_M			=               0x01,
  LSM303D_FULLSCALE_8_M			=               0x02,
  LSM303D_FULLSCALE_12_M		=               0x03
} LSM303D_Fullscale_M_t;

typedef enum {
  LSM303D_HPM_NORMAL_MODE_RES_A		=		0x00,
  LSM303D_HPM_REF_SIGNAL_A		=		0x01,
  LSM303D_HPM_NORMAL_MODE_A		=		0x02,
  LSM303D_HPM_AUTORESET_INT_A		=		0x03
} LSM303D_HPFMode_A_t;

typedef enum {
  LSM303D_CONTINUOUS_MODE_M		=		0x00,
  LSM303D_SINGLE_MODE_M			=		0x01,
  LSM303D_POWER_DOWN_MODE_M		=		0x02
} LSM303D_Mode_M_t;

typedef enum {
  LSM303D_FIFO_BYPASS_MODE              =               0x00,
  LSM303D_FIFO_MODE                     =               0x01,
  LSM303D_FIFO_STREAM_MODE              =               0x02,
  LSM303D_FIFO_STREAM_TO_FIFO_MODE      =               0x03,
  LSM303D_FIFO_BYPASS_TO_STREAM_MODE    =               0x04,
  LSM303D_FIFO_DISABLE_MODE		=               0x05
} LSM303D_FifoMode_t;

typedef enum {
  LSM303D_INT_MODE_OR                   =               0x00,
  LSM303D_INT_MODE_6D_MOVEMENT          =               0x01,
  LSM303D_INT_MODE_AND                  =               0x02,
  LSM303D_INT_MODE_6D_POSITION          =               0x03  
} LSM303D_IntMode_t;

typedef enum {
  LSM303D_INT1_6D_4D_DISABLE            =               0x00,
  LSM303D_INT1_6D_ENABLE                =               0x01,
  LSM303D_INT1_4D_ENABLE                =               0x02 
} LSM303D_INT_6D_4D_t;

typedef enum {
  LSM303D_UP_SX                         =               0x44,
  LSM303D_UP_DX                         =               0x42,
  LSM303D_DW_SX                         =               0x41,
  LSM303D_DW_DX                         =               0x48,
  LSM303D_TOP                           =               0x60,
  LSM303D_BOTTOM                        =               0x50
} LSM303D_POSITION_6D_t;

//interrupt click response
//  b7 = don't care   b6 = IA  b5 = DClick  b4 = Sclick  b3 = Sign  
//  b2 = z      b1 = y     b0 = x
typedef enum {
LSM303D_DCLICK_Z_P                      =               0x24,
LSM303D_DCLICK_Z_N                      =               0x2C,
LSM303D_SCLICK_Z_P                      =               0x14,
LSM303D_SCLICK_Z_N                      =               0x1C,
LSM303D_DCLICK_Y_P                      =               0x22,
LSM303D_DCLICK_Y_N                      =               0x2A,
LSM303D_SCLICK_Y_P                      =               0x12,
LSM303D_SCLICK_Y_N			=		0x1A,
LSM303D_DCLICK_X_P                      =               0x21,
LSM303D_DCLICK_X_N                      =               0x29,
LSM303D_SCLICK_X_P                      =               0x11,
LSM303D_SCLICK_X_N                      =               0x19,
LSM303D_NO_CLICK                        =               0x00
} LSM303D_Click_Response; 


/* Exported constants --------------------------------------------------------*/

#ifndef __SHARED__CONSTANTS
#define __SHARED__CONSTANTS

#define MEMS_SET                                        0x01
#define MEMS_RESET                                      0x00

#endif /*__SHARED__CONSTANTS*/

#define LSM303D_MEMS_I2C_ADDRESS                        0x3A

//Register Definition

/**************CONTROL REGISTERS*****************/

/***************CTRL0***************/
#define LSM303D_CTRL0					0x1F

//BITS
#define LSM303D_BOOT                                    BIT(7) 
#define LSM303D_FIFO_EN                                 BIT(6) 
#define LSM303D_FTH_EN                                  BIT(5) 
#define LSM303D_HPCLICK                                 BIT(2)
#define LSM303D_HPIS1                                   BIT(1)
#define LSM303D_HPIS2                                   BIT(0)

/***************CTRL1***************/
//ADDRESS
#define LSM303D_CTRL1                                  0x20

//BITS
#define LSM303D_ODR_BIT_A                               BIT(4)
#define LSM303D_BDU					BIT(3)
#define LSM303D_AZEN					BIT(2)
#define LSM303D_AYEN					BIT(1)
#define LSM303D_AXEN					BIT(0)

/***************CTRL2***************/
//ADDRESS
#define LSM303D_CTRL2                                  0x21

//BITS
#define LSM303D_ABW_A                                   BIT(6)
#define LSM303D_FS_A                                    BIT(3)
#define LSM303D_ST_A                                    BIT(1)
#define LSM303D_SIM                                     BIT(0)

/***************CTRL3***************/
//ADDRESS
#define LSM303D_CTRL3                                  0x22

//BITS
#define LSM303D_P1_BOOT                                 BIT(7)
#define LSM303D_P1_CLICK                                  BIT(6)
#define LSM303D_P1_IG1                                 BIT(5)
#define LSM303D_P1_IG2                                 BIT(4)
#define LSM303D_P1_IGM                                 BIT(3)
#define LSM303D_P1_DRDYA                                BIT(2)
#define LSM303D_P1_DRDYM                                BIT(1)
#define LSM303D_P1_EMPTY                                BIT(0)

//BITMASK
#define LSM303D_BOOT_ON_PIN_INT1_ENABLE			0x80
#define LSM303D_BOOT_ON_PIN_INT1_DISABLE		0x00
#define LSM303D_CLICK_ON_PIN_INT1_ENABLE			0x40
#define LSM303D_CLICK_ON_PIN_INT1_DISABLE			0x00
#define LSM303D_IG1_ON_PIN_INT1_ENABLE			0x20
#define LSM303D_IG1_ON_PIN_INT1_DISABLE		0x00
#define LSM303D_IG2_ON_PIN_INT1_ENABLE			0x10
#define LSM303D_IG2_ON_PIN_INT1_DISABLE		0x00
#define LSM303D_IGM_ON_PIN_INT1_ENABLE			0x08
#define LSM303D_IGM_ON_PIN_INT1_DISABLE		0x00
#define LSM303D_DRDYA_ON_PIN_INT1_ENABLE		0x04
#define LSM303D_DRDYA_ON_PIN_INT1_DISABLE		0x00
#define LSM303D_DRDYM_ON_PIN_INT1_ENABLE		0x02
#define LSM303D_DRDYM_ON_PIN_INT1_DISABLE		0x00
#define LSM303D_EMPTY_ON_PIN_INT1_ENABLE		0x01
#define LSM303D_EMPTY_ON_PIN_INT1_DISABLE		0x00

/***************CTRL4***************/
//ADDRESS
#define LSM303D_CTRL4                                  0x23

//BITS
#define LSM303D_P2_CLICK					BIT(7)
#define LSM303D_P2_IG1					BIT(6)
#define LSM303D_P2_IG2                                 BIT(5)
#define LSM303D_P2_INTM                                 BIT(4)
#define LSM303D_P2_DRDYA                                BIT(3)
#define LSM303D_P2_DRDYM                                BIT(2)
#define LSM303D_P2_OVERRUN                              BIT(1)
#define LSM303D_P2_WTM					BIT(0)

//BITMASK
#define LSM303D_CLICK_ON_PIN_INT2_ENABLE                  0x80
#define LSM303D_CLICK_ON_PIN_INT2_DISABLE                 0x00
#define LSM303D_IG1_ON_PIN_INT2_ENABLE                 0x40
#define LSM303D_IG1_ON_PIN_INT2_DISABLE                0x00
#define LSM303D_IG2_ON_PIN_INT2_ENABLE                 0x20
#define LSM303D_IG2_ON_PIN_INT2_DISABLE                0x00
#define LSM303D_INTM_ON_PIN_INT2_ENABLE			0x10
#define LSM303D_INTM_ON_PIN_INT2_DISABLE		0x00
#define LSM303D_DRDYA_ON_PIN_INT2_ENABLE		0x08
#define LSM303D_DRDYA_ON_PIN_INT2_DISABLE		0x00
#define LSM303D_DRDYM_ON_PIN_INT2_ENABLE		0x04
#define LSM303D_DRDYM_ON_PIN_INT2_DISABLE		0x00
#define LSM303D_OVERRUN_ON_PIN_INT2_ENABLE		0x02
#define LSM303D_OVERRUN_ON_PIN_INT2_DISABLE		0x00
#define LSM303D_WTM_ON_PIN_INT2_ENABLE			0x01
#define LSM303D_WTM_ON_PIN_INT2_DISABLE			0x00

/***************CTRL5***************/
//ADDRESS
#define LSM303D_CTRL5                                  0x24

//BITS
#define LSM303D_TEMP_EN                                 BIT(7)
#define LSM303D_MRES									BIT(5)
#define LSM303D_ODR_BIT_M                               BIT(2)
#define LSM303D_LIR2                                    BIT(1)
#define LSM303D_LIR1                                    BIT(0)

/***************CTRL6***************/
//ADDRESS
#define LSM303D_CTRL6                                  0x25

//BITS
#define LSM303D_FS_M                                    BIT(5)

/***************CTRL7***************/
//ADDRESS
#define LSM303D_CTRL7                                  0x26

//BITS
#define LSM303D_AHPM                                    BIT(6)
#define LSM303D_AFDS                                    BIT(5)
#define LSM303D_T_ONLY                                  BIT(4)
#define LSM303D_MLP                                     BIT(2)
#define LSM303D_MD                                      BIT(0)

/***************OFFSET REGISTERS******************/

/*******OFFSET_X_L_M, OFFSET_X_H_M********/
//ADDRESS
#define LSM303D_OFFSET_X_L_M                            0x16
#define LSM303D_OFFSET_X_H_M                            0x17

/******OFFSET_Y_L_M, OFFSET_Y_L_M*********/
//ADDRESS
#define LSM303D_OFFSET_Y_L_M                            0x18
#define LSM303D_OFFSET_Y_H_M                            0x19

/******OFFSET_Z_L_M, OFFSET_Z_L_M*********/
//ADDRESS
#define LSM303D_OFFSET_Z_L_M                            0x1A
#define LSM303D_OFFSET_Z_H_M                            0x1B

/**************REFERENCE REGISTERS****************/

/**************REFERENCE_X****************/
//ADDRESS
#define LSM303D_REFERENCE_X				0x1C

/**************REFERENCE_Y****************/
//ADDRESS
#define LSM303D_REFERENCE_Y				0x1D

/**************REFERENCE_Z****************/
//ADDRESS
#define LSM303D_REFERENCE_Z				0x1E

/****************FIFO REGISTERS******************/

/**************FIFO_CNTRL_REG****************/
//ADDRESS
#define LSM303D_FIFO_CNTRL_REG				0x2E

//BITS
#define LSM303D_FM                                      BIT(5)
#define LSM303D_FTH                                     BIT(0)

/**************FIFO_SRC_REG****************/
//ADDRESS
#define LSM303D_FIFO_SRC_REG				0x2F

//BITMASK
#define LSM303D_FIFO_SRC_FTH                            0x80
#define LSM303D_FIFO_SRC_OVRUN                          0x40
#define LSM303D_FIFO_SRC_EMPTY                          0x20

/**************MAGNETOMETER INTERRUPT REGISTERS***************/

/**************INT_CTRL_M****************/
//ADDRESS
#define LSM303D_INT_CTRL_M				0x12

//BITS
#define LSM303D_4D                                      BIT(1)

//BITMASK
#define LSM303D_INT_XMIEN_ENABLE                        0x80
#define LSM303D_INT_XMIEN_DISABLE                       0x00
#define LSM303D_INT_YMIEN_ENABLE                        0x40
#define LSM303D_INT_YMIEN_DISABLE                       0x00
#define LSM303D_INT_ZMIEN_ENABLE                        0x20
#define LSM303D_INT_ZMIEN_DISABLE                       0x00
#define LSM303D_INT_PPOD_ENABLE                         0x10
#define LSM303D_INT_PPOD_DISABLE                        0x00
#define LSM303D_INT_MIEA_ENABLE                         0x08
#define LSM303D_INT_MIEA_DISABLE                        0x00
#define LSM303D_INT_MIEL_ENABLE                         0x04
#define LSM303D_INT_MIEL_DISABLE                        0x00
#define LSM303D_INT_4D_ENABLE                           0x02
#define LSM303D_INT_4D_DISABLE                          0x00
#define LSM303D_INT_MIEN_ENABLE                         0x01
#define LSM303D_INT_MIEN_DISABLE                        0x00

/**************INT_SRC_M****************/
//ADDRESS
#define LSM303D_INT_SRC_M				0x13

//BITMASK
#define LSM303D_INT_SRC_M_PTH_X                         0x80
#define LSM303D_INT_SRC_M_PTH_Y                         0x40
#define LSM303D_INT_SRC_M_PTH_Z                         0x20
#define LSM303D_INT_SRC_M_NTH_X                         0x10
#define LSM303D_INT_SRC_M_NTH_Y                         0x08
#define LSM303D_INT_SRC_M_NTH_Z                         0x04
#define LSM303D_INT_SRC_MROI                            0x02
#define LSM303D_INT_SRC_MINT                            0x01

/**********INT_THS_L_M; INT_THS_H_M**********/
//ADDRESS
#define LSM303D_INT_THS_L_M				0x14
#define LSM303D_INT_THS_H_M				0x15

/**************ACCELEROMETER INTERRUPT REGISTERS***************/

/**************IG_CFG1***************/
//ADDRESS
#define LSM303D_IG_CFG1				0x2F

//BITS
#define LSM303D_ANDOR                                   BIT(7)
#define LSM303D_INT_6D                                  BIT(6)

//BITMASK
#define LSM303D_INT_AND                                 0x80
#define LSM303D_INT_OR                                  0x00
#define LSM303D_INT_6D_ENABLE				0x40
#define LSM303D_INT_6D_DISABLE				0x00
#define LSM303D_INT_ZHIE_ENABLE                         0x20
#define LSM303D_INT_ZHIE_DISABLE                        0x00
#define LSM303D_INT_ZLIE_ENABLE                         0x10
#define LSM303D_INT_ZLIE_DISABLE                        0x00
#define LSM303D_INT_YHIE_ENABLE                         0x08
#define LSM303D_INT_YHIE_DISABLE                        0x00
#define LSM303D_INT_YLIE_ENABLE                         0x04
#define LSM303D_INT_YLIE_DISABLE                        0x00
#define LSM303D_INT_XHIE_ENABLE                         0x02
#define LSM303D_INT_XHIE_DISABLE                        0x00
#define LSM303D_INT_XLIE_ENABLE                         0x01
#define LSM303D_INT_XLIE_DISABLE                        0x00

/**************IG_SRC1***************/
//ADDRESS
#define LSM303D_IG_SRC1				0x31

//BITMASK
#define LSM303D_INT_SRC_IA                              0x40
#define LSM303D_INT_SRC_ZH                              0x20
#define LSM303D_INT_SRC_ZL                              0x10
#define LSM303D_INT_SRC_YH                              0x08
#define LSM303D_INT_SRC_YL                              0x04
#define LSM303D_INT_SRC_XH                              0x02
#define LSM303D_INT_SRC_XL                              0x01

/***************IG_THS1**************/
//ADDRESS
#define LSM303D_IG_THS1				0x32

/************IG_DUR1************/
//ADDRESS
#define LSM303D_IG_DUR1			0x33

/**************IG_CFG2***************/
//ADDRESS
#define LSM303D_IG_CFG2				0x34

/**************IG_SRC2***************/
//ADDRESS
#define LSM303D_IG_SRC2				0x35

/***************IG_THS2**************/
//ADDRESS
#define LSM303D_IG_THS2				0x36

/************IG_DUR2************/
//ADDRESS
#define LSM303D_IG_DUR2			        0x37

/*********************CLICK REGISTERS**********************/

/************CLICK_CFG************/
//ADDRESS
#define LSM303D_CLICK_CFG       			0x38

//BITS
//STRUCTURES
//BITMASK
#define LSM303D_ZD_ENABLE                               0x20
#define LSM303D_ZD_DISABLE                              0x00
#define LSM303D_ZS_ENABLE                               0x10
#define LSM303D_ZS_DISABLE                              0x00
#define LSM303D_YD_ENABLE                               0x08
#define LSM303D_YD_DISABLE                              0x00
#define LSM303D_YS_ENABLE                               0x04
#define LSM303D_YS_DISABLE                              0x00
#define LSM303D_XD_ENABLE                               0x02
#define LSM303D_XD_DISABLE                              0x00
#define LSM303D_XS_ENABLE                               0x01
#define LSM303D_XS_DISABLE                              0x00

/************CLICK_SRC************/
//ADDRESS
#define LSM303D_CLICK_SRC       			0x38

//BITMASK
#define LSM303D_IA                                      0x40
#define LSM303D_DCLICK                                  0x20
#define LSM303D_SCLICK                                  0x10
#define LSM303D_CLICK_SIGN                              0x08
#define LSM303D_CLICK_Z                                 0x04
#define LSM303D_CLICK_Y                                 0x02
#define LSM303D_CLICK_X                                 0x01

/************CLICK_THS************/
//ADDRESS
#define LSM303D_CLICK_THS				0x3A

/************TIME_LIMIT***********/
//ADDRESS
#define LSM303D_CLICK_TIME_LIMIT			0x3B

/**********TIME_LATENCY***********/
//ADDRESS
#define LSM303D_CLICK_TIME_LATENCY			0x3C

/***********TIME_WINDOW***********/
//ADDRESS
#define LSM303D_CLICK_TIME_WINDOW			0x3D

/********************SLEEP TO WAKE REGISTERS******************/
/***********Act_THS***********/
//ADDRESS
#define LSM303D_Act_THS          			0x3E

/***********Act_DUR***********/
//ADDRESS
#define LSM303D_Act_DUR          			0x3F

/**********ACCELEROMETER: STATUS AND OUTPUT REGISTERS***********/

/***********STATUS_A***********/
//ADDRESS
#define LSM303D_STATUS_A    			0x27

//BITMASK
#define LSM303D_STATUS_ZYXAOR                      0x80    // 1	:	new data set has over written the previous one
                                                                // 0	:	no overrun has occurred (default)	
#define LSM303D_STATUS_ZAOR                        0x40    // 0	:	no overrun has occurred (default)
                                                                // 1	:	new Z-axis data has over written the previous one
#define LSM303D_STATUS_YAOR                        0x20    // 0	:	no overrun has occurred (default)
                                                                // 1	:	new Y-axis data has over written the previous one
#define LSM303D_STATUS_XAOR                        0x10    // 0	:	no overrun has occurred (default)
                                                                // 1	:	new X-axis data has over written the previous one
#define LSM303D_STATUS_ZYXADA                      0x08    // 0	:	a new set of data is not yet available
                                                                // 1	:	a new set of data is available 
#define LSM303D_STATUS_ZADA                        0x04    // 0	:	a new data for the Z-Axis is not available
                                                                // 1	:	a new data for the Z-Axis is available
#define LSM303D_STATUS_YADA                        0x02    // 0	:	a new data for the Y-Axis is not available
                                                                // 1	:	a new data for the Y-Axis is available
#define LSM303D_STATUS_XADA                        0x01    // 0	:	a new data for the X-Axis is not available
                                                                // 1	:	a new data for the X-Axis is available

#define LSM303D_DATAREADY_BIT_A                         LSM303D_STATUS_ZYXADA

/***********OUT_*_*_A***********/
//ADDRESS
#define LSM303D_OUT_X_L_A       			0x28
#define LSM303D_OUT_X_H_A       			0x29
#define LSM303D_OUT_Y_L_A       			0x2A
#define LSM303D_OUT_Y_H_A       			0x2B
#define LSM303D_OUT_Z_L_A       			0x2C
#define LSM303D_OUT_Z_H_A       			0x2D

/***********MAGNETOMETER: STATUS AND OUTPUT REGISTERS***********/

/***********STATUS_M***********/
//ADDRESS
#define LSM303D_STATUS_M    			0x07

//BITMASK
#define LSM303D_STATUS_ZYXMOR                      0x80    // 1	:	new data set has over written the previous one
                                                                // 0	:	no overrun has occurred (default)	
#define LSM303D_STATUS_ZMOR                        0x40    // 0	:	no overrun has occurred (default)
                                                                // 1	:	new Z-axis data has over written the previous one
#define LSM303D_STATUS_YMOR                        0x20    // 0	:	no overrun has occurred (default)
                                                                // 1	:	new Y-axis data has over written the previous one
#define LSM303D_STATUS_XMOR                        0x10    // 0	:	no overrun has occurred (default)
                                                                // 1	:	new X-axis data has over written the previous one
#define LSM303D_STATUS_ZYXMDA                      0x08    // 0	:	a new set of data is not yet available
                                                                // 1	:	a new set of data is available 
#define LSM303D_STATUS_ZMDA                        0x04    // 0	:	a new data for the Z-Axis is not available
                                                                // 1	:	a new data for the Z-Axis is available
#define LSM303D_STATUS_YMDA                        0x02    // 0	:	a new data for the Y-Axis is not available
                                                                // 1	:	a new data for the Y-Axis is available
#define LSM303D_STATUS_XMDA                        0x01    // 0	:	a new data for the X-Axis is not available
                                                                // 1	:	a new data for the X-Axis is available

#define LSM303D_TEMPOR_BIT                              LSM303D_STATUS_ZYXMOR
#define LSM303D_DATAREADY_BIT_M                         LSM303D_STATUS_ZYXMDA
#define LSM303D_TEMPDA                                  LSM303D_STATUS_ZYXMDA

/***********OUT_*_*_M***********/
//ADDRESS
#define LSM303D_OUT_X_L_M       			0x08
#define LSM303D_OUT_X_H_M       			0x09
#define LSM303D_OUT_Y_L_M       			0x0A
#define LSM303D_OUT_Y_H_M       			0x0B
#define LSM303D_OUT_Z_L_M       			0x0C
#define LSM303D_OUT_Z_H_M       			0x0D

/********************TEMPERATURE REGISTERS*******************/
//ADDRESS
#define LSM303D_OUT_TEMP_L       			0x05
#define LSM303D_OUT_TEMP_H       			0x06


/* Exported macro ------------------------------------------------------------*/

#ifndef __SHARED__MACROS

#define __SHARED__MACROS
#define ValBit(VAR,Place)         (VAR & (1<<Place))
#define BIT(x) ( (x) )

#endif /*__SHARED__MACROS*/


/* Exported functions --------------------------------------------------------*/
//Sensor Configuration Functions
status_t LSM303D_BootEnable(State_t boot);
status_t LSM303D_SetAccODR(LSM303D_ODR_A_t ov);
status_t LSM303D_SetAccMode(LSM303D_Mode_A_t md);	
status_t LSM303D_SetBDU(State_t bdu);
status_t LSM303D_SetAxis(LSM303D_Axis_t axis);
status_t LSM303D_SetAccABW(LSM303D_ABW_A_t abw);
status_t LSM303D_SetAccFullScale(LSM303D_Fullscale_A_t fs);
status_t LSM303D_SetAccSelfTest(State_t st);
status_t LSM303D_SetSPIMode(LSM303D_SPIMode_t spimode);
status_t LSM303D_SetInt1Pin(LSM303D_IntPinConf_t pinConf);
status_t LSM303D_SetInt2Pin(LSM303D_IntPinConf_t pinConf);
status_t LSM303D_EnableTemperature(State_t state);
status_t LSM303D_SetAccRefX(u8_t ref);
status_t LSM303D_SetAccRefY(u8_t ref);
status_t LSM303D_SetAccRefZ(u8_t ref);
status_t LSM303D_SetTemperatureOnly(State_t state);
status_t LSM303D_SetActTHS(u8_t val);
status_t LSM303D_SetActDUR(u8_t val);

//Filtering Functions
status_t LSM303D_SetHPFMode(LSM303D_HPFMode_A_t hpf);
status_t LSM303D_SetAccFilterDataSel(State_t state);
status_t LSM303D_HPFClickEnable(State_t hpfe);
status_t LSM303D_HPFAOI1Enable(State_t hpfe);
status_t LSM303D_HPFAOI2Enable(State_t hpfe);

//Interrupt Functions
status_t LSM303D_SetAccInt1Configuration(LSM303D_IntConf_t ic);
status_t LSM303D_SetAccInt1Mode(LSM303D_IntMode_t ic);

status_t LSM303D_SetInt6D4DConfiguration(LSM303D_INT_6D_4D_t ic);
status_t LSM303D_EnableInt1Latch(State_t latch);
status_t LSM303D_ResetInt1Latch(void);
status_t LSM303D_SetAccInt1Threshold(u8_t ths);
status_t LSM303D_SetAccInt1Duration(u8_t id);
status_t LSM303D_GetAccInt1Src(u8_t* val);
status_t LSM303D_GetAccInt1SrcBit(u8_t statusBIT, u8_t *val);

status_t LSM303D_SetAccInt2Configuration(LSM303D_IntConf_t ic);
status_t LSM303D_SetAccInt2Mode(LSM303D_IntMode_t ic);
status_t LSM303D_EnableInt2Latch(State_t latch);
status_t LSM303D_ResetInt2Latch(void);
status_t LSM303D_SetAccInt2Threshold(u8_t ths);
status_t LSM303D_SetAccInt2Duration(u8_t id);
status_t LSM303D_GetAccInt2Src(u8_t* val);
status_t LSM303D_GetAccInt2SrcBit(u8_t statusBIT);

status_t LSM303D_SetClickCFG(u8_t status);
status_t LSM303D_SetClickTHS(u8_t ths);
status_t LSM303D_SetClickLIMIT(u8_t ths);
status_t LSM303D_SetClickLATENCY(u8_t ths);
status_t LSM303D_SetClickWINDOW(u8_t ths);
status_t LSM303_GetClickResponse(u8_t* val);

//FIFO Functions
status_t LSM303D_FIFOModeEnable(LSM303D_FifoMode_t fm);
status_t LSM303D_FIFOWaterMarkEnable(State_t boot);
status_t LSM303D_FIFOSetWaterMark(u8_t wm); 
status_t LSM303D_GetFifoSourceReg(u8_t* val);
status_t LSM303D_GetFifoSourceBit(u8_t statusBIT, u8_t* val);
status_t LSM303D_GetFifoSourceFSS(u8_t* val);

//Other Reading Functions
status_t LSM303D_Get6DPosition(u8_t* val);
status_t LSM303D_GetTempRaw(i16_t* val);
status_t LSM303D_GetAccStatusReg(u8_t* val);
status_t LSM303D_GetAccStatusBit(u8_t statusBIT, u8_t *val);
status_t LSM303D_GetAccAxesRaw(AxesRaw_t* buff);

//Magnetometer Functions
status_t LSM303D_SetMagFullScale(LSM303D_Fullscale_M_t fs);
status_t LSM303D_SetMagResolution(LSM303D_RES_M_t magres);
status_t LSM303D_SetMagODR(LSM303D_ODR_M_t ov);
status_t LSM303D_SetMagLPMode(State_t state);                
status_t LSM303D_SetMagMode(LSM303D_Mode_M_t Mode);
status_t LSM303D_SetMagOffsetX(i16_t offset);
status_t LSM303D_SetMagOffsetY(i16_t offset);
status_t LSM303D_SetMagOffsetZ(i16_t offset);
status_t LSM303D_SetMagIntConfiguration(LSM303D_IntConf_t ic);
status_t LSM303D_SetMagIntThreshold(i16_t ths); 

status_t LSM303D_GetMagStatusReg(u8_t* val);
status_t LSM303D_GetMagStatusBit(u8_t statusBIT, u8_t* val);
status_t LSM303D_GetMagAxesRaw(AxesRaw_t* buff);
status_t LSM303D_GetMagIntSrcReg(u8_t* val);
status_t LSM303D_GetMagIntSrcBit(u8_t statusBIT, u8_t *val);


//Generic
u8_t LSM303D_ReadReg(u8_t deviceAddr, u8_t Reg, u8_t* Data);
u8_t LSM303D_WriteReg(u8_t deviceAddress, u8_t WriteAddr, u8_t Data); 

#endif /* __LSM303D__H */

/******************* (C) COPYRIGHT 2012 STMicroelectronics *****END OF FILE****/
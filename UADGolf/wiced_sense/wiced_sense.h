/*
 * Copyright 2015, Broadcom Corporation
 * All Rights Reserved.
 *
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Broadcom Corporation;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Broadcom Corporation.
 */

/******************************************************
 *                     Features
 ******************************************************/
// Whether to support the accelerometer
#define WICED_SENSE_SUPPORT_ACCEL           1

// Whether to support gyro or not.
#define WICED_SENSE_SUPPORT_GYRO            1

// Whether to support the humidity sensor
#define WICED_SENSE_SUPPORT_HUMIDITY        0

// Whether to support the magnetometer
#define WICED_SENSE_SUPPORT_MAGNETOMETER    0

// Whether to support the pressure sensor
#define WICED_SENSE_SUPPORT_PRESSURE        0

// Whether to support the temperature sensor
#define WICED_SENSE_SUPPORT_TEMPERATURE     0

// Whether we need to put hte gyro in sleep mode when not in active use
// #define WICED_SENSE_ACTIVATE_GYRO_ON_DENAMD 1

#define WICED_SENSE_NUM_BAT_MEASUREMENTS_TO_AVERAGE   8
/******************************************************
 *                     Types
 ******************************************************/

#pragma pack(1)
//host information for NVRAM
typedef PACKED struct
{
    // BD address of the bonded host
    BD_ADDR  bdaddr;

    // Current value of the client configuration descriptor
    UINT16  characteristic_client_configuration;

}  HOSTINFO;
#pragma pack()

enum
{
	// LED is idle
	WICED_SENSE_LED_STATE_IDLE,

	// LED is in state 1
	WICED_SENSE_LED_STATE_RUNNING,

	// LED is in state 2
	WICED_SENSE_LED_STATE_2,

	// LED is in state 3
	WICED_SENSE_LED_STATE_3,

	// LED is in state 4
	WICED_SENSE_LED_STATE_4,

	// LED is in state 5
	WICED_SENSE_LED_STATE_5,

	// LED is in state 6
	WICED_SENSE_LED_STATE_6,

	// LED state max
	WICED_SENSE_LED_STATE_MAX
};

enum
{
	// Low frequency noise
	WICED_SENSE_BUZZER_LOW,

	// Low-Mid frequency noise
	WICED_SENSE_BUZZER_MID_LOW,

	// Mid frequency noise
	WICED_SENSE_BUZZER_MID,

	// High-Mid frequency noise
	WICED_SENSE_BUZZER_MID_HIGH,

	// High frequency noise
	WICED_SENSE_BUZZER_HIGH
};

/******************************************************
 *                     Constants
 ******************************************************/

// application and version info for this product
#define WICED_SENSE_APP_ID             0x3A20
#define WICED_SENSE_APP_VERSION_MAJOR  1
#define WICED_SENSE_APP_VERSION_MINOR  4

#define BROADCOM_COMPANY_ID            0x0f  			// Broadcom Company Identifier
#define BROADCOM_ADVERTISEMENT_TYPE    0x0202

#define NVRAM_ID_HOST_LIST					      0x10	// ID of the memory block used for NVRAM access

// Please note that all UUIDs need to be reversed when publishing in the database

// {739298B6-87B6-4984-A5DC-BDC18B068985}
// static const GUID UUID_WICED_SENSE_SERVICE               = { 0x739298b6, 0x87b6, 0x4984, { 0xa5, 0xdc, 0xbd, 0xc1, 0x8b, 0x6, 0x89, 0x85 } };
#define UUID_WICED_SENSE_SERVICE                  0x85, 0x89, 0x06, 0x8b, 0xc1, 0xbd, 0xdc, 0xa5, 0x84, 0x49, 0xb6, 0x87, 0xb6, 0x98, 0x92, 0x73

// {33EF9113-3B55-413E-B553-FEA1EAADA459}
// static const GUID UUID_WICED_SENSE_CHARACTERISTIC_NOTIFY = { 0x33ef9113, 0x3b55, 0x413e, { 0xb5, 0x53, 0xfe, 0xa1, 0xea, 0xad, 0xa4, 0x59 } };
#define UUID_WICED_SENSE_CHARACTERISTIC_NOTIFY    0x59, 0xa4, 0xad, 0xea, 0xa1, 0xfe, 0x53, 0xb5, 0x3e, 0x41, 0x55, 0x3b, 0x13, 0x91, 0xef, 0x33

// GATT Database handles
#define HANDLE_WICED_SENSE_SERVICE_UUID                    0x28
#define HANDLE_WICED_SENSE_VALUE_NOTIFY                    0x2a
#define HANDLE_WICED_SENSE_CLIENT_CONFIGURATION_DESCRIPTOR 0x2b
#define HANDLE_WICED_SENSE_BATTERY_LEVEL                   0x63

// Masks for the sensor data included in the notification.
// Mask for the accelerometer
#define WICED_SENSE_NOTIFICATION_HEADER_INCLUDES_ACCELEROMETER   (1 << 0)
// Mask for the Gyro
#define WICED_SENSE_NOTIFICATION_HEADER_INCLUDES_GYRO            (1 << 1)
// Mask for the Humidity sensor
#define WICED_SENSE_NOTIFICATION_HEADER_INCLUDES_HUMIDITY        (1 << 2)
// Mask for the Magnetometer
#define WICED_SENSE_NOTIFICATION_HEADER_INCLUDES_MAGNETOMETER    (1 << 3)
// Mask for the Pressure sensor
#define WICED_SENSE_NOTIFICATION_HEADER_INCLUDES_PRESSURE        (1 << 4)
// Mask for the Temperature sensor
#define WICED_SENSE_NOTIFICATION_HEADER_INCLUDES_TEMPERATURE     (1 << 5)
// Number of finetimeouts to beep the buzzer on some event.
#define WICED_SENSE_NUM_FINE_TIMEOUTS_TO_PLAY                     (5)
// PWM init value for toggling LED1
#define WICED_SENSE_LED1_DEFAULT_INIT_VALUE                       (0x0)
// PWM step size
#define WICED_SENSE_LED1_DEFAULT_STEP                              (0x20)
// PWM start value
#define WICED_SENSE_LED1_TOGGLE_VALUE_START                        (0x20)
// Full battery level
#define WICED_SENSE_BATTERY_LEVEL_FULL                             (3000)
// Empty battery level
#define WICED_SENSE_BATTERY_LEVEL_EMPTY                             (1800)

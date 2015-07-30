/*
 * Copyright 2015, Broadcom Corporation
 * All Rights Reserved.
 *
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Broadcom Corporation;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Broadcom Corporation.
 */

/** @file
*
* Broadcom WICED Sense sample application
*
* Application uses the sensors on the Broadcom reference
* WICED Sense board to connect to the reference peer application
* to demonstrate sensor capabilities.
*  - Demonstrates the use of I2C interface to read sensor data
*  - Demonstrates vendor specific service and characteristic to
*       notify multiple sensor data to the peer app
*  - Use of advance TX opportunity so the sensors can be polled just in time
*  		to conserve power
*  - Use of deep sleep.
*  - Secure over the air upgrade of the firmware
*  - Use of the RSA library to calculate the SHA256 hash and verify RSA signature
*
* To demonstrate the app, work through the following steps.
* 1. Remove battery from and connect the WICED Sense board over USB.
* 2. If you do not want to provide Over-the-Air Secure Firmware Upgrade
*    build and download the application to the WICED Sense board.
*
* Over-the-Air Secure Firmware Upgrade feature, is described in details in
* the Doc/WICED-Secure-Over-the-Air-Firmware-Upgrade.pdf.
*
* 2. Add OTA_SECURE_UPGRADE=1 to the Make Target for example
*    "wiced_sense-BCM920737TAG_Q32 OTA_SECURE_UPGRADE=1 download"
* 3. Generate rsa_pub.c using WsRsaKeyGen and copy to the project directory
* 5. Build and download the application to the WICED Sense board.
* 6. To securely update the application:
* 	6a. Sign the build output
*       wiced_sense-BCM920737TAG_Q32-rom-ram-Wiced-release.ota.bin
*    	file using WsRsaSign utility
* 	6b. Pair with a client
* 	6c. Use WsSecOtaUpgrade application to try over the air upgrade
*    	wiced_sense-BCM920737TAG_Q32-rom-ram-Wiced-release.ota.bin.signed
*    	file created during the build.
*
*/
// Enable/disable tracing
#define BLE_TRACE_DISABLE

#include "bleprofile.h"
#include "gpiodriver.h"
#include "string.h"
#include "platform.h"
#include "wiced_sense.h"
#include "bleapp.h"
#include "bleappconfig.h"
#include "bleappfwu.h"
#include "i2cm.h"
#include "sparcommon.h"
#include "devicelpm.h"
#include "miadriver.h"
#include "pwm.h"
#include "rtc.h"
#include "adc.h"

// Include sensor headers
#include "hts221_driver.h"
#include "l3gd20_driver.h"
#include "lis3dsh_driver.h"
#include "lps25h_driver.h"
#include "lsm303d_driver.h"

// include secure firmware upgrade feature
#ifdef OTA_SECURE_UPGRADE
#include "ws_sec_upgrade_ota.h"
#endif

#include "thread_and_mem_mgmt.h"

/******************************************************
 *                     Structures
 ******************************************************/

/******************************************************
 *                      Constants
 ******************************************************/

/******************************************************
 *               Function Prototypes
 ******************************************************/
static void  wiced_sense_create(void);
static void  wiced_sense_timeout( UINT32 count );
static void  wiced_sense_fine_timeout( UINT32 finecount );
static void  wiced_sense_connection_up( void );
static void  wiced_sense_connection_down( void );
static void  wiced_sense_advertisement_stopped( void );
static void  wiced_sense_smp_bond_result( LESMP_PARING_RESULT result );
static void  wiced_sense_encryption_changed( HCI_EVT_HDR *evt );
static int   wiced_sense_write_handler( LEGATTDB_ENTRY_HDR *p );
static void  wiced_sense_gpio_interrupt_handler(void* unsued, UINT8 gpio);
static void  wiced_sense_next_led1_state(void);
static void  wiced_sense_stop_led1(void);
static void  wiced_sense_start_led1(void);

static void  wiced_sense_short_beep_buzzer(UINT8 frequency);
static void  wiced_sense_start_led2(void);
static void  wiced_sense_stop_led2(void);
static void  wiced_sense_toggle_led2(void);

static void wiced_sense_scale_battery_level_to_percentage_and_update(void);
static void wiced_sense_measure_battery_level_and_average(void);
static void  wiced_sense_initialize_sensors(void);

static void wiced_sense_enter_hidoff(void);
static void wiced_sense_abort_hidoff(void);

#if WICED_SENSE_SUPPORT_ACCEL
static void     wiced_sense_initialize_lis3dsh(void);
static status_t wiced_sense_get_accelerometer_instantaneous_data(AxesRaw_t* dataA);
#endif

#if WICED_SENSE_SUPPORT_MAGNETOMETER
static void     wiced_sense_initialize_lsm303d(void);
static status_t wiced_sense_get_magnetometer_instantaneous_data(AxesRaw_t* dataM);
#endif

#if WICED_SENSE_SUPPORT_PRESSURE || WICED_SENSE_SUPPORT_TEMPERATURE
static void     wiced_sense_initialize_lps25h(void);
static status_t wiced_sense_get_pressure_temp_instantaneous_data(LPS25H_MeasureTypeDef_st* data);
#endif

#if WICED_SENSE_SUPPORT_GYRO
static void     wiced_sense_initialize_l3gd20(void);
static status_t wiced_sense_get_gyro_instantaneous_data(AxesRaw_t* gdata);
#endif

#if WICED_SENSE_SUPPORT_HUMIDITY
static void     wiced_sense_initialize_hts221(void);
static status_t wiced_sense_get_humidity_temp_instantaneous_data(UINT16* humidity, INT16* temperature);
#endif

static void   wiced_sense_power_down_sensors(void);

static UINT8  wiced_sense_poll_hpt_and_fill_notification_packet(UINT16* data_ptr, UINT8* size);
static UINT8  wiced_sense_poll_agm_and_fill_notification_packet(UINT16* data_ptr, UINT8* size);

// Notification that a TX opportunity is coming up
static void   wiced_sense_tx_opportunity_notice(void* context, UINT32 unused);

/******************************************************
 *               Import Function Prototypes
 ******************************************************/
extern void blecm_connectionEventNotifiationInit(void);
extern void blecm_connectionEventNotifiationEnable(void (*clientCallback)(void*, UINT32),
        UINT32 clientContext,
        INT16 offset,
        UINT16 defaultPeriod, UINT32 connHandle);
extern void blecm_connectionEventNotifiationDisable(void);

/******************************************************
 *               Variables Definitions
 ******************************************************/
#ifdef OTA_SECURE_UPGRADE
const WS_UPGRADE_APP_INFO WsUpgradeAppInfo =
{
    /* ID            = */ WICED_SENSE_APP_ID,
    /* Version_Major = */ WICED_SENSE_APP_VERSION_MAJOR,
    /* Version_Minor = */ WICED_SENSE_APP_VERSION_MINOR,
};
#endif

/*
 * This is the GATT database for the WICED Sense application.  It is
 * currently empty with only required GAP and GATT services.
 */
const UINT8 wiced_sense_gatt_database[]=
{
    // Handle 0x01: GATT service
	// Service change characteristic is optional and is not present
    PRIMARY_SERVICE_UUID16 (0x0001, UUID_SERVICE_GATT),

    // Handle 0x14: GAP service
    // Device Name and Appearance are mandatory characteristics.  Peripheral
    // Privacy Flag only required if privacy feature is supported.  Reconnection
    // Address is optional and only when privacy feature is supported.
    // Peripheral Preferred Connection Parameters characteristic is optional
    // and not present.
    PRIMARY_SERVICE_UUID16 (0x0014, UUID_SERVICE_GAP),

    // Handle 0x15: characteristic Device Name, handle 0x16 characteristic value.
    // Any 16 byte string can be used to identify the sensor.  Just need to
    // replace the "Hello" string below.  Keep it short so that it fits in
    // advertisement data along with 16 byte UUID.
    CHARACTERISTIC_UUID16 (0x0015, 0x0016, UUID_CHARACTERISTIC_DEVICE_NAME,
    					   LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE, 16),
       'W','I','C','E','D',' ','S','e','n','s','e',' ','K','i','t',0x00,

    // Handle 0x17: characteristic Appearance, handle 0x18 characteristic value.
    // List of approved appearances is available at bluetooth.org.  Current
    // value is set to 0x200 - Generic Tag
    CHARACTERISTIC_UUID16 (0x0017, 0x0018, UUID_CHARACTERISTIC_APPEARANCE,
    					   LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE, 2),
        BIT16_TO_8(APPEARANCE_GENERIC_TAG),

    // Handle 0x28: WICED Sense Service.
    // This is the main proprietary service of WICED SEnse. Note that
    // UUID of the vendor specific service is 16 bytes, unlike standard Bluetooth
    // UUIDs which are 2 bytes.  _UUID128 version of the macro should be used.
    PRIMARY_SERVICE_UUID128 (HANDLE_WICED_SENSE_SERVICE_UUID, UUID_WICED_SENSE_SERVICE),

    // Handle 0x29: characteristic Sensor Notification, handle 0x2a characteristic value
    // we support both notification and indication.  Peer need to allow notifications
    // or indications by writing in the Characteristic Client Configuration Descriptor
    // (see handle 2b below).  Note that UUID of the vendor specific characteristic is
    // 16 bytes, unlike standard Bluetooth UUIDs which are 2 bytes.  _UUID128 version
    // of the macro should be used.
    CHARACTERISTIC_UUID128 (0x0029, HANDLE_WICED_SENSE_VALUE_NOTIFY, UUID_WICED_SENSE_CHARACTERISTIC_NOTIFY,
                           LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_NOTIFY,
                           LEGATTDB_PERM_READABLE, 20),
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,

	// Handle 0x2b: Characteristic Client Configuration Descriptor.
    // This is standard GATT characteristic descriptor.  2 byte value 0 means that
    // message to the client is disabled.  Peer can write value 1 or 2 to enable
    // notifications or indications respectively.  Not _WRITABLE in the macro.  This
    // means that attribute can be written by the peer.
    CHAR_DESCRIPTOR_UUID16_WRITABLE (HANDLE_WICED_SENSE_CLIENT_CONFIGURATION_DESCRIPTOR,
                                     UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                     LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ, 2),
        0x00,0x00,

    // Handle 0x4d: Device Info service
    // Device Information service helps peer to identify manufacture or vendor
    // of the device.  It is required for some types of the devices (for example HID,
    // and medical, and optional for others.  There are a bunch of characteristics
    // available, out of which Hello Sensor implements 3.
    PRIMARY_SERVICE_UUID16 (0x004d, UUID_SERVICE_DEVICE_INFORMATION),

    // Handle 0x4e: characteristic Manufacturer Name, handle 0x4f characteristic value
    CHARACTERISTIC_UUID16 (0x004e, 0x004f, UUID_CHARACTERISTIC_MANUFACTURER_NAME_STRING, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE, 8),
        'B','r','o','a','d','c','o','m',

    // Handle 0x50: characteristic Model Number, handle 0x51 characteristic value
    CHARACTERISTIC_UUID16 (0x0050, 0x0051, UUID_CHARACTERISTIC_MODEL_NUMBER_STRING, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE, 8),
        '0','0','0','1',0x00,0x00,0x00,0x00,

    // Handle 0x52: characteristic System ID, handle 0x53 characteristic value
    CHARACTERISTIC_UUID16 (0x0052, 0x0053, UUID_CHARACTERISTIC_SYSTEM_ID, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE, 8),
        0xa5,0xdc,0xbd,0xc1,0x8b,0x6,0x89,0x85,

    // Handle 0x61: Battery service
    // This is an optional service which allows peer to read current battery level.
    PRIMARY_SERVICE_UUID16 (0x0061, UUID_SERVICE_BATTERY),

    // Handle 0x62: characteristic Battery Level, handle 0x63 characteristic value
    CHARACTERISTIC_UUID16 (0x0062, 0x0063, UUID_CHARACTERISTIC_BATTERY_LEVEL,
                           LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE, 1),
        0x64,

#ifdef OTA_SECURE_UPGRADE
   	// Handle 0xff00: Broadcom vendor specific WICED Smart Secure Upgrade Service.
   	// The service has 2 characteristics.  The first is the control point.  Client
   	// sends commands, and sensor sends status notifications. Note that
   	// UUID of the vendor specific service is 16 bytes, unlike standard Bluetooth
   	// UUIDs which are 2 bytes.  _UUID128 version of the macro should be used.
   	PRIMARY_SERVICE_UUID128 (HANDLE_WS_UPGRADE_SERVICE, UUID_WS_SECURE_UPGRADE_SERVICE),

   	// Handle 0xff01: characteristic WS Control Point, handle 0xff02 characteristic value.
   	// This characteristic can be used by the client to send commands to this device
   	// and to send status notifications back to the client.  Client has to enable
   	// notifications by updating Characteristic Client Configuration Descriptor
   	// (see handle ff03 below).  Note that UUID of the vendor specific characteristic is
   	// 16 bytes, unlike standard Bluetooth UUIDs which are 2 bytes.  _UUID128 version
   	// of the macro should be used.  Also note that characteristic has to be _WRITABLE
   	// to correctly enable writes from the client.
   	CHARACTERISTIC_UUID128_WRITABLE (HANDLE_WS_UPGRADE_CHARACTERISTIC_CONTROL_POINT,
   									 HANDLE_WS_UPGRADE_CONTROL_POINT, UUID_WS_SECURE_UPGRADE_CHARACTERISTIC_CONTROL_POINT,
   									 LEGATTDB_CHAR_PROP_WRITE | LEGATTDB_CHAR_PROP_NOTIFY,
   									 LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_VARIABLE_LENGTH, 5),
   		0x00,0x00,0x00,0x00,0x00,

   	// Handle 0xff03: Characteristic Client Configuration Descriptor.
   	// This is a standard GATT characteristic descriptor.  2 byte value 0 means that
   	// message to the client is disabled.  Peer can write value 1 to enable
   	// notifications or respectively.  Note _WRITABLE in the macro.  This
   	// means that attribute can be written by the peer.
   	CHAR_DESCRIPTOR_UUID16_WRITABLE (HANDLE_WS_UPGRADE_CLIENT_CONFIGURATION_DESCRIPTOR,
   									 UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
   									 LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ, 2),
   		0x00,0x00,

   	// Handle 0xff04: characteristic WS Data, handle 0xff05 characteristic value
   	// This characteristic is used to send next portion of the FW.  Similar to the
   	// control point, characteristic should be _WRITABLE and 128bit version of UUID is used.
   	CHARACTERISTIC_UUID128_WRITABLE (HANDLE_WS_UPGRADE_CHARACTERISTIC_DATA,
   									 HANDLE_WS_UPGRADE_DATA, UUID_WS_SECURE_UPGRADE_CHARACTERISTIC_DATA,
   									 LEGATTDB_CHAR_PROP_WRITE,
   									 LEGATTDB_PERM_VARIABLE_LENGTH | LEGATTDB_PERM_WRITE_REQ,  20),
   		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
   		0x00,0x00,0x00,0x00,

   	// Handle 0xff06: characteristic Application Info, handle 0xff07 characteristic value
   	// Client can read value of this characteristic to figure out which application id is
   	// running as well as version information.  Characteristic UUID is 128 bits.
   	CHARACTERISTIC_UUID128 (HANDLE_WS_UPGRADE_CHARACTERISTIC_APP_INFO,
   							HANDLE_WS_UPGRADE_APP_INFO, UUID_WS_SECURE_UPGRADE_CHARACTERISTIC_APP_INFO,
   							LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE,  4),
   		 WICED_SENSE_APP_ID & 0xff, (WICED_SENSE_APP_ID >> 8) & 0xff, WICED_SENSE_APP_VERSION_MAJOR, WICED_SENSE_APP_VERSION_MINOR,
#endif
};

/*
 * This is the application configuration.
 * */
const BLE_PROFILE_CFG wiced_sense_cfg =
{
        /*.fine_timer_interval            =*/ 12.5, // ms
        /*.default_adv                    =*/ NO_DISCOVERABLE, // current sensor is not discoverable
        /*.button_adv_toggle              =*/ 0,    // pairing button make adv toggle (if 1) or always on (if 0)
        /*.high_undirect_adv_interval     =*/ 32,   // slots
        /*.low_undirect_adv_interval      =*/ 1024, // slots
        /*.high_undirect_adv_duration     =*/ 10,   // seconds
        /*.low_undirect_adv_duration      =*/ 10,  // seconds
        /*.high_direct_adv_interval       =*/ 0,    // seconds
        /*.low_direct_adv_interval        =*/ 0,    // seconds
        /*.high_direct_adv_duration       =*/ 0,    // seconds
        /*.low_direct_adv_duration        =*/ 0,    // seconds
        /*.local_name                     =*/ "WICED Sense Kit",  // [LOCAL_NAME_LEN_MAX];
        /*.cod                            =*/ BIT16_TO_8(APPEARANCE_GENERIC_TAG),0x00, // [COD_LEN];
        /*.ver                            =*/ "1.1",         // [VERSION_LEN];
        /*.encr_required                  =*/ 1,    // data encrypted and device sends security request on every connection
        /*.disc_required                  =*/ 0,    // if 1, disconnection after confirmation
        /*.test_enable                    =*/ 1,    // TEST MODE is enabled when 1
        /*.tx_power_level                 =*/ 0x04, // dbm
        /*.con_idle_timeout               =*/ 3,    // second  0-> no timeout
        /*.powersave_timeout              =*/ 0,    // second  0-> no timeout
        /*.hdl                            =*/ {0x00, 0x00, 0x00, 0x00, 0x00}, // [HANDLE_NUM_MAX];
        /*.serv                           =*/ {0x00, 0x00, 0x00, 0x00, 0x00},
        /*.cha                            =*/ {0x00, 0x00, 0x00, 0x00, 0x00},
        /*.findme_locator_enable          =*/ 0,    // if 1 Find me locator is enable
        /*.findme_alert_level             =*/ 0,    // alert level of find me
        /*.client_grouptype_enable        =*/ 0,    // if 1 grouptype read can be used
        /*.linkloss_button_enable         =*/ 0,    // if 1 linkloss button is enable
        /*.pathloss_check_interval        =*/ 0,    // second
        /*.alert_interval                 =*/ 0,    // interval of alert
        /*.high_alert_num                 =*/ 0,    // number of alert for each interval
        /*.mild_alert_num                 =*/ 0,    // number of alert for each interval
        /*.status_led_enable              =*/ 0,    // if 1 status LED is enable
        /*.status_led_interval            =*/ 0,    // second
        /*.status_led_con_blink           =*/ 0,    // blink num of connection
        /*.status_led_dir_adv_blink       =*/ 0,    // blink num of dir adv
        /*.status_led_un_adv_blink        =*/ 0,    // blink num of undir adv
        /*.led_on_ms                      =*/ 0,    // led blink on duration in ms
        /*.led_off_ms                     =*/ 0,    // led blink off duration in ms
        /*.buz_on_ms                      =*/ 0,    // buzzer on duration in ms
        /*.button_power_timeout           =*/ 0,    // seconds
        /*.button_client_timeout          =*/ 0,    // seconds
        /*.button_discover_timeout        =*/ 0,    // seconds
        /*.button_filter_timeout          =*/ 0,    // seconds
    #ifdef BLE_UART_LOOPBACK_TRACE
        /*.button_uart_timeout            =*/ 15,   // seconds
    #endif
};

// Following structure defines UART configuration
const BLE_PROFILE_PUART_CFG wiced_sense_puart_cfg =
{
    /*.baudrate   =*/ 115200,
    /*.txpin      =*/ GPIO_PIN_UART_TX,
    /*.rxpin      =*/ GPIO_PIN_UART_RX,
};

// Following structure defines GPIO configuration used by the application
const BLE_PROFILE_GPIO_CFG wiced_sense_gpio_cfg =
{
    /*.gpio_pin =*/
    {
        GPIO_PIN_WP,      // This need to be used to enable/disable NVRAM write protect
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 // other GPIOs are not used
    },
    /*.gpio_flag =*/
    {
        GPIO_SETTINGS_WP,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    }
};

// Remote address when connected.
BD_ADDR wiced_sense_remote_addr          = {0, 0, 0, 0, 0, 0};
UINT16 	wiced_sense_connection_handle    = 0;   // HCI handle of connection, not zero when connected
UINT8   wiced_sense_num_to_write         = 0;   // Number of messages we need to send
UINT8   wiced_sense_pwm_playing          = 0;   // Buzzer tone is playing
UINT8   wiced_sense_led1_state           = WICED_SENSE_LED_STATE_IDLE;   // LED1 blink state
UINT16  wiced_sense_led1_toggle_value    = 0;   // PWM starting toggle value for LED1
UINT16  wiced_sense_led1_init_value      = 0;   // PWM starting init value for LED1
UINT8   wiced_sense_led1_playing         = WICED_SENSE_NUM_FINE_TIMEOUTS_TO_PLAY;  // Number of timeouts to play LED1 at a given PWM freq
UINT8   wiced_sense_led2_state           = 0;   // LED2 state
UINT8   wiced_sense_polls_since_last_hpt = 0;   // Number of TX opportunities since we sent Humidity, Pressure and Temp notification
UINT8   wiced_sense_send_data            = 0;   // Whether to send data or not.
// NVRAM save area
HOSTINFO wiced_sense_hostinfo;
INT16   wiced_sense_battery_measurements[WICED_SENSE_NUM_BAT_MEASUREMENTS_TO_AVERAGE];
UINT8   wiced_sense_battery_measurement_count = 0;
INT32   wiced_sense_battery_measurements_sum = 0;
UINT8   wiced_sense_battery_measurements_oldest_index = 0;
INT16   wiced_sense_battery_measurements_average = 0;
UINT8   wiced_sense_battery_level_in_percentage = 0;
/******************************************************
 *               Function Definitions
 ******************************************************/

// Application initialization
APPLICATION_INIT()
{
	// RSA stack requires to increase the stack size.  Make it 4K.
	blecm_SetApplicationThreadStackSizeInWords(1024);

	// Also RSA code requires a bit larger data size than default 264 bytes.
	cfa_mm_ConfigureMemoryPool(CFA_MM_POOL_2, 272, 10);

    bleapp_set_cfg((UINT8 *)wiced_sense_gatt_database,
                   sizeof(wiced_sense_gatt_database),
                   (void *)&wiced_sense_cfg,
                   (void *)&wiced_sense_puart_cfg,
                   (void *)&wiced_sense_gpio_cfg,
                   wiced_sense_create);
#ifdef BLE_TRACE_DISABLE
    BLE_APP_DISABLE_TRACING();
#endif

}

// Create WICED Sense application
void wiced_sense_create(void)
{
    extern UINT32 blecm_configFlag;
    BLEPROFILE_DB_PDU db_pdu;
    UINT32 i;

    // If timed wake is enabled, clear P39 interrupt if waking from timed wake.
    gpio_clearPinInterruptStatus(GPIO_PIN_P39 / GPIO_MAX_NUM_PINS_PER_PORT, GPIO_PIN_P39 % GPIO_MAX_NUM_PINS_PER_PORT);

    // No logs.
    // blecm_configFlag |= BLECM_DBGUART_LOG;

    ble_trace0("wiced_sense_create()\n");

    // to verify that RSA does not corrupt the stack, initialize the stack checking.
    // Actual check is called after RSA is done with the signature verification.
    blecm_StackCheckInit();

    // dump the database to debug uart.
#ifndef BLE_TRACE_DISABLE
    legattdb_dumpDb();
#endif

    bleprofile_Init(bleprofile_p_cfg);
    bleprofile_GPIOInit(bleprofile_gpio_p_cfg);

	// Disable GPIOs double bonded with the ones we plan to use in this app.
	gpio_configurePin((GPIO_PIN_P10) / 16, (GPIO_PIN_P10) % 16, GPIO_INPUT_DISABLE, 0);
	gpio_configurePin((GPIO_PIN_P11) / 16, (GPIO_PIN_P11) % 16, GPIO_INPUT_DISABLE, 0);
	gpio_configurePin((GPIO_PIN_P28) / 16, (GPIO_PIN_P28) % 16, GPIO_PULL_DOWN, 0);
	gpio_configurePin((GPIO_PIN_P38) / 16, (GPIO_PIN_P38) % 16, GPIO_INPUT_DISABLE, 0);

    // Initialize connection event notification mechanism. When connected, we will register
	// with the FW to receive notifications of upcoming TX opportunities.
    blecm_connectionEventNotifiationInit();

    // register connection up and connection down handler.
    bleprofile_regAppEvtHandler(BLECM_APP_EVT_LINK_UP, wiced_sense_connection_up);
    bleprofile_regAppEvtHandler(BLECM_APP_EVT_LINK_DOWN, wiced_sense_connection_down);
    bleprofile_regAppEvtHandler(BLECM_APP_EVT_ADV_TIMEOUT, wiced_sense_advertisement_stopped);

    bleprofile_regAppEvtHandler(BLECM_APP_EVT_ENTERING_HIDOFF,(BLECM_NO_PARAM_FUNC)wiced_sense_enter_hidoff);
    bleprofile_regAppEvtHandler(BLECM_APP_EVT_ABORTING_HIDOFF,(BLECM_NO_PARAM_FUNC)wiced_sense_abort_hidoff);

    // handler for Encryption changed.
    blecm_regEncryptionChangedHandler(wiced_sense_encryption_changed);

    // handler for Bond result
    lesmp_regSMPResultCb((LESMP_SINGLE_PARAM_CB) wiced_sense_smp_bond_result);

    // register to process client writes
    legattdb_regWriteHandleCb((LEGATTDB_WRITE_CB)wiced_sense_write_handler);

    // Initializer the RTC so we can use it for deep sleep/timed wake if required.
    rtc_init();

    // Read value of the service from GATT DB.
    bleprofile_ReadHandle(HANDLE_WICED_SENSE_SERVICE_UUID, &db_pdu);
    ble_tracen((char *)db_pdu.pdu, db_pdu.len);

	// total length should be less than 31 bytes
	BLE_ADV_FIELD adv[3];

	// flags
	adv[0].len      = 1 + 1;
	adv[0].val      = ADV_FLAGS;
	adv[0].data[0]  = LE_LIMITED_DISCOVERABLE | BR_EDR_NOT_SUPPORTED;

	adv[1].len      = 8 + 1;
    adv[1].val      = ADV_MANUFACTURER_DATA; 				// (AD_TYPE == 0xff)
    adv[1].data[0]  = BROADCOM_COMPANY_ID & 0xff;  			// Apple (Company Identifier 2 bytes)
    adv[1].data[1]  = (BROADCOM_COMPANY_ID >> 8) & 0xff;
    adv[1].data[2]  = BROADCOM_ADVERTISEMENT_TYPE & 0xff;
	adv[1].data[3]  = (BROADCOM_ADVERTISEMENT_TYPE >> 8) & 0xff;
	adv[1].data[6]  = WICED_SENSE_APP_ID & 0xff;  			    // ProductID (2 bytes)
	adv[1].data[7]  = (WICED_SENSE_APP_ID >> 8) & 0xff;

	// name
	adv[2].len      = strlen(bleprofile_p_cfg->local_name) + 1;
	adv[2].val      = ADV_LOCAL_NAME_COMP;
	memcpy(adv[2].data, bleprofile_p_cfg->local_name, adv[2].len - 1);

	bleprofile_GenerateADVData(adv, 3);

	// Set TX power when connected/advertising to 0 dBm.
    blecm_setTxPowerInADV(0);
    blecm_setTxPowerInConnection(0);

    // Become discoverable (start adv).
    bleprofile_Discoverable(HIGH_UNDIRECTED_DISCOVERABLE, wiced_sense_remote_addr);

    // We are discoverable, start blinking LED1
    wiced_sense_start_led1();

    // Start timers
    bleprofile_regTimerCb(wiced_sense_fine_timeout, wiced_sense_timeout);
    bleprofile_StartTimer();

    // initialize ADC
    adc_config();

    // Fill up the average measurements
    for(i = 0; i < WICED_SENSE_NUM_BAT_MEASUREMENTS_TO_AVERAGE; i++)
    {
    	// Get the first few battery level readings.
    	wiced_sense_measure_battery_level_and_average();
    }

    // Power down all sensors.
    wiced_sense_power_down_sensors();

#ifdef OTA_SECURE_UPGRADE
    // initialize support for secure upgrade
    ws_upgrade_ota_init();
#endif

    // Register for interrupts from GPIOs
    {
    	UINT16 masks[3] = {(1 << 4), 0 , 0};
    	gpio_registerForInterrupt(masks, wiced_sense_gpio_interrupt_handler, 0);
    }
}

// Starts LED1
void wiced_sense_start_led1(void)
{
	wiced_sense_led1_state = WICED_SENSE_LED_STATE_RUNNING;

	// Set to initial toggle and init values for the PWM
	wiced_sense_led1_toggle_value = WICED_SENSE_LED1_TOGGLE_VALUE_START;
	wiced_sense_led1_init_value = WICED_SENSE_LED1_DEFAULT_INIT_VALUE;

	// Bring the PWM out of reset state
	pwm_setReset(1 << PWM1, 0);

    // Start the LED state machine.
    wiced_sense_next_led1_state();
}

// Run the LED state machine
void wiced_sense_next_led1_state(void)
{
	switch(wiced_sense_led1_state)
	{
	case WICED_SENSE_LED_STATE_RUNNING:
		pwm_start(PWM1, LHL_CLK, wiced_sense_led1_toggle_value, wiced_sense_led1_init_value);
		break;
	default:
		break;
	}

	// Enable output on the PWM to drive the LED.
	gpio_configurePin((GPIO_PIN_P27) / 16, (GPIO_PIN_P27) % 16, PWM1_OUTPUT_ENABLE_P27, 0);

	// Move to the next state
	wiced_sense_led1_toggle_value += WICED_SENSE_LED1_DEFAULT_STEP;

	// Once we reach the max brightness we want, start with the lowest brightness.
	if(wiced_sense_led1_toggle_value > (0x140))
		wiced_sense_led1_toggle_value = WICED_SENSE_LED1_TOGGLE_VALUE_START;
}

// Stops LED1.
void wiced_sense_stop_led1(void)
{
	// Stop the LED
	pwm_setReset(1 << PWM1, 1);
	gpio_configurePin((GPIO_PIN_P27) / 16, (GPIO_PIN_P27) % 16, GPIO_INPUT_ENABLE | GPIO_PULL_UP, 0);

	wiced_sense_led1_state = WICED_SENSE_LED_STATE_IDLE;
	wiced_sense_led1_playing = WICED_SENSE_NUM_FINE_TIMEOUTS_TO_PLAY;
}

// Start LED2 PWM. LED 2 draws a lot of current. Disable for now.
void wiced_sense_start_led2(void)
{
//	pwm_setReset(1 << PWM2, 0);
//
//	pwm_start(PWM2, LHL_CLK, 0x280, 0x50);
//
//	gpio_configurePin((GPIO_PIN_P14) / 16, (GPIO_PIN_P14) % 16, PWM2_OUTPUT_ENABLE_P14, 0);
}

// Stops LED2.
void wiced_sense_stop_led2(void)
{
//	// Stop the LED
//	pwm_setReset(1 << PWM2, 1);
//	gpio_configurePin((GPIO_PIN_P14) / 16, (GPIO_PIN_P14) % 16, GPIO_INPUT_ENABLE | GPIO_PULL_UP, 0);
}

// Toggles the state of LED2.
void wiced_sense_toggle_led2(void)
{
	if(wiced_sense_led2_state)
		wiced_sense_stop_led2();
	else
		wiced_sense_start_led2();

	wiced_sense_led2_state ^= 1;
}

// Starts a beep on the buzzer.
void wiced_sense_short_beep_buzzer(UINT8 frequency)
{
    pwm_setReset(1 << PWM3, 0);

    switch(frequency)
    {
    case WICED_SENSE_BUZZER_LOW:
    	pwm_start(PWM3, LHL_CLK, 0x3BF, 0x37E);
    	break;
    case WICED_SENSE_BUZZER_MID_LOW:
    	pwm_start(PWM3, LHL_CLK, 0x360, 0x300);
    	break;
    case WICED_SENSE_BUZZER_MID:
    	pwm_start(PWM3, LHL_CLK, 0x2FF, 0x200);
    	break;
    case WICED_SENSE_BUZZER_MID_HIGH:
    	pwm_start(PWM3, LHL_CLK, 0x2FF, 0x100);
    	break;
    case WICED_SENSE_BUZZER_HIGH:
    	pwm_start(PWM3, LHL_CLK, 0x2FF, 0x000);
    	break;
    }

	// Enable Output of the PWM
	gpio_configurePin((GPIO_PIN_P13) / 16, (GPIO_PIN_P13) % 16, PWM3_OUTPUT_ENABLE_P13, 0);

	// Set PWM state
	wiced_sense_pwm_playing = WICED_SENSE_NUM_FINE_TIMEOUTS_TO_PLAY;
}

// This function will be called on every connection establishment
void wiced_sense_connection_up(void)
{
	wiced_sense_connection_handle = (UINT16)emconinfo_getConnHandle();

    // Initialize all sensors
	wiced_sense_initialize_sensors();

	// The gyro draws a lot of current when active. Allow turning
	// it on off on-demand (seems to be a bit too noisy when enabled).
#if WICED_SENSE_ACTIVATE_GYRO_ON_DENAMD && WICED_SENSE_SUPPORT_GYRO
	// Put Gyro in sleep
	if(L3GD20_SetMode(L3GD20_SLEEP) == MEMS_SUCCESS)
		ble_trace0("L3GD20_SetMode Successful");
	else
		ble_trace0("L3GD20_SetMode Failed.");
#endif

	// Request a connection interval that seems to be responsive enough,
	// yet, a bit mindful of power.
	//bleprofile_SendConnParamUpdateReq(64, 96, 0, 500);
	bleprofile_SendConnParamUpdateReq(10.0, 10.0, 0, 500);
	// Register for notifications from the baseband of upcoming TX opportunities.
	// wiced_sense_tx_opportunity_notice - the callback to be invoked
	// No context needs to be supplied back
	// notifications needed 6.25ms before the TX opportunity (so we have enough time to poll sensors
	//         and enqueue the notification)  = 6250uS / 625uS in BT slots
	// if not connected, notify every 30ms = 30000us/625 in BT slots (will not be used because we enable notis only when connected).
	// wiced_sense_connection_handle - the connection handle of the connection for which we need the notis.
	blecm_connectionEventNotifiationEnable((void (*)(void*, UINT32))wiced_sense_tx_opportunity_notice, 0, 6250/625, 30000/625, wiced_sense_connection_handle);

    // save address of the connected device and print it out.
    memcpy(wiced_sense_remote_addr, (UINT8 *)emconninfo_getPeerAddr(), sizeof(wiced_sense_remote_addr));

    ble_trace3("wiced_sense_connection_up: %08x%04x %d\n",
                (wiced_sense_remote_addr[5] << 24) + (wiced_sense_remote_addr[4] << 16) +
                (wiced_sense_remote_addr[3] << 8) + wiced_sense_remote_addr[2],
                (wiced_sense_remote_addr[1] << 8) + wiced_sense_remote_addr[0],
                wiced_sense_connection_handle);

    // Stop advertising
    bleprofile_Discoverable(NO_DISCOVERABLE, NULL);

    bleprofile_StopConnIdleTimer();

    // as we require security for every connection, we will not send any notificationss until
    // encryption is done.
    if (bleprofile_p_cfg->encr_required != 0)
    {
    	if (emconninfo_deviceBonded())
    	{
            ble_trace0("device bonded");
    	}
    	else
    	{
    	    ble_trace0("device not bonded");
            lesmp_sendSecurityRequest();
    	}
        return;
    }

    // Beep a low frequency tone on the buzzer
    wiced_sense_short_beep_buzzer(WICED_SENSE_BUZZER_LOW);

    // Stop LED1 and start LED2
	wiced_sense_stop_led1();
	wiced_sense_start_led2();
}


// This function will be called when connection goes down
void wiced_sense_connection_down(void)
{
	// Power down all sensors to save some power.
	wiced_sense_power_down_sensors();

	wiced_sense_send_data = 0;

	// Disable connection event notification.
	blecm_connectionEventNotifiationDisable();

    ble_trace3("wiced_sense_connection_down:%08x%04x handle:%d\n",
                (wiced_sense_remote_addr[5] << 24) + (wiced_sense_remote_addr[4] << 16) +
                (wiced_sense_remote_addr[3] << 8) + wiced_sense_remote_addr[2],
                (wiced_sense_remote_addr[1] << 8) + wiced_sense_remote_addr[0],
                wiced_sense_connection_handle);

	memset (wiced_sense_remote_addr, 0, 6);

	wiced_sense_connection_handle = 0;
	wiced_sense_polls_since_last_hpt = 0;

	// Start advertisements again.
	bleprofile_Discoverable(HIGH_UNDIRECTED_DISCOVERABLE, wiced_sense_remote_addr);

	gpio_configurePin((GPIO_PIN_P13) / 16, (GPIO_PIN_P13) % 16, GPIO_INPUT_ENABLE | GPIO_PULL_DOWN, 0);
	pwm_setReset(1 << PWM3, 1);
	// Beep a higher frequency tone on the buzzer.
	wiced_sense_short_beep_buzzer(WICED_SENSE_BUZZER_MID);

	// Stop LED2 and start LED1
	wiced_sense_stop_led2();
	wiced_sense_start_led1();
}

// Advertisement stopped callback
void wiced_sense_advertisement_stopped(void)
{
	ble_trace0("Adv stopped, powerind down sensors, enabling wake.");

	// Stop all timers when adv stops.
	bleprofile_KillTimer();

	// Power down all sensors so we conserve battery.
	wiced_sense_power_down_sensors();

	// Stop LE1
	wiced_sense_stop_led1();

	// Wake on GPIO interrupt.
	gpio_configurePin(GPIO_PIN_P4 / 16, GPIO_PIN_P4 % 16, GPIO_EN_INT_RISING_EDGE, GPIO_PIN_OUTPUT_LOW);

	{
		ble_trace0("Entering deep sleep.");

		// Configure the low power manager to enter deep sleep.
		devLpmConfig.disconnectedLowPowerMode = DEV_LPM_DISC_LOW_POWER_MODES_HID_OFF;

		// Configure the wake time in mS.
		devLpmConfig.wakeFromHidoffInMs = 72000000;

		// Wait for at most 700mS after entering deep sleep before giving up
		// and aborting. If we do enter deep sleep, we will never get out of
		// this wait loop.
		miaDriverConfig.delayAfterEnteringHidOffInUs= 700000;

		// Use the external 32k.
		devLpmConfig.wakeFromHidoffRefClk = HID_OFF_TIMED_WAKE_CLK_SRC_128KHZ;

		gpio_configurePin(GPIO_PIN_P0 / 16, GPIO_PIN_P0 % 16, GPIO_INPUT_ENABLE | GPIO_PULL_UP, 0);

		// Enter deep-sleep now. Will not return.
		devlpm_enterLowPowerMode();
	}
}


// Process SMP bonding result.  If we successfully paired with the
// central device, save its BDADDR in the NVRAM and initialize
// associated data
void wiced_sense_smp_bond_result(LESMP_PARING_RESULT  result)
{
    ble_trace3("hello_sample, bond result %02x smpinfo addr type:%d emconninfo type:%d\n",
    		result, lesmp_pinfo->lesmpkeys_bondedInfo.adrType, emconninfo_getPeerAddrType());

	gpio_configurePin((GPIO_PIN_P13) / 16, (GPIO_PIN_P13) % 16, GPIO_INPUT_ENABLE | GPIO_PULL_DOWN, 0);
	pwm_setReset(1 << PWM3, 1);

	wiced_sense_stop_led1();

    // do some noise
    wiced_sense_short_beep_buzzer(WICED_SENSE_BUZZER_HIGH);

    if (result == LESMP_PAIRING_RESULT_BONDED)
    {
        // saving bd_addr in nvram
        UINT8 *bda;
        UINT8 writtenbyte;

        bda = (UINT8 *)emconninfo_getPeerPubAddr();

        memcpy(wiced_sense_hostinfo.bdaddr, bda, sizeof(BD_ADDR));
        wiced_sense_hostinfo.characteristic_client_configuration = 0;

        ble_trace2("Bond successful %08x%04x\n", (bda[5] << 24) + (bda[4] << 16) + (bda[3] << 8) + bda[2], (bda[1] << 8) + bda[0]);
        writtenbyte = bleprofile_WriteNVRAM(NVRAM_ID_HOST_LIST, sizeof(wiced_sense_hostinfo), (UINT8 *)&wiced_sense_hostinfo);
        ble_trace1("NVRAM write:%04x\n", writtenbyte);
    }
}

// Process notification from the stack that encryption has been set.  If connected
// client is registered for notification, it is a good time to send it out
void wiced_sense_encryption_changed(HCI_EVT_HDR *evt)
{
    BLEPROFILE_DB_PDU db_pdu;

    UINT8 *bda = emconninfo_getPeerPubAddr();

    wiced_sense_send_data = 1;

    ble_trace2("hello_sample, encryption changed %08x%04x\n",
                (bda[5] << 24) + (bda[4] << 16) +
                (bda[3] << 8) + bda[2],
                (bda[1] << 8) + bda[0]);

	gpio_configurePin((GPIO_PIN_P13) / 16, (GPIO_PIN_P13) % 16, GPIO_INPUT_ENABLE | GPIO_PULL_DOWN, 0);
	pwm_setReset(1 << PWM3, 1);

	wiced_sense_stop_led1();

    wiced_sense_short_beep_buzzer(WICED_SENSE_BUZZER_MID_LOW);

    // Connection has been encrypted meaning that we have correct/paired device
    // restore values in the database
//    bleprofile_ReadNVRAM(NVRAM_ID_HOST_LIST, sizeof(wiced_sense_hostinfo), (UINT8 *)&wiced_sense_hostinfo);
//
//    // Need to setup value of Client Configuration descriptor in our database because peer
//    // might decide to read and stack sends answer without asking application.
//    db_pdu.len = 2;
//    db_pdu.pdu[0] = wiced_sense_hostinfo.characteristic_client_configuration & 0xff;
//    db_pdu.pdu[1] = (wiced_sense_hostinfo.characteristic_client_configuration >> 8) & 0xff;

	//bleprofile_WriteHandle(HANDLE_WICED_SENSE_CLIENT_CONFIGURATION_DESCRIPTOR, &db_pdu);

    ble_trace3("EncOn %08x%04x client_configuration:%04x\n",
                (wiced_sense_hostinfo.bdaddr[5] << 24) + (wiced_sense_hostinfo.bdaddr[4] << 16) +
                (wiced_sense_hostinfo.bdaddr[3] << 8) + wiced_sense_hostinfo.bdaddr[2],
                (wiced_sense_hostinfo.bdaddr[1] << 8) + wiced_sense_hostinfo.bdaddr[0],
                wiced_sense_hostinfo.characteristic_client_configuration);
}


//
// Process write request or command from peer device
//
int wiced_sense_write_handler(LEGATTDB_ENTRY_HDR *p)
{
    UINT8  writtenbyte;
    UINT16 handle   = legattdb_getHandle(p);
    int    len      = legattdb_getAttrValueLen(p);
    UINT8  *attrPtr = legattdb_getAttrValue(p);

	gpio_configurePin((GPIO_PIN_P13) / 16, (GPIO_PIN_P13) % 16, GPIO_INPUT_ENABLE | GPIO_PULL_DOWN, 0);
	pwm_setReset(1 << PWM3, 1);

    ble_trace1("wiced_sense_write_handler: handle %04x\n", handle);

    // By writing into Characteristic Client Configuration descriptor
    // peer can enable or disable notifications
    if ((len == 2) && (handle == HANDLE_WICED_SENSE_CLIENT_CONFIGURATION_DESCRIPTOR))
    {
    	wiced_sense_hostinfo.characteristic_client_configuration = attrPtr[0] + (attrPtr[1] << 8);
        ble_trace1("wiced_sense_write_handler: client_configuration %04x\n", wiced_sense_hostinfo.characteristic_client_configuration);
    }
#ifdef OTA_SECURE_UPGRADE
    // Support Over the Air firmware upgrade need to call corresponding upgrade functions
    else if ((len >= 1) && (handle == HANDLE_WS_UPGRADE_CONTROL_POINT))
    {
        return (ws_upgrade_ota_handle_command (attrPtr, len));
    }
    else if ((len == 2) && (handle == HANDLE_WS_UPGRADE_CLIENT_CONFIGURATION_DESCRIPTOR))
    {
        return (ws_upgrade_ota_handle_configuration (attrPtr, len));
    }
    else if ((len > 0)  && (len <= WS_UPGRADE_MAX_DATA_LEN) && (handle == HANDLE_WS_UPGRADE_DATA))
    {
        return (ws_upgrade_ota_handle_data (attrPtr, len));
    }
    else
    {
        ble_trace2("wiced_sense_write_handler: bad write len:%d handle:0x%x\n", len, handle);
    	return 0x80;
    }
#endif
    // Save update to NVRAM.  Client does not need to set it on every connection.
//    writtenbyte = bleprofile_WriteNVRAM(NVRAM_ID_HOST_LIST, sizeof(wiced_sense_hostinfo), (UINT8 *)&wiced_sense_hostinfo);
//    ble_trace1("wiced_sense_write_handler: NVRAM write:%04x\n", writtenbyte);

    return 0;
}

//Scale battery level into percentage and update GATT DB.
void wiced_sense_scale_battery_level_to_percentage_and_update(void)
{
	UINT32 temp;
	BLEPROFILE_DB_PDU db_pdu;

	wiced_sense_measure_battery_level_and_average();

	ble_trace1("Battery: wiced_sense_battery_measurements_average  = %d", wiced_sense_battery_measurements_average);

	if(wiced_sense_battery_measurements_average > WICED_SENSE_BATTERY_LEVEL_FULL)
		wiced_sense_battery_level_in_percentage = 100;
	else if (wiced_sense_battery_measurements_average <= WICED_SENSE_BATTERY_LEVEL_EMPTY)
		wiced_sense_battery_level_in_percentage = 0;
	else
	{
		temp = wiced_sense_battery_measurements_average - WICED_SENSE_BATTERY_LEVEL_EMPTY;
		temp *= 100;   // for percentage.
		temp /= (WICED_SENSE_BATTERY_LEVEL_FULL - WICED_SENSE_BATTERY_LEVEL_EMPTY);
		wiced_sense_battery_level_in_percentage = temp;
	}

	bleprofile_ReadHandle(HANDLE_WICED_SENSE_BATTERY_LEVEL, &db_pdu);
	db_pdu.len = 1;
	db_pdu.pdu[0] = wiced_sense_battery_level_in_percentage;
	bleprofile_WriteHandle(HANDLE_WICED_SENSE_BATTERY_LEVEL, &db_pdu);

	ble_trace1("Battery: Level = %d", wiced_sense_battery_level_in_percentage);
}

// Measures the average ADC reading of the battery level
void wiced_sense_measure_battery_level_and_average(void)
{
	// P15 is connected to the battery.
	INT16 new_measurement = adc_readVoltage(ADC_INPUT_P15);

	// Subtract the oldest
	wiced_sense_battery_measurements_sum -= wiced_sense_battery_measurements[wiced_sense_battery_measurements_oldest_index];

	// Add the newest
	wiced_sense_battery_measurements[wiced_sense_battery_measurements_oldest_index] = new_measurement;
	wiced_sense_battery_measurements_sum += new_measurement;

	// Adjust the oldest index and wraparound
	if(++wiced_sense_battery_measurements_oldest_index >= WICED_SENSE_NUM_BAT_MEASUREMENTS_TO_AVERAGE)
		wiced_sense_battery_measurements_oldest_index = 0;

	wiced_sense_battery_measurements_average = wiced_sense_battery_measurements_sum / WICED_SENSE_NUM_BAT_MEASUREMENTS_TO_AVERAGE;
	if((wiced_sense_battery_measurements_sum % WICED_SENSE_NUM_BAT_MEASUREMENTS_TO_AVERAGE) >= (WICED_SENSE_NUM_BAT_MEASUREMENTS_TO_AVERAGE / 2))
		wiced_sense_battery_measurements_average++;
}

// One second timer expired. Read the sensor data.
void wiced_sense_timeout(UINT32 arg)
{

}

// Fine timer timeout.
void wiced_sense_fine_timeout(UINT32 arg)
{
	// Check if we need to stop the buzzer
	if(wiced_sense_pwm_playing &&
			(--wiced_sense_pwm_playing == 0) )
	{
		gpio_configurePin((GPIO_PIN_P13) / 16, (GPIO_PIN_P13) % 16, GPIO_INPUT_ENABLE | GPIO_PULL_DOWN, 0);

		// Disable the channel.
		pwm_setReset(1 << PWM3, 1);
	}

	// Check if we need to transition LED1 to the next brightness level
	if(wiced_sense_led1_playing && wiced_sense_led1_state != WICED_SENSE_LED_STATE_IDLE &&
			(--wiced_sense_led1_playing == 0) )
	{
		wiced_sense_led1_playing = WICED_SENSE_NUM_FINE_TIMEOUTS_TO_PLAY;

		// Move LED blink to the next state.
		wiced_sense_next_led1_state();
	}

	if(wiced_sense_connection_handle != 0)
		wiced_sense_scale_battery_level_to_percentage_and_update();
}

// The GPIO interrupt handler.
void wiced_sense_gpio_interrupt_handler(void* unsued, UINT8 gpio)
{
	switch(gpio)
	{
	case GPIO_PIN_P4:
		// Disable interrupt config
		gpio_configurePin(0, 4, GPIO_INTERRUPT_DISABLE, GPIO_PIN_OUTPUT_LOW);

		// Start blinking LEDs
		wiced_sense_start_led1();

		// Start timers.
		bleprofile_StartTimer();

		// Interrupt on P4, become discoverable.
		bleprofile_Discoverable(HIGH_UNDIRECTED_DISCOVERABLE, wiced_sense_remote_addr);
		break;
	default:
		break;
	}
}

void wiced_sense_enter_hidoff(void)
{
	// Do nothing
}

void wiced_sense_abort_hidoff(void)
{
	// Do nothing
}

#if WICED_SENSE_SUPPORT_GYRO
/// Get the instantaneous gyro data.
status_t wiced_sense_get_gyro_instantaneous_data(AxesRaw_t* gdata)
{
	if(L3GD20_GetAngRateRaw(gdata) == MEMS_SUCCESS)
		return MEMS_SUCCESS;

	return MEMS_ERROR;
}
#endif

#if WICED_SENSE_SUPPORT_MAGNETOMETER
/// Get the instantaneous magnetometer data.
status_t wiced_sense_get_magnetometer_instantaneous_data(AxesRaw_t* dataM)
{
	if(LSM303D_GetMagAxesRaw(dataM) == MEMS_SUCCESS)
		return MEMS_SUCCESS;

	return MEMS_ERROR;
}
#endif

#if WICED_SENSE_SUPPORT_ACCEL
/// Get the instantaneous accelerometer data.;
status_t wiced_sense_get_accelerometer_instantaneous_data(AxesRaw_t* dataA)
{
	if(GetAccAxesRaw(dataA) == MEMS_SUCCESS)
		return MEMS_SUCCESS;

	return MEMS_ERROR;
}
#endif

#if WICED_SENSE_SUPPORT_PRESSURE || WICED_SENSE_SUPPORT_TEMPERATURE
/// Get the instantaneous accelerometer data.
status_t wiced_sense_get_pressure_temp_instantaneous_data(LPS25H_MeasureTypeDef_st* data)
{
	if(LPS25H_Get_Measurement(data) == LPS25H_OK)
		return MEMS_SUCCESS;

	return MEMS_ERROR;
}
#endif

#if WICED_SENSE_SUPPORT_HUMIDITY
status_t wiced_sense_get_humidity_temp_instantaneous_data(UINT16* humidity, INT16* temperature)
{
	if(HTS221_StartOneShotMeasurement() == HTS221_OK)
	{
		if(HTS221_Get_Measurement(humidity, temperature) == HTS221_OK)
			return MEMS_SUCCESS;
	}

	return MEMS_ERROR;
}
#endif


#if WICED_SENSE_SUPPORT_GYRO
// Initialize the gyro sensor
void wiced_sense_initialize_l3gd20(void)
{
	if(L3GD20_SetMode(L3GD20_NORMAL) == MEMS_SUCCESS)
		ble_trace0("L3GD20_SetMode Successful");
	else
		ble_trace0("L3GD20_SetMode Failed.");

	if(L3GD20_BootMemEnable(MEMS_DISABLE) == MEMS_SUCCESS)
		ble_trace0("L3GD20_BootMemEnable Disable Successful");
	else
		ble_trace0("L3GD20_BootMemEnable Disable Failed.");

	// Initialize L3GD20 - the gyro
	if(L3GD20_SetODR(L3GD20_ODR_190Hz_BW_25) == MEMS_SUCCESS)
		ble_trace0("L3GD20_SetODR Successful");
	else
		ble_trace0("L3GD20_SetODR Failed.");

	if(L3GD20_SetFullScale(L3GD20_FULLSCALE_250) == MEMS_SUCCESS)
		ble_trace0("L3GD20_SetFullScale Successful");
	else
		ble_trace0("L3GD20_SetFullScale Failed.");

	if(L3GD20_SetAxis(L3GD20_X_ENABLE | L3GD20_Y_ENABLE | L3GD20_Z_ENABLE) == MEMS_SUCCESS)
		ble_trace0("L3GD20_SetAxis Successful");
	else
		ble_trace0("L3GD20_SetAxis Failed.");

	if(L3GD20_SetBDU(MEMS_ENABLE) == MEMS_SUCCESS)
		ble_trace0("L3GD20_SetBDU Successful");
	else
		ble_trace0("L3GD20_SetBDU Failed");
}
#endif

#if WICED_SENSE_SUPPORT_ACCEL
// Initialize accelerometer
void wiced_sense_initialize_lis3dsh(void)
{
	if(SetODR(ODR_100Hz) == MEMS_SUCCESS)
		ble_trace0("LIS3DSH_SetODR Successful");
	else
		ble_trace0("LIS3DSH_SetODR Failed.");

	if(SetFullScale(FULLSCALE_2) == MEMS_SUCCESS)
		ble_trace0("LIS3DSH_SetFullScale Successful");
	else
		ble_trace0("LIS3DSH_SetFullScale Failed.");

	if(SetAxis(X_ENABLE | Y_ENABLE | Z_ENABLE) == MEMS_SUCCESS)
		ble_trace0("LIS3DSH_SetAxis Successful");
	else
		ble_trace0("LIS3DSH_SetAxis Failed.");

	if(VectFiltEnable(MEMS_DISABLE) == MEMS_SUCCESS)
		ble_trace0("LIS3DSH_VectFiltEnable Successful");
	else
		ble_trace0("LIS3DSH_VectFiltEnable Failed.");

	if(SoftReset(MEMS_DISABLE) == MEMS_SUCCESS)
		ble_trace0("LIS3DSH_SoftReset Successful");
	else
		ble_trace0("LIS3DSH_SoftReset Failed.");

	if(SetBDU(MEMS_ENABLE) == MEMS_SUCCESS)
		ble_trace0("SetBDU Successful");
	else
		ble_trace0("SetBDU Failed");
}
#endif

#if WICED_SENSE_SUPPORT_MAGNETOMETER
// Initialize magnetometer/accelerometer sensor
void wiced_sense_initialize_lsm303d(void)
{
	// We will use only the magnetometer in this sensor because we have
	// a dedicated accelerometer
	// Turn off LSM303D accelerometer
	if(LSM303D_SetAccMode(LSM303D_POWER_DOWN_MODE_A) == MEMS_SUCCESS)
		ble_trace0("LSM303D_SetAccMode PD Successful");
	else
		ble_trace0("LSM303D_SetAccMode PD Failed.");

	if(LSM303D_SetMagMode(LSM303D_CONTINUOUS_MODE_M) == MEMS_SUCCESS)
		ble_trace0("LSM303D_SetMagMode Successful");
	else
		ble_trace0("LSM303D_SetMagMode Failed");

	if(LSM303D_SetMagODR(LSM303D_ODR_100Hz_M) == MEMS_SUCCESS)
		ble_trace0("LSM303D_SetMagODR Successful");
	else
		ble_trace0("LSM303D_SetMagODR Failed");

	if(LSM303D_SetMagFullScale(LSM303D_FULLSCALE_2_M) == MEMS_SUCCESS)
		ble_trace0("LSM303D_SetMagFullScale Successful");
	else
		ble_trace0("LSM303D_SetMagFullScale Failed");

	if(LSM303D_SetBDU(MEMS_ENABLE) == MEMS_SUCCESS)
		ble_trace0("LSM303D_SetBDU Successful");
	else
		ble_trace0("LSM303D_SetBDU Failed");
}
#endif

#if WICED_SENSE_SUPPORT_HUMIDITY
// Initialize temp and humidity sensor
void wiced_sense_initialize_hts221(void)
{
	if(HTS221_Activate() == HTS221_OK)
		ble_trace0("HTS221_Activate Successful");
	else
		ble_trace0("HTS221_Activate Failed.");

	if(HTS221_Set_Odr(HTS221_ODR_7HZ) == HTS221_OK)
		ble_trace0("HTS221_Set_Odr Successful");
	else
		ble_trace0("HTS221_Set_Odr Failed.");

	if(HTS221_Set_AvgHT(HTS221_AVGH_8, HTS221_AVGT_4) == HTS221_OK)
		ble_trace0("HTS221_Set_AvgHT Successful");
	else
		ble_trace0("HTS221_Set_AvgHT failed");

	if(HTS221_Set_BduMode(1) == HTS221_OK)
		ble_trace0("HTS221_Set_BduMode Successful");
	else
		ble_trace0("HTS221_Set_BduMode failed");
}
#endif

#if WICED_SENSE_SUPPORT_PRESSURE || WICED_SENSE_SUPPORT_TEMPERATURE
// Initialize pressure sensor
void wiced_sense_initialize_lps25h(void)
{
	if(LPS25H_Activate() == LPS25H_OK)
		ble_trace0("LPS25H_Activate Successful");
	else
		ble_trace0("LPS25H_Activate Failed.");

	if(LPS25H_Set_Odr(LPS25H_ODR_7HZ) == LPS25H_OK)
		ble_trace0("LPS25H_Set_Odr Successfully");
	else
		ble_trace0("LPS25H_Set_Odr Failed");

	if(LPS25H_Set_Bdu(LPS25H_BDU_NO_UPDATE) == LPS25H_OK)
		ble_trace0("LPS25H_Set_BDU successful");
	else
		ble_trace0("LPS25H_Set_BDU failed");
}
#endif


// Initializes all sensors connected over the I2C interface.
void wiced_sense_initialize_sensors(void)
{
#if WICED_SENSE_SUPPORT_GYRO
	// Initialize gyro
	wiced_sense_initialize_l3gd20();
#endif

#if WICED_SENSE_SUPPORT_ACCEL
	// Initialize accelerometer
	wiced_sense_initialize_lis3dsh();
#endif

#if WICED_SENSE_SUPPORT_MAGNETOMETER
	// Initialize magnetometer/accelerometer
	wiced_sense_initialize_lsm303d();
#endif

#if WICED_SENSE_SUPPORT_HUMIDITY
	// Initialize temp and humidity sensor
	wiced_sense_initialize_hts221();
#endif

#if WICED_SENSE_SUPPORT_PRESSURE || WICED_SENSE_SUPPORT_TEMPERATURE
	// Initialize pressure sensor
	wiced_sense_initialize_lps25h();
#endif
}

void wiced_sense_power_down_sensors(void)
{
	// Unconditionally power down all sensors on the device.
	if(L3GD20_BootMemEnable(MEMS_ENABLE) == MEMS_SUCCESS)
		ble_trace0("L3GD20_BootMemEnable Disable Successful");
	else
		ble_trace0("L3GD20_BootMemEnable Disable Failed.");

	if(L3GD20_SetAxis(0) == MEMS_SUCCESS)
		ble_trace0("L3GD20_SetAxis Disable Successful");
	else
		ble_trace0("L3GD20_SetAxis Disable Failed.");

	// Turn off L3GD20
	if(L3GD20_SetMode(L3GD20_POWER_DOWN) == MEMS_SUCCESS)
		ble_trace0("L3GD20_SetMode PD Successful");
	else
		ble_trace0("L3GD20_SetMode PD Failed.");

	if(SetAxis(X_DISABLE | Y_DISABLE | Z_DISABLE) == MEMS_SUCCESS)
		ble_trace0("LIS3DSH_SetAxis Disable Successful");
	else
		ble_trace0("LIS3DSH_SetAxis Disable Failed.");

	// Turn off LIS3DSH
	if(SetODR(POWER_DOWN) == MEMS_SUCCESS)
		ble_trace0("LIS3DSH_SetODR PD Successful");
	else
		ble_trace0("LIS3DSH_SetODR PD Failed.");


	// Turn off LSM303D accelerometer
	if(LSM303D_SetAccMode(LSM303D_POWER_DOWN_MODE_A) == MEMS_SUCCESS)
		ble_trace0("LSM303D_SetAccMode PD Successful");
	else
		ble_trace0("LSM303D_SetAccMode PD Failed.");

	// Turn off LSM303D magnetometer.
	if(LSM303D_SetMagMode(LSM303D_POWER_DOWN_MODE_M) == MEMS_SUCCESS)
		ble_trace0("LSM303D_SetMagMode PD Successful");
	else
		ble_trace0("LSM303D_SetMagMode PD Failed.");

	// Turn off HTS221
	if(HTS221_DeActivate() == HTS221_OK)
		ble_trace0("HTS221_DeActivate Successful");
	else
		ble_trace0("HTS221_DeActivate Failed.");

	// Turn off LPS25H
	if(LPS25H_DeActivate() == LPS25H_OK)
		ble_trace0("LPS25H_DeActivate Successful");
	else
		ble_trace0("LPS25H_DeActivate Failed.");
}

/// Polls the humidity, pressure and temperature sensors and returns
/// their data in the given buffer, each 16 bits wide.
UINT8 wiced_sense_poll_hpt_and_fill_notification_packet(UINT16* data_ptr, UINT8* size)
{
	UINT16 humidity = 0;
	INT16 temperature = 0;
	LPS25H_MeasureTypeDef_st measurement;
	UINT8 header = 0;

#if WICED_SENSE_SUPPORT_HUMIDITY
	// Read humidity first
	if(wiced_sense_get_humidity_temp_instantaneous_data(&humidity, &temperature) == MEMS_SUCCESS)
	{
		//ble_trace2("Humi*10=%6d, Temp*10=%6d", humidity, temperature);
		header |= (WICED_SENSE_NOTIFICATION_HEADER_INCLUDES_HUMIDITY);
		*data_ptr++ = humidity;
		*size += sizeof(humidity);
	}
	else
	{
		ble_trace0("Reading humidity sensor failed.");
	}
#endif

#if WICED_SENSE_SUPPORT_PRESSURE || WICED_SENSE_SUPPORT_TEMPERATURE
	// Then read temp and pressure.
	if(wiced_sense_get_pressure_temp_instantaneous_data(&measurement) == MEMS_SUCCESS)
	{
		//ble_trace2("LPS25H Temp = %6d, Press=%6d", measurement.Tout, measurement.Pout);
#if WICED_SENSE_SUPPORT_PRESSURE
		header |= WICED_SENSE_NOTIFICATION_HEADER_INCLUDES_PRESSURE;
		*data_ptr++ = (measurement.Pout / 10) & 0xFFFF;
		*size += sizeof(UINT16);
#endif

#if WICED_SENSE_SUPPORT_TEMPERATURE
		header |= WICED_SENSE_NOTIFICATION_HEADER_INCLUDES_TEMPERATURE;
		*data_ptr++ = measurement.Tout & 0xFFFF;
		*size += sizeof(UINT16);
#endif
	}
	else
	{
		ble_trace0("LPS25H_Get_Measurement failed.");
	}
#endif

	return header;
}

/// Polls the accelerometer, gyro and magnetometer and returns their data
/// in the given buffer, X, Y and Z.
UINT8 wiced_sense_poll_agm_and_fill_notification_packet(UINT16* data_ptr, UINT8* size)
{
	UINT8 header = 0;
	AxesRaw_t* axes_ptr = (AxesRaw_t*)data_ptr;

#if WICED_SENSE_ACTIVATE_GYRO_ON_DENAMD && WICED_SENSE_SUPPORT_GYRO
	// Configure the gyro
	wiced_sense_initialize_l3gd20();
#endif

#if WICED_SENSE_SUPPORT_ACCEL
	// Get accelerometer data first
	if(wiced_sense_get_accelerometer_instantaneous_data(axes_ptr) == MEMS_SUCCESS)
	{
		//ble_trace3("Accelerometer: X=%6d, Y=%6d, Z=%6d", axes_ptr->AXIS_X, axes_ptr->AXIS_Y, axes_ptr->AXIS_Z);
		header |= WICED_SENSE_NOTIFICATION_HEADER_INCLUDES_ACCELEROMETER;
		axes_ptr++;
		*size += sizeof(AxesRaw_t);
	}
	else
	{
		ble_trace0("Reading Accelerometer failed.");
	}
#endif

#if WICED_SENSE_SUPPORT_GYRO
	if(wiced_sense_get_gyro_instantaneous_data(axes_ptr) == MEMS_SUCCESS)
	{
		//ble_trace3("Gyro: X=%6d, Y=%6d, Z=%6d", axes_ptr->AXIS_X, axes_ptr->AXIS_Y, axes_ptr->AXIS_Z);
		header |= WICED_SENSE_NOTIFICATION_HEADER_INCLUDES_GYRO;
		axes_ptr++;
		*size += sizeof(AxesRaw_t);
	}
	else
	{
		ble_trace0("Reading gyro failed.");
	}

#if WICED_SENSE_ACTIVATE_GYRO_ON_DENAMD
	// Put Gyro in sleep
	if(L3GD20_SetMode(L3GD20_SLEEP) == MEMS_SUCCESS)
		ble_trace0("L3GD20_SetMode Successful");
	else
		ble_trace0("L3GD20_SetMode Failed.");
#endif

#endif

#if WICED_SENSE_SUPPORT_MAGNETOMETER
	if(wiced_sense_get_magnetometer_instantaneous_data(axes_ptr) == MEMS_SUCCESS)
	{
		//ble_trace3("Magnetometer: X=%6d, Y=%6d, Z=%6d", axes_ptr->AXIS_X, axes_ptr->AXIS_Y, axes_ptr->AXIS_Z);
		header |= WICED_SENSE_NOTIFICATION_HEADER_INCLUDES_MAGNETOMETER;
		*size += sizeof(AxesRaw_t);
	}
	else
	{
		ble_trace0("Reading Magnetometer failed.");
	}
#endif

	return header;
}

// TX opportunity notice.
// Function to be called the configurable number of BT slots before
// an upcoming TX opportunity so we can poll and collect sensor data
// just in time to send notification to the central. This will maximize
// sleep time.
void wiced_sense_tx_opportunity_notice(void* context, UINT32 unused)
{
	BLEPROFILE_DB_PDU db_pdu;
	UINT8* header_ptr = &db_pdu.pdu[0];
	UINT16* data_ptr = (UINT16*)&db_pdu.pdu[1];    // Unaligned access is OK on CM3.
	db_pdu.len = 0;

	// If client has not registered for notifications, do not need to do anything
	if ( wiced_sense_hostinfo.characteristic_client_configuration == 0 || wiced_sense_connection_handle == 0  /* || wiced_sense_send_data == 0 */ )
		return;

	// Increment polls since last humidity, pressure, temperature measurement cycle.
	wiced_sense_polls_since_last_hpt++;

	memset (&db_pdu, 0x00, sizeof(db_pdu));

	// Now try to poll the sensors.
	if (wiced_sense_polls_since_last_hpt >= 10)
	{
		// If it has been a while since we polled humidity, pressure and temp sensors,
		// send just these three. Will use a better algorithm later. HPT don't change very
		// often, so polling every 10th connection event should be sufficient.
		// A better way to do this would be to transmit only if something changed since last
		// poll.
		*header_ptr = wiced_sense_poll_hpt_and_fill_notification_packet(data_ptr, (UINT8*)&db_pdu.len);

		// Poll Accelerometer, Gyro and Magnetometer for a while.
		wiced_sense_polls_since_last_hpt = 0;
	}
	else
	{
		*header_ptr = wiced_sense_poll_agm_and_fill_notification_packet(data_ptr, (UINT8*)&db_pdu.len);
	}

	if(*header_ptr)
	{
		db_pdu.len++;

		// Write to DB so if the client reads it, it will get the latest value.
		bleprofile_WriteHandle(HANDLE_WICED_SENSE_VALUE_NOTIFY, &db_pdu);

		ble_trace1("Noti length: %d", db_pdu.len);
		ble_tracen(&db_pdu.pdu[0], db_pdu.len);

		// Send notification if required. If its backed up, packet will automatically be dropped.
		bleprofile_sendNotification(HANDLE_WICED_SENSE_VALUE_NOTIFY, (UINT8 *)db_pdu.pdu, db_pdu.len);

		// Blink LED2 to indicate that a packet was sent out.
		wiced_sense_toggle_led2();
	}
}

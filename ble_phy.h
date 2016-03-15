#ifndef _BLE_PHY_H_
#define _BLE_PHY_H_

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "led.h"
#include "sensor.h"

#include "dfu_app_handler.h"
#include "bootloader_util.h"
#include "nrf.h"
#include "nrf_sdm.h"
#include "ble_gatt.h"
#include "ble_gatts.h"
#include "dfu_ble_svc.h"
#include "device_manager.h"


#define BLE_APPEARANCE_PHYSICS_SENSOR								2015
///wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww
/// Phy sensor structures
///---------------------------------------------------------------------<
typedef enum
{
    BLE_PSS_EVT_NOTIFICATION_ENABLED,                            		// notification enabled event
    BLE_PSS_EVT_NOTIFICATION_DISABLED                            		// notification disabled event
} ble_pss_evt_type_t;



typedef struct
{
    ble_pss_evt_type_t evt_type;                                  		// type of event
} ble_pss_evt_t;

// Forward declaration of the ble_pss_t type. 
typedef struct ble_pss_s ble_pss_t;

typedef void (*ble_pss_evt_handler_t) (ble_pss_t * p_pss, ble_pss_evt_t * p_evt);

typedef struct
{
    ble_pss_evt_handler_t         evt_handler;                    		// event handler 
    bool                          support_notification;          		// true if notification is supported
    ble_srv_report_ref_t *        p_report_ref;                   		// if not NULL, a Report Reference descriptor with the specified value will be added tocharacteristic 
    ble_srv_cccd_security_mode_t  phy_sens_char_attr_md;     			// Initial security level for physim characteristics attribute 
    ble_gap_conn_sec_mode_t       phy_sens_report_read_perm; 			// Initial security level for physim report read attribute 
} ble_pss_init_t;


typedef struct ble_pss_s												// various status information for the service
{
    ble_pss_evt_handler_t         evt_handler;                    		// event handler
    uint16_t                      service_handle;               		// handle of physim Service
    uint16_t                      service_handle_w;                       // handle of physim Service
    ble_gatts_char_handles_t      phy_sen_level_handles;          		// handles related to the phy sensor characteristic
    ble_gatts_char_handles_t      phy_sen_level_handles_w;
    uint16_t                      report_ref_handle;              		// handle of the Report Reference descriptor.
    uint16_t                      conn_handle;                    		// handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). 
    bool                          is_notification_supported;      		// true if notification is supported.
    uint8_t                    	  uuid_type;
} ble_pss_t;

typedef enum 
{

    BLE_SHOT_MODE = 0,
    BLE_OTHER_MODE=1,
    BLE_SETTINGS_MODE=2,
} ble_mode_t;

#define BLE_UUID_PHY_SENSOR_SERVICE 	0x2000
#define PHY_SENSOR_DATA_CHAR		 	0x2E00
#define PHY_SENSOR_WRITE_CHAR           0x2E01
#define PHY_SENSOR_LED_CHAR 			0x2E10
//#define PHY_SENSOR_UUID_BASE {0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00}
#define PHY_SENSOR_UUID_BASE {0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}


#define SETTINGS_READ                   0x02
#define SETTINGS_NEW                    0x03
#define DATA                            0x04
#define DATA_READY                      0x05
#define DATA_END                        0x06
#define DATA_START                      0x08
#define DATA_DRAFT                      0x0A

uint32_t initBlePHYSEN(ble_pss_t * p_pss, const ble_pss_init_t * p_pss_init);
static uint32_t addCharPHYSEN(ble_pss_t * p_pss, const ble_pss_init_t * p_pss_init);
void onBleEvenPHYSEN(ble_pss_t * p_pss, ble_evt_t * p_ble_evt);
static void onWritePHYSEN(ble_pss_t * p_pss, ble_evt_t * p_ble_evt);
static void onWriteAccessPHYSEN(ble_pss_t * p_pss, ble_evt_t * p_ble_evt);
static void onDiscPHYSEN(ble_pss_t * p_pss, ble_evt_t * p_ble_evt);
static void onConnPHYSEN(ble_pss_t * p_pss, ble_evt_t * p_ble_evt);
uint32_t sendDataPHYSENS(ble_pss_t * p_pss);


extern volatile uint8_t g_ble_conn;
extern volatile uint8_t settings_flag;
extern volatile ble_mode_t ble_mode;
extern volatile uint8_t g_battery_int;
extern volatile uint8_t g_cooked_data[6];
extern volatile uint8_t g_settings[18];
extern volatile uint8_t g_settings_new[18];
extern volatile uint8_t g_handle_settings;
extern volatile uint8_t g_valid;
extern volatile uint8_t g_state;
extern volatile uint16_t g_real_index;
extern volatile uint16_t g_remember;
extern volatile uint16_t g_shot_br25s_index;
#endif 


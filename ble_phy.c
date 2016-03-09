#include "ble_phy.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
volatile uint8_t g_ble_conn = 0;

extern int m_send_packet;

uint32_t initBlePHYSEN(ble_pss_t * p_pss, const ble_pss_init_t * p_pss_init)
{/// init BLE physical sensor service
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // initialize service structure
    p_pss->evt_handler               = p_pss_init->evt_handler;			
    p_pss->conn_handle               = BLE_CONN_HANDLE_INVALID;
    p_pss->is_notification_supported = p_pss_init->support_notification;
    
    // add service
    ble_uuid128_t base_uuid= {PHY_SENSOR_UUID_BASE};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_pss->uuid_type);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    ble_uuid.type = p_pss->uuid_type;
    ble_uuid.uuid = BLE_UUID_PHY_SENSOR_SERVICE;
    
	err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_pss->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
   
    // Add physics sensor characteristic
    return addCharPHYSEN(p_pss, p_pss_init);
}

static uint32_t addCharPHYSEN(ble_pss_t * p_pss, const ble_pss_init_t * p_pss_init)
{/// add phy sensor characteristics
    ble_gatts_char_md_t char_md;
    ble_gatts_char_md_t char_md_w;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_gatts_attr_t    attr_char_value_w;
    ble_uuid_t          ble_uuid;
    ble_uuid_t          ble_uuid_w;
    ble_gatts_attr_md_t attr_md;
    ble_gatts_attr_md_t attr_md_w;
    char user_desc[] = "Physics sensor";
    
	memset(&cccd_md, 0, sizeof(cccd_md));
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
	cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    
    memset(&char_md, 0, sizeof(char_md));
    
    char_md.char_props.read   = 1;
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;
    char_md.p_char_user_desc  = (uint8_t *) user_desc;
    char_md.char_user_desc_size = strlen(user_desc);
    char_md.char_user_desc_max_size = strlen(user_desc);

    memset(&char_md_w, 0, sizeof(char_md_w));
    
    char_md_w.char_props.write  = 1;
    char_md_w.p_char_user_desc  = NULL;
    char_md_w.p_char_pf         = NULL;
    char_md_w.p_user_desc_md    = NULL;
    char_md_w.p_cccd_md         = &cccd_md;
    char_md_w.p_sccd_md         = NULL;
    char_md_w.p_char_user_desc  = (uint8_t *) user_desc;
    char_md_w.char_user_desc_size = strlen(user_desc);
    char_md_w.char_user_desc_max_size = strlen(user_desc);
    
    
    ble_uuid.type = p_pss->uuid_type;
    ble_uuid.uuid = PHY_SENSOR_DATA_CHAR;

    ble_uuid_w.type = p_pss->uuid_type;
    ble_uuid_w.uuid = PHY_SENSOR_WRITE_CHAR;
    
    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_md_w, 0, sizeof(attr_md_w));

    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md_w.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md_w.write_perm);
    attr_md_w.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md_w.rd_auth    = 0;
    attr_md_w.wr_auth    = 0;
    attr_md_w.vlen       = 0;
       
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = (SENSOR_ROW_SIZE);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = (SENSOR_ROW_SIZE);
    attr_char_value.p_value      = NULL;

    memset(&attr_char_value_w, 0, sizeof(attr_char_value_w));

    attr_char_value_w.p_uuid       = &ble_uuid_w;
    attr_char_value_w.p_attr_md    = &attr_md_w;
    attr_char_value_w.init_len     = (SENSOR_ROW_SIZE);
    attr_char_value_w.init_offs    = 0;
    attr_char_value_w.max_len      = (SENSOR_ROW_SIZE);
    attr_char_value_w.p_value      = NULL;
    
    sd_ble_gatts_characteristic_add(p_pss->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_pss->phy_sen_level_handles);
    return sd_ble_gatts_characteristic_add(p_pss->service_handle_w, &char_md_w,
                                               &attr_char_value_w,
                                               &p_pss->phy_sen_level_handles_w) ;
}

void onBleEvenPHYSEN(ble_pss_t * p_pss, ble_evt_t * p_ble_evt)
{/// call functions for appropriate ble events
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            onConnPHYSEN(p_pss, p_ble_evt);
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            onDiscPHYSEN(p_pss, p_ble_evt);
            break;
            
        case BLE_GATTS_EVT_WRITE:
            onWriteAccessPHYSEN(p_pss, p_ble_evt);
            break;
            
        default:
            // No implementation needed.
            break;
    }
}

static void onConnPHYSEN(ble_pss_t * p_pss, ble_evt_t * p_ble_evt)
{/// handle connect event
    p_pss->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}

static void onDiscPHYSEN(ble_pss_t * p_pss, ble_evt_t * p_ble_evt)
{/// handle disconnect event
    UNUSED_PARAMETER(p_ble_evt);
    p_pss->conn_handle = BLE_CONN_HANDLE_INVALID;
}

static void onWritePHYSEN(ble_pss_t * p_pss, ble_evt_t * p_ble_evt)
{/// handle write event
	ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
	uint32_t utmp32 = p_evt_write->data[0];
	settings_flag += 0x15;
	utmp32 = p_evt_write->handle;
	printUSART0("-> BLE: Accessing handle: [%h]\n",&utmp32);



	if((p_evt_write->handle >= 0x000C) && (p_evt_write->handle <= 0x0010))
	{
		utmp32 = p_evt_write->data[0];
		printUSART0("-> BLE: Physics sensors data[0]: [%h]\n",&utmp32);
		//actLED(p_evt_write->data[0]);
		
		if (p_evt_write->data[0] == 0x05)
		{
			g_sensor_ridx = (SENSOR_COL_SIZE) - 1;
			g_sensor_widx = 0; 
			g_sensor_rcnt = 0x0000;
		}

        // Access door for settings
        if (p_evt_write->data[0] == 0x01) {
            //SETTINGS_EVALUATION
            //settings_flag = p_evt_write->data[0] + p_evt_write->data[1] + p_evt_write->data[2] + p_evt_write->data[3];
        } else if (p_evt_write->data[0] == 0x03) {
            //SETTINGS_NEW
            //settings_flag = p_evt_write->data[0] + p_evt_write->data[1] + p_evt_write->data[2] + p_evt_write->data[3];

        } else {
    		if (p_pss->is_notification_supported)
    		{
    			if ((p_evt_write->handle == p_pss->phy_sen_level_handles.cccd_handle)&&(p_evt_write->len == 1))
    			{
    				// CCCD written, call application event handler
    				if (p_pss->evt_handler != NULL)
    				{
    					ble_pss_evt_t evt;
    					
    					if (p_evt_write->data[0] == 0x05)
    					{
    						evt.evt_type = BLE_PSS_EVT_NOTIFICATION_ENABLED;
    					}
    					else
    					{
    						evt.evt_type = BLE_PSS_EVT_NOTIFICATION_DISABLED;
    					}

    					p_pss->evt_handler(p_pss, &evt);
    				}
    			}
    		}
        }
	}
}

static void onWriteAccessPHYSEN(ble_pss_t * p_pss, ble_evt_t * p_ble_evt)
{/// handle write event
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    int len = p_evt_write->len;
    int iter = 0;
    if (p_evt_write->handle == p_pss->phy_sen_level_handles_w.value_handle) {
        settings_flag = 5 + p_evt_write->len;
    }
    for (; iter < len; iter++) {
        //Accessing memory
    } 

    if (p_pss->is_notification_supported)
    {
        if ((p_evt_write->handle == p_pss->phy_sen_level_handles.cccd_handle))
        {
            // CCCD written, call application event handler
            if (p_pss->evt_handler != NULL)
            {
                ble_pss_evt_t evt;
                evt.evt_type = BLE_PSS_EVT_NOTIFICATION_ENABLED;
                p_pss->evt_handler(p_pss, &evt);
            }
        }
    }
}

uint32_t sendDataPHYSENS(ble_pss_t * p_pss)
{
    uint32_t err_code = NRF_SUCCESS;
	uint16_t len = (SENSOR_ROW_SIZE);


	if ((p_pss->conn_handle != BLE_CONN_HANDLE_INVALID) && p_pss->is_notification_supported)
	{
		g_sensor_ridx++;
		if(g_sensor_ridx == (SENSOR_COL_SIZE))				
			g_sensor_ridx = 0;											// roll over
		
		
		ble_gatts_hvx_params_t hvx_params;
		
		memset(&hvx_params, 0, sizeof(hvx_params));
		
		hvx_params.handle   = p_pss->phy_sen_level_handles.value_handle;
		hvx_params.type     = BLE_GATT_HVX_NOTIFICATION;
		hvx_params.p_len    = &len;
		hvx_params.p_data   = g_sensor_data[g_sensor_ridx];
		
        //nrf_delay_ms(30);
		err_code = sd_ble_gatts_hvx(p_pss->conn_handle, &hvx_params);
	    /*if (err_code == 0) {
            nrf_drv_gpiote_out_toggle(6);
        }*/
    }
	else
	{
		err_code = NRF_ERROR_INVALID_STATE;
	}
    

    return err_code;
}




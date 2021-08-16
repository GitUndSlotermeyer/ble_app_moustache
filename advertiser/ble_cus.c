#include "ble_cus.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "ble.h"
#include "nrf_log.h"
#include "app_error.h"
#include "bsp.h"


static uint8_t char_value = 0;

static void on_write(ble_cus_t * p_cus, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    NRF_LOG_INFO("inside on_write funciton");
    bsp_board_led_invert(0);
    // Custom Value Characteristic Written to.

    if(p_evt_write->len == 2) 
    {
        //NRF_LOG_INFO("CCCD value was written.");

        //NRF_LOG_INFO("CCCD value is %d", p_evt_write->data[0]);
       // NRF_LOG_INFO("CCCD value is %d", p_evt_write->data[1]);
        NRF_LOG_INFO("CCCD Handle is %d", p_ble_evt->evt.gatts_evt.params.write.handle);

    }

}

static void on_connect(ble_cus_t * p_cus, ble_evt_t const * p_ble_evt)
{
    ble_cus_evt_t evt;
    evt.evt_type = BLE_CUS_EVT_CONNECTED;

    p_cus->evt_handler(p_cus, &evt);
}

static void on_disconnect(ble_cus_t * p_cus, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);

    p_cus->conn_handle = BLE_CONN_HANDLE_INVALID;
}

static uint32_t char0_add(ble_cus_t * p_cus, const ble_cus_init_t * p_cus_init)
{
    uint32_t            err_code = NRF_SUCCESS;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    
    memset(&char_md, 0, sizeof(ble_gatts_char_md_t));

    char_md.char_props.write = 1;
    char_md.char_user_desc_max_size = 10;
    char_md.char_user_desc_size = 2;
	
    ble_uuid.type = p_cus->uuid_type;
    ble_uuid.uuid = CHAR0_UUID;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    
    attr_char_value.p_uuid = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.max_len   = sizeof(uint8_t);
    attr_char_value.p_value = &char_value;

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.write_perm = p_cus_init->custom_value_char_attr_md.write_perm;


    err_code = sd_ble_gatts_characteristic_add(p_cus->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_cus->custom_value_handles[0]);
    
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("Characteristic 0 successfully added + value handle is %d.", p_cus->custom_value_handles[0].value_handle);


    return NRF_SUCCESS;
}

static uint32_t char1_add(ble_cus_t * p_cus, const ble_cus_init_t * p_cus_init)
{
    uint32_t            err_code = NRF_SUCCESS;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    
    memset(&char_md, 0, sizeof(ble_gatts_char_md_t));

    char_md.char_props.notify = 1;
    char_md.char_user_desc_max_size = 10;
    char_md.char_user_desc_size = 2;
    char_md.p_cccd_md = &cccd_md;


    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

	
    ble_uuid.type = p_cus->uuid_type;
    ble_uuid.uuid = CHAR1_UUID;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    
    attr_char_value.p_uuid = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.max_len   = sizeof(uint8_t);
    attr_char_value.p_value = &char_value;

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
//    attr_md.read_perm  = p_cus_init->custom_value_char_attr_md.read_perm;
    attr_md.write_perm = p_cus_init->custom_value_char_attr_md.write_perm;


    err_code = sd_ble_gatts_characteristic_add(p_cus->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_cus->custom_value_handles[1]);
    
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("Characteristic 1 successfully added + value handle is %d.", p_cus->custom_value_handles[1].value_handle);

    return NRF_SUCCESS;
}

void ble_cus_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_cus_t * p_cus = (ble_cus_t *) p_context;
    
    if (p_cus == NULL || p_ble_evt == NULL)
    {
        return;
    }
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_cus, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_cus, p_ble_evt);
            break;
        
        case BLE_GATTS_EVT_WRITE:
            on_write(p_cus, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}

uint32_t ble_cus_init(ble_cus_t * p_cus, const ble_cus_init_t * p_cus_init)
{
    if (p_cus == NULL || p_cus_init == NULL) return NRF_ERROR_NULL;
    
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_cus->conn_handle               =  BLE_CONN_HANDLE_INVALID;
    p_cus->evt_handler               =  p_cus_init->evt_handler;

    // Add Custom Service UUID
    ble_uuid128_t base_uuid = {CUSTOM_SERVICE_UUID_BASE};
    err_code =  sd_ble_uuid_vs_add(&base_uuid, &p_cus->uuid_type);
    APP_ERROR_CHECK(err_code);
    
    ble_uuid.type = p_cus->uuid_type;
    ble_uuid.uuid = CUSTOM_SERVICE_UUID;

    // Add the Custom Service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_cus->service_handle);
    APP_ERROR_CHECK(err_code);

    err_code = char0_add(p_cus, p_cus_init);
    APP_ERROR_CHECK(err_code);

    err_code = char1_add(p_cus, p_cus_init);
    APP_ERROR_CHECK(err_code);
    
    return err_code;
}

uint32_t ble_cus_value_update_and_notify(ble_cus_t * p_cus, uint8_t new_value, uint8_t char_index)
{
    if (p_cus == NULL) return NRF_ERROR_NULL;

    uint32_t err_code = NRF_SUCCESS;
    // Update database.

    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));
    gatts_value.len     = sizeof(uint8_t);
    gatts_value.p_value = &new_value;

    err_code = sd_ble_gatts_value_set(p_cus->conn_handle, p_cus->custom_value_handles[char_index].value_handle,
                                      &gatts_value);
    APP_ERROR_CHECK(err_code);
        

    if (p_cus->conn_handle != BLE_CONN_HANDLE_INVALID) 
    {
            
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_cus->custom_value_handles[char_index].value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        //NRF_LOG_INFO("%d = Conn handle: %d, Custom value handle : %d, CCCD handle : %d", i, p_cus->conn_handle, p_cus->custom_value_handles[0].value_handle, p_cus->custom_value_handles[0].cccd_handle );
        err_code = sd_ble_gatts_hvx(p_cus->conn_handle, &hvx_params);
        NRF_LOG_INFO("Error is %d", err_code);
        APP_ERROR_CHECK(err_code);
        
    }
 
    return err_code;
}

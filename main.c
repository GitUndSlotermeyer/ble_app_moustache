#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "../../../components/softdevice/common/nrf_sdh.h"
#include "../../../components/softdevice/common/nrf_sdh_ble.h"
#include "../../../components/softdevice/common/nrf_sdh_soc.h"
#include "../../../components/libraries/pwr_mgmt/nrf_pwr_mgmt.h"
#include "../../../components/libraries/timer/app_timer.h"
#include "../../../components/boards/boards.h"
#include "../../../components/libraries/bsp/bsp.h"
#include "../../../components/libraries/bsp/bsp_btn_ble.h"
#include "../../../components/softdevice/s140/headers/ble.h"
#include "../../../components/softdevice/s140/headers/ble_hci.h"
#include "../../../components/ble/ble_advertising/ble_advertising.h"
#include "../../../components/ble/common/ble_conn_params.h"
#include "../../../components/ble/nrf_ble_qwr/nrf_ble_qwr.h"
#include "ble_lbs_c.h"
#include "../../../components/ble/nrf_ble_gatt/nrf_ble_gatt.h"
#include "../../../components/ble/nrf_ble_scan/nrf_ble_scan.h"
#include "../../../components/libraries/usbd/app_usbd_types.h"
#include "../../../components/libraries/usbd/app_usbd.h"
#include "../../../components/libraries/usbd/app_usbd_core.h"
#include "../../../components/libraries/usbd/class/hid/generic/app_usbd_hid_generic.h"
#include "app_usbd_hid_kbd.h"


#include "../../../components/libraries/log/nrf_log.h"
#include "../../../components/libraries/log/nrf_log_ctrl.h"
#include "../../../components/libraries/log/nrf_log_default_backends.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_power.h"

#define DEVICE_NAME                     "Nordic_Server"                       /**< Name of device. Will be included in the advertising data. */
#define CENTRAL_SCANNING_LED            BSP_BOARD_LED_1                     /**< Scanning LED will be on when the device is scanning. */
#define CENTRAL_CONNECTED_LED           BSP_BOARD_LED_2                    /**< Connected LED will be on when the device is connected. */

#define APP_BLE_CONN_CFG_TAG            1                                   /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO           3                                   /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */


#define USBD_POWER_DETECTION true

#define CONFIG_KBD_LETTER       APP_USBD_HID_KBD_SPACEBAR
#define APP_USBD_INTERFACE_KBD      1

// SERVICE & CHARACTERISTIC UUID'S
// CHAR0 represents state of usb
// CHAR1 represents state of LED
// CHAR2 represents ID

#define CUSTOM_SERVICE_UUID_BASE         {0xBC, 0x8A, 0xBF, 0x45, 0xCA, 0x05, 0x50, 0xBA, 0x40, 0x42, 0xB0, 0x00, 0xC9, 0xAD, 0x64, 0xF3}
#define CUSTOM_SERVICE_UUID               0x1400
#define CHAR0_UUID                        0x1401
#define CHAR1_UUID                        0x1402
#define CHAR2_UUID                        0x1403


// CUSTOM SERVICE STRUCTURES START

typedef struct ble_cus_s ble_cus_t;

typedef enum
{
    BLE_CUS_EVT_DISCONNECTED,
    BLE_CUS_EVT_CONNECTED,

} ble_cus_evt_type_t;

typedef struct
{
    ble_cus_evt_type_t evt_type;                                  /**< Type of event. */
} ble_cus_evt_t;

typedef void (*ble_cus_evt_handler_t) (ble_cus_t * p_cus, ble_cus_evt_t * p_evt);

typedef struct
{
    uint8_t                       initial_custom_value;           /**< Initial custom value */
    ble_cus_evt_handler_t         evt_handler;                    /**< Event handler to be called for handling events in the Custom Service. */
    ble_srv_cccd_security_mode_t  custom_value_char_attr_md;     /**< Initial security level for Custom characteristics attribute */
} ble_cus_init_t;


void ble_cus_on_ble_evt( ble_evt_t const * p_ble_evt, void * p_context);

#define BLE_CUS_DEF(_name)  static ble_cus_t _name; \
NRF_SDH_BLE_OBSERVER(_name ## _obs, BLE_HRS_BLE_OBSERVER_PRIO, ble_cus_on_ble_evt, &_name)


struct ble_cus_s
{
    ble_cus_evt_handler_t         evt_handler;                    /**< Event handler to be called for handling events in the Custom Service. */
    uint16_t                      service_handle;                 /**< Handle of Custom Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t      custom_value_handles[NRF_SDH_BLE_CENTRAL_LINK_COUNT];           /**< Handles related to the Custom Value characteristic. */
    uint16_t                      conn_handle[NRF_SDH_BLE_CENTRAL_LINK_COUNT];                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint8_t                       uuid_type; 
};

static uint8_t m_custom_value = 0;
static uint8_t current_conn_index = 0;

// CUSTOM SERVICE STRUCTURES START

static void hid_kbd_user_ev_handler(app_usbd_class_inst_t const * p_inst, app_usbd_hid_user_event_t event);

static char const m_target_periph_name[] = "nRF Connect";     /**< Name of the device we try to connect to. This name is searched in the scan report data*/

NRF_BLE_SCAN_DEF(m_scan);                                       /**< Scanning module instance. */
NRF_BLE_GATT_DEF(m_gatt);                                       /**< GATT module instance. */
NRF_BLE_GQ_DEF(m_ble_gatt_queue, NRF_SDH_BLE_CENTRAL_LINK_COUNT, NRF_BLE_GQ_QUEUE_SIZE);
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/
BLE_CUS_DEF(m_cus);
APP_USBD_HID_KBD_GLOBAL_DEF(m_app_hid_kbd,
                            APP_USBD_INTERFACE_KBD,
                            NRFX_USBD_EPIN5 ,
                            hid_kbd_user_ev_handler,
                            APP_USBD_HID_SUBCLASS_BOOT
);


// CUSTOM VALUE CODE START

static uint8_t char_value = 0;

static void on_write(ble_cus_t * p_cus, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    //NRF_LOG_INFO("inside on_write funciton");
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
    p_cus->conn_handle[current_conn_index++] = p_ble_evt->evt.gap_evt.conn_handle;
    NRF_LOG_INFO(" %d = Connection handle is %d", current_conn_index - 1, p_cus->conn_handle[current_conn_index - 1]);

    ble_cus_evt_t evt;
    evt.evt_type = BLE_CUS_EVT_CONNECTED;

    p_cus->evt_handler(p_cus, &evt);
}

static void on_disconnect(ble_cus_t * p_cus, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);

    for(int i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
    {
        if(p_cus->conn_handle[i] == p_ble_evt->evt.gatts_evt.conn_handle) 
        {
            p_cus->conn_handle[i] = BLE_CONN_HANDLE_INVALID;
            current_conn_index = i;
            break;
        }
    }

}

static uint32_t char0_add(ble_cus_t * p_cus, const ble_cus_init_t * p_cus_init)
{
    uint32_t            err_code = NRF_SUCCESS;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    
    memset(&char_md, 0, sizeof(ble_gatts_char_md_t));

    char_md.char_ext_props.wr_aux = 1;
    char_md.char_ext_props.reliable_wr = 1;
    char_md.char_props.read = 1;
    char_md.char_props.write = 1;
    char_md.char_props.notify = 1;
    char_md.char_user_desc_max_size = 10;
    char_md.char_user_desc_size = 2;
    char_md.p_cccd_md = &cccd_md;


    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

	
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
    attr_md.read_perm  = p_cus_init->custom_value_char_attr_md.read_perm;
    attr_md.write_perm = p_cus_init->custom_value_char_attr_md.write_perm;


    err_code = sd_ble_gatts_characteristic_add(p_cus->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_cus->custom_value_handles[0]);
    
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("Characteristic 0 successfully added+ value handle is %d.", p_cus->custom_value_handles[0].value_handle);


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

    char_md.char_ext_props.wr_aux = 1;
    char_md.char_ext_props.reliable_wr = 1;
    char_md.char_props.read = 1;
    char_md.char_props.write = 1;
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
    attr_md.read_perm  = p_cus_init->custom_value_char_attr_md.read_perm;
    attr_md.write_perm = p_cus_init->custom_value_char_attr_md.write_perm;


    err_code = sd_ble_gatts_characteristic_add(p_cus->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_cus->custom_value_handles[1]);
    
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("Characteristic 1 successfully added + value handle is %d.", p_cus->custom_value_handles[1].value_handle);

    return NRF_SUCCESS;
}

static uint32_t char2_add(ble_cus_t * p_cus, const ble_cus_init_t * p_cus_init)
{
    uint32_t            err_code = NRF_SUCCESS;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    
    memset(&char_md, 0, sizeof(ble_gatts_char_md_t));

    char_md.char_ext_props.wr_aux = 1;
    char_md.char_ext_props.reliable_wr = 1;
    char_md.char_props.read = 1;
    char_md.char_props.write = 1;
    char_md.char_props.notify = 1;
    char_md.char_user_desc_max_size = 10;
    char_md.char_user_desc_size = 2;
    char_md.p_cccd_md = &cccd_md;


    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

	
    ble_uuid.type = p_cus->uuid_type;
    ble_uuid.uuid = CHAR2_UUID;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    
    attr_char_value.p_uuid = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.max_len   = sizeof(uint8_t);
    attr_char_value.p_value = &char_value;

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.read_perm  = p_cus_init->custom_value_char_attr_md.read_perm;
    attr_md.write_perm = p_cus_init->custom_value_char_attr_md.write_perm;

    err_code = sd_ble_gatts_characteristic_add(p_cus->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_cus->custom_value_handles[2]);
    
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("Characteristic 2 successfully added + value handle is %d.", p_cus->custom_value_handles[2].value_handle);

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
    if (p_cus == NULL || p_cus_init == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    for(int i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
    {
        p_cus->conn_handle[i]               = BLE_CONN_HANDLE_INVALID;
    }
    p_cus->evt_handler               = p_cus_init->evt_handler;

    // Add Custom Service UUID
    ble_uuid128_t base_uuid = {CUSTOM_SERVICE_UUID_BASE};
    err_code =  sd_ble_uuid_vs_add(&base_uuid, &p_cus->uuid_type);
    VERIFY_SUCCESS(err_code);
    
    ble_uuid.type = p_cus->uuid_type;
    ble_uuid.uuid = CUSTOM_SERVICE_UUID;

    // Add the Custom Service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_cus->service_handle);
    APP_ERROR_CHECK(err_code);

    err_code = char0_add(p_cus, p_cus_init);
    APP_ERROR_CHECK(err_code);

    err_code = char1_add(p_cus, p_cus_init);
    APP_ERROR_CHECK(err_code);

    err_code = char2_add(p_cus, p_cus_init);
    APP_ERROR_CHECK(err_code);
    
    return err_code;
}

uint32_t ble_cus_custom_value_update(ble_cus_t * p_cus, uint8_t custom_value){
    if (p_cus == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = sizeof(uint8_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = &custom_value;

    // Update database.

    for(int i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
    {
       if (p_cus->conn_handle[i] != BLE_CONN_HANDLE_INVALID) 
        {
            err_code = sd_ble_gatts_value_set(p_cus->conn_handle[i],
                                                p_cus->custom_value_handles[i].value_handle,
                                                &gatts_value);
            APP_ERROR_CHECK(err_code);
        }
    }

    for(int i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
    {
        if (p_cus->conn_handle[i] != BLE_CONN_HANDLE_INVALID) 
        {
            
            ble_gatts_hvx_params_t hvx_params;

            memset(&hvx_params, 0, sizeof(hvx_params));

            hvx_params.handle = p_cus->custom_value_handles[2].value_handle;
            hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset = gatts_value.offset;
            hvx_params.p_len  = &gatts_value.len;
            hvx_params.p_data = gatts_value.p_value;

            NRF_LOG_INFO("%d = Conn handle: %d, Custom value handle : %d, CCCD handle : %d", i, p_cus->conn_handle[i], p_cus->custom_value_handles[i].value_handle, p_cus->custom_value_handles[i].cccd_handle );
            err_code = sd_ble_gatts_hvx(p_cus->conn_handle[i], &hvx_params);
            NRF_LOG_INFO("Error is %d", err_code);
            APP_ERROR_CHECK(err_code);
        }

    }
 
    return err_code;
}

// CUSTOM VALUE CODE END

// APPLICATION CODE START

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void leds_init()
{
    bsp_board_init(BSP_INIT_LEDS);
}

static void scan_start()
{
    ret_code_t err_code;

    err_code = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(err_code);

    bsp_board_led_off(CENTRAL_CONNECTED_LED);
    bsp_board_led_on(CENTRAL_SCANNING_LED);
}

static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    // For readability.
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        // Upon connection, check which peripheral has connected (HR or RSC), initiate DB
        // discovery, update LEDs status and resume scanning if necessary. */
        case BLE_GAP_EVT_CONNECTED:
        
            NRF_LOG_INFO("Connected %d.", p_ble_evt->evt.gap_evt.conn_handle);

            // Update LEDs status, and check if we should be looking for more
            // peripherals to connect to.


            if(p_ble_evt->evt.gap_evt.conn_handle + 1 == NRF_SDH_BLE_CENTRAL_LINK_COUNT)
            {
                NRF_LOG_INFO("Maximum peripherals connected.");
                bsp_board_led_on(CENTRAL_CONNECTED_LED);
                bsp_board_led_off(CENTRAL_SCANNING_LED);
            }
            else 
            {
                NRF_LOG_INFO("Continue scanning.");
                scan_start();
            }
            break;

        // Upon disconnection, reset the connection handle of the peer which disconnected, update
        // the LEDs status and start scanning again.
        case BLE_GAP_EVT_DISCONNECTED:
        {
            NRF_LOG_INFO("Disconnected.");
            bsp_board_led_off(CENTRAL_CONNECTED_LED);
            bsp_board_led_on(CENTRAL_SCANNING_LED);
            scan_start();
        } break;

        case BLE_GAP_EVT_TIMEOUT:
        {
            // We have not specified a timeout for scanning, so only connection attemps can timeout.
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_DEBUG("Connection request timed out.");
            }
        } break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        {
            // Accept parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                        &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
        {
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_TIMEOUT:
        {
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        } break;

        default:
            // No implementation needed.
           // NRF_LOG_INFO("Some event happened");
            break;
    }
}

static void ble_stack_init()
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

static void button_event_handler(bsp_event_t bsp_event)
{
    ret_code_t err_code;
    switch ((unsigned int)bsp_event)
    {
       case BSP_EVENT_KEY_0:
            NRF_LOG_INFO("Button is pressed.");

            // USBD STOP THE MOVIE CODE USE IT LATER
           /* err_code = app_usbd_hid_kbd_key_control(&m_app_hid_kbd, CONFIG_KBD_LETTER, true);
            APP_ERROR_CHECK(err_code);
            err_code = app_usbd_hid_kbd_key_control(&m_app_hid_kbd, CONFIG_KBD_LETTER, false);
            APP_ERROR_CHECK(err_code);*/

     
            // Increment the value of m_custom_value before nortifing it.
            m_custom_value++;

            err_code = ble_cus_custom_value_update(&m_cus, m_custom_value);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            return; // no implementation needed
    }
}

static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;

    switch(p_scan_evt->scan_evt_id)
    {
        case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
            err_code = p_scan_evt->params.connecting_err.err_code;
            APP_ERROR_CHECK(err_code);
            break;

        default:
          break;
    }
} 

static void buttons_init()
{
    ret_code_t err_code;

    err_code = bsp_init(BSP_INIT_BUTTONS, button_event_handler);
    APP_ERROR_CHECK(err_code);
}

static void log_init()
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

static void timer_init()
{
    ret_code_t err_code;

    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

static void power_management_init()
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

static void scan_init()
{
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;

    memset(&init_scan, 0, sizeof(init_scan));

    init_scan.connect_if_match = true;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);

    // Setting filters for scanning.
    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_NAME_FILTER, false);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER, m_target_periph_name);
    APP_ERROR_CHECK(err_code);
}

static void gatt_init()
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}

static void idle_state_handle()
{
    NRF_LOG_FLUSH();
    nrf_pwr_mgmt_run();
}

static void on_cus_evt(ble_cus_t * p_cus_service, ble_cus_evt_t * p_evt)
{
    switch(p_evt->evt_type)
    {
        case BLE_CUS_EVT_CONNECTED:
            break;

        case BLE_CUS_EVT_DISCONNECTED:
            break;

        default:
            break;
    }
}

static void services_init()
{
    ret_code_t         err_code;
    nrf_ble_qwr_init_t qwr_init = {0};
    ble_cus_init_t     cus_init;

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize CUS Service init structure to zero.
    memset(&cus_init, 0, sizeof(cus_init));

    cus_init.evt_handler                = on_cus_evt;
	
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init.custom_value_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init.custom_value_char_attr_md.write_perm);

    err_code = ble_cus_init(&m_cus, &cus_init);
    APP_ERROR_CHECK(err_code);	
} 

static void gap_params_init()
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);


    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

// USB CODE START
static bool m_usb_connected = false;

static void hid_kbd_user_ev_handler(app_usbd_class_inst_t const * p_inst, app_usbd_hid_user_event_t event)
{
    switch (event) {
        case APP_USBD_HID_USER_EVT_SET_BOOT_PROTO:
            hid_kbd_clear_buffer(p_inst);
            break;
        default:
            break;
    }
}

static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    switch (event)
    {
        case APP_USBD_EVT_STOPPED:
            app_usbd_disable();
            break;
        case APP_USBD_EVT_POWER_DETECTED:
            NRF_LOG_INFO("USB power detected");
            if (!nrfx_usbd_is_enabled())
            {
                app_usbd_enable();
            }
            break;
        case APP_USBD_EVT_POWER_REMOVED:
            NRF_LOG_INFO("USB power removed");
            m_usb_connected = false;
            app_usbd_stop();
            break;
        case APP_USBD_EVT_POWER_READY:
            NRF_LOG_INFO("USB ready");
            m_usb_connected = true;
            app_usbd_start();
            break;

        default:
            break;
    }
}

// USB CODE ENDED

int main(void)
{
    // Initialize.
    ret_code_t ret;
    static const app_usbd_config_t usbd_config = {
        .ev_state_proc = usbd_user_ev_handler,
    };
    log_init();
    timer_init();
    buttons_init();
    leds_init();

    ret = nrf_drv_clock_init();
    APP_ERROR_CHECK(ret);

    ret = app_usbd_init(&usbd_config);
    APP_ERROR_CHECK(ret);

    app_usbd_class_inst_t const * class_inst_kbd;
    class_inst_kbd = app_usbd_hid_kbd_class_inst_get(&m_app_hid_kbd);

    ret = app_usbd_class_append(class_inst_kbd);
    APP_ERROR_CHECK(ret);
  
    power_management_init();
    ble_stack_init();
    gap_params_init();
    services_init();
    scan_init();
    gatt_init();


    // Start execution.
    NRF_LOG_INFO("Server started");
    scan_start();

      if (USBD_POWER_DETECTION)
    {
        NRF_LOG_INFO("USB Power detection true.")
        ret = app_usbd_power_events_enable();
        APP_ERROR_CHECK(ret);
    }
    else
    {
        NRF_LOG_INFO("No USB power detection enabled.");

        app_usbd_enable();
        app_usbd_start();
    }

    // Enter main loop.
    for (;;)
    {
        while (app_usbd_event_queue_process())
        {
        }
        idle_state_handle();
    }
}

/*
MEMORY
{
  FLASH (rx) : ORIGIN = 0x27000, LENGTH = 0xda000
  RAM (rwx) :  ORIGIN = 0x20002250, LENGTH = 0x3dd20
}
*/

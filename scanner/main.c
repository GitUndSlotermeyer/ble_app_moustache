#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "nrf_pwr_mgmt.h"
#include "app_timer.h"
#include "boards.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_ble_qwr.h"
#include "ble_lbs_c.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_scan.h"
#include "app_usbd_types.h"
#include "app_usbd.h"
#include "app_usbd_core.h"
#include "app_usbd_hid_generic.h"
#include "app_usbd_hid_kbd.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_power.h"

#define DEVICE_NAME                     "nRF Connect"                       /**< Name of device. Will be included in the advertising data. */
#define CENTRAL_SCANNING_LED            BSP_BOARD_LED_1                     /**< Scanning LED will be on when the device is scanning. */
#define CENTRAL_CONNECTED_LED           BSP_BOARD_LED_2                    /**< Connected LED will be on when the device is connected. */

#define APP_BLE_CONN_CFG_TAG            1                                   /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO           3                                   /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define NUMBER_OF_CHARACTERISTIC 2
#define USBD_POWER_DETECTION true

#define CONFIG_KBD_LETTER       APP_USBD_HID_KBD_SPACEBAR
#define APP_USBD_INTERFACE_KBD      1

#define LED_CHARACTERISTIC 0
#define BTN_CHARACTERISTIC 1

// SERVICE & CHARACTERISTIC UUID'S
#define CUSTOM_SERVICE_UUID_BASE         {0xBC, 0x8A, 0xBF, 0x45, 0xCA, 0x05, 0x50, 0xBA, 0x40, 0x42, 0xB0, 0x00, 0xC9, 0xAD, 0x64, 0xF3}
#define CUSTOM_SERVICE_UUID               0x1400

static void hid_kbd_user_ev_handler(app_usbd_class_inst_t const * p_inst, app_usbd_hid_user_event_t event);

static char const m_target_periph_name[] = "nRF Connect";     /**< Name of the device we try to connect to. This name is searched in the scan report data*/

NRF_BLE_SCAN_DEF(m_scan);                                       /**< Scanning module instance. */
NRF_BLE_GATT_DEF(m_gatt);                                       /**< GATT module instance. */
NRF_BLE_GQ_DEF(m_ble_gatt_queue, NRF_SDH_BLE_CENTRAL_LINK_COUNT, NRF_BLE_GQ_QUEUE_SIZE);
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/
BLE_DB_DISCOVERY_DEF(m_db_disc);
APP_USBD_HID_KBD_GLOBAL_DEF(m_app_hid_kbd,
                            APP_USBD_INTERFACE_KBD,
                            NRFX_USBD_EPIN5 ,
                            hid_kbd_user_ev_handler,
                            APP_USBD_HID_SUBCLASS_BOOT
);

// APPLICATION CODE START

static ble_gatt_db_char_t characteristic[NRF_SDH_BLE_CENTRAL_LINK_COUNT][NUMBER_OF_CHARACTERISTIC];
static uint8_t peripheral_conn_handles[NRF_SDH_BLE_CENTRAL_LINK_COUNT];
static uint8_t connected_devices = 0;
static uint8_t video_paused = 0;
static uint8_t winner_conn_handle = 0;
 
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

static void write_to_LED(uint8_t conn_handle, uint8_t value, uint8_t char_index)
{
    ret_code_t err_code;

    //find out index of conn handle
    uint8_t index = -1;
    for(int i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
    {
        if(conn_handle == peripheral_conn_handles[i])
        {
            index = i;
            break;
        }
    }
    
    if (index == -1)
    {
        return;
    }

    ble_gattc_write_params_t const  write_params = 
    {
        .len = sizeof(uint8_t),
        .offset = 0,
        .handle = characteristic[index][char_index].characteristic.handle_value,
        .flags = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE,
        .write_op = BLE_GATT_OP_WRITE_CMD,
        .p_value = &value
    };

    err_code = sd_ble_gattc_write(conn_handle, &write_params);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Wrote to the characteristic.");
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
        
           // NRF_LOG_INFO("Connected %d.", p_ble_evt->evt.gap_evt.conn_handle);

            // Update LEDs status, and check if we should be looking for more
            // peripherals to connect to.

            memset(&m_db_disc, 0x00, sizeof(m_db_disc));
            err_code = ble_db_discovery_start(&m_db_disc, p_ble_evt->evt.gap_evt.conn_handle);
            APP_ERROR_CHECK(err_code);

           // NRF_LOG_INFO("DB_DISCOVERY started.");

           // find out what field in array is uninitialized
           for(uint8_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
           {
               if(peripheral_conn_handles[i] == 0xFF) 
               {
                peripheral_conn_handles[i] = p_ble_evt->evt.gap_evt.conn_handle;
               // NRF_LOG_INFO("Index = %d, value = %d", i,  p_ble_evt->evt.gap_evt.conn_handle);
                break;
               }
           }

            if(++connected_devices == NRF_SDH_BLE_CENTRAL_LINK_COUNT) 
            {
                NRF_LOG_INFO("Max peripherals connected");
                bsp_board_led_off(CENTRAL_SCANNING_LED);
                bsp_board_led_on(CENTRAL_CONNECTED_LED); 
            }
            else 
            {
                NRF_LOG_INFO("Continue scanning");
                scan_start();
            }
        break;

        // Upon disconnection, reset the connection handle of the peer which disconnected, update
        // the LEDs status and start scanning again.
        case BLE_GAP_EVT_DISCONNECTED:
        {
            NRF_LOG_INFO("Disconnected.");
            connected_devices--;

            // uninitialize field of disconnected peripheral in array 
            for(uint8_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
            {
                if(peripheral_conn_handles[i] == p_ble_evt->evt.gap_evt.conn_handle)
                {
                    peripheral_conn_handles[i] = 0xff;
                    break;
                }
            }
            bsp_board_led_off(CENTRAL_CONNECTED_LED);
            bsp_board_led_on(CENTRAL_SCANNING_LED);
            scan_start();
        } 
        break;

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
        } 
        break; 

        case BLE_GATTC_EVT_HVX :
        {
            //NRF_LOG_INFO("Notification is delivered.");
            //NRF_LOG_INFO("New value is %d", p_ble_evt->evt.gattc_evt.params.hvx.data[0]);

                switch (video_paused)
                {
                    case 0:
                        NRF_LOG_INFO("Video is not paused. Winner is %d.", p_ble_evt->evt.gattc_evt.conn_handle);
                        video_paused++;
                        winner_conn_handle = p_ble_evt->evt.gattc_evt.conn_handle;
                        write_to_LED(winner_conn_handle, 0x01, LED_CHARACTERISTIC);
                        break;
                    
                    case 1:
                        if(winner_conn_handle == p_ble_evt->evt.gattc_evt.conn_handle)
                        {    
                            NRF_LOG_INFO("Video is paused. Winner clicked.");
                            video_paused--;
                            write_to_LED(p_ble_evt->evt.gattc_evt.conn_handle, 0x00, LED_CHARACTERISTIC);
                        }
                        else
                        {
                            NRF_LOG_INFO("Someone clicked who is not winner");
                        }
                        break;

                    default:
                        break;
                }

            }
        
        default:
            // No implementation needed.
            //NRF_LOG_INFO("Some event happened");
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
    //ret_code_t err_code;
    switch ((unsigned int)bsp_event)
    {
       case BSP_EVENT_KEY_0:
            NRF_LOG_INFO("Button is pressed.");

            // USBD STOP THE MOVIE CODE USE IT LATER
           /* err_code = app_usbd_hid_kbd_key_control(&m_app_hid_kbd, CONFIG_KBD_LETTER, true);
            APP_ERROR_CHECK(err_code);
            err_code = app_usbd_hid_kbd_key_control(&m_app_hid_kbd, CONFIG_KBD_LETTER, false);
            APP_ERROR_CHECK(err_code);*/

            

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

static void idle_state_handle()
{
    NRF_LOG_FLUSH();
    nrf_pwr_mgmt_run();
}

static void services_init()
{
    ret_code_t         err_code;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
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

static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ret_code_t err_code;
    
    switch (p_evt->evt_type)
    {
        // structure needed for differentiation of various peripheral characterstic is filled
        case BLE_DB_DISCOVERY_COMPLETE:
        { 
           // NRF_LOG_INFO("BLE_DB_DISCOVERY_COMPLETE event triggered.");

            ble_gatt_db_srv_t * discovered_db = &(p_evt->params.discovered_db);
            uint8_t index = 0xff;

            // find index of conn_handle
            for(uint8_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
            {
                if(peripheral_conn_handles[i] == p_evt->conn_handle)
                {
                    index = i;
                    break;
                }
            }

            // Save each characteristic to seperate row
            // Rows are indexed by connection handles
            for(int i = 0; i < discovered_db->char_count; i++) 
            {
                characteristic[index][i] = discovered_db->charateristics[i];
            }

            uint16_t const value = 0x0001;

            // Enable notification for second characteristic for each client
            // Set CCCD value to 1
            ble_gattc_write_params_t const write_params = {
                .offset = 0,
                .flags = BLE_GATT_OP_WRITE_CMD,
                .handle = characteristic[index][1].cccd_handle,
                .len = sizeof(value),
                .write_op = BLE_GATT_OP_WRITE_CMD,
                .p_value = (uint8_t *)&value
            };

            err_code = sd_ble_gattc_write(peripheral_conn_handles[index], &write_params);
            APP_ERROR_CHECK(err_code);        
        }
        break;
    
    case BLE_DB_DISCOVERY_ERROR:
        NRF_LOG_INFO("Database Discovery error.");
        break;
    
    case BLE_DB_DISCOVERY_SRV_NOT_FOUND:
        NRF_LOG_INFO("Service is not found.");
        break;
    
    default:
        break;
    }
}

static void db_discovery_init()
{
    ble_db_discovery_init_t db_init;

    memset(&db_init, 0, sizeof(db_init));

    db_init.evt_handler  = db_disc_handler;
    db_init.p_gatt_queue = &m_ble_gatt_queue;

    ret_code_t err_code = ble_db_discovery_init(&db_init);
    APP_ERROR_CHECK(err_code);

    ble_uuid_t const service_uuid = 
    {
        .type = BLE_UUID_TYPE_VENDOR_BEGIN,
        .uuid = 0x1400
    };

    ble_uuid_t helper_uuid = 
    {
        .type = BLE_UUID_TYPE_VENDOR_BEGIN
    };
    ble_uuid128_t const base_uuid = {CUSTOM_SERVICE_UUID_BASE};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &(helper_uuid.type));
    NRF_LOG_INFO("sd_vs_add error is %d", err_code);
    APP_ERROR_CHECK(err_code);

    err_code = ble_db_discovery_evt_register(&service_uuid);
    APP_ERROR_CHECK(err_code);
}

static void handle_struct_init()
{
    for(uint8_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++ )
    {
        peripheral_conn_handles[i] = 0xff;
    }
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
    handle_struct_init();

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
    db_discovery_init();


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

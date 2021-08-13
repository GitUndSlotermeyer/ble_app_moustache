#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "../../../components/libraries/util/nordic_common.h"
#include "../../../modules/nrfx/mdk/nrf.h"
#include "../../../components/libraries/util/app_error.h"
#include "../../../components/softdevice/s140/headers/ble.h"
#include "../../../components/softdevice/s140/headers/ble_hci.h"
#include "../../../components/ble/common/ble_srv_common.h"
#include "../../../components/ble/common/ble_advdata.h"
#include "../../../components/ble/ble_advertising/ble_advertising.h"
#include "../../../components/ble/common/ble_conn_params.h"
#include "../../../components/softdevice/common/nrf_sdh.h"
#include "../../../components/softdevice/common/nrf_sdh_soc.h"
#include "../../../components/softdevice/common/nrf_sdh_ble.h"
#include "../../../components/libraries/timer/app_timer.h"
#include "../../../components/libraries/fds/fds.h"
#include "../../../components/ble/peer_manager/peer_manager.h"
#include "../../../components/ble/peer_manager/peer_manager_handler.h"
#include "../../../components/libraries/bsp/bsp_btn_ble.h"
#include "../../../components/libraries/sensorsim/sensorsim.h"
#include "../../../components/ble/common/ble_conn_state.h"
#include "../../../components/ble/nrf_ble_gatt/nrf_ble_gatt.h"
#include "../../../components/ble/nrf_ble_qwr/nrf_ble_qwr.h"
#include "../../../components/libraries/pwr_mgmt/nrf_pwr_mgmt.h"
#include "../../../components/libraries/util/app_util_platform.h"
#include "../../../components/boards/boards.h"
#include "../../../components/libraries/bsp/bsp.h"
#include "../../../components/libraries/pwm/app_pwm.h"
#include "../../../modules/nrfx/drivers/include/nrfx_pwm.h"
#include "../../../components/ble/nrf_ble_gq/nrf_ble_gq.h"
#include "../../../components/ble/ble_db_discovery/ble_db_discovery.h"
#include "../../../components/ble/common/ble_gatt_db.h"

#include "../../../components/libraries/log/nrf_log.h"
#include "../../../components/libraries/log/nrf_log_ctrl.h"
#include "../../../components/libraries/log/nrf_log_default_backends.h"
#include "ble_cus.h"

#define DEVICE_NAME                     "nRF Connect"                       /**< Name of device. Will be included in the advertising data. */
#define APP_ADV_INTERVAL                300                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

#define APP_ADV_DURATION                18000                                   /**< The advertising duration (180 seconds) in units of 10 milliseconds. */
#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

// Connection negotiation parameters
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

// LEDS for visual recognition
#define CENTRAL_ADV_LED         1
#define CENTRAL_CONN_LED        2

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

// Instance init
NRF_BLE_GQ_DEF(m_ble_gatt_queue, NRF_SDH_BLE_CENTRAL_LINK_COUNT, NRF_BLE_GQ_QUEUE_SIZE);
NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
//NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                             /**< Advertising module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);
BLE_CUS_DEF(m_cus);

// Global variables init
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/*
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}*/

static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void timers_init()
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
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

static void gap_params_init()
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    switch (p_evt -> evt_type)
    {
        case BLE_CONN_PARAMS_EVT_FAILED:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("Connection negotiation failed.");
            break;
        
        case BLE_CONN_PARAMS_EVT_SUCCEEDED:
            NRF_LOG_INFO("Connection negotiation succeeded.");
            break;
    }
}

static void conn_params_init()
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

static void sleep_mode_enter()
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}

static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
            break;

        case BLE_ADV_EVT_IDLE:
            NRF_LOG_INFO("Entering sleep mode.");
            sleep_mode_enter();
            break;

        default:
            break;
    }
}

static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
            bsp_board_led_off(CENTRAL_CONN_LED);
            bsp_board_led_on(CENTRAL_ADV_LED);
            break;

        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.");
            bsp_board_led_off(CENTRAL_ADV_LED);
            bsp_board_led_on(CENTRAL_CONN_LED);

            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
          //  err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            //APP_ERROR_CHECK(err_code);

            break;

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
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_SCAN_REQ_REPORT:
            NRF_LOG_INFO("Scan request has landed.");
            break;

        case BLE_GATTC_EVT_READ_RSP:
            NRF_LOG_INFO("Read response value is %d", p_ble_evt->evt.gattc_evt.params.read_rsp.data[0]);
            NRF_LOG_INFO("Read response value is %d", p_ble_evt->evt.gattc_evt.params.read_rsp.data[1]);
            break;
        
        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
            NRF_LOG_INFO("Hand value notification complete.");
            break;

        case BLE_GATTS_EVT_WRITE :
            NRF_LOG_INFO("CCCD descriptor is changed.");
            bsp_board_led_on(3);
            break;
        default:
            // No implementation needed.
          //  NRF_LOG_INFO("Some event happened");
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

static void btn_evt_handler(bsp_event_t event)
{
    ret_code_t err_code;
    switch (event)
    {
        case BSP_EVENT_KEY_0:
            NRF_LOG_INFO("Button 0 is pressed.");
            if(m_conn_handle != BLE_CONN_HANDLE_INVALID)
            {
            err_code =  ble_cus_value_update_and_notify(&m_cus, 0x01, 1);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("Value is set and scanner is notified.");
            }
            break;

        case BSP_EVENT_KEY_1:
            break;

        default:
            break;
    }
}
static void advertising_init() 
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

static void buttons_leds_init()
{
    bsp_board_init(BSP_INIT_LEDS);
  
    ret_code_t err_code = bsp_init(BSP_INIT_BUTTONS, btn_evt_handler);
    APP_ERROR_CHECK(err_code);
}

static void log_init()
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

static void power_management_init()
{
    ret_code_t err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

static void idle_state_handle()
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

static void advertising_start()
{
    ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    bsp_board_led_on(CENTRAL_ADV_LED);

    APP_ERROR_CHECK(err_code);
}

static void services_init()
{
    ret_code_t         err_code;
    //nrf_ble_qwr_init_t qwr_init = {0};
    ble_cus_init_t     cus_init;

    // Initialize Queued Write Module.
  //  qwr_init.error_handler = nrf_qwr_error_handler;

   /* err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);*/

    // Initialize CUS Service init structure to zero.
    memset(&cus_init, 0, sizeof(cus_init));

    cus_init.evt_handler                = on_cus_evt;
	
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init.custom_value_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init.custom_value_char_attr_md.write_perm);

    err_code = ble_cus_init(&m_cus, &cus_init);
    APP_ERROR_CHECK(err_code);	

}

static void gatt_init()
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}

int main()
{
    timers_init();
    log_init();
    buttons_leds_init();
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();

    // Start execution.

    advertising_start();
    NRF_LOG_INFO("Client started.");

    for (;;)
    {
        idle_state_handle();
    }
}
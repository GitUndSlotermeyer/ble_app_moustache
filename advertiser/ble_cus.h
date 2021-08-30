#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "ble.h"
#include "ble_srv_common.h"

#define CUSTOM_SERVICE_UUID_BASE         {0xBC, 0x8A, 0xBF, 0x45, 0xCA, 0x05, 0x50, 0xBA, 0x40, 0x42, 0xB0, 0x00, 0xC9, 0xAD, 0x64, 0xF3}
#define CUSTOM_SERVICE_UUID               0x1400
#define NUMBER_OF_CHARACTERISTIC 2
#define CHAR0_UUID                        0x1401
#define CHAR1_UUID                        0x1402

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
    ble_gatts_char_handles_t      custom_value_handles[NUMBER_OF_CHARACTERISTIC];           /**< Handles related to the Custom Value characteristic. */
    uint16_t                      conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint8_t                       uuid_type; 
};


uint32_t ble_cus_init(ble_cus_t * p_cus, const ble_cus_init_t * p_cus_init);

uint32_t ble_cus_value_update_and_notify(ble_cus_t * p_cus, uint8_t new_value, uint8_t char_index);
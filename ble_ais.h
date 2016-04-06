

#ifndef BLE_AIS_H__
#define BLE_AIS_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"


// UUID : a51455bf-d64a-4579-b51c-f27eb4c9c343
#define AIS_UUID_BASE {0x43, 0xC3, 0xC9, 0xB4, 0x7E, 0xF2, 0x1C, 0xB5, 0x79, 0x45, 0x4A, 0xD6, 0xBF, 0x55, 0x14, 0xA5}
#define AIS_UUID_SERVICE 0x2623
#define AIS_UUID_BUTTON_CHAR 0x2524
#define AIS_UUID_DATA_CHAR 0x3000

// MTU length
#define BLE_AIS_DATA_CHAR_LEN 20

/**@brief Service event type. */
// Copied from Nordic example template
typedef enum
{
    BLE_AIS_EVT_NOTIFICATION_ENABLED,              
    BLE_AIS_EVT_NOTIFICATION_DISABLED                         
} ble_ais_evt_type_t;

/**@brief Service event. */
// Copied from Nordic example template
typedef struct
{
    ble_ais_evt_type_t evt_type;                                  /**< Type of event. */
} ble_ais_evt_t;

// Forward declaration of the ble_ais_t type. 
typedef struct ble_ais_s ble_ais_t;
typedef void (*ble_ais_evt_handler_t) (ble_ais_t * p_ais, uint8_t new_state);

typedef struct
{
    ble_ais_evt_handler_t evt_write_handler;                    /**< Event handler. */
} ble_ais_init_t;


typedef struct ble_ais_s
{
    uint16_t                    service_handle;
    ble_gatts_char_handles_t    data_char_handles;
    uint8_t                     uuid_type;
    uint16_t                    conn_handle;
    ble_ais_evt_handler_t 			evt_write_handler;
} ble_ais_t;



// Initialize the service
uint32_t ble_ais_init(ble_ais_t * p_ais, const ble_ais_init_t * p_ais_init);

// Function for handling the Application's BLE Stack events.
// Copied from Nordic example template
void ble_ais_on_ble_evt(ble_ais_t * p_ais, ble_evt_t * p_ble_evt);

// Function for sending a data packet notification
uint32_t ble_ais_data_send(ble_ais_t * p_ais, uint8_t data[BLE_AIS_DATA_CHAR_LEN]);

#endif // BLE_AIS_H__

/** @} */

#ifndef BLE_REC_H__
#define BLE_REC_H__

#include <stdint.h>
#include <stdbool.h>
#include "sdk_config.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"

#define BLE_UUID_REC_SERVICE 0x0002                      /**< The UUID of the Nordic UART Service. */

#define OPCODE_LENGTH 1
#define HANDLE_LENGTH 2

/**@brief   Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
#if defined(NRF_SDH_BLE_GATT_MAX_MTU_SIZE) && (NRF_SDH_BLE_GATT_MAX_MTU_SIZE != 0)
    #define BLE_NUS_MAX_DATA_LEN (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH)
#else
    #define BLE_NUS_MAX_DATA_LEN (BLE_GATT_MTU_SIZE_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH)
    #warning NRF_SDH_BLE_GATT_MAX_MTU_SIZE is not defined.
#endif

#define BLE_REC_DEF(_name)                                                                          \
static ble_rec_t _name;                                                                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_HRS_BLE_OBSERVER_PRIO,                                                     \
                     ble_rec_on_ble_evt, &_name)

// Event types for rec service
typedef enum {
    BLE_REC_EVT_RX_DATA
} ble_rec_evt_type_t;

typedef struct
{
    uint8_t const * p_data;           /**< A pointer to the buffer with received data. */
    uint16_t        length;           /**< Length of received data. */
} ble_rec_evt_rx_data_t;

// Forward delcaration for the rec type
typedef struct ble_rec_s ble_rec_t;

// Event structure for rec
typedef struct {
    ble_rec_evt_type_t evt_type; // Type of event
    ble_rec_t* p_rec; // Pointer to what made this event happen
    union{
        ble_rec_evt_rx_data_t rx_data; // Data for the event
    } params;
} ble_rec_evt_t;

// Forward delcaration for the rec type
typedef struct ble_rec_s ble_rec_t;

// Type for data handler used
typedef void (*ble_rec_data_handler_t) (ble_rec_evt_t * p_rec_evt);

// Struct needed to init things
typedef struct {
    ble_rec_data_handler_t data_handler;
} ble_rec_init_t;

// Structure for the receive service
struct ble_rec_s {
    uint8_t uuid_type;
    ble_rec_data_handler_t data_handler; // Handler used to receive data
    ble_gatts_char_handles_t rx_handles;              /**< Handles related to the RX characteristic (as provided by the SoftDevice). */
    uint16_t service_handle;
    uint16_t conn_handle;

};

// Function to handle init
uint32_t ble_rec_init(ble_rec_t* p_hrs, ble_rec_init_t const * p_rec_init);

// Function to handle this service's event
void ble_rec_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

#endif
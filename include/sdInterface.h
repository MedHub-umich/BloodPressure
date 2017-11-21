// bleInterface.h
// THis is meant to abstract the crap of sd
#ifndef SOFT_DEVICE_INTERFACE_H
#define SOFT_DEVICE_INTERFACE_H

#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_bas.h"
#include "ble_hrs.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "sensorsim.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_freertos.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "bsp_btn_ble.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ble_conn_state.h"
#include "fds.h"

#define DEVICE_NAME                         "TYLERSBLE"                            /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                   "MedHub"                   /**< Manufacturer. Will be passed to Device Information Service. */

#define APP_BLE_OBSERVER_PRIO               1                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG                1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_FEATURE_NOT_SUPPORTED           BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2    /**< Reply when unsupported features are requested. */


// Defines for BLE
#define DEAD_BEEF                           0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

// Defines for Peer Manager
#define SEC_PARAM_BOND                      1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                      0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                      0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS                  0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES           BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                       0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE              7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE              16   

// Defines for GAP
#define MIN_CONN_INTERVAL                   MSEC_TO_UNITS(10, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.4 seconds). */
#define MAX_CONN_INTERVAL                   MSEC_TO_UNITS(650, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.65 second). */
#define SLAVE_LATENCY                       0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                    MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory time-out (4 seconds). */

#define APP_ADV_INTERVAL                    300                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS          180                                     /**< The advertising time-out in units of seconds. */

#define FIRST_CONN_PARAMS_UPDATE_DELAY      5000                                    /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY       30000                                   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT        3                                       /**< Number of attempts before giving up the connection parameter negotiation. */


// Define services for the BLE
NRF_BLE_GATT_DEF(m_gatt);                                           /**< GATT module instance. */
BLE_ADVERTISING_DEF(m_advertising);                                 /**< Advertising module instance. */

// Public functions
void peer_manager_init(void);
void advertising_start(void * p_erase_bonds); // "Start" BLE (by starting to advertise)
void gap_params_init(void);
void gatt_init(void);
void delete_bonds(void);
void advertising_init(void);
void on_adv_evt(ble_adv_evt_t ble_adv_evt);
void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context);
void ble_stack_init(void);
void bsp_event_handler(bsp_event_t event);
void sleep_mode_enter(void);
void conn_params_init(ble_hrs_t* m_hrs);
void conn_params_error_handler(uint32_t nrf_error);
void on_conn_params_evt(ble_conn_params_evt_t * p_evt);

#endif //SOFT_DEVICE_INTERFACE_H
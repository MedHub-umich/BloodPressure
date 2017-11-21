// Implementation of our receive service Nordic

#include "ble_rec.h"
#include "sdk_common.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/* TO WRITE TO ME!
    1. Run primary to find the attribute handle (should be 0x0017) TODO: determine if this is always the case
    2. Add two to the attr handle (which is for the rx characterisitc)
    3. Write using char-write-req 0x<handle> <data>
*/

#define BLE_UUID_REC_RX_CHARACTERISTIC 0x0002                      /**< The UUID of the RX Characteristic. */

#define BLE_REC_MAX_RX_CHAR_LEN        BLE_NUS_MAX_DATA_LEN        /**< Maximum length of the RX Characteristic (in bytes). */

#define REC_BASE_UUID                  {{0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x00, 0x00, 0x40, 0x6E}} /**< Used vendor specific UUID. */

// Function to handle the BLE_GAP_EVT_CONNECTTED event
static void on_connect(ble_rec_t *p_rec, ble_evt_t const * p_ble_evt){
    p_rec->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}

// Function to handle the BLE_GAP_EVT_DISCONNECTED EVENT
static void on_disconnect(ble_rec_t* p_rec, ble_evt_t const * p_ble_evt) {
    UNUSED_PARAMETER(p_ble_evt);
    p_rec->conn_handle = BLE_CONN_HANDLE_INVALID;
}


// Function to handle write events (when our device is written to (I think))
static void on_write(ble_rec_t* p_rec, ble_evt_t const * p_ble_evt) {
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    ble_rec_evt_t evt;
    evt.p_rec = p_rec;

    if ( (p_evt_write->handle == p_rec->rx_handles.value_handle)
            && (p_rec->data_handler != NULL)) {
        evt.params.rx_data.p_data = p_evt_write->data;
        evt.params.rx_data.length = p_evt_write->len;
        evt.evt_type = BLE_REC_EVT_RX_DATA;
        p_rec->data_handler(&evt);    
    }
}

// Function to initialize our rx characterisitc
static uint32_t rx_char_add(ble_rec_t* p_rec, const ble_rec_init_t * p_rec_init) {
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t attr_char_value;
    ble_uuid_t ble_uuid;
    ble_gatts_attr_md_t attr_md;
    uint32_t err_code;

    unsigned char * name = (unsigned char *)"TEST";

    // Define the character attributes for the rx
    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.write            = 1;
    char_md.char_props.write_wo_resp    = 1;
    // TODO Figure out the below
    char_md.p_char_user_desc            = name; // TODO: Change this to be more descriptive
    char_md.char_user_desc_size         = 4;
    char_md.char_user_desc_max_size     = 10;
    char_md.p_char_pf                   = NULL; 
    char_md.p_cccd_md                   = NULL;
    char_md.p_sccd_md                   = NULL;

    ble_uuid.type = p_rec->uuid_type;
    ble_uuid.uuid = BLE_UUID_REC_RX_CHARACTERISTIC;

    // Set the attribute metadata qualities
    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen= 1;

    // Set the attributecharacter values
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid = &ble_uuid; // Pointer to the attribute uuid
    attr_char_value.p_attr_md = &attr_md; // Pointer to the attribute metatdata
    attr_char_value.init_len = 1; // Initial lenght of attribute
    attr_char_value.init_offs = 0; // Offset of where the attribute stats (ignore)
    attr_char_value.max_len = BLE_REC_MAX_RX_CHAR_LEN; // Maximum size the attribute can be
    NRF_LOG_INFO("Here");
    // Register the attribute with the device
    err_code =  sd_ble_gatts_characteristic_add(p_rec->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_rec->rx_handles);

    NRF_LOG_INFO("%d", err_code);
    return err_code;
}

// Handler for bluetooth events
void ble_rec_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context) {    
    if ((p_context == NULL) || (p_ble_evt == NULL)) {
        return;
    }

    ble_rec_t * p_rec = (ble_rec_t *)p_context;

    switch(p_ble_evt->header.evt_id) {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_rec, p_ble_evt);
            break;
        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_rec, p_ble_evt);
            break;
        case BLE_GATTS_EVT_WRITE:
            on_write(p_rec, p_ble_evt);
            break;
        default:
            break;
    }
}

// Initializer for the event
uint32_t ble_rec_init(ble_rec_t* p_rec, ble_rec_init_t const * p_rec_init) {
    uint32_t err_code;
    ble_uuid_t ble_uuid;
    // ble_uuid128_t rec_base_uuid = REC_BASE_UUID;

    VERIFY_PARAM_NOT_NULL(p_rec);
    VERIFY_PARAM_NOT_NULL(p_rec_init);

    // Initialze the service
    p_rec->conn_handle = BLE_CONN_HANDLE_INVALID;
    p_rec->data_handler = p_rec_init->data_handler;
    
    // Add custome base UUID
    // err_code = sd_ble_uuid_vs_add(&rec_base_uuid, &p_rec->uuid_type);
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_REC_SERVICE);
    NRF_LOG_INFO("STARTING CHECKS");
    // ble_uuid.type = p_rec->uuid_type;
    // ble_uuid.uuid = BLE_UUID_REC_SERVICE;
    p_rec->uuid_type = ble_uuid.type;
    // Add (register) the new service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_rec->service_handle);
    VERIFY_SUCCESS(err_code);
    // Add the RX characteristic
    err_code = rx_char_add(p_rec, p_rec_init);
    VERIFY_SUCCESS(err_code);
    NRF_LOG_INFO("SUCCESS IN INIT!");
    return NRF_SUCCESS;
}
// The interactions we wish to have the BLE
#ifndef BLE_INTERFACE_H
#define BLE_INTERFACE_H
#include "sdInterface.h"
#include "ble_rec.h"

#define MIN_PACKET_SIZE 2
#define MAX_REC_TYPES 256
#define MAX_PACKET_SIZE 10

typedef struct {
    uint8_t rec_type;
    uint8_t data_length;
    uint8_t data[MAX_PACKET_SIZE - MIN_PACKET_SIZE];
} rec_data_t;

// Type for main function handle registering
typedef void(*rec_data_hook_t) (rec_data_t* rec_data);

rec_data_hook_t hooks[MAX_REC_TYPES];

void bleInit(ble_hrs_t* m_hrs, ble_rec_t* m_rec);
void bleBegin(void * p_erase_bonds);
int sendData(ble_hrs_t* m_hrs, uint8_t* data, size_t length);
void debugErrorMessage(ret_code_t err_code);
int registerDataHook(uint8_t index, rec_data_hook_t hook);


// Private
void services_init(ble_hrs_t* m_hrs, ble_rec_t* m_rec);

#endif //BLE_INTERFACE_H
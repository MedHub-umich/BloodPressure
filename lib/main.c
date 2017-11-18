/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/** @file
 *
 * @defgroup blinky_example_main main.c
 * @{
 * @ingroup blinky_example
 * @brief Blinky Example Application main file.
 *
 * This file contains the source code for a sample application to blink LEDs.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "nrf_twi_mngr.h"
#include "bp.h"

#include "app_error.h"
#include "boards.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define TWI_INSTANCE_ID             0

#define MAX_PENDING_TRANSACTIONS    5

NRF_TWI_MNGR_DEF(m_nrf_twi_mngr, MAX_PENDING_TRANSACTIONS, TWI_INSTANCE_ID);
bpMonitor bpDevice;
uint8_t memRegAddr[] = {98, 172, 186, 200, 214, 228, 242, 0, 14, 28, 42, 56, 70, 84};
uint8_t MEM_INDEX_ADDR = 96;


// TWI (with transaction manager) initialization.
static void twi_config(void)
{
    uint32_t err_code;

    nrf_drv_twi_config_t const config = {
       .scl                = 26,
       .sda                = 25,
       .frequency          = 16395475,
       .interrupt_priority = APP_IRQ_PRIORITY_LOWEST,
       .clear_bus_init     = false
    };

    err_code = nrf_twi_mngr_init(&m_nrf_twi_mngr, &config);
    APP_ERROR_CHECK(err_code);
}

void bloodPressureReadCB(ret_code_t result, void * p_user_data) {
    if (result != NRF_SUCCESS)
    {
        NRF_LOG_INFO("FailedRead");
        NRF_LOG_WARNING("read_bp_registers_cb - error: %d", (int)result);
        return;
    } else {
        bpDevice.diastolic += 25; //fix up the value
        bpDevice.findingBloodPressure = 0; //notify that we are done finding blood pressure
        NRF_LOG_INFO("BP diastolic pressure %d", bpDevice.diastolic);
        NRF_LOG_INFO("BP systolic pressure %d", bpDevice.systolic);
        NRF_LOG_INFO("BP heart rate %d", bpDevice.heartRate);
        NRF_LOG_FLUSH();
    }
}

void getMostRecentBPReadingFromUpper() {

    // [these structures have to be "static" - they cannot be placed on stack
    //  since the transaction is scheduled and these structures most likely
    //  will be referred after this function returns]
    int err_code;
    static nrf_twi_mngr_transfer_t const transfers[] =
    {
        NRF_TWI_MNGR_WRITE(BP_UPPER_ADDR, &bpDevice.currentMemLocation, sizeof(bpDevice.currentMemLocation),  0),
        NRF_TWI_MNGR_READ (BP_UPPER_ADDR, &bpDevice.diastolic, 3, 0),
        
    };
    static nrf_twi_mngr_transaction_t NRF_TWI_MNGR_BUFFER_LOC_IND transaction =
    {
        .callback            = bloodPressureReadCB,
        .p_user_data         = NULL,
        .p_transfers         = transfers,
        .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])
    };

    err_code = nrf_twi_mngr_schedule(&m_nrf_twi_mngr, &transaction);
    if (err_code == NRF_ERROR_NO_MEM) {
        NRF_LOG_INFO("Error Code %d", err_code);
    }
} 

void getMostRecentBPReadingFromLower() {

    // [these structures have to be "static" - they cannot be placed on stack
    //  since the transaction is scheduled and these structures most likely
    //  will be referred after this function returns]
    int err_code;
    static nrf_twi_mngr_transfer_t const transfers[] =
    {
        NRF_TWI_MNGR_WRITE(BP_LOWER_ADDR, &bpDevice.currentMemLocation, sizeof(bpDevice.currentMemLocation),  0),
        NRF_TWI_MNGR_READ (BP_LOWER_ADDR, &bpDevice.diastolic, 3, 0),
        
    };
    static nrf_twi_mngr_transaction_t NRF_TWI_MNGR_BUFFER_LOC_IND transaction =
    {
        .callback            = bloodPressureReadCB,
        .p_user_data         = NULL,
        .p_transfers         = transfers,
        .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])
    };

    err_code = nrf_twi_mngr_schedule(&m_nrf_twi_mngr, &transaction);
    if (err_code == NRF_ERROR_NO_MEM) {
        NRF_LOG_INFO("Error Code %d", err_code);
    }
} 

void getMostRecentBPReading() {
    if (bpDevice.currentMemLocation < 100) {
        getMostRecentBPReadingFromLower();
    } else {
        getMostRecentBPReadingFromUpper();
    }
}

void indexCB(ret_code_t result, void * p_user_data) {
    if (result != NRF_SUCCESS)
    {
        NRF_LOG_INFO("FailedRead");
        NRF_LOG_WARNING("read_bp_registers_cb - error: %d", (int)result);
        return;
    } else {
        bpDevice.currentMemLocation = memRegAddr[bpDevice.currentMemLocation];
        bpDevice.findingIndex = 0; //notify that we are done finding the index in memory of the current BP
        NRF_LOG_INFO("Got the index which is %d", bpDevice.currentMemLocation);
        NRF_LOG_FLUSH();
        getMostRecentBPReading();
    }
}



void getMemIndex() {

    // [these structures have to be "static" - they cannot be placed on stack
    //  since the transaction is scheduled and these structures most likely
    //  will be referred after this function returns]
    int err_code;
    NRF_LOG_INFO("I am going to schedule a transactions");
    NRF_LOG_FLUSH();
    static nrf_twi_mngr_transfer_t const transfers[] =
    {
        NRF_TWI_MNGR_WRITE(BP_UPPER_ADDR, &MEM_INDEX_ADDR, sizeof(MEM_INDEX_ADDR),  0),
        NRF_TWI_MNGR_READ (BP_UPPER_ADDR, &bpDevice.currentMemLocation, 1, 0),
        
    };
    static nrf_twi_mngr_transaction_t NRF_TWI_MNGR_BUFFER_LOC_IND transaction =
    {
        .callback            = indexCB,
        .p_user_data         = NULL,
        .p_transfers         = transfers,
        .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])
    };

    err_code = nrf_twi_mngr_schedule(&m_nrf_twi_mngr, &transaction);
    if (err_code == NRF_ERROR_NO_MEM) {
        NRF_LOG_INFO("Error Code %d", err_code);
    }
} 



/**
 * @brief Function for application main entry.
 */
int main(void)
{
    /* Configure board. */
    bsp_board_leds_init();

    /* Configure TWI */
    twi_config();
    
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    NRF_LOG_INFO("Starting BP Readings!");
    if (NRF_LOG_PROCESS() == false) {}

    /* Toggle LEDs. */
    while (true) {

        bpDevice.findingIndex = 1;
        getMemIndex();
        // bpDevice.findingBloodPressure = 1;
        // getMostRecentBPReading();
        // while(bpDevice.findingBloodPressure) {}
        // NRF_LOG_INFO("Current diastolic pressure %d", bpDevice.diastolic);
        // NRF_LOG_FLUSH();
        nrf_delay_ms(3000);
        //read_bp_registers();
    }
}

/**
 *@}
 **/

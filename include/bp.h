/**
 * Copyright (c) 2015 - 2017, Nordic Semiconductor ASA
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
#ifndef BP_H
#define BP_H


#include "nrf_twi_mngr.h"

#ifdef __cplusplus
extern "C" {
#endif


// 0x90 is the LM75B's address in the mbed Application Shield, it contains
// R/W bit and "nrf_drv_twi" (and consequently "nrf_twi_mngr") requires slave
// address without this bit, hence shifting.
#define BP_ADDR          (0xA0U >> 1)

#define BP_REG_DATA      0x30


extern uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND bp_data_addr;


#define BP_READ(p_reg_addr, p_buffer, byte_cnt) \
    NRF_TWI_MNGR_WRITE(BP_ADDR, p_reg_addr, 1,        NRF_TWI_MNGR_NO_STOP), \
    NRF_TWI_MNGR_READ (BP_ADDR, p_buffer,   byte_cnt, 0)


#define BP_INIT_TRANSFER_COUNT 1

extern nrf_twi_mngr_transfer_t const bp_init_transfers[BP_INIT_TRANSFER_COUNT];

#ifdef __cplusplus
}
#endif

#endif // BP_H

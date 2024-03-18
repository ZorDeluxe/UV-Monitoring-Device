/**
 * Copyright (c) 2015 - 2020, Nordic Semiconductor ASA
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

/* 
 * Code written by Grayson Mynott for ECE, University of Canterbury, 2020.
 * 
 * Based off Nordic SDK 17.0, eeprom_simulator example program.
 *
 * Contains functions intended to be used for UV_Dosimeter application.
 *
 */

#ifndef EEPROM_MODULE_H__
#define EEPROM_MODULE_H__

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include "nrf_drv_twi.h"
#include "nrf_gpio.h"
#include "app_error.h"
#include "nrf.h"
#include "bsp.h"
#include "app_util_platform.h"
#include "app_timer.h"
#include "nrf_drv_clock.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


//#define EEPROM_SIZE                       0x1FFFF

// Maximum number of bytes writable to the EEPROM at once.
// Datasheet did not specify, so I arbitrarily chose 200. Example code had 255
#define EEPROM_SEQ_WRITE_MAX_BYTES        200
#define EEPROM_SEQ_READ_MAX_BYTES         200

#define MAX_ADDRESS                       0x1FFFF     // For 1Mb EEPROM
//#define MAX_ADDRESS                       0x3FFFF     // For 2Mb EEPROM

// EEPROM Chip Address. See datasheet.
#define EEPROM_ADDR                       0b1010000

// EEPROM addressing byte length
#define EEPROM_ADDRESS_LEN_BYTES    2

// I2C Configuration
#define MASTER_TWI_INST     0       // TWI interface used.
#define SCL_PIN             20      // I2C SCL pin.
#define SDA_PIN             2       // I2C SDA pin.


// Format for I2C requests.
// Addr: 1010[WW][R/W] [WWWWWWWW] [WWWWWWWW] [DDDDDDDD]


// Initiate a EEPROM write sequence.
ret_code_t eeprom_write(uint8_t w_0, uint8_t w_1, uint8_t w_2, uint8_t const * pdata, size_t size);

// Initiate a EEPROM read sequence.
ret_code_t eeprom_read(uint8_t w_0, uint8_t w_1, uint8_t w_2, uint8_t * pdata, size_t size);

// Initialise the TWI/I2C module.
ret_code_t twi_master_init(void);

// Initialise the circular buffer. (Just resets the write address).
ret_code_t init_circ_buf(void);

// Write a value to the circular buffer. 
ret_code_t eeprom_write_circ_buf(uint8_t * value);

// Reads a value from the SED address in EEPROM.
ret_code_t eeprom_read_dosage(uint8_t * value);

// Writes a value to the SED address in EEPROM.
ret_code_t eeprom_write_dosage(uint8_t * value);

// Pings EEPROM until it acknowledges.
void eeprom_wait(void);

#endif // EEPROM_MODULE_H__
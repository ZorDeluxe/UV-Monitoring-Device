/**
 * Copyright (c) 2014 - 2020, Nordic Semiconductor ASA
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
  * Based off Nordic SDK 17.0, board definition pca10040.h.
  *
  * Contains definitions intended to be used for UV_Dosimeter application.
  *
 */


#ifndef TAIYO_H
#define TAIYO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nrf_gpio.h"

// Board definitions for UV Dosimeter
#define LEDS_NUMBER    0                                        // Number of LEDs

#define BUTTONS_NUMBER 1                                        // Number of buttons

#define BUTTON_1       9                                        // Button pin number

#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP                      // Button idle state (Pull-Up/Pull-Down)

#define BUTTONS_ACTIVE_STATE 0                                  // Button active state

#define BUTTONS_LIST { BUTTON_1 }                               // List of buttons. Position in list is how the button is referenced

#define BSP_BUTTON_0   BUTTON_1                                 // Button definition

#ifdef __cplusplus
}
#endif

#endif // TAIYO_H
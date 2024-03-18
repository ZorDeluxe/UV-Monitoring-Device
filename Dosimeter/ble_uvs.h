/**
 * Copyright (c) 2012 - 2020, Nordic Semiconductor ASA
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
 * Based off Nordic SDK 17.0, ble_hrs.h.
 *
 * Contains functions and definitions used to define a UV BLE service intended 
 * to be used for UV_Dosimeter application.
 *
 */

/** @file
 *
 * @defgroup ble_uvs UV Service
 * @{
 * @ingroup ble_sdk_srv
 * @brief UV Service module.
 *
 * @details This module implements the UV Service with the UV Index,
 *          UV Dosage, and UV Data Dump characteristics.
 *          During initialization it adds the UV Service, UV Index, and UV Dosage
 *          characteristic to the BLE stack database.
 *
 *          If enabled, notification of the UV Index characteristic is performed
 *          when the application calls ble_uvs_uv_index_update().
 *
 *          The UV Service also provides a set of functions for manipulating the
 *          various fields in the UV Measurement characteristic.
 *
 *          If an event handler is supplied by the application, the UV Service will
 *          generate UV Service events to the application.
 *
 * @note    The application must register this module as BLE event observer using the
 *          NRF_SDH_BLE_OBSERVER macro. Example:
 *          @code
 *              ble_uvs_t instance;
 *              NRF_SDH_BLE_OBSERVER(anything, BLE_UVS_BLE_OBSERVER_PRIO,
 *                                   ble_uvs_on_ble_evt, &instance);
 *          @endcode
 *
 * @note Attention!
 *  To maintain compliance with Nordic Semiconductor ASA Bluetooth profile
 *  qualification listings, this section of source code must not be modified.
 */

#ifndef BLE_UVS_H__
#define BLE_UVS_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"

#define BLE_UVS_BLE_OBSERVER_PRIO 2

#ifdef __cplusplus
extern "C" {
#endif

/**@brief   Macro for defining a ble_uvs instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_UVS_DEF(_name)                                                                          \
static ble_uvs_t _name;                                                                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_UVS_BLE_OBSERVER_PRIO,                                                     \
                     ble_uvs_on_ble_evt, &_name)


/**@brief UV Service event type. */
typedef enum
{
    BLE_UVS_EVT_NOTIFICATION_ENABLED,   /**< UV value notification enabled event. */
    BLE_UVS_EVT_NOTIFICATION_DISABLED   /**< UV value notification disabled event. */
} ble_uvs_evt_type_t;

/**@brief UV Service event. */
typedef struct
{
    ble_uvs_evt_type_t evt_type;    /**< Type of event. */
} ble_uvs_evt_t;

// Forward declaration of the ble_uvs_t type.
typedef struct ble_uvs_s ble_uvs_t;

/**@brief UV Service event handler type. */
typedef void (*ble_uvs_evt_handler_t) (ble_uvs_t * p_uvs, ble_uvs_evt_t * p_evt);

/**@brief UV Service init structure. This contains all options and data needed for
 *        initialization of the service. */
typedef struct
{
    ble_uvs_evt_handler_t        evt_handler;                                          /**< Event handler to be called for handling events in the UV Service. */
    security_req_t               uvi_cccd_wr_sec;                                      /**< Security requirement for writing the UVI characteristic CCCD. */
    security_req_t               uvd_cccd_wr_sec;                                      /**< Security requirement for writing the UVD characteristic value. */
    security_req_t               uvtx_cccd_wr_sec;                                     /**< Security requirement for writing the UVTX characteristic value. */
    security_req_t               uvi_rd_sec;
    security_req_t               uvd_rd_sec;
    security_req_t               uvtx_rd_sec;
} ble_uvs_init_t;

/**@brief UV Service structure. This contains various status information for the service. */
struct ble_uvs_s
{
    ble_uvs_evt_handler_t        evt_handler;                                          /**< Event handler to be called for handling events in the UV Service. */
    uint16_t                     service_handle;                                       /**< Handle of UV Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t     uvi_handles;                                          /**< Handles related to the UV Index characteristic. */
    ble_gatts_char_handles_t     uvd_handles;                                          /**< Handles related to the UV Dosage characteristic. */
    ble_gatts_char_handles_t     uvtx_handles;                                         /**< Handles related to the UV Data Dump characteristic. */
    uint16_t                     conn_handle;                                          /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint8_t                      max_uvi_len;                                          /**< Current maximum UV data length, adjusted according to the current ATT MTU. */
    uint8_t                      prev_index;                                           /**< Previous UV index value. */
    uint8_t                      prev_dosage;                                          /**< Previous UV dosage value. */
};


/**@brief Function for initializing the UV Service.
 *
 * @param[out]  p_uvs       UV Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_uvs_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_uvs_init(ble_uvs_t * p_uvs, ble_uvs_init_t const * p_uvs_init);


/**@brief Function for handling the GATT module's events.
 *
 * @details Handles all events from the GATT module of interest to the UV Service.
 *
 * @param[in]   p_uvs      UV Service structure.
 * @param[in]   p_gatt_evt  Event received from the GATT module.
 */
void ble_uvs_on_gatt_evt(ble_uvs_t * p_uvs, nrf_ble_gatt_evt_t const * p_gatt_evt);


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the UV Service.
 *
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 * @param[in]   p_context   UV Service structure.
 */
void ble_uvs_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


/**@brief Function for sending UV measurement if notification has been enabled.
 *
 * @details The application calls this function after having performed a UV measurement.
 *          If notification has been enabled, the UV measurement data is encoded and sent to
 *          the client.
 *
 * @param[in]   p_uvs                       UV Service structure.
 * @param[in]   uv                          New UV measurement.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_uvs_uv_index_update(ble_uvs_t * p_uvs, uint16_t uv);


/**@brief Function for sending UV dosage if notification has been enabled.
 *
 * @details The application calls this function after having performed a UV measurement.
 *          If notification has been enabled, the UV dosage data is encoded and sent to
 *          the client.
 *
 * @param[in]   p_uvs                       UV Service structure.
 * @param[in]   dosage                      New UV measurement.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_uvs_uv_dosage_update(ble_uvs_t * p_uvs, uint32_t dosage);


/**@brief Function for sending UV index data dump if notification has been enabled.
 *
 * @details The application calls this function after the button is held down.
 *          If notification has been enabled, the UV data is encoded and sent to
 *          the client.
 *
 * @param[in]   p_uvs                       UV Service structure.
 * @param[in]   value                       UV index measurement.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_uvs_uv_data_dump(ble_uvs_t *p_uvs, uint8_t value);


#ifdef __cplusplus
}
#endif

#endif // BLE_UVS_H__

/** @} */

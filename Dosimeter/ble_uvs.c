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
  * Based off Nordic SDK 17.0, ble_hrs.c.
  *
  * Contains functions and definitions used to define a UV BLE service intended 
  * to be used for UV_Dosimeter application.
  *
  */

#include "sdk_common.h"
#if NRF_MODULE_ENABLED(BLE_UVS)
#include "ble_uvs.h"
#include <string.h>
#include "ble_srv_common.h"


#define OPCODE_LENGTH 1                                                                     /**< Length of opcode inside packet. */
#define HANDLE_LENGTH 2                                                                     /**< Length of handle inside packet. */
#define MAX_UVS_LEN      (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH)    /**< Maximum size of a transmitted UV Measurement. */

#define INITIAL_VALUE_UVI                       0                                           /**< Initial UV Index value. */
#define INITIAL_VALUE_UVD                       0                                           /**< Initial UV Dosage value. */
#define INVALID_INDEX                           255
#define INVALID_DOSAGE                          255

// UV Measurement flag bits
#define UVI_FLAG_MASK_UV_VALUE_16BIT            (0x01 << 0)                                 /**< UV Value Format bit. */


/**@brief Function for handling write events to the UV Measurement characteristic.
 *
 * @param[in]   p_uvs         UV Service structure.
 * @param[in]   p_evt_write   Write event received from the BLE stack.
 */
static void on_uv_cccd_write(ble_uvs_t * p_uvs, ble_gatts_evt_write_t const * p_evt_write)
{
    if (p_evt_write->len == 2)
    {
        // CCCD written, update notification state
        if (p_uvs->evt_handler != NULL)
        {
            ble_uvs_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_UVS_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_UVS_EVT_NOTIFICATION_DISABLED;
            }

            p_uvs->evt_handler(p_uvs, &evt);
        }
    }
}


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_uvs       UV Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_uvs_t * p_uvs, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (p_evt_write->handle == p_uvs->uvi_handles.cccd_handle)
    {
        on_uv_cccd_write(p_uvs, p_evt_write);
    }
}



/****************************************************
 * Not sure what this is for.
 ***************************************************/
void ble_uvs_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_uvs_t * p_uvs = (ble_uvs_t *) p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            p_uvs->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            p_uvs->conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_uvs, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for encoding a UV Measurement.
 *
 * @param[in]   p_uvs              UV Service structure.
 * @param[in]   index              Measurement to be encoded.
 * @param[out]  p_encoded_buffer   Buffer where the encoded data will be written.
 *
 * @return      Size of encoded data.
 */
static uint8_t uvi_encode(ble_uvs_t * p_uvs, uint16_t index, uint8_t * p_encoded_buffer)
{
    uint8_t flags = 0;
    uint8_t len   = 1;
    int     i;

    // Encode UV measurement
    if (index > 0xff)// If index is more than 1 byte
    {
        flags |= UVI_FLAG_MASK_UV_VALUE_16BIT; // Set flag that indicates two bytes
        len   += uint16_encode(index, &p_encoded_buffer[len]);
    }
    else
    {
        p_encoded_buffer[len++] = (uint8_t)index;
    }

    // Add flags
    p_encoded_buffer[0] = flags;

    return len;
}

/**@brief Function for encoding a UV Measurement.
 *
 * @param[in]   p_uvs              UV Service structure.
 * @param[in]   uv         Measurement to be encoded.
 * @param[out]  p_encoded_buffer   Buffer where the encoded data will be written.
 *
 * @return      Size of encoded data.
 */
static uint8_t uvd_encode(ble_uvs_t * p_uvs, 
                          uint32_t    dosage, 
                          uint8_t   * p_encoded_buffer)
{
    uint8_t flags = 0;
    uint8_t len   = 1;
    int     i;

    // Encode UV measurement
    if (dosage > 0xff)
    {
        flags |= UVI_FLAG_MASK_UV_VALUE_16BIT;
        len   += uint32_encode(dosage, &p_encoded_buffer[len]);
    }
    else
    {
        p_encoded_buffer[len++] = (uint8_t)dosage;
    }

    // Add flags
    p_encoded_buffer[0] = flags;

    return len;
}


/****************************************************
 * Sets up the UV Index Characteristic.
 ***************************************************/
uint32_t add_uvi_char(ble_uvs_t * p_uvs, const ble_uvs_init_t * p_uvs_init)
{
    uint32_t              err_code;
    ble_uuid_t            ble_uuid;
    ble_add_char_params_t add_char_params;
    uint8_t               encoded_initial_uvi[MAX_UVS_LEN];

    // Add UV Index characteristic
    memset(&add_char_params, 0, sizeof(add_char_params));

    add_char_params.uuid              = BLE_UUID_UV_INDEX_CHAR;
    add_char_params.max_len           = MAX_UVS_LEN;
    add_char_params.init_len          = uvi_encode(p_uvs, INITIAL_VALUE_UVI, encoded_initial_uvi);
    add_char_params.p_init_value      = encoded_initial_uvi;
    add_char_params.is_var_len        = true;
    add_char_params.char_props.notify = 1;
    add_char_params.char_props.read   = 1;
    add_char_params.cccd_write_access = p_uvs_init->uvi_cccd_wr_sec;
    add_char_params.read_access       = p_uvs_init->uvi_rd_sec;

    err_code = characteristic_add(p_uvs->service_handle, &add_char_params, &(p_uvs->uvi_handles));
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

/****************************************************
 * Sets up the SED Characteristic.
 ***************************************************/
uint32_t add_uvd_char(ble_uvs_t * p_uvs, const ble_uvs_init_t * p_uvs_init)
{
    uint32_t              err_code;
    ble_uuid_t            ble_uuid;
    ble_add_char_params_t add_char_params;
    uint8_t               encoded_initial_uvd[MAX_UVS_LEN];

    // Add UV Dosage characteristic
    memset(&add_char_params, 0, sizeof(add_char_params));

    add_char_params.uuid              = BLE_UUID_HEART_RATE_MEASUREMENT_CHAR;
    add_char_params.max_len           = MAX_UVS_LEN;
    add_char_params.init_len          = uvi_encode(p_uvs, INITIAL_VALUE_UVD, encoded_initial_uvd);
    add_char_params.p_init_value      = encoded_initial_uvd;
    add_char_params.is_var_len        = true;
    add_char_params.char_props.notify = 1;
    add_char_params.char_props.read   = 1;
    add_char_params.cccd_write_access = p_uvs_init->uvd_cccd_wr_sec;
    add_char_params.read_access       = p_uvs_init->uvd_rd_sec;

    err_code = characteristic_add(p_uvs->service_handle, &add_char_params, &(p_uvs->uvd_handles));
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}


/****************************************************
 * Sets up the EEPROM Download Characteristic.
 ***************************************************/
uint32_t add_uvtx_char(ble_uvs_t * p_uvs, const ble_uvs_init_t * p_uvs_init)
{
    uint32_t              err_code;
    ble_uuid_t            ble_uuid;
    ble_add_char_params_t add_char_params;
    uint8_t               encoded_initial_uvtx[MAX_UVS_LEN];

    // Add UV Dosage characteristic
    memset(&add_char_params, 0, sizeof(add_char_params));

    add_char_params.uuid              = BLE_UUID_GLUCOSE_MEASUREMENT_CHAR;
    add_char_params.max_len           = MAX_UVS_LEN;
    add_char_params.init_len          = uvi_encode(p_uvs, INITIAL_VALUE_UVI, encoded_initial_uvtx);
    add_char_params.p_init_value      = encoded_initial_uvtx;
    add_char_params.is_var_len        = true;
    add_char_params.char_props.notify = 1;
    add_char_params.char_props.read   = 1;
    add_char_params.cccd_write_access = p_uvs_init->uvtx_cccd_wr_sec;
    add_char_params.read_access       = p_uvs_init->uvtx_rd_sec;

    err_code = characteristic_add(p_uvs->service_handle, &add_char_params, &(p_uvs->uvtx_handles));
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}


/****************************************************
 * Initialise the UV Service.
 * Adds characteristics.
 ***************************************************/
uint32_t ble_uvs_init(ble_uvs_t * p_uvs, const ble_uvs_init_t * p_uvs_init)
{
    uint32_t              err_code;
    ble_uuid_t            ble_uuid;

    // Initialize service structure
    p_uvs->evt_handler                 = p_uvs_init->evt_handler;
    p_uvs->conn_handle                 = BLE_CONN_HANDLE_INVALID;
    p_uvs->max_uvi_len                 = MAX_UVS_LEN;
    p_uvs->prev_index                  = INVALID_INDEX;
    p_uvs->prev_dosage                 = INVALID_DOSAGE;

    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_ENVIRONMENTAL_SERVICE);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_uvs->service_handle);

    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    err_code = add_uvi_char(p_uvs, p_uvs_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = add_uvd_char(p_uvs, p_uvs_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = add_uvtx_char(p_uvs, p_uvs_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

/****************************************************
 * Function that updates the UV Index Characteristic
 ***************************************************/
uint32_t ble_uvs_uv_index_update(ble_uvs_t *p_uvs,uint16_t index)
{
    uint32_t err_code;

    // Send value if connected and notifying
    if (p_uvs->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint8_t                encoded_uvi[MAX_UVS_LEN];
        uint16_t               len;
        uint16_t               hvx_len;
        ble_gatts_hvx_params_t hvx_params;

        // Update only if there is a change in value
        if (index != p_uvs->prev_index)
        {
          len     = uvi_encode(p_uvs, index, encoded_uvi);
          hvx_len = len;

          memset(&hvx_params, 0, sizeof(hvx_params));

          hvx_params.handle = p_uvs->uvi_handles.value_handle;
          hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
          hvx_params.offset = 0;
          hvx_params.p_len  = &hvx_len;
          hvx_params.p_data = encoded_uvi;

          err_code = sd_ble_gatts_hvx(p_uvs->conn_handle, &hvx_params);
          if ((err_code == NRF_SUCCESS) && (hvx_len != len))
          {
              err_code = NRF_ERROR_DATA_SIZE;
          }

          p_uvs->prev_index = index;
        }
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}


/****************************************************
 * Function that updates the SED characteristic
 ***************************************************/
uint32_t ble_uvs_uv_dosage_update(ble_uvs_t *p_uvs, uint32_t dosage)
{
    uint32_t err_code;

    // Send value if connected and notifying
    if (p_uvs->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint8_t                encoded_uvi[MAX_UVS_LEN];
        uint16_t               len;
        uint16_t               hvx_len;
        ble_gatts_hvx_params_t hvx_params;

        // Update only if there is a change in value
        if (dosage != p_uvs->prev_dosage)
        {
          len     = uvi_encode(p_uvs, dosage, encoded_uvi);
          hvx_len = len;

          memset(&hvx_params, 0, sizeof(hvx_params));

          hvx_params.handle = p_uvs->uvd_handles.value_handle;
          hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
          hvx_params.offset = 0;
          hvx_params.p_len  = &hvx_len;
          hvx_params.p_data = encoded_uvi;

          err_code = sd_ble_gatts_hvx(p_uvs->conn_handle, &hvx_params);
          if ((err_code == NRF_SUCCESS) && (hvx_len != len))
          {
              err_code = NRF_ERROR_DATA_SIZE;
          }
          p_uvs->prev_dosage = dosage;
        }
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}


/****************************************************
 * Function that performs the EEPROM download 
 * process.
 ***************************************************/
uint32_t ble_uvs_uv_data_dump(ble_uvs_t *p_uvs, uint8_t value)
{
    uint32_t err_code;

    // Send value if connected and notifying
    if (p_uvs->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint8_t                encoded_uvi[MAX_UVS_LEN];
        uint16_t               len;
        uint16_t               hvx_len;
        ble_gatts_hvx_params_t hvx_params;
        
        // Encode value
        len     = uvi_encode(p_uvs, value, encoded_uvi);
        hvx_len = len;

        // Set object parameters
        memset(&hvx_params, 0, sizeof(hvx_params));
        hvx_params.handle = p_uvs->uvtx_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = encoded_uvi;
        
        // Send object
        err_code = sd_ble_gatts_hvx(p_uvs->conn_handle, &hvx_params);
        if ((err_code == NRF_SUCCESS) && (hvx_len != len))
        {
            err_code = NRF_ERROR_DATA_SIZE;
        }
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}



/****************************************************
 * I am not sure what this is for.
 ***************************************************/
void ble_uvs_on_gatt_evt(ble_uvs_t * p_uvs, nrf_ble_gatt_evt_t const * p_gatt_evt)
{
    if (    (p_uvs->conn_handle == p_gatt_evt->conn_handle)
        &&  (p_gatt_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        p_uvs->max_uvi_len = p_gatt_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
    }
}
#endif // NRF_MODULE_ENABLED(BLE_UVS)

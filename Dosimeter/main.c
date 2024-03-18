/* 
  * Code written by Grayson Mynott for ECE, University of Canterbury, 2020.
  * 
  * Contains functions intended to be used for UV_Dosimeter application.
  *
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_sdm.h"
#include "app_error.h"                          // Application Error Module               (Used to check if functions perform successfully or failed)
#include "app_timer.h"                          // Application Timer Module               (Used for UV sampling timer)
#include "app_util_platform.h"
#include "ble.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_state.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_bas.h"                            // Battery Service Module
#include "ble_dis.h"                            // Device Information Service Module
#include "ble_conn_params.h"  
#include "boards.h"
#include "nrf_ble_gatt.h"                       // Generic Attribute Module               (Used for BT Services and Characteristics)
#include "nrf_ble_lesc.h"
#include "nrf_ble_qwr.h"                        // Queued Write Module
#include "nrf_delay.h"                          // Delay module
#include "nrf_drv_timer.h"                      // Timer module
#include "nrf_sdh.h"                            // SoftDevice Handler
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "bsp_btn_ble.h"                        // Board Support Package                  (Used to define buttons/LEDs)
#include "peer_manager.h"                       // Peer Management                        (Used to manage BLE pairing/bonds)
#include "peer_manager_handler.h"
#include "fds.h"                                // Flash Data Storage                     (Used to manage the on-chip flash memory)

#include "nrf_pwr_mgmt.h"                       // Used for low-power mode

// Logging modules
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "ble_uvs.h"                            // UV Service Module
#include "saadc_module.h"                       // SAADC Module
#include "eeprom_module.h"                      // EEPROM module


// ==============================================================================================================================================================
// ============================================================================================================================================ DEFINITIONS START
// ==============================================================================================================================================================
#define DEVICE_NAME                         "UV Dosimeter"                          // Name of device. Will be included in the advertising data.
#define MANUFACTURER_NAME                   "University of Canterbury"              // Manufacturer. Will be passed to Device Information Service.
#define APP_ADV_INTERVAL_F                  300                                     // The advertising interval (187.5 milliseconds) in units of 0.625 ms.
#define APP_ADV_DURATION_F                  18000                                   // The advertising duration (180 seconds) in units of 10 milliseconds.
#define APP_ADV_INTERVAL_S                  800                                     // The advertising interval (500 milliseconds)in units of 0.625 ms.
#define APP_ADV_DURATION_S                  60000                                   // The advertising duration (6000 seconds) in units of 10 milliseconds.

#define APP_BLE_CONN_CFG_TAG                1                                       // A tag identifying the SoftDevice BLE configuration.
#define APP_BLE_OBSERVER_PRIO               3                                       // Application's BLE observer priority. You shouldn't need to modify this value.

#define MIN_CONN_INTERVAL                   MSEC_TO_UNITS(20, UNIT_1_25_MS)         // Minimum acceptable connection interval (0.02 seconds).
#define MAX_CONN_INTERVAL                   MSEC_TO_UNITS(75, UNIT_1_25_MS)         // Maximum acceptable connection interval (0.075 second).
#define SLAVE_LATENCY                       0                                       // Slave latency. */
#define CONN_SUP_TIMEOUT                    MSEC_TO_UNITS(4000, UNIT_10_MS)         // Connection supervisory timeout (4 seconds).

#define FIRST_CONN_PARAMS_UPDATE_DELAY      APP_TIMER_TICKS(5000)                   // Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds).
#define NEXT_CONN_PARAMS_UPDATE_DELAY       APP_TIMER_TICKS(30000)                  // Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds).
#define MAX_CONN_PARAMS_UPDATE_COUNT        3                                       // Number of attempts before giving up the connection parameter negotiation.

#define LESC_DEBUG_MODE                     0                                       // Set to 1 to use LESC debug keys, allows you to use a sniffer to inspect traffic.

#define SEC_PARAM_BOND                      1                                       // Perform bonding.
#define SEC_PARAM_MITM                      0                                       // Man In The Middle protection not required.
#define SEC_PARAM_LESC                      1                                       // LE Secure Connections enabled.
#define SEC_PARAM_KEYPRESS                  0                                       // Keypress notifications not enabled.
#define SEC_PARAM_IO_CAPABILITIES           BLE_GAP_IO_CAPS_NONE                    // No I/O capabilities.
#define SEC_PARAM_OOB                       0                                       // Out Of Band data not available.
#define SEC_PARAM_MIN_KEY_SIZE              7                                       // Minimum encryption key size.
#define SEC_PARAM_MAX_KEY_SIZE              16                                      // Maximum encryption key size.

#define DEAD_BEEF                           0xDEADBEEF                              // Value used as error code on stack dump, can be used to identify stack location on stack unwind.

BLE_UVS_DEF(m_uvs);                                                                 // UV service instance.
BLE_BAS_DEF(m_bas);                                                                 // Battery service instance.
NRF_BLE_GATT_DEF(m_gatt);                                                           // GATT module instance.
NRF_BLE_QWR_DEF(m_qwr);                                                             // Context for the Queued Write module.
BLE_ADVERTISING_DEF(m_advertising);                                                 // Advertising module instance.


uint16_t m_conn_handle =                    BLE_CONN_HANDLE_INVALID;                // Handle of the current connection. (Initially invalid until a connection is established.)

// Set UUIDs
ble_uuid_t m_adv_uuids[] =                                                          // UUIDs for the services.
{
    {BLE_UUID_ENVIRONMENTAL_SERVICE,        BLE_UUID_TYPE_BLE},
    {BLE_UUID_BATTERY_SERVICE,              BLE_UUID_TYPE_BLE},
    {BLE_UUID_DEVICE_INFORMATION_SERVICE,   BLE_UUID_TYPE_BLE}
};

#define UV_SENSOR_MEAS_INTERVAL             APP_TIMER_TICKS(1000)                   // UV sensor measurement converted to ticks from milliseconds.                    -- Ideally this would be set through Bluetooth.
APP_TIMER_DEF(m_uv_sensor_timer_id);                                                // UV measurement timer.

uint8_t DD_COMPLETE = 1;                                                            // Used to indicate whether all data has been transferred to host machine.        -- Ideally replace this with something else. No one likes global variables
// ==============================================================================================================================================================
// ============================================================================================================================================== DEFINITIONS END
// ==============================================================================================================================================================



// ==============================================================================================================================================================
// =========================================================================================================================================== NRF CALLBACK START
// ==============================================================================================================================================================
 /****************************************************
 * Callback function for asserts in the SoftDevice.
 * Called in the case where an error occurs.
 * Essentially stalls program until reset occurs.
 ***************************************************/
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}
// ==============================================================================================================================================================
// ============================================================================================================================================= NRF CALLBACK END
// ==============================================================================================================================================================





// ==============================================================================================================================================================
// ============================================================================================================================================== BLUETOOTH START
// ==============================================================================================================================================================

/****************************************************
 * Handler for Queued Write Module Errors
 * Just passes the error through to Application
 * Error Handler.
 ***************************************************/
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


 /****************************************************
 * GAP (Generic Access Profile) Initialisation
 * Sets device parameters like device name, 
 * appearance, connection parameters.
 ***************************************************/
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,(const uint8_t *)DEVICE_NAME,strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_UNKNOWN);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


 /****************************************************
 * Initialises the BLE services.
 * That is Battery Service, 
 * Device Information Service,
 * UV Service,
 * and Queued Write Service
 *
 * Sets Security Level Parameters, event handlers, 
 * etc.
 ***************************************************/
static void services_init(void)
{
    ret_code_t         err_code;
    ble_uvs_init_t     uvs_init;
    ble_bas_init_t     bas_init;
    ble_dis_init_t     dis_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize UV Service.
    memset(&uvs_init, 0, sizeof(uvs_init));

    uvs_init.evt_handler          = NULL;
    uvs_init.uvi_cccd_wr_sec      = SEC_OPEN;
    uvs_init.uvd_cccd_wr_sec      = SEC_OPEN;
    uvs_init.uvtx_cccd_wr_sec     = SEC_OPEN;
    uvs_init.uvi_rd_sec           = SEC_OPEN;
    uvs_init.uvd_rd_sec           = SEC_OPEN;
    uvs_init.uvtx_rd_sec          = SEC_OPEN;

    err_code = ble_uvs_init(&m_uvs, &uvs_init);
    APP_ERROR_CHECK(err_code);


    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;
    bas_init.bl_rd_sec            = SEC_OPEN;
    bas_init.bl_cccd_wr_sec       = SEC_OPEN;
    bas_init.bl_report_rd_sec     = SEC_OPEN;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);


    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);
    dis_init.dis_char_rd_sec      = SEC_OPEN;

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
}

//------------------------------------------------------------------------------------------------------------------------- CONN_PARAMS START
/****************************************************
 * Connection parameter event handler.
 * Currently just disconnects on connection 
 * parameter failure.
 ***************************************************/
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/****************************************************
 * Function for handling connection parameter errors.
 ***************************************************/
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/****************************************************
 * Initialises connection parameters.
 * Not quite sure what this does.
 ***************************************************/
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = m_uvs.uvi_handles.cccd_handle;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}
//------------------------------------------------------------------------------------------------------------------------- CONN_PARAMS END
//------------------------------------------------------------------------------------------------------------------------- BLE_EVENTS START

/****************************************************
 * BLE Event Handler
 * Function is called whenever there is a change in
 * Bluetooth connection, GATT server status, etc.
 ***************************************************/
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected, reason %d.",
                          p_ble_evt->evt.gap_evt.params.disconnected.reason);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            NRF_LOG_DEBUG("BLE_GAP_EVT_SEC_PARAMS_REQUEST");
            break;

        case BLE_GAP_EVT_AUTH_KEY_REQUEST:
            NRF_LOG_INFO("BLE_GAP_EVT_AUTH_KEY_REQUEST");
            break;

        case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
            NRF_LOG_INFO("BLE_GAP_EVT_LESC_DHKEY_REQUEST");
            break;

         case BLE_GAP_EVT_AUTH_STATUS:
             NRF_LOG_INFO("BLE_GAP_EVT_AUTH_STATUS: status=0x%x bond=0x%x lv4: %d kdist_own:0x%x kdist_peer:0x%x",
                          p_ble_evt->evt.gap_evt.params.auth_status.auth_status,
                          p_ble_evt->evt.gap_evt.params.auth_status.bonded,
                          p_ble_evt->evt.gap_evt.params.auth_status.sm1_levels.lv4,
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_own),
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_peer));
            break;

        default:
            // No implementation needed.
            break;
    }
}
//------------------------------------------------------------------------------------------------------------------------- BLE_EVENTS END
//------------------------------------------------------------------------------------------------------------------------- BLE_INIT START
/****************************************************
 * Initialises the BLE stack/SoftDevice.
 * Not really sure what that is though.
 ***************************************************/
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();                          // Request to enable the SoftDevice
    APP_ERROR_CHECK(err_code);

    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG,  // Sets the SoftDevice configuration, gets the RAM start address
          &ram_start);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_sdh_ble_enable(&ram_start);                    // Enables the SoftDevice/BLE Stack
    APP_ERROR_CHECK(err_code);

    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO,   // Register a handler for BLE events
          ble_evt_handler, NULL);
}
//------------------------------------------------------------------------------------------------------------------------- BLE_INIT END



//------------------------------------------------------------------------------------------------------------------------- ADVERTISING START
/****************************************************
 * Handler for advertising events, such as mode 
 * switching.
 * Currently does nothing super important, just
 * debugging info.
 ***************************************************/
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
            break;

        case BLE_ADV_EVT_SLOW:
            NRF_LOG_INFO("Slow advertising.");
            break;
        
        case BLE_ADV_EVT_IDLE:
            NRF_LOG_INFO("Stop advertising.");
            break;

        default:
            break;
    }
}

/****************************************************
 * Advertising Initialisation.
 * This function sets advertising name, intervals, 
 * advertising modes (Fast/Slow), etc.
 ***************************************************/
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME; // Advertise the full device name
    init.advdata.include_appearance      = true;                  // Include device appearance while advertising
    init.advdata.flags                   =                        // Sets advertising flags
          BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids)    // Number of services
          / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;           // All UUIDs of the inlcuded services

    init.config.ble_adv_fast_enabled  = true;                     // Enable or disable fast advertising mode
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL_F;       // Advertising interval for fast advertising
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION_F;       // Time-out (in units of 10ms) for fast advertising

    init.config.ble_adv_slow_enabled  = true;                     // Enable or disable slow advertising mode
    init.config.ble_adv_slow_interval = APP_ADV_INTERVAL_S;       // Advertising interval for slow advertising
    init.config.ble_adv_slow_timeout  = APP_ADV_DURATION_S;       // Time-out (in units of 10ms) for slow advertising

    init.evt_handler = on_adv_evt;                                // Sets the Advertising Event handler

    err_code = ble_advertising_init(&m_advertising, &init);       // Initializes advertising with the defined configuration
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising,              // Sets a tag indicating the SoftDevice BLE configuration
          APP_BLE_CONN_CFG_TAG);
}

/****************************************************
 * Delete Existing Bonds.
 * Bonds are like previous connections.
 ***************************************************/
static void delete_bonds(void)
{
    ret_code_t err_code;

    err_code = pm_peers_delete();                                 // Delete existing Peer Manager bonds
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Erased bonds.");
}


/****************************************************
 * Start Bluetooth Advertising. Allows connections.
 ***************************************************/
static void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();                                           // Calls to function to delete PM bonds
    }
    else
    {
        ret_code_t err_code;

        err_code = ble_advertising_start(&m_advertising,         // Start Fast Advertising
              BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);
    }
}
//------------------------------------------------------------------------------------------------------------------------- ADVERTISING END



//------------------------------------------------------------------------------------------------------------------------- PEER MANAGER START
/****************************************************
 * Peer Manager Event Handler.
 ***************************************************/
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);                                  // Checks the Peer Manager Event
    pm_handler_flash_clean(p_evt);                                

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:                       // If Peer Manager Event was deleting bonds, start advertising
            advertising_start(false);
            break;

        default:
            break;
    }
}

/****************************************************
 * Peer Manager Initialisation
 ***************************************************/
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();                                         // Initialize the Peer Management module
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));          // Create a memory space for security parameters

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;                    // Perform bonding
    sec_param.mitm           = SEC_PARAM_MITM;                    // Enable Man-In-The-Middle protection
    sec_param.lesc           = SEC_PARAM_LESC;                    // Enable LE Secure Connection pairing
    sec_param.keypress       = SEC_PARAM_KEYPRESS;                // Enable generation of keypress notifications
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;         // IO capabilities
    sec_param.oob            = SEC_PARAM_OOB;                     // Out Of Band Data Flag
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;            // Minimum encryption key size in octets
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;            // Maximum encryption key size in octets
    sec_param.kdist_own.enc  = 1;                                 // Long Term Key and Master Identification (Own)
    sec_param.kdist_own.id   = 1;                                 // Identity Resolving Key and Identity Address Information (Own)
    sec_param.kdist_peer.enc = 1;                                 // Long Term Key and Master Identification (Peer)
    sec_param.kdist_peer.id  = 1;                                 // Identity Resolving Key and Identity Address Information (Peer)

    err_code = pm_sec_params_set(&sec_param);                     // Sets the defined Security Parameters
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);                       // Registers the Peer Manager event handler function
    APP_ERROR_CHECK(err_code);
}
//------------------------------------------------------------------------------------------------------------------------- PEER MANAGER END



//------------------------------------------------------------------------------------------------------------------------- GATT START
/****************************************************
 * GATT Event Handler
 ***************************************************/
static void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, 
      nrf_ble_gatt_evt_t const * p_evt)
{
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        NRF_LOG_INFO("GATT ATT MTU on connection 0x%x changed to %d.",
                     p_evt->conn_handle,
                     p_evt->params.att_mtu_effective);
    }

    ble_uvs_on_gatt_evt(&m_uvs, p_evt);                           // What is this for? Who knows? Something to do with setting the MTU length?
}


/****************************************************
 * Initialises the GATT module.
 ***************************************************/
static void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);      // Initializes GATT module, sets GATT event handler
    APP_ERROR_CHECK(err_code);
}
//------------------------------------------------------------------------------------------------------------------------- GATT END
// ==============================================================================================================================================================
// ================================================================================================================================================ BLUETOOTH END
// ==============================================================================================================================================================





// ==============================================================================================================================================================
// ========================================================================================================================================== BATTERY LEVEL START
// ==============================================================================================================================================================

/****************************************************
 * Converts ADC value [1-1024] into a percentage.
 * All dependent on battery voltage.
 ***************************************************/
 /*
 * Honestly I don't think I've got this right.
 * Perhaps the voltage divider I was using is wrong.
 
 * A voltage divider of: BATT -> 180k -> ADC -> 120k -> GND
 * should drop 4.2V (Li-Ion Max) to 1.68V.

 * A voltage divider of: BATT -> 130k -> ADC -> 160k -> GND
 * should drop 3V (Li Max) to 1.66V.
 ***************************************************/
static uint8_t convertToPercent(int32_t value)
{   // ADC_READING = [VOLTAGE] * GAIN/REFERENCE * 2^(RESOLUTION)

    // Percentage = (ADC Reading - Battery_Low_Reading) / (Battery_High_Reading - Battery_Low_Reading)
    double percentage = (value - 276) / (double) (416 - 276);
    
    NRF_LOG_INFO("Batt_LVL: " NRF_LOG_FLOAT_MARKER, 
          NRF_LOG_FLOAT(percentage));

    return (uint8_t) (percentage*100);
}

/****************************************************
 * Samples the Battery ADC channel, converts this 
 * value to percentage.
 * Bluetooth value is updated.
 ***************************************************/
static void battery_level_update(void)
{
    ret_code_t err_code;
    int32_t  batt_adc;
    uint8_t  battery_level;
    
    batt_adc  = sampleADC(BATT_ADC_CH);                           // Sample the Battery ADC Channel

    battery_level = (uint8_t)convertToPercent(batt_adc);          // Convert the ADC reading into a percentage

    NRF_LOG_INFO("Battery: %d%%", battery_level);

    
    // Update the battery level GATT Characteristic
    err_code = ble_bas_battery_level_update(&m_bas, 
          battery_level, BLE_CONN_HANDLE_ALL);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
}
// ==============================================================================================================================================================
// ============================================================================================================================================ BATTERY LEVEL END
// ==============================================================================================================================================================





// ==============================================================================================================================================================
// ========================================================================================================================================= UV MEASUREMENT START
// ==============================================================================================================================================================

/****************************************************
 * Function for converting the ADC value [0-1024]
 * into a UV Index value.
 * This formula is based off of the Adafruit 
 * recommendation, however needs calibration.
 ***************************************************/
static float convertToIndex(uint16_t value)
{
    float index;
    float voltage;

    // Set supply voltage. (V/4) is used as the ADC reference value.
    //float VDD = 3.3;
    //float VDD = 3;
    float VDD = 1.8;

    // ADC_READING = [VOLTAGE] * GAIN/REFERENCE * 2^(RESOLUTION)
    // Currently:
    // GAIN = 1/4
    // REFERENCE = VDD/4
    // RESOLUTION = 10-bit

    voltage = (value*VDD)/(float)(1024);
    NRF_LOG_INFO("Voltage: " NRF_LOG_FLOAT_MARKER, 
          NRF_LOG_FLOAT(voltage));

    //photoCurrent = voltage/4.3;
    // UV Index = voltage/0.1 -- From Adafruit
    index = 10*voltage;

    return index;
}

/****************************************************
 * Samples the UV ADC channel, converts this value 
 * to UV Index, and then recalculates the SED.
 * These values are then stored in EEPROM and 
 * Bluetooth values are updated.
 ***************************************************/
void uv_meas_update(void)
{
    ret_code_t      err_code;
    uint16_t        uv_reading;
    float           uv_index;
    uint8_t         uv_index_int;
    float           dosage = 0;
    int32_t         uv_adc;

    // Sample the UV SAADC channel and convert voltage to UV Index
    uv_adc    = sampleADC(UV_ADC_CH);
    if (uv_adc < 0){uv_adc = 0;}
    uv_index = convertToIndex(uv_adc);
    NRF_LOG_INFO("UV Index: " NRF_LOG_FLOAT_MARKER, 
          NRF_LOG_FLOAT(uv_index));
    uv_index_int = 10*uv_index;                                                   // Allows for 1 decimal precision in a single byte. [UV Index of 1.2 -> 12, Uv Index of 10.9 -> 109]
    
    // Write UV Index (*10) to EEPROM circular buffer
    err_code = eeprom_write_circ_buf((uint8_t *) &uv_index_int);
    APP_ERROR_CHECK(err_code);

    // Read current SED from EEPROM                                               // Could store SED as a 16-bit value. Divide by 4000 in app. Allows for an SED of up to 16
    err_code = eeprom_read_dosage((uint8_t *) &dosage);
    APP_ERROR_CHECK(err_code);
    dosage += ((float)uv_index/4000);
    NRF_LOG_INFO("SED: " NRF_LOG_FLOAT_MARKER, 
          NRF_LOG_FLOAT(dosage));
    NRF_LOG_INFO("Dosage: %d", dosage*4000);

    // Write updated SED back to EEPROM
    err_code = eeprom_write_dosage((uint8_t *) &dosage);
    APP_ERROR_CHECK(err_code);


    // Update UV Index Characteristic
    err_code = ble_uvs_uv_index_update(&m_uvs, uv_index_int);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }

    // Update SED Characteristic
    err_code = ble_uvs_uv_dosage_update(&m_uvs, dosage);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
    
    NRF_LOG_INFO("--------------------");
}


/****************************************************
 * This is the timeout handler for the UV sampling
 * timer. (m_uv_sensor_timer_id).
 * Makes a call to the UV sampling function.
 * On every second timeout, it also makes a call to 
 * the battery level update function.
 ***************************************************/
static void uv_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    static bool batt_update = true;
    if (batt_update){
      battery_level_update();                                     // Call the handler that updates the battery level characteristic (occurs once for every two timer calls)
    }
    batt_update = !batt_update;

    uv_meas_update();                                             // Call the handler that updates the UV characteristics
}



// --------------------------------------------------------------------------------------------------------------------------------------------------------------------- APPLICATION TIMER START

/****************************************************
 * Initialises the app timer module and starts the 
 * application timer.
 * The relevent timer (m_uv_sensor_timer_id) is 
 * defined at the top of this document.
 ***************************************************/
static void timers_init(void)
{
    ret_code_t err_code;

    // Initialize timer module.
    err_code = app_timer_init();                                  // Initialize application timers
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_uv_sensor_timer_id,            // Create timer to periodically initiate UV measurements
                                APP_TIMER_MODE_REPEATED,
                                uv_meas_timeout_handler);         
    APP_ERROR_CHECK(err_code);
}


/****************************************************
 * Starts the application timer used to trigger UV 
 * sampling events.
 * The relevent timer (m_uv_sensor_timer_id) is 
 * defined at the top of this document.
 ***************************************************/
static void application_timers_start(void)
{
    ret_code_t err_code;

    err_code = app_timer_start(m_uv_sensor_timer_id,              // Start timer that initiates UV measurement
          UV_SENSOR_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}
// --------------------------------------------------------------------------------------------------------------------------------------------------------------------- TIMER INITIALIZATION END


/****************************************************
 * This is the function used for the EEPROM download
 * required by the data-logger.
 * It simply iterates through the EEPROM, updating
 * the values over Bluetooth as it goes.
 ***************************************************/
static void uv_index_data_dump(void)
{
    ret_code_t  err_code;
    uint8_t     r_0;
    uint8_t     r_1;
    uint8_t     r_2;
    uint8_t     value = 0;
    static uint32_t    r_index = 0x14;              // Note: This should be changed to a non-static value in order to achieve correct circular buffer operation.
    uint32_t    s_index;


    // Pause the UV sampling timer.
    err_code = app_timer_stop(m_uv_sensor_timer_id);
    APP_ERROR_CHECK(err_code);

    // Read the current circular buffer write address. This indicates the latest UV value, and therefore where to stop the EEPROM read.
    while (eeprom_read(0, 0, 0, (uint8_t *) &s_index, 3) != NRF_SUCCESS)
    {
        eeprom_wait();
    }

    NRF_LOG_INFO("Addr: %d", s_index);
    while(r_index < s_index)                                                                    // While the read index is less than the stop index
    {
        r_0 = (((0b00000011 << 16) & r_index) >> 16);                                           // Convert the read index into 3 addr bytes
        r_1 = (((0b11111111 << 8) & r_index) >> 8);
        r_2 = ((0b11111111) & r_index);

        while (eeprom_read(r_0, r_1, r_2, (uint8_t *) &value, 1) != NRF_SUCCESS)                // Read the value at the read index
        {
            eeprom_wait();
        }

        err_code = ble_uvs_uv_data_dump(&m_uvs, value);                                         // Update BLE characteristic
        if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
        )
        {
            APP_ERROR_HANDLER(err_code);
        }
        if (err_code == NRF_SUCCESS)                                                            // If successfully updated, increase the read index
        {
          r_index += 1;
        }else if (err_code = NRF_ERROR_RESOURCES)                                               // If there is a resource error, save index and exit
        {
          return;
        }        
    }


    // Could potentially restart the circular buffer here.
    //err_code = init_circ_buf();
    //APP_ERROR_CHECK(err_code);

    // Data-dump is complete, restart UV sampling timer
    DD_COMPLETE = 1;
    r_index = 0x14;
    NRF_LOG_INFO("Finished")
    application_timers_start();


}
// ==============================================================================================================================================================
// =========================================================================================================================================== UV MEASUREMENT END
// ==============================================================================================================================================================




// ==============================================================================================================================================================
// ==================================================================================================================================================== BSP START
// ==============================================================================================================================================================
/****************************************************
 * Handler for BSP events.
 * On a button press, this function is called, where
 * 'event' relates to the button that was pressed.
 * Currently differentiates between short and long
 * presses of a singular button.
 ***************************************************/
void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_ADVERTISING_START:                         // Event Advertising Start
            //NRF_LOG_INFO("Short Press");

            if (m_advertising.adv_mode_current == BLE_ADV_MODE_IDLE)
            {
              advertising_start(true);                            // Start advertising
            }
            break;

        case 10:                                                  // Not sure why this is ever a case, but it occasionally occurs on normal presses
            //NRF_LOG_INFO("Short Press");

            if (m_advertising.adv_mode_current == BLE_ADV_MODE_IDLE)
            {
              advertising_start(true);                            // Start advertising
            }
            break;

        case BSP_EVENT_KEY_0:                                     // Event Key 0
            //NRF_LOG_INFO("Long Press");
            if (m_uvs.conn_handle != BLE_CONN_HANDLE_INVALID)
            {
              uv_index_data_dump();
              DD_COMPLETE = 0;
            }
            break;

        case 4:                                                   // Not sure why this is ever a case, but it occasionally occurs on long presses
            //NRF_LOG_INFO("Long Press");
            if (m_uvs.conn_handle != BLE_CONN_HANDLE_INVALID)
            {
              uv_index_data_dump();
              DD_COMPLETE = 0;
            }
            break;

        default:
            NRF_LOG_INFO("BUTTON_EVENT: %d", event);
            break;
    }
}


/****************************************************
 * Initialises the BSP module.
 * Assigns events to button press actions.
 * Currently sets a short and long press of the 
 * button.
 ***************************************************/
static void buttons_init(void)
{
    ret_code_t err_code;

    err_code = bsp_init(BSP_INIT_BUTTONS, bsp_event_handler);     // Initialize BSP Module
    APP_ERROR_CHECK(err_code);

    err_code = bsp_event_to_button_action_assign(0,               // Assign short press to BSP Event Advertising Start
          BSP_BUTTON_ACTION_PUSH, BSP_EVENT_ADVERTISING_START);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_event_to_button_action_assign(0,               // Assign long press (1000ms) to BSP Event Key 0
          BSP_BUTTON_ACTION_LONG_PUSH, BSP_EVENT_KEY_0);
    APP_ERROR_CHECK(err_code);
}

// ==============================================================================================================================================================
// ====================================================================================================================================================== BSP END
// ==============================================================================================================================================================





// ==============================================================================================================================================================
// ================================================================================================================================================ LOGGING START
// ==============================================================================================================================================================
/****************************************************
 * Initialises the logging module.
 * Events are logged to serial output when enabled
 * in config.h
 ***************************************************/
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);                     // Initialize NRF logging module
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();                              // Initialize logging backends
}
// ==============================================================================================================================================================
// ================================================================================================================================================== LOGGING END
// ==============================================================================================================================================================





// ==============================================================================================================================================================
// ======================================================================================================================================= POWER MANAGEMENT START
// ==============================================================================================================================================================
/****************************************************
 * Initialises the power management module.
 ***************************************************/
static void power_management_init(void)
{
    ret_code_t err_code = nrf_pwr_mgmt_init();                    // Initialize power management
    APP_ERROR_CHECK(err_code);
}


/****************************************************
 * Function for putting the chip into a 'System-Off'
 * sleep mode.
 * This mode is only able to be exited on a system 
 * reset. Not ideal.
 ***************************************************/
/*
void sleep_mode_enter(void)
{
    NRF_LOG_INFO("ENTERING SLEEP MODE.");
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}
*/
// ==============================================================================================================================================================
// ========================================================================================================================================= POWER MANAGEMENT END
// ==============================================================================================================================================================





// ==============================================================================================================================================================
// ============================================================================================================================================== IDLE HOOK START
// ==============================================================================================================================================================
/****************************************************
 * Idle state hook.
 * Runs whenever there is no other function running.
 * i.e. in between sampling periods and connection 
 * packets.
 ***************************************************/
static void idle_state_handle(void)
{
    ret_code_t err_code = nrf_ble_lesc_request_handler();         // I believe LESC is Bluetooth security. I am unsure why this function would be constantly running.
    APP_ERROR_CHECK(err_code);
    
    NRF_LOG_FLUSH();                                              // Send logs to UART


    // The data-dump can only send a certain number of packets in one loop without transmission errors.
    // Therefore, if a data-dump process is started and stalled, this statement will resume it.
    // Probably better ways to do this, but it works.
    if(!DD_COMPLETE && m_uvs.conn_handle != BLE_CONN_HANDLE_INVALID)
    {
      uv_index_data_dump();
    }

    // If log queue is empty, enter System ON sleep mode.
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}
// ==============================================================================================================================================================
// ================================================================================================================================================ IDLE HOOK END
// ==============================================================================================================================================================





// ==============================================================================================================================================================
// =================================================================================================================================================== MAIN START
// ==============================================================================================================================================================
/****************************************************
 * Main function.
 * Makes calls to all initialising functions.
 ***************************************************/
int main(void)
{
    bool erase_bonds = true;
    uint32_t err_code;

    log_init();                                                   // Initialize logging
    timers_init();                                                // Initialize application timers
    buttons_init();                                               // Initialize buttons module
    power_management_init();                                      // Initialize power management
    ble_stack_init();                                             // Initialize BLE stack
    gap_params_init();                                            // Initialize GAP parameters
    gatt_init();                                                  // Initialize GATT
    services_init();                                              // Initialize services
    advertising_init();                                           // Initialize advertising
    conn_params_init();                                           // Initialize connection parameters
    peer_manager_init();                                          // Initialize peer manager

    saadc_init();                                                 // Initialize SAADC module
    twi_master_init();                                            // Initialize TWI/I2C module


    // This block resets the UV index circular buffer write address and SED value.
    const uint16_t reset = 0;
    err_code = init_circ_buf();                                   // Reset EEPROM write index
    APP_ERROR_CHECK(err_code);
    err_code = eeprom_write_dosage((uint8_t * ) &reset);          // Reset SED
    APP_ERROR_CHECK(err_code);


    // Start execution.
    NRF_LOG_INFO("UV Sensor module started.");
    application_timers_start();                                   // Start application timers
    advertising_start(erase_bonds);                               // Start advertising

    while(1)
    {
        idle_state_handle();                                      // Idle state function
    }
}
// ==============================================================================================================================================================
// ===================================================================================================================================================== MAIN END
// ==============================================================================================================================================================

/* 
  * Code written by Grayson Mynott for ECE, University of Canterbury, 2020.
  * 
  * Based off Nordic SDK 17.0, saadc example program.
  *
  * Contains functions intended to be used for UV_Dosimeter application.
  *
 */


#include "saadc_module.h"

const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(1);  // SAADC Timer Instance
nrf_ppi_channel_t     m_ppi_channel;                        // PPI Channel, links ADC to timer


/****************************************************
 * Handler for ADC timer time-out.
 * Does nothing.
 ***************************************************/
void timer_handler(nrf_timer_event_t event_type, void * p_context)
{
}


/****************************************************
 * SAADC Sampling Event Initialisation.
 * To be honest I am not sure what most of this does.
 * It links the SAADC module to the SAADC timer,
 * however I am not sure why the timer is required
 * at all, as we are just triggering a sampling
 * event from the UV sampling function in 'main.c'.
 *
 * SDK SAADC Example did not explain anything.
 ***************************************************/
void saadc_sampling_event_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    err_code = nrf_drv_timer_init(&m_timer, &timer_cfg, timer_handler);
    APP_ERROR_CHECK(err_code);

    /* setup m_timer for compare event every 400ms */
    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer, 400);
    nrf_drv_timer_extended_compare(&m_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   ticks,
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   false);
    nrf_drv_timer_enable(&m_timer);

    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer,
                                                                                NRF_TIMER_CC_CHANNEL0);
    uint32_t saadc_sample_task_addr   = nrf_drv_saadc_sample_task_get();

    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel,
                                          timer_compare_event_addr,
                                          saadc_sample_task_addr);
    APP_ERROR_CHECK(err_code);
}


/****************************************************
 * Enable SAADC Samplin Event Timer
 ***************************************************/
void saadc_sampling_event_enable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);

    APP_ERROR_CHECK(err_code);
}


/****************************************************
 * SAADC callback function.
 * Not sure when this is called to ne honest.
 * Also not sure what would be put in here.
 ***************************************************/
void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
}


/****************************************************
 * Function for calibrating the ADC.
 * Should occur periodically.
 ***************************************************/
void calibrate_saadc(void)
{
    nrfx_err_t nrfx_err_code = NRFX_SUCCESS;

    // Stop ADC
    nrf_saadc_int_disable(NRF_SAADC_INT_ALL);
    NRFX_IRQ_DISABLE(SAADC_IRQn);
    nrf_saadc_task_trigger(NRF_SAADC_TASK_STOP);

    // Wait for ADC being stopped.
    bool result;
    NRFX_WAIT_FOR(nrf_saadc_event_check(NRF_SAADC_EVENT_STOPPED), HW_TIMEOUT, 0, result);
    NRFX_ASSERT(result);

    // Start calibration
    NRFX_IRQ_ENABLE(SAADC_IRQn);
    nrfx_err_code = nrfx_saadc_calibrate_offset();
    APP_ERROR_CHECK(nrfx_err_code);
    while(nrfx_saadc_is_busy()){};
    
    // Stop ADC
    nrf_saadc_int_disable(NRF_SAADC_INT_ALL);
    NRFX_IRQ_DISABLE(SAADC_IRQn);
    nrf_saadc_task_trigger(NRF_SAADC_TASK_STOP);

    // Wait for ADC being stopped. 
    NRFX_WAIT_FOR(nrf_saadc_event_check(NRF_SAADC_EVENT_STOPPED), HW_TIMEOUT, 0, result);
    NRFX_ASSERT(result);
    
    // Enable IRQ
    NRFX_IRQ_ENABLE(SAADC_IRQn);
}


/****************************************************
 * Function that samples the relevant ADC channel.
 * Channel is taken as input parameter, returns 
 * value in range [0-1024]
 ***************************************************/
int16_t sampleADC(int8_t channel)
{
    int16_t result;
    nrfx_err_t nrfx_err_code;
    nrf_saadc_int_disable(NRF_SAADC_INT_ALL);
    while(nrfx_saadc_is_busy()){};
    nrfx_err_code = nrf_drv_saadc_sample_convert(channel, &result);
    APP_ERROR_CHECK(nrfx_err_code);

    return result;
}


/****************************************************
 * Initialises the SAADC module.
 * Pins, Channels, ADC reference, etc.
 ***************************************************/
void saadc_init(void)
{
    ret_code_t err_code;
    // Initialize UV Sensor ADC Channel Config
    nrf_saadc_channel_config_t uv_ch_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(UV_ADC_IN);
    uv_ch_config.gain       = NRF_SAADC_GAIN1_4;
    uv_ch_config.reference  = NRF_SAADC_REFERENCE_VDD4;

    // Initialize Battery ADC Channel Config
    nrf_saadc_channel_config_t batt_ch_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(BATT_ADC_IN);
    batt_ch_config.gain       = NRF_SAADC_GAIN1_4;
    batt_ch_config.reference  = NRF_SAADC_REFERENCE_VDD4;


    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);
    calibrate_saadc();

    // Initialize UV Sensor ADC Channel
    err_code = nrf_drv_saadc_channel_init(UV_ADC_CH, &uv_ch_config);
    APP_ERROR_CHECK(err_code);

    // Initialize Battery ADC Channel
    err_code = nrf_drv_saadc_channel_init(BATT_ADC_CH, &batt_ch_config);
    APP_ERROR_CHECK(err_code);
}

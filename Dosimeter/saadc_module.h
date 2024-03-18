/* 
  * Code written by Grayson Mynott for ECE, University of Canterbury, 2020.
  * 
  * Based off Nordic SDK 17.0, saadc example program.
  *
  * Contains functions intended to be used for UV_Dosimeter application.
  *
 */


#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrf.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "app_error.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define SAMPLES_IN_BUFFER     1                                     // Number of samples in the ADC buffer. Usually this would be more and averaged out, but ADC confuses me so not this time.
#define HW_TIMEOUT            10000                                 // Number of times to attempt sampling ADC before raising an error.

#define UV_ADC_IN             NRF_SAADC_INPUT_AIN1                  // Define the input pin for UV sensor
#define BATT_ADC_IN           NRF_SAADC_INPUT_AIN2                  // Define the input pin for Battery Level
#define UV_ADC_CH             0                                     // Define the UV sensor ADC channel
#define BATT_ADC_CH           1                                     // Define the battery level ADC channel

extern const nrf_drv_timer_t m_timer;                               // SAADC Timer Instance
extern nrf_ppi_channel_t     m_ppi_channel;                         // PPI Channel, links ADC to timer


// SAADC Timer Handler
void timer_handler(nrf_timer_event_t event_type, void * p_context);

// SAADC Sampling Event Initilaisation
void saadc_sampling_event_init(void);

// Enable SAADC Sampling Event
void saadc_sampling_event_enable(void);

// SAADC Event Handler
void saadc_callback(nrf_drv_saadc_evt_t const * p_event);

// Calibrate SAADC
void calibrate_saadc(void);

// Sample given ADC Channel
int16_t sampleADC(int8_t channel);

// Initialise SAADC
void saadc_init(void);
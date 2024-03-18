/***********************************************************************
 * Outdoor Display with Wifi Capabilities
 * 
 * Author: Zoren Dela Cruz        Edited: 28/08/2020
 * Description: This module is the code used to transfer the sensor 
 *              readings to the website. The code also include a light
 *              sleep mode to reduce the power consumption of device.
 *              Light sleep mode is used to retain the RAM
***********************************************************************/
#include "DEV_Config.h"
#include "utility/EPD_12in48.h"
#include "GUI_Paint.h"
#include "imagedata.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <Arduino.h>
#include <driver/adc.h>
#include <stdlib.h>
void e_paper_display();

//***************************************************************************
// Global Variables
//***************************************************************************
int uv_rounded = 0;               // Constant Set-up
int previous_UV = 20;             // Set Previous UV to random value
int Calibration = 0;              // Display calibration
int count = 0;                    // Count to exit while loop

// Configure ADC Pin
const int ADC_Pin = 34;          

// Replace with your network credentials
const char* ssid     = "HUAWEI-4271";         // Mobile WiFi SSID and Password
const char* password = "7EJMGRQAMYR";

// REPLACE with your Domain name and URL path or IP address with path
const char* serverName = "http://132.181.51.108/?page_id=798";

// Keep this API Key value to be compatible with the PHP code provided in the project page.
// If you change the apiKeyValue value, the PHP file /post-data.php also needs to have the same key
String apiKeyValue = "tPmAT5Ab3j7F8";

// Setting up the light sleep mode for the ESP32
#define uS_TO_S_FACTOR 1000000      /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 90            /* Time ESP32 will go to sleep (in seconds) */

/* Entry point ----------------------------------------------------------------*/
void setup()
{
  // Serial port initialization
  Serial.begin(115200);

  delay(2000);

  // Program start-up
  if (Calibration <= 0)
  {
    // Start up of WaveShare Display
    delay(20000);
    printf("Starting...\r\n");
    Serial.print("...Up\r\n");

    // Setting up Analog UV Sensor
    analogReadResolution(12);         

    //SPI initialization
    delay(2000);
    DEV_ModuleInit();
    printf("Display Initialization...\r\n");
    DEV_TestLED();
    EPD_12in48_Init();
    EPD_12in48_Clear();

    // Put into development sleep for awhile
    DEV_Delay_ms(6000);
    //EPD_12in48_Clear();
    EPD_12in48_Sleep();
    printf("###Asleep for two minutes\r\n");
    DEV_Delay_ms(120000);

    // Set-up image on program
    printf("Finished \r\n");
    //Apply for a drawing cache
    UBYTE *Image = NULL;
    UWORD xsize = 1304, ysize = 984 / 2;//1304 x 492// Not enough memory to allocate 1304*984 of memory
    UDOUBLE Imagesize = ((xsize % 8 == 0) ? (xsize / 8 ) : (xsize / 8 + 1)) * ysize;
    if ((Image = (UBYTE *)malloc(Imagesize)) == NULL)
    {
      printf("Failed to apply for black memory...\r\n");
      while (1);
    }
    Calibration = 1;  // Calibration is completed
  }
}

/* The main loop -------------------------------------------------------------*/
void loop()
{
  //************************************************************
  // Analog UV Sensor readings
  //************************************************************
  delay(2000);
  int sensorValue = analogRead(ADC_Pin);
  float Calcs = sensorValue * 33;
  float UV_Value = Calcs / 4096;
  uv_rounded = abs(round(UV_Value));

  //************************************************************
  // Update the display if there is a change in UV Index reading
  // Display code (Martin Allen's code)
  //************************************************************
  if (previous_UV != uv_rounded)
  {
    // Clear out display before updating the new display
    Serial.print("###EPD_12in48 Init\r\n");
    EPD_12in48_Init();
    EPD_12in48_Clear();
    // 1.Apply for a drawing cache
    UBYTE *Image = NULL;
    UWORD xsize = 1304, ysize = 984 / 2;//1304 x 492// Not enough memory to allocate 1304*984 of memory
    UDOUBLE Imagesize = ((xsize % 8 == 0) ? (xsize / 8 ) : (xsize / 8 + 1)) * ysize;
    if ((Image = (UBYTE *)malloc(Imagesize)) == NULL)
    {
      printf("Failed to apply for black memory...\r\n");
      while (1);
    }
    printf("Paint New Image\r\n");
    Paint_NewImage(Image, 1304, 492, 0, WHITE);
    // 2.Drawing on the img
    printf("1.Drawing top half screen black\r\n");
    Paint_Clear(WHITE);

    // Draw the upper part of the E-ink Display (Martin Allen's code)
    if (uv_rounded == 0) Paint_DrawImage(zero_top, 18, 148, 1024, 344);
    if (uv_rounded == 1) Paint_DrawImage(one_top, 18, 368, 1024, 124);
    if (uv_rounded == 2) Paint_DrawImage(two_top, 18, 148, 1024, 344);
    if (uv_rounded == 3) Paint_DrawImage(three_top, 18, 148, 1024, 344);
    if (uv_rounded == 4) Paint_DrawImage(four_top, 18, 148, 1024, 344);
    if (uv_rounded == 5) Paint_DrawImage(five_top, 18, 148, 1024, 344);
    if (uv_rounded == 6) Paint_DrawImage(six_top, 18, 148, 1024, 344);
    if (uv_rounded == 7) Paint_DrawImage(seven_top, 18, 148, 1024, 344);
    if (uv_rounded == 8) Paint_DrawImage(eight_top, 18, 148, 1024, 344);
    if (uv_rounded == 9) Paint_DrawImage(nine_top, 18, 148, 1024, 344);
    if (uv_rounded == 10) Paint_DrawImage(ten_zero, 18, 44, 1024, 448);
    if (uv_rounded >= 11) Paint_DrawImage(one, 18, 122, 1024, 248);

    if (uv_rounded >= 10.5) Paint_DrawImage(extreme_top, 1048, 0, 256, 492);
    else if (uv_rounded >= 7.5) Paint_DrawImage(very_high_top, 1048, 0, 256, 492);
    else if (uv_rounded >= 5.5) Paint_DrawImage(high_top, 1048, 0, 256, 492);
    else if (uv_rounded >= 2.5) Paint_DrawImage(medium_top, 1048, 0, 256, 492);
    else if (uv_rounded < 2.5) Paint_DrawImage(low_top, 1048, 0, 256, 492);
    EPD_12in48_SendBlack1(Image);

    printf("2.Drawing bottom half screen black\r\n");
    Paint_Clear(WHITE);

    // Draw the lower part of the E-ink Display (Martin Allen's code)
    if (uv_rounded == 0) Paint_DrawImage(zero_bottom, 18, 0, 1024, 344);
    if (uv_rounded == 1) Paint_DrawImage(one_bottom, 18, 0, 1024, 124);
    if (uv_rounded == 2) Paint_DrawImage(two_bottom, 18, 0, 1024, 344);
    if (uv_rounded == 3) Paint_DrawImage(three_bottom, 18, 0, 1024, 344);
    if (uv_rounded == 4) Paint_DrawImage(four_bottom, 18, 0, 1024, 344);
    if (uv_rounded == 5) Paint_DrawImage(five_bottom, 18, 0, 1024, 344);
    if (uv_rounded == 6) Paint_DrawImage(six_bottom, 18, 0, 1024, 344);
    if (uv_rounded == 7) Paint_DrawImage(seven_bottom, 18, 0, 1024, 344);
    if (uv_rounded == 8) Paint_DrawImage(eight_bottom, 18, 0, 1024, 344);
    if (uv_rounded == 9) Paint_DrawImage(nine_bottom, 18, 0, 1024, 344);
    if (uv_rounded == 10) Paint_DrawImage(one, 18, 160, 1024, 248);
    if (uv_rounded >= 11) Paint_DrawImage(one, 18, 122, 1024, 248);

    if (uv_rounded >= 10.5) Paint_DrawImage(extreme_bottom, 1048, 0, 256, 492);
    else if (uv_rounded >= 7.5) Paint_DrawImage(very_high_bottom, 1048, 0, 256, 492);
    else if (uv_rounded >= 5.5) Paint_DrawImage(high_bottom, 1048, 0, 256, 492);
    else if (uv_rounded >= 2.5) Paint_DrawImage(medium_bottom, 1048, 0, 256, 492);
    else if (uv_rounded < 2.5) Paint_DrawImage(low_bottom, 1048, 0, 256, 492);
    EPD_12in48_SendBlack2(Image);

    // Turn off the display for awhile
    Serial.print("EPD_12in48_Display\r\n");
    EPD_12in48_TurnOnDisplay();
    DEV_Delay_ms(60000);

    //clear for Long-term preservation
    Serial.print("###EPD_12in48 Asleep\r\n");
    //EPD_12in48_Clear();
    EPD_12in48_Sleep();

    free(Image);
    Image = NULL;
    Serial.print("###Asleep Now\r\n");
  }

  // Update the previous UV reading
  previous_UV = uv_rounded;
  delay(2000);
  
  //************************************************************
  // Set-up the wifi settings. Code is designed that if no 
  // connection is found, it will exit the while loop
  //************************************************************
  WiFi.begin(ssid, password);
  Serial.println("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
    // Check if there is Wifi connection 
    if (count >= 100)
    {
      count = 0;
      break;    // Exit after 100 polls if WiFi fails to connect
    }
    count++;
  }
  
  delay(2000);
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
  delay(2000);

  //************************************************************
  // Send sensor readings to the website through an HTTP request
  //************************************************************
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;

    // Your Domain name with URL path or IP address with path
    http.begin(serverName);

    // Specify content-type header
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");

    // Prepare your HTTP POST request data
    String httpRequestData = "api_key=" + apiKeyValue + "&value1=" + String(readUVIntensity())
                             + "";
    Serial.print("httpRequestData: ");
    Serial.println(httpRequestData);


    // Send HTTP POST request
    int httpResponseCode = http.POST(httpRequestData);

    if (httpResponseCode > 0) {
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
    }
    else {
      Serial.print("Error code: ");
      Serial.println(httpResponseCode);
    }
    // Free resources
    http.end();
  }
  else {
    Serial.println("WiFi Disconnected");
  }

  //************************************************************
  // Put ESP32 into light sleep mode
  //************************************************************
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  esp_light_sleep_start();
}

//*********************************************************************************
// Function I use to send the sensor readings to the website. This function returns 
// a string which would be sent to the website
//*********************************************************************************
String readUVIntensity()
{
  int UV_Index1 = analogRead(ADC_Pin);
  float UV_Index2 = UV_Index1 * 33; 
  float UV_Index = UV_Index2 / 4096;
  // Check if any reads failed and exit early
  if (isnan(UV_Index) || (UV_Index < 0))
  {
    Serial.println("Failed to read from UV Sensor");
    return String(0.00);
  } else {
    Serial.println(UV_Index);
    return String(UV_Index);
  }
}

/***********************************************************************
 * Outdoor Display with Wifi Capabilities
 * 
 * Author: Zoren Dela Cruz and Mark Arunchayanon     Edited: 13/10/2020
 * Description: This module is the code used to display real-time UV
 *              intensity with a scrolling feature with the sharp LCD
 *              display. It also uses BLE to transmit UV dose, index and
 *              battery levels. 
***********************************************************************/
#include <bluefruit.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SharpMem.h>
#include <math.h>
#include "imagedata.h"

//***************************************************************************
// Global Variables
//***************************************************************************
// SPI Pins was used
#define SHARP_SCK  SCK
#define SHARP_MOSI MOSI
#define SHARP_SS   MISO

// Set the size of the display here, e.g. 144x168!
Adafruit_SharpMem display(SHARP_SCK, SHARP_MOSI, SHARP_SS, 144, 168);
// The currently-available SHARP Memory Display (144x168 pixels)
// requires > 4K of microcontroller RAM; it WILL NOT WORK on Arduino Uno
// or other <4K "classic" devices!  The original display (96x96 pixels)
// does work there, but is no longer produced.

#define BLACK 0
#define WHITE 1

// Sampling interval in seconds
#define samplingInt 4 

//***************************************************************************
// Bluetooth Low Energy Set-up using Environmental Sensing Services
//***************************************************************************
/* GATT Services https://www.bluetooth.com/specifications/gatt/services/
    Name: Environmental Sensing
    Uniform Type Identifier: org.bluetooth.service.environmental_sensing
    Assigned Number: 0x181A
    Specification: GSS
*/
#define UUID16_SVC_ENVIRONMENTAL_SENSING 0x181A

/* GATT Characteristics https://www.bluetooth.com/specifications/gatt/characteristics/
    Name: UV Index
    Uniform Type Identifier: org.bluetooth.characteristic.uv_index
    Assigned Number: 0x2A76
    Specification: GSS
*/
#define UUID16_CHR_UV_INDEX 0x2A76

/* GATT Characteristics https://www.bluetooth.com/specifications/gatt/characteristics/
    Name: Temp Sensing -> Used for UV dose
    Uniform Type Identifier: org.bluetooth.characteristic.uv_index
    Assigned Number: 0x2A1F
    Specification: GSS
*/#define UUID16_CHR_TEMP 0x2A1F

BLEService        ess = BLEService(UUID16_SVC_ENVIRONMENTAL_SENSING);
BLECharacteristic uvic = BLECharacteristic(UUID16_CHR_UV_INDEX);
BLECharacteristic uvtc = BLECharacteristic(UUID16_CHR_TEMP);

BLEDis bledis;    // DIS (Device Information Service) helper class instance
BLEBas blebas;

// UV index GATT Characteristic format is in uint8
uint8_t uvindexvalue = 0x42;
uint8_t uvindexrawvalue = 0x42;

// Sum of UV Index readings, Sum(UVI sensor reading * sampling interval)/4000
float uvSum = 0x42;
float uvDiv = 0x42;
uint8_t uvIntegral = 0x42;
uint8_t uvRemainder = 0x42;

// Analog Pin
int ADC_Pin = A0;
int VBat_Pin = A1;

// Display Set-up
int minorHalfSize; // 1/2 of lesser of display width or height
byte disp_height = 0;
byte disp_width = 0;
void Paint_DrawImage();

// Inside Main Loop
byte display_count = 0;
int uv_rounded = 0;
int old_uv_rounded = 22;

// Monitoring Battery Voltage
float Vbat = 0;
int VBatPercentage = 0;

// Advanced function prototypes
void startAdv(void);
void setupESService(void);
void connect_callback(uint16_t conn_handle);
void disconnect_callback(uint16_t conn_handle, uint8_t reason);

//***************************************************************************
// Set-up stage
//***************************************************************************
void setup() {
  Serial.begin(115200);
  //while (!Serial) delay(10);   // for nrf52840 with native usb

  Serial.println("Bluefruit52 UV Index sensor example");
  Serial.println("-------------------------------------\n");

  // Set the analog reference to 3.0V (default = 3.6V)
  analogReference(AR_INTERNAL_3_0);

  // Set the resolution to 10-bit (0..1023)
  analogReadResolution(10); // Can be 8, 10, 12 or 14

  // start & clear the display
  display.begin();
  display.clearDisplay();

  // Several shapes are drawn centered on the screen.  Calculate 1/2 of
  // lesser of display width or height, this is used repeatedly later.
  minorHalfSize = min(display.width(), display.height()) / 2;

  // Initialise the Bluefruit module
  Serial.println("Initialise the Bluefruit nRF52 module");
  Bluefruit.begin();
  Bluefruit.setName("SunSmart FYP");

  // Set the connect/disconnect callback handlers
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // Configure and Start the Device Information Service
  Serial.println("Configuring the Device Information Service");
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
  bledis.begin();;

  Serial.println("Configuring the Battery Service");
  blebas.begin();
  blebas.write(100);

  // Setup the Environmental Sensing service using
  // BLEService and BLECharacteristic classes
  Serial.println("Configuring the Environmental Sensing Service");
  setupESService();

  // Setup the advertising packet(s)
  Serial.println("Setting up the advertising payload(s)");
  startAdv();

  Serial.println("\nAdvertising");

  // Turn off blue LED
  digitalWrite(LED_BLUE, LOW);

}

void loop() {
  //  digitalToggle(LED_BLUE);
  digitalWrite(LED_BLUE, LOW);

  //***************************************************************************
  // UV Index calculation
  //***************************************************************************
  int sensorValue = analogRead(ADC_Pin);
  float voltage = sensorValue * (3.0 / 1023);
  uvindexvalue = round(abs(voltage / 0.1)); // convert float to uint8_t
  uvindexrawvalue = abs(voltage/0.1) * 10;  // Adds a 10 multiplier for Mark's app

  //***************************************************************************
  // UV Dose calculation
  //***************************************************************************
  uvSum = uvSum + ((uvindexrawvalue / 10) * samplingInt);  //Sum(UVI sensor reading * sampling interval)/40000
  uvDiv = (uvSum / 4000);
  float temp = trunc(uvDiv);
  uvIntegral = uint8_t(uvDiv);
  uvRemainder = uint8_t(uvDiv - temp);

  //***************************************************************************
  // Battery Monitoring from pin A1
  //***************************************************************************
  int sensorValue1 = analogRead(VBat_Pin);
  Vbat = sensorValue1 * (3.0 / 1023);
  float VbatConvert = Vbat - 1.8;
  float VbatPercentage = VbatConvert / 1.2;
  VBatPercentage = round(VbatPercentage * 100);

  //***************************************************************************
  // Serial Prints for Debugging
  //***************************************************************************
  Serial.println("");
  Serial.print("Sensor Value: ");
  Serial.println(sensorValue);

  Serial.println("");
  Serial.print("Raw UV Index (float): ");
  Serial.println(voltage);

  Serial.println("");
  Serial.print("Raw UV Index (uint8_t): ");
  Serial.println(uvindexvalue);

  Serial.println("");
  Serial.print("Battery Voltage (uint8_t): ");
  Serial.println(Vbat);

  Serial.println("");
  Serial.print("Battery Percentage (uint8_t): ");
  Serial.println(VBatPercentage);
  
  Serial.println("");
  Serial.print("UV Dose (uint8_t): ");
  Serial.println(uvIntegral);

  Serial.println("");
  Serial.print("UV Dos (float): ");
  Serial.println(uvDiv);

  Serial.println("");
  Serial.print("UV Sum (float): ");
  Serial.println(uvSum);

  //***************************************************************************
  // Displaying Slip, slap, slop and wrap
  //***************************************************************************
  if (display_count >= 5 || (old_uv_rounded != uvindexvalue))
  {
    display_count = 0;
    Scrolling_Message();
    Update_Display(uvindexvalue);
  }
  display_count++;                  // Update Display count
  old_uv_rounded = uvindexvalue;    // Display if there is change in UV Index


  //***************************************************************************
  // Bluetooth Transmission
  //***************************************************************************
  if (Bluefruit.connected()) {
    // Note: We use .indicate instead of .write!
    // If it is connected but CCCD is not enabled
    // The characteristic's value is still updated although indicate is not sent

    // Sends UV Index
    if (uvic.notify(&uvindexrawvalue, sizeof(uvindexrawvalue))) {
      //Serial.print("UV Index Measurement updated to: ");
      Serial.println(uvindexrawvalue);
    } else {
      Serial.println("ERROR: Indicate not set in the CCCD or not connected!");
    }

    // Sends UV Dose
    if (uvtc.notify(&uvIntegral, sizeof(uvIntegral))) {
      //Serial.print("UV Index Measurement updated to: ");
      Serial.print("UV Dose (uint8_t): ");
      Serial.println(uvIntegral);
    } else {
      Serial.println("ERROR: Indicate not set in the CCCD or not connected!");
    }

    // Sends Battery Level
    if (blebas.notify(VBatPercentage)) {
    Serial.println(VBatPercentage);
    } else {
      Serial.println("ERROR: Indicate not set in the CCCD or not connected!");
    }
  }
  delay(2000);
}

//***************************************************************************
// Advertising protocol used for BLE. This is where you select the device name
// and set-up advertising processes. 
//***************************************************************************
void startAdv(void) {
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include HTM Service UUID
  Bluefruit.Advertising.addService(ess);

  // Include Name
  Bluefruit.Advertising.addName();

  /* Start Advertising
     - Enable auto advertising if disconnected
     - Interval:  fast mode = 20 ms, slow mode = 1000 ms
     - Timeout for fast mode is 10 seconds
     - Start(timeout) with timeout = 0 will advertise forever (until connected)

     For recommended advertising interval
     https://developer.apple.com/library/content/qa/qa1931/_index.html
  */
  Bluefruit.Advertising.restartOnDisconnect(true);

  // in unit of 0.625 ms
  Bluefruit.Advertising.setInterval(32, 1600);

  // number of seconds in fast mode
  Bluefruit.Advertising.setFastTimeout(10);

  // 0 = Don't stop advertising after n seconds
  Bluefruit.Advertising.start(0);
}


//***************************************************************************
// Function used to setup the environmental sensing services. This is where
// UV index and UV dose (Temperature sensing) is configured by setting 
// its properties and callbacks. 
//***************************************************************************
void setupESService(void) {
  // Configure the Environmental Sensing service
  // See: https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Characteristics/org.bluetooth.characteristic.uv_index.xml
  // Supported Characteristics:
  // Name                         UUID    Requirement Properties
  // ---------------------------- ------  ----------- ----------
  // UV Index                     0x2A76  Mandatory   Read
  ess.begin();

  // Note: You must call .begin() on the BLEService before calling .begin() on
  // any characteristic(s) within that service definition.. Calling .begin() on
  // a BLECharacteristic will cause it to be added to the last BLEService that
  // was 'begin()'ed!

  // Configure the UV Index characteristic
  // See:https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Characteristics/org.bluetooth.characteristic.uv_index.xml
  // Properties = Indicate
  // Min Len    = 1
  // Max Len    = 1
  // B0         = UINT8 - UV Index measurement unitless
  uvic.setProperties(CHR_PROPS_NOTIFY);
  uvic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  uvic.setFixedLen(1);
  uvic.setCccdWriteCallback(cccd_callback);  // Optionally capture CCCD updates
  uvic.begin();
  uvic.write(&uvindexvalue, sizeof(uvindexvalue));

  uvtc.setProperties(CHR_PROPS_NOTIFY);
  uvtc.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  uvtc.setFixedLen(1);
  uvtc.setCccdWriteCallback(cccd_callback);  // Optionally capture CCCD updates
  uvtc.begin();
  uvtc.write(&uvIntegral, sizeof(uvIntegral));
}


//***************************************************************************
// Gives an acknowledgement that device has been connected
//***************************************************************************
void connect_callback(uint16_t conn_handle) {
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);
}


//***************************************************************************
//   Callback invoked when a connection is dropped
//   @param conn_handle connection where this event happens
//   @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
//***************************************************************************
void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  (void) conn_handle;
  (void) reason;

  Serial.println("Disconnected");
  Serial.println("Advertising!");
}


//***************************************************************************
//   Callback function
//***************************************************************************
void cccd_callback(uint16_t conn_hdl,
                   BLECharacteristic* chr, uint16_t cccd_value) {
  // Display the raw request packet
  Serial.print("CCCD Updated: ");

  // Serial.printBuffer(request->data, request->len);
  Serial.print(cccd_value);
  Serial.println("");

  // Check the characteristic this CCCD update is associated with in case
  // this handler is used for multiple CCCD records.
  if (chr->uuid == uvic.uuid) {
    if (chr->indicateEnabled(conn_hdl)) {
      Serial.println("UV Index Measurement 'Indicate' enabled");
    } else {
      Serial.println("UV Index Measurement 'Indicate' disabled");
    }
  }

  // Check the characteristic this CCCD update is associated with in case
  // this handler is used for multiple CCCD records.
  if (chr->uuid == uvtc.uuid) {
    if (chr->indicateEnabled(conn_hdl)) {
      Serial.println("UV Integral Measurement 'Indicate' enabled");
    } else {
      Serial.println("UV Integral Measurement 'Indicate' disabled");
    }
  }
}

//***************************************************************************
//   Draw image for the Sharp LCD Display
//***************************************************************************
void Paint_DrawImage(const unsigned char *image, int xStart, int yStart, int W_Image, int H_Image)
{
  int x, y, i;
  int WByte = (W_Image % 8 == 0) ? (W_Image / 8 ) : (W_Image / 8 + 1);
  int imagedata, ramdata;

  for (y = 0; y < H_Image; y++)
  {
    for (x = 0; x < W_Image; x++)
    {
      imagedata = image[x / 8 + y * WByte];
      ramdata = (imagedata << (x % 8)) & 0x80 ? 0 : 1;
      display.drawPixel(x + xStart, y + yStart, ramdata);
    }
  }
}

//***************************************************************************
//   Show the scrolling feature of the UV Index Value
//***************************************************************************
void Update_Display(int uv_rounded)
{
  int i;
  for (i = -168; i <= 0; i = i + 24)
  {
    if (uv_rounded == 0) Paint_DrawImage(zero_low, 0, i, 144, 168);
    if (uv_rounded == 1) Paint_DrawImage(one_low, 0, i, 144, 168);
    if (uv_rounded == 2) Paint_DrawImage(two_low, 0, i, 144, 168);
    if (uv_rounded == 3) Paint_DrawImage(three_medium, 0, i, 144, 168);
    if (uv_rounded == 4) Paint_DrawImage(four_medium, 0, i, 144, 168);
    if (uv_rounded == 5) Paint_DrawImage(five_medium, 0, i, 144, 168);
    if (uv_rounded == 6) Paint_DrawImage(six_high, 0, i, 144, 168);
    if (uv_rounded == 7) Paint_DrawImage(seven_high, 0, i, 144, 168);
    if (uv_rounded == 8) Paint_DrawImage(eight_very_high, 0, i, 144, 168);
    if (uv_rounded == 9) Paint_DrawImage(nine_very_high, 0, i, 144, 168);
    if (uv_rounded == 10) Paint_DrawImage(ten_very_high, 0, i, 144, 168);
    if (uv_rounded >= 11) Paint_DrawImage(eleven_extreme, 0, i, 144, 168);
    display.refresh();
    delay(100);
  }

  if (uv_rounded == 0) Paint_DrawImage(zero_low, 0, 0, 144, 168);
  if (uv_rounded == 1) Paint_DrawImage(one_low, 0, 0, 144, 168);
  if (uv_rounded == 2) Paint_DrawImage(two_low, 0, 0, 144, 168);
  if (uv_rounded == 3) Paint_DrawImage(three_medium, 0, 0, 144, 168);
  if (uv_rounded == 4) Paint_DrawImage(four_medium, 0, 0, 144, 168);
  if (uv_rounded == 5) Paint_DrawImage(five_medium, 0, 0, 144, 168);
  if (uv_rounded == 6) Paint_DrawImage(six_high, 0, 0, 144, 168);
  if (uv_rounded == 7) Paint_DrawImage(seven_high, 0, 0, 144, 168);
  if (uv_rounded == 8) Paint_DrawImage(eight_very_high, 0, 0, 144, 168);
  if (uv_rounded == 9) Paint_DrawImage(nine_very_high, 0, 0, 144, 168);
  if (uv_rounded == 10) Paint_DrawImage(ten_very_high, 0, 0, 144, 168);
  if (uv_rounded >= 11) Paint_DrawImage(eleven_extreme, 0, 0, 144, 168);
  display.refresh();
  delay(200);
}

//***************************************************************************
//   Show the scrolling feature of the slip, slop, slap and wrap
//***************************************************************************
void Scrolling_Message (void)
{
  int i;
  // Prints slip
  for (i = 0; i > -320; i = i - 48)
  {
    Paint_DrawImage(slip, i, 0, 432, 168);
    display.refresh();
    delay(100);
  }

  // Prints Slap
  for (i = 0; i > -320; i = i - 48)
  {
    Paint_DrawImage(slap, i, 0, 432, 168);
    display.refresh();
    delay(100);
  }

  // Prints Slop
  for (i = 0; i > -320; i = i - 48)
  {
    Paint_DrawImage(slop, i, 0, 432, 168);
    display.refresh();
    delay(100);
  }

  // Prints Wrap
  for (i = 0; i > -320; i = i - 48)
  {
    Paint_DrawImage(wrap, i, 0, 432, 168);
    display.refresh();
    delay(100);
  }
  delay(200);
  display.clearDisplay();
  delay(200);
}

#include <TinyGPS++.h>
#include <IridiumSBD.h>
#include <PString.h> // String buffer formatting: http://arduiniana.org
#include <Adafruit_NeoPixel.h> // Support for the WB2812B
#include "wiring_private.h"
#include <Wire.h>
#include <Adafruit_MPL3115A2.h>

/* Uncomment this to allow debug prints to console */
#define DEBUG
#define PRLTEST

/* ----------  Begin: Beacon_V5M Setup ------------------------------- */

// Stores both source and destination RockBLOCK serial numbers in flash.
// The default values are:
#define RB_destination 0 // Serial number of the destination RockBLOCK (int). Set to zero to disable RockBLOCK message forwarding
#define RB_source 0 // Serial number of this unit (int)

// Power for the 9603N is switched by a DMG3415 P-channel MOSFET
// A 2N2222 NPN transistor pulls the MOSFET gate low
// A high on A5 (D19) enables power to the 9603N

// Power can be provided by: two PowerFilm MPT3.6-150 solar panels; USB; 3 x Energiser Ultimate Lithium AA batteries
// GNSS data is provided by u-blox MAX-M8Q
// WB2812B NeoPixel is connected to D13 in parallel with a standard Red LED (for the bootloader)
// 1.25V precision voltage reference is connected to A0 to allow lower battery voltages to be measured

// The Iridium_9603_Beacon PCB is based extensively on the Adafruit Feather M0 (Adalogger)
// https://www.adafruit.com/products/2796
// GPS data provided by u-blox MAX-M8Q
// https://www.u-blox.com/en/product/max-m8-series
// Pressure (altitude) and temperature provided by MPL3115A2
// Requires Adafruit's MPL3115A2 library
// https://github.com/adafruit/Adafruit_MPL3115A2_Library

// Support for the WB2812B is provided by Adafruit:
// https://github.com/adafruit/Adafruit_NeoPixel

// Iridium 9603N is interfaced to M0 using Serial2
// D6 (Port A Pin 20) = Enable (Sleep) : Connect to 9603 ON/OFF Pin 5
// D10 (Port A Pin 18) = Serial2 TX : Connect to 9603 Pin 6
// D12 (Port A Pin 19) = Serial2 RX : Connect to 9603 Pin 7
// A3 / D17 (Port A Pin 4) = Ring Indicator : Connect to 9603 Pin 12

// Power to the 9603N is switched by a P-channel MOSFET
// The MOSFET gate is pulled high by a 10K resistor. The gate is pulled low by a 2N2222 NPN transistor
// Power is enabled by pulling the base of the transistor high
// The transistor base is connected to A5 / D19 (Port B Pin 2)

// Iridium 9603 is powered from Linear Technology LTC3225 SuperCapacitor Charger
// (fitted with 2 x 10F 2.7V caps e.g. Bussmann HV1030-2R7106-R)
// (or 2 x 1F 2.7V caps e.g. Bussmann HV0810-2R7105-R)
// to provide the 1.3A peak current when the 9603 is transmitting.
// Charging 10F capacitors to 5.3V at 60mA could take ~7 minutes!
// (~6.5 mins to PGOOD, ~7 mins to full charge)
// 1F capacitors charge in approx. 1/10 of that time.
// Setting the charging current to 150mA will reduce the time further.
// 5.3V is OK as the 9603N has an extended supply voltage range of +5 V +/- 0.5 V
// http://www.linear.com/product/LTC3225

// Port > D8 (Port A Pin 06, Physical Pin 11) = LTC3225 ~Shutdown

// A1 / D15 (Port B Pin 8) = LTC3225 PGOOD

// MAX-M8Q GNSS is interfaced to M0 using Serial1
// D1 (Port A Pin 10) = Serial1 TX : Connect to GPS RX
// D0 (Port A Pin 11) = Serial1 RX : Connect to GPS TX
// D11 (Port A Pin 16) = GPS ENable : Connect to GPS EN(ABLE)

// MPL3115A2 Pressure (Altitude) and Temperature Sensor
// D20 (Port A Pin 22) = SDA : Connect to MPL3115A2 SDA
// D21 (Port A Pin 23) = SCL : Connect to MPL3115A2 SCL

// D13 (Port A Pin 17) = WB2812B NeoPixel + single Red LED
// D9 (Port A Pin 7) = AIN 7 : Bus Voltage / 2
// D14 (Port A Pin 2) = AIN 0 : 1.25V precision voltage reference

// D7 (Port A Pin 21) = OMRON relay set coil (pull low to energise the coil via a P-channel MOSFET)
// D3 (Port A Pin 09) = OMRON relay reset coil (pull low to energise the coil via a P-channel MOSFET)

// Red LED on D13 shows when the SAMD is in bootloader mode (LED will fade up/down)

// WB2812B on D13 indicates what the software is doing:
// Magenta at power up (loop_step == init) (~10 seconds)
// Blue when waiting for a GNSS fix (loop_step == start_GPS or read_GPS or read_pressure) (could take 5 mins)
// Cyan when waiting for supercapacitors to charge (loop_step == start_LTC3225 or wait_LTC3225) (could take 7 mins)
// White during Iridium transmit (loop_step == start_9603) (could take 5 mins)
// Green flash (2 seconds) indicates successful transmission
// Red flash (2+ seconds) entering sleep
// LED will flash Red after: Iridium transmission (successful or failure); low battery detected; no GNSS data; supercapacitors failed to charge
// WB2812B blue LED has the highest forward voltage and is slightly dim at 3.3V. The red and green values are adapted accordingly (222 instead of 255).


// Flash Storage
#include <FlashStorage.h>
typedef struct { // Define a struct to hold the flash variable(s)
  int PREFIX; // Flash storage prefix (0xB5); used to test if flash has been written to before
  int INTERVAL; // Message interval in minutes
  // RockBLOCK source serial number: stored as an int; i.e. the RockBLOCK serial number of the 9603N attached to this beacon
  int RBSOURCE;
  // RockBLOCK destination serial number: stored as an int; i.e. the RockBLOCK serial number of the 9603N you would like the messages delivered _to_
  int RBDESTINATION; // Set this to zero to disable RockBLOCK gateway message forwarding
  int CSUM; // Flash storage checksum; the modulo-256 sum of PREFIX and INTERVAL; used to check flash data integrity
} FlashVarsStruct;
FlashStorage(flashVarsMem, FlashVarsStruct); // Reserve memory for the flash variables
FlashVarsStruct flashVars; // Define the global to hold the variables
int RBSOURCE = RB_source;
int RBDESTINATION = RB_destination;

// MPL3115A2
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();

// **************  BEGIN: SERIAL PORT DEFINITIONS *******************
// Serial Port Definisions
// SERCOM0 -> Serial1 -> GPS_SERIAL
// SERCOM1 -> Serial2 -> IRIDIUM_SERIAL
// SERCOM2 -> Serial4 -> XDATA_SERIAL [Enclosure Label SERIAL#1]
// SERCOM3 -> I2C -> PTU
// SERCOM4 -> Serial3 -> IMET_SERIAL [Enclosure Label SERIAL#2]
// SERCOM5 -> Serial -> DEBUG_SERIAL (USB Serial)

// SERCOM0 -> Serial1 -> GPS_SERIAL
#define GPS_SERIAL Serial1 // Use M0 Serial1 to interface to the MAX-M8Q

// SERCOM1 -> Serial2 -> Iridium
// Serial2 pin and pad definitions (in Arduino files Variant.h & Variant.cpp)
#define PIN_SERIAL2_RX       (34ul)               // Pin description number for PIO_SERCOM on D12 (Physical Pin 28)
#define PIN_SERIAL2_TX       (36ul)               // Pin description number for PIO_SERCOM on D10 (Physical Pin 27)
#define PAD_SERIAL2_TX       (UART_TX_PAD_2)      // SERCOM1 pad 2 (SC1PAD2)
#define PAD_SERIAL2_RX       (SERCOM_RX_PAD_3)    // SERCOM1 pad 3 (SC1PAD3)
// Instantiate the Serial2 class
Uart Serial2(&sercom1, PIN_SERIAL2_RX, PIN_SERIAL2_TX, PAD_SERIAL2_RX, PAD_SERIAL2_TX);
HardwareSerial &IRIDIUM_SERIAL(Serial2);

// Interrupt handler for SERCOM1 (essential for Serial2 comms)
void SERCOM1_Handler()
{
  Serial2.IrqHandler();
}

// SERCOM2 -> Serial4 -> XDATA_SERIAL [Enclosure Label SERIAL#1]
// Serial4 pin and pad definitions (in Arduino files Variant.h & Variant.cpp)
// Tx  is Digital Pin 2, Port A Pin 14, SERCOM2 Pad 2, Serial4 Tx, Physical Pin 23
// Rx  is Digital Pin 5, Port A Pin 15, SERCOM2 Pad 3, Serial4 Rx, Physical Pin 24
#define PIN_SERIAL4_RX       (5ul)               // Pin description number for PIO_SERCOM on D5
#define PIN_SERIAL4_TX       (2ul)               // Pin description number for PIO_SERCOM on D2
#define PAD_SERIAL4_TX       (UART_TX_PAD_2)      // SERCOM2 Pad 2 (SC2PAD2)
#define PAD_SERIAL4_RX       (SERCOM_RX_PAD_3)    // SERCOM2 Pad 3 (SC2PAD3)
// Instantiate the Serial3 class
Uart Serial4(&sercom2, PIN_SERIAL4_RX, PIN_SERIAL4_TX, PAD_SERIAL4_RX, PAD_SERIAL4_TX);
HardwareSerial &XDATA_SERIAL(Serial4);    //The hardware serial port that the XData instrument is attached to.

// Interrupt handler for SERCOM2 (essential for Serial4 comms)
void SERCOM2_Handler()
{
  Serial4.IrqHandler();
}

// SERCOM4 -> Serial3 -> IMET_SERIAL [Enclosure Label SERIAL#2]
// Serial3 pin and pad definitions (in Arduino files Variant.h & Variant.cpp)
// Tx  is connected to MOSI (Digital Pin 23, Port B Pin 10, SERCOM4 Pad 2, Serial3 Tx)
// Rx  is connected to SCK (Digital Pin 24, Port B Pin 11, SERCOM4 Pad 3, Serial3 Rx)
#define PIN_SERIAL3_RX       (24ul)               // Pin description number for PIO_SERCOM on D24
#define PIN_SERIAL3_TX       (23ul)               // Pin description number for PIO_SERCOM on D23
#define PAD_SERIAL3_TX       (UART_TX_PAD_2)      // SERCOM4 Pad 2 (SC4PAD2)
#define PAD_SERIAL3_RX       (SERCOM_RX_PAD_3)    // SERCOM4 Pad 3 (SC4PAD3)
// Instantiate the Serial3 class
Uart Serial3(&sercom4, PIN_SERIAL3_RX, PIN_SERIAL3_TX, PAD_SERIAL3_RX, PAD_SERIAL3_TX);
HardwareSerial &IMET_SERIAL(Serial3);   //The hardware serial port that the iMet is attached to.

// Interrupt handler for SERCOM4 (essential for Serial3 comms)
void SERCOM4_Handler()
{
  Serial3.IrqHandler();
}

#define DEBUG_SERIAL Serial //USB serial for debug

// **************  END: SERIAL PORT DEFINITIONS *******************

// Leave the "#define GALILEO" uncommented to use: GPS + Galileo + GLONASS + SBAS
// Comment the "#define GALILEO" out to use the default u-blox M8 GNSS: GPS + SBAS + QZSS + GLONASS
#define GALILEO

// Set Nav Mode to Portable
static const uint8_t setNavPortable[] = {
  0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
  0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// Set Nav Mode to Pedestrian
static const uint8_t setNavPedestrian[] = {
  0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
  0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// Set Nav Mode to Automotive
static const uint8_t setNavAutomotive[] = {
  0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x04, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
  0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// Set Nav Mode to Sea
static const uint8_t setNavSea[] = {
  0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x05, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
  0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// Set Nav Mode to Airborne <1G
static const uint8_t setNavAir[] = {
  0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
  0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const int len_setNav = 42;

// Set NMEA Config
// Set trackFilt to 1 to ensure course (COG) is always output
// Set Main Talker ID to 'GP' to avoid having to modify TinyGPS
static const uint8_t setNMEA[] = {
  0xb5, 0x62, 0x06, 0x17, 0x14, 0x00, 0x20, 0x40, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
static const int len_setNMEA = 26;

// Set GNSS Config to GPS + Galileo + GLONASS + SBAS (Causes the M8 to restart!)
static const uint8_t setGNSS[] = {
  0xb5, 0x62, 0x06, 0x3e, 0x3c, 0x00,
  0x00, 0x20, 0x20, 0x07,
  0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01,
  0x01, 0x01, 0x03, 0x00, 0x01, 0x00, 0x01, 0x01,
  0x02, 0x04, 0x08, 0x00, 0x01, 0x00, 0x01, 0x01,
  0x03, 0x08, 0x10, 0x00, 0x00, 0x00, 0x01, 0x01,
  0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x03,
  0x05, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x05,
  0x06, 0x08, 0x0e, 0x00, 0x01, 0x00, 0x01, 0x01
};
static const int len_setGNSS = 66;

static const int ringIndicator = 17; // 9602 Ring Indicator on pin D17
static const int LTC3225shutdown = 8; // LTC3225 ~Shutdown on pin D8
static const int LTC3225PGOOD = 15; // LTC3225 PGOOD on pin A1 / D15
static const int Enable_9603N = 19; // 9603N Enable (enables EXT_PWR via P-MOSFET)
static const int IridiumSleepPin = 6; // Iridium Sleep connected to D6

long iterationCounter = 0; // Increment each time a transmission is attempted

static const int ledPin = 13; // WB2812B + Red LED on pin D13

//#define NoLED // Uncomment this line to disable the LED
#define swap_red_green // Uncomment this line if your WB2812B has red and green reversed
#ifdef swap_red_green
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(1, ledPin, NEO_GRB + NEO_KHZ800); // GRB WB2812B
#else
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(1, ledPin, NEO_RGB + NEO_KHZ800); // RGB WB2812B
#endif
#define LED_Brightness 32 // 0 - 255 for WB2812B

static const int GPS_EN = 11; // GPS & MPL3115A2 Enable on pin D11
#define GPS_ON LOW
#define GPS_OFF HIGH
#define VAP A7 // Bus voltage analog pin (bus voltage divided by 2)
#define VREF A0 // 1.25V precision voltage reference
#define VBUS_NORM 3.3 // Normal bus voltage for battery voltage calculations
#define VREF_NORM 1.25 // Normal reference voltage for battery voltage calculations
#define VBAT_LOW 3.05 // Minimum voltage for LTC3225

static const int set_coil = 7; // OMRON G6SK relay set coil (pull low to energise coil)
static const int reset_coil = 3; // OMRON G6SK relay reset coil (pull low to energise coil) [D3]

unsigned long tnow;
int PGOOD;

int BEACON_INTERVAL = 12; // seconds (may not be used)


void LED_off() // Turn NeoPixel off
{
  pixels.setPixelColor(0, 0, 0, 0);
  pixels.show();
}

void LED_dim_white() // Set LED to dim white
{
  pixels.setBrightness(LED_Brightness / 2); // Dim the LED brightness
  pixels.setPixelColor(0, pixels.Color(222, 222, 255)); // Set color.
  pixels.show(); // This sends the updated pixel color to the hardware.
  pixels.setBrightness(LED_Brightness); // Reset the LED brightness
}

void LED_dim_blue() // Set LED to dim blue
{
  pixels.setBrightness(LED_Brightness / 2); // Dim the LED brightness
  pixels.setPixelColor(0, pixels.Color(0, 0, 255)); // Set color.
  pixels.show(); // This sends the updated pixel color to the hardware.
  pixels.setBrightness(LED_Brightness); // Reset the LED brightness
}

void LED_dim_cyan() // Set LED to dim cyan
{
  pixels.setBrightness(LED_Brightness / 2); // Dim the LED brightness
  pixels.setPixelColor(0, pixels.Color(0, 222, 255)); // Set color.
  pixels.show(); // This sends the updated pixel color to the hardware.
  pixels.setBrightness(LED_Brightness); // Reset the LED brightness
}

void LED_white() // Set LED to white
{
  pixels.setPixelColor(0, pixels.Color(222, 222, 255)); // Set color.
  pixels.show(); // This sends the updated pixel color to the hardware.
}

void LED_red() // Set LED to red
{
  pixels.setPixelColor(0, pixels.Color(222, 0, 0)); // Set color.
  pixels.show(); // This sends the updated pixel color to the hardware.
}

void LED_green() // Set LED to green
{
  pixels.setPixelColor(0, pixels.Color(0, 222, 0)); // Set color.
  pixels.show(); // This sends the updated pixel color to the hardware.
}

void LED_blue() // Set LED to blue
{
  pixels.setPixelColor(0, pixels.Color(0, 0, 255)); // Set color.
  pixels.show(); // This sends the updated pixel color to the hardware.
}

void LED_cyan() // Set LED to cyan
{
  pixels.setPixelColor(0, pixels.Color(0, 222, 255)); // Set color.
  pixels.show(); // This sends the updated pixel color to the hardware.
}

void LED_magenta() // Set LED to magenta
{
  pixels.setPixelColor(0, pixels.Color(222, 0, 255)); // Set color.
  pixels.show(); // This sends the updated pixel color to the hardware.
}

void set_relay()
{
  pinMode(set_coil, OUTPUT); // Make relay set_coil pin an output
  digitalWrite(set_coil, LOW); // Pull pin low
  delay(20); // Pull pin low for 20msec
  digitalWrite(set_coil, HIGH); // Pull pin high again
  pinMode(set_coil, INPUT_PULLUP); // Make relay set_coil pin high-impedance
}

void reset_relay()
{
  pinMode(reset_coil, OUTPUT); // Make relay reset_coil pin an output
  digitalWrite(reset_coil, LOW); // Pull pin low
  delay(20); // Pull pin low for 20msec
  digitalWrite(reset_coil, HIGH); // Pull pin high again
  pinMode(reset_coil, INPUT_PULLUP); // Make relay reset_coil pin high-impedance
}

// Send message in u-blox UBX format
// Calculates and appends the two checksum bytes
// Doesn't add the 0xb5 and 0x62 sync chars (these need to be included at the start of the message)
void sendUBX(const uint8_t *message, const int len) {
  int csum1 = 0; // Checksum bytes
  int csum2 = 0;
  for (int i = 0; i < len; i++) { // For each byte in the message
    GPS_SERIAL.write(message[i]); // Write the byte
    if (i >= 2) { // Don't include the sync chars in the checksum
      csum1 = csum1 + message[i]; // Update the checksum bytes
      csum2 = csum2 + csum1;
    }
  }
  csum1 = csum1 & 0xff; // Limit checksums to 8-bits
  csum2 = csum2 & 0xff;
  GPS_SERIAL.write((uint8_t)csum1); // Send the checksum bytes
  GPS_SERIAL.write((uint8_t)csum2);
}


// Read 'battery' voltage
float get_vbat() {

  float vbat = 0.0;
  float vref = VREF_NORM;
  float vrail = VBUS_NORM;

  // Measure the reference voltage and calculate the rail voltage
  vref = analogRead(VREF) * (VBUS_NORM / 1023.0);
  vrail = VREF_NORM * VBUS_NORM / vref;
  vbat = analogRead(VAP) * (2.0 * vrail / 1023.0); // Read 'battery' voltage from resistor divider, correcting for vrail

  return vbat;

}


float getBeaconT() {

  float pascals = 0.0;
  float tempC = 0.0;

  if (baro.begin()) {
    // Read pressure twice to avoid first erroneous value
    pascals = baro.getPressure();
    pascals = baro.getPressure();
    tempC = baro.getTemperature();    // This call seems to hang the program if getPressure not called first
  }
  else {
    Serial.println("[ERROR : read_pressure] ***!!! Error initialising MPL3115A2 !!!***");
  }

  return tempC;
}
/* ---------- End: Beacon_V5M Setup ----------------------------------- */


/* Define the number of XDATA packets to accumulate for each iridium packet.
   Note the Iridium SBD is limited to 340 Char and about 10s between packets
   so don't exceed these limits
*/
#define PACKETS_TO_SEND 7
#define SEND_TIMEOUT 12000

/* Incoming data flags */
bool newXData = false;
bool newIMetData = false;
bool newGPSData = false;

/* Setup Flag */
bool inSetup = true;

/* CNC variables in XDATA string */
int16_t CNC_300;    //OPC 300nm channel counts aka CN counts
int16_t CNC_500;    //OPC 500nm channel counts
int16_t CNC_700;    //OPC 700nm channel counts
int8_t Pump1_PWM;  //Pump1 PWM drive percent (0-100)
int8_t Pump2_PWM;  //Pump2 PWM drive percent (0-100)
int16_t TempCN;   //Temperature of the staurator C 0.01 resolution
int8_t TempIce;  //Temperature of Ice Jacket C 1 resolution
int8_t TempPCB;  //Temperature of the control boars C 1 resolution

/*iMet Global Variables */
float iMet_p, iMet_t, iMet_u;
String XDataInput = "";
String IMET_Buff = "";

/*Iridium Global Variables */
uint8_t IridiumBuffer[340];
int IridiumBufferIndex = 0;
uint8_t SendBuffer[340];
int SendBufferIndex = 0;
int XdataCnt = 0;
unsigned long  SendStartTime = 0;
float BeaconBatteryV = 0.0;
float BeaconT = 23.2;

float lat, lon, alt;

IridiumSBD isbd(IRIDIUM_SERIAL, IridiumSleepPin);
TinyGPSPlus gps;


/*
    Initialization of Beacon Board
*/
void setup() {

  bool ok = true;

  // Turn ON LED (Magenta) at start of setup
  pixels.begin(); // This initializes the NeoPixel library.
  delay(100); // Seems necessary to make the NeoPixel start reliably
  pixels.setBrightness(LED_Brightness); // Initialize the LED brightness
  LED_off(); // Turn NeoPixel off
#ifndef NoLED
  LED_magenta(); // Set LED to Magenta
#endif

#ifdef DEBUG
  DEBUG_SERIAL.begin(9600);
  delay(2000);
  DEBUG_SERIAL.println("[INFO : setup] INITIALIZING Beacon Board - Please Standby...");
#endif

  // define Pins and initial state of the LTC3225 (capacitor charger)
  pinMode(LTC3225shutdown, OUTPUT); // LTC3225 supercapacitor charger shutdown pin
  digitalWrite(LTC3225shutdown, LOW); // Disable the LTC3225 supercapacitor charger
  pinMode(LTC3225PGOOD, INPUT); // Define an input for the LTC3225 Power Good signal

  pinMode(Enable_9603N, OUTPUT); // 9603N enable via P-FET and NPN transistor
  digitalWrite(Enable_9603N, LOW); // Disable the 9603N

  pinMode(GPS_EN, OUTPUT); // GPS & MPL3115A2 enable
  digitalWrite(GPS_EN, GPS_OFF); // Disable the GPS and MPL3115A2

  //pinMode(IridiumSleepPin, OUTPUT); // The call to IridiumSBD should have done this - but just in case
  //digitalWrite(IridiumSleepPin, LOW); // Disable the Iridium 9603
  pinMode(ringIndicator, INPUT); // Define an input for the Iridium 9603 Ring Indicator signal

  pinMode(set_coil, INPUT_PULLUP); // Initialise relay set_coil pin
  pinMode(reset_coil, INPUT_PULLUP); // Initialise relay reset_coil pin

  // See if global variables have already been stored in flash
  // If they have, read them. If not, initialise them.
  flashVars = flashVarsMem.read(); // Read the flash memory
  int csum = flashVars.PREFIX + flashVars.INTERVAL + flashVars.RBSOURCE + flashVars.RBDESTINATION; // Sum the prefix and data
  csum = csum & 0xff; // Limit checksum to 8-bits
  if ((flashVars.PREFIX == 0xB5) and (csum == flashVars.CSUM)) { // Check prefix and checksum match
    // Flash data is valid so update globals using the stored values
    BEACON_INTERVAL = flashVars.INTERVAL;
    RBSOURCE = flashVars.RBSOURCE;
    RBDESTINATION = flashVars.RBDESTINATION;
  }
  else {
    // Flash data is corrupt or hasn't been initialised so do that now
    flashVars.PREFIX = 0xB5; // Initialise the prefix
    flashVars.INTERVAL = BEACON_INTERVAL; // Initialise the beacon interval
    flashVars.RBSOURCE = RBSOURCE; // Initialise the source RockBLOCK serial number
    flashVars.RBDESTINATION = RBDESTINATION; // Initialise the destination RockBLOCK serial number
    csum = flashVars.PREFIX + flashVars.INTERVAL + flashVars.RBSOURCE + flashVars.RBDESTINATION; // Initialise the checksum
    csum = csum & 0xff;
    flashVars.CSUM = csum;
    flashVarsMem.write(flashVars); // Write the flash variables
    reset_relay(); // Reset the relay as this is the first time the code has been run
  }

  ok = ok && start_GPS();        // Turn ON and configure GPS receiver
  ok = ok && start_LTC3225();    // Turn on the supercapacitor charger and wait for capacitors to be charged.
  ok = ok && start_Iridium();    // Power ON and check communication with Iridium 9603 modem


#ifdef DEBUG
  DEBUG_SERIAL.println("[INFO : setup] Starting iMET and XDATA Serial Ports.");
#endif

  IMET_SERIAL.begin(9600);
  //IMET_SERIAL.setTimeout(2000);

  XDATA_SERIAL.begin(9600);
  //XDATA_SERIAL.setTimeout(2000);
  pinPeripheral(2, PIO_SERCOM);
  pinPeripheral(5, PIO_SERCOM);

  if (!ok) {
    delay(10000);  // Keep looping through setup if initialization failed
    setup();
  }

}

void loop() {
  bool ok = true;

#ifndef NoLED
  LED_off(); // Turn LED off
#endif

  if (inSetup) {
#ifdef DEBUG
    DEBUG_SERIAL.println("[INFO : loop] Starting Main Loop.");
#endif
    inSetup = false;
  }

  checkForSerial();

  if (newXData)  //this code must also be copied into the ISBDCallback function
  {

#ifdef PRLTEST
    DEBUG_SERIAL.println();
    DEBUG_SERIAL.println(" ************************************************");
    DEBUG_SERIAL.print("newXData : XDataInput = "); DEBUG_SERIAL.print(XDataInput); DEBUG_SERIAL.print(" XDataCnt = "); DEBUG_SERIAL.println(XdataCnt);
    DEBUG_SERIAL.println(" ************************************************");
    DEBUG_SERIAL.println();
    DEBUG_SERIAL.print("IridiumBufferIndex: ");  DEBUG_SERIAL.println(IridiumBufferIndex);
#endif

    ok = parseXDATA();         //Parse the incoming string.  Also does a simple check to see if header of string is valid.

    if (ok) {
      IridiumBufferIndex = addXDataToIridium(IridiumBufferIndex);  //Add the values to the string to send via Iridium
      XdataCnt++;            //Increment the counter
    }

    XDataInput = "";
  }

  checkForSerial();

  if (XdataCnt >= PACKETS_TO_SEND)
  {
    BeaconT = getBeaconT();
    BeaconBatteryV = get_vbat();

#ifdef PRLTEST
    DEBUG_SERIAL.println();
    DEBUG_SERIAL.print("BeaconT: "); DEBUG_SERIAL.println(BeaconT);
    DEBUG_SERIAL.print("BeaconBatteryV: "); DEBUG_SERIAL.println(BeaconBatteryV);
#endif

    IridiumBuffer[IridiumBufferIndex + 1] = uint8_t(BeaconT);
    IridiumBuffer[IridiumBufferIndex + 2] = uint8_t(BeaconBatteryV * 10);
    SendBufferIndex = IridiumBufferIndex + 2;
    memcpy(SendBuffer, IridiumBuffer, SendBufferIndex); //copy the buffer/indx to a new one so we can start refilling the old one

#ifdef PRLTEST
    DEBUG_SERIAL.println();
    DEBUG_SERIAL.print("SendBufferIndex: "); DEBUG_SERIAL.println(SendBufferIndex);
    DEBUG_SERIAL.print("SendBuffer: "); DEBUG_SERIAL.write(SendBuffer, SendBufferIndex);
    DEBUG_SERIAL.println("SendBuffer in HEX: ");
    for (int i = 0; i < SendBufferIndex; i++) {
      DEBUG_SERIAL.print(SendBuffer[i], HEX);
    }
    DEBUG_SERIAL.println();
#endif

    XdataCnt = 0;
    IridiumBufferIndex = 0;
    memset(IridiumBuffer, 0, sizeof(IridiumBuffer));
    SendStartTime =  millis();
    int err = isbd.sendSBDBinary(SendBuffer, SendBufferIndex);
    if (err != ISBD_SUCCESS)
    {
      Serial.println();
      Serial.print("[ERROR : loop : sendSBDBinary]  Failed to send SBD, moving on: error ");
      Serial.println(err);
    }
#ifdef PRLTEST
    else {
      DEBUG_SERIAL.println();
      DEBUG_SERIAL.println("##     ## ########  ######   ######     ###     ######   ########     ######  ######## ##    ## ######## ");
      DEBUG_SERIAL.println("###   ### ##       ##    ## ##    ##   ## ##   ##    ##  ##          ##    ## ##       ###   ##    ##    ");
      DEBUG_SERIAL.println("#### #### ##       ##       ##        ##   ##  ##        ##          ##       ##       ####  ##    ##    ");
      DEBUG_SERIAL.println("## ### ## ######    ######   ######  ##     ## ##   #### ######       ######  ######   ## ## ##    ##    ");
      DEBUG_SERIAL.println("##     ## ##             ##       ## ######### ##    ##  ##                ## ##       ##  ####    ##    ");
      DEBUG_SERIAL.println("##     ## ##       ##    ## ##    ## ##     ## ##    ##  ##          ##    ## ##       ##   ###    ##    ");
      DEBUG_SERIAL.println("##     ## ########  ######   ######  ##     ##  ######   ########     ######  ######## ##    ##    ##    ");
      DEBUG_SERIAL.println();
    }
#endif

  }

}

/*
   This function checks for new serial data on hardware serial ports, and creates strings
   using the terminating characters (either \r or \n).  Once it has a complete string on a given port
   it sets a flag to show the string is available and/or calls a parsing function to process the string.
   It is *CRITICAL* that this function is called all the time, or launched from an ISR to make sure
   we are capturing all the incoming characters.
*/

void checkForSerial(void)
{

  int numChars;

  if ((numChars = XDATA_SERIAL.available()) > 0) {
    //DEBUG_SERIAL.print(numChars);
    char XDATA_Char = XDATA_SERIAL.read();
    //DEBUG_SERIAL.print("XDATA_Char: ");DEBUG_SERIAL.println(XDATA_Char);
    IMET_SERIAL.write(XDATA_Char);  //Pass through any xdata chars to the iMet.
    if (XDATA_Char == '\r')
    {
#ifdef DEBUG
      DEBUG_SERIAL.println();
      DEBUG_SERIAL.print("XDATA String: ");
      DEBUG_SERIAL.println(XDataInput);
#endif
      newXData = true;
    } else
    {
      XDataInput += XDATA_Char;
    }
  }

  if (GPS_SERIAL.available()) {
    gps.encode(GPS_SERIAL.read()); //this sends incoming NEMA data to a parser, need to do this regularly
  }

  if (IMET_SERIAL.available()) {
    char IMET_Char = IMET_SERIAL.read();
    //DEBUG_SERIAL.print("IMET_Char: ");DEBUG_SERIAL.println(IMET_Char);
    XDATA_SERIAL.write(IMET_Char); //Pass through any iMet chars to the XData instruments.
    if (IMET_Char == '\n')
    {
      newIMetData = true; //Raise the flag.
      parseIMET();
      IMET_Buff = "";
    } else
    {
      IMET_Buff += IMET_Char;
    }
  }

}

/*
   This function parses the ASCII Imet strings to extract the p, t and u values
   from these strings and save them in global variables. For now it ignores the iMet GPS
   assuming the instuments are relying on their own GPS
*/

void parseIMET()
{
  char * strtokIndx; // this is used by strtok() as an index
  char IMETArray[64];
  if (IMET_Buff.startsWith("PTUX")) //we have a PTU string
  {
    (IMET_Buff.substring(6)).toCharArray(IMETArray, 64);
    strtokIndx = strtok(IMETArray, ", ");
    iMet_p = atof(strtokIndx);
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    iMet_t = atof(strtokIndx);
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    iMet_u = atof(strtokIndx);
  }
#ifdef DEBUG
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.print("iMET Variables: ");
  DEBUG_SERIAL.print(iMet_p);
  DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.print(iMet_t);
  DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.println(iMet_u);
#endif
}

/*
   This function parses incoming XData strings to their component values.
   It decides which parser to use based on the 'instrument ID' field in the Xdata.  It puts the component
   values into global variables that can then be packaged up to send via Iridium.
*/

bool parseXDATA()
{
  bool ok;
  char XDATAArray[128];
  int c = -99;
  int Pump1_PWM_TEMP = 0;
  int Pump2_PWM_TEMP = 0;
  int TempIce_TEMP = 0;
  int TempPCB_TEMP = 0;


  CNC_300 = 0;    //OPC 300nm channel counts aka CN counts
  CNC_500 = 0;    //OPC 500nm channel counts
  CNC_700 = 0;    //OPC 700nm channel counts
  Pump1_PWM = 0;  //Pump1 PWM drive percent (0-100)
  Pump2_PWM = 0;  //Pump2 PWM drive percent (0-100)
  TempCN = 0;   //Temperature of the staurator C 0.01 resolution

  if (XDataInput.startsWith("xdata=41")) //we have a CNC string
  {
    (XDataInput.substring(10)).toCharArray(XDATAArray, 128); //drop the first 10 chars ('xdata=4102') and process the rest

    //#ifdef PRLTEST
    //DEBUG_SERIAL.print("XDATAArray: "); DEBUG_SERIAL.println(XDATAArray);
    /*
        DEBUG_SERIAL.print(XDATAArray[0]); DEBUG_SERIAL.print(" "); DEBUG_SERIAL.print(XDATAArray[1]); DEBUG_SERIAL.print(" "); DEBUG_SERIAL.print(XDATAArray[2]); DEBUG_SERIAL.print(" "); DEBUG_SERIAL.print(XDATAArray[3]); DEBUG_SERIAL.println(" ");
        DEBUG_SERIAL.print(XDATAArray[4]); DEBUG_SERIAL.print(" "); DEBUG_SERIAL.print(XDATAArray[5]); DEBUG_SERIAL.print(" "); DEBUG_SERIAL.print(XDATAArray[6]); DEBUG_SERIAL.print(" "); DEBUG_SERIAL.print(XDATAArray[7]); DEBUG_SERIAL.println(" ");
        DEBUG_SERIAL.print(XDATAArray[8]); DEBUG_SERIAL.print(" "); DEBUG_SERIAL.print(XDATAArray[9]); DEBUG_SERIAL.print(" "); DEBUG_SERIAL.print(XDATAArray[10]); DEBUG_SERIAL.print(" "); DEBUG_SERIAL.print(XDATAArray[11]); DEBUG_SERIAL.println(" ");
        DEBUG_SERIAL.print(XDATAArray[12]); DEBUG_SERIAL.print(" "); DEBUG_SERIAL.print(XDATAArray[13]); DEBUG_SERIAL.print(" "); DEBUG_SERIAL.print(XDATAArray[14]); DEBUG_SERIAL.print(" "); DEBUG_SERIAL.print(XDATAArray[15]); DEBUG_SERIAL.println(" ");
        DEBUG_SERIAL.print(XDATAArray[16]); DEBUG_SERIAL.print(" "); DEBUG_SERIAL.print(XDATAArray[17]); DEBUG_SERIAL.print(" "); DEBUG_SERIAL.print(XDATAArray[18]); DEBUG_SERIAL.print(" "); DEBUG_SERIAL.print(XDATAArray[19]); DEBUG_SERIAL.println(" ");
        DEBUG_SERIAL.print(XDATAArray[20]); DEBUG_SERIAL.print(" "); DEBUG_SERIAL.print(XDATAArray[21]); DEBUG_SERIAL.print(" "); DEBUG_SERIAL.print(XDATAArray[22]); DEBUG_SERIAL.print(" "); DEBUG_SERIAL.print(XDATAArray[23]); DEBUG_SERIAL.println(" ");
    */
    //#endif
    c = sscanf(XDATAArray, "%4hx %4hx %4hx %2x %2x %4hx %2x %2x", &CNC_300, &CNC_500, &CNC_700, &Pump1_PWM_TEMP, &Pump2_PWM_TEMP, &TempCN, &TempIce_TEMP, &TempPCB_TEMP);
    ok = true;
  }
  else {
    ok = false;
  }

  // Required as using hh in formated string did not return the expected value.
  Pump1_PWM = (int8_t) Pump1_PWM_TEMP;
  Pump2_PWM = (int8_t) Pump2_PWM_TEMP;
  TempIce = (int8_t) TempIce_TEMP;
  TempPCB = (int8_t) TempPCB_TEMP;

#ifdef DEBUG
  DEBUG_SERIAL.print("XDATA Valid = "); DEBUG_SERIAL.println(ok);
  DEBUG_SERIAL.print("CNC Variables: ");
  DEBUG_SERIAL.print(CNC_300); DEBUG_SERIAL.print(' ');
  DEBUG_SERIAL.print(CNC_500); DEBUG_SERIAL.print(' ');
  DEBUG_SERIAL.print(CNC_700); DEBUG_SERIAL.print(' ');
  DEBUG_SERIAL.print(Pump1_PWM); DEBUG_SERIAL.print(' ');
  DEBUG_SERIAL.print(Pump2_PWM); DEBUG_SERIAL.print(' ');
  DEBUG_SERIAL.print(float(TempCN) / 100.0); DEBUG_SERIAL.print(' '); // Convert this back to a float
  DEBUG_SERIAL.print(float(TempIce) - 100.0); DEBUG_SERIAL.print(' '); // Convert this back to a float
  DEBUG_SERIAL.print(float(TempPCB) - 100.0); DEBUG_SERIAL.println(); // Convert this back to a float
#endif

  memset(XDATAArray, 0, sizeof(XDATAArray));
  newXData = false;

  return ok;
}

/*
   Creates and array of bytes (uint8_t) to send via iridium.   The function is passed the exisitng
   array index, and returns the new array index with bytes added to it.   First is checks that there is enough space
   in the array for the new bytes, and returns the existing index if there is not enough space.
*/
int addXDataToIridium(int start_index)
{
  if (start_index > 337 - 29) //If there is not enough space in the SBD message don't add this data
    return start_index;

  /* Add the formatted XData */
  IridiumBuffer[start_index + 1] = highByte(CNC_300);
  IridiumBuffer[start_index + 2] += lowByte(CNC_300);
  IridiumBuffer[start_index + 3] += highByte(CNC_500);
  IridiumBuffer[start_index + 4] += lowByte(CNC_500);
  IridiumBuffer[start_index + 5] += highByte(CNC_700);
  IridiumBuffer[start_index + 6] += lowByte(CNC_700);
  IridiumBuffer[start_index + 7] += lowByte(Pump1_PWM);
  IridiumBuffer[start_index + 8] += lowByte(Pump2_PWM);
  IridiumBuffer[start_index + 9] += highByte(TempCN);
  IridiumBuffer[start_index + 10] += lowByte(TempCN);
  IridiumBuffer[start_index + 11] += lowByte(TempIce);
  IridiumBuffer[start_index + 12] += lowByte(TempPCB);
  /*Add the most recent p,t,u */

  IridiumBuffer[start_index + 13] += highByte(uint16_t(iMet_p * 100.0));
  IridiumBuffer[start_index + 14] += lowByte(uint16_t(iMet_p * 100.0));
  IridiumBuffer[start_index + 15] += highByte(uint16_t((iMet_t + 273.15) * 100.0));
  IridiumBuffer[start_index + 16] += lowByte(uint16_t((iMet_t + 273.15) * 100.0));
  IridiumBuffer[start_index + 17] += lowByte(uint8_t(iMet_u));

  /*Add the most recent lat/lon/alt */
  /* Latitude + 90.0 then shifted 1e6 to the left and sent as 4 byte int
      The horizontal resolution > 1m
  */
  lat = gps.location.lat();
  lon = gps.location.lng();
  alt = gps.altitude.meters();

#ifdef PRLTEST
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.print("Lat: "); DEBUG_SERIAL.println(lat);
  DEBUG_SERIAL.print("Long: "); DEBUG_SERIAL.println(lon);
  DEBUG_SERIAL.print("Alt: "); DEBUG_SERIAL.println(alt);
#endif

  IridiumBuffer[start_index + 18] += highByte(uint32_t( (lat + 90.0) * 1000000.0) & 0xFFFF0000 >> 16);
  IridiumBuffer[start_index + 19] += lowByte(uint32_t( (lat + 90.0) * 1000000.0) & 0xFFFF0000 >> 16);
  IridiumBuffer[start_index + 20] += highByte(uint32_t( (lat + 90.0) * 1000000.0));
  IridiumBuffer[start_index + 21] += lowByte(uint32_t( (lat + 90.0) * 1000000.0));
  /* Longitude + 180.0 then shifted 1e6 to the left and sent as 4 byte int */
  IridiumBuffer[start_index + 22] += highByte(uint32_t( (lon + 180.0) * 1000000.0) & 0xFFFF0000 >> 16);
  IridiumBuffer[start_index + 23] += lowByte(uint32_t( (lon + 180.0) * 1000000.0) & 0xFFFF0000 >> 16);
  IridiumBuffer[start_index + 24] += highByte(uint32_t( (lon + 180.0) * 1000000.0));
  IridiumBuffer[start_index + 25] += lowByte(uint32_t( (lon + 180.0) * 1000000.0));
  /*Altitude in meters sent as a 2 byte in 0 - 65535 meters */
  IridiumBuffer[start_index + 26] += highByte(uint16_t( alt));
  IridiumBuffer[start_index + 27] += lowByte(uint16_t(alt));

#ifdef DEBUG
  DEBUG_SERIAL.print("Iridium Payload: ");
  DEBUG_SERIAL.write(IridiumBuffer, start_index + 27);
  DEBUG_SERIAL.println();
  for (int i = 0; i < 340; i++) {
    DEBUG_SERIAL.print(IridiumBuffer[i], HEX);
  }
  DEBUG_SERIAL.println();
#endif

  return start_index + 27;
}

/*
   This is the ISBD call back function that continues to check serial ports for
   new data and proces that data while we wait for the SBD to send. After 12 seconds
   it gives up, clears the output buffer and moves on to the next sample.
*/

bool ISBDCallback()
{
  /*
    if (!inSetup) {
    if ((millis() >  SendStartTime + SEND_TIMEOUT)  or (XdataCnt >= PACKETS_TO_SEND))   //probably should not do it this way.
      return false;

    checkForSerial();

    if (newXData)
    {

    #ifdef PRLTEST
      DEBUG_SERIAL.println();
      DEBUG_SERIAL.println(" ************************************************");
      DEBUG_SERIAL.print("INCALLBACK newXData : XDataInput = "); DEBUG_SERIAL.print(XDataInput); DEBUG_SERIAL.print(" XDataCnt = "); DEBUG_SERIAL.println(XdataCnt);
      DEBUG_SERIAL.println(" ************************************************");
      DEBUG_SERIAL.println();
    #endif

      parseXDATA();         //Parse the incoming string
      IridiumBufferIndex = addXDataToIridium(IridiumBufferIndex);  //Add the values to the string to send via Iridium
      XdataCnt++;            //Increment the counter
      XDataInput = "";
    }
    }
  */

  // 'Flash' the LED
  if ((millis() / 1000) % 2 == 1) {
    LED_dim_white();
  }
  else {
    LED_white();
  }


  return true;
}

/*
   in Debug mode print the SBD Modem Traffic.
*/


#ifdef DEBUG
void ISBDConsoleCallback(IridiumSBD *device, char c)
{
  DEBUG_SERIAL.write(c);
}

void ISBDDiagsCallback(IridiumSBD *device, char c)
{
  DEBUG_SERIAL.write(c);
}
#endif

/*
   Powers ON the GPS Reciever and initiates the serial communication.
   Also configures the GPS Reciever.
*/


bool start_GPS() {

  bool ok = true;

#ifndef NoLED
  LED_blue(); // Set LED to Blue
#endif

  // Power up the GPS and MPL3115A2
#ifdef DEBUG
  DEBUG_SERIAL.println("[INFO : start_GPS] Powering up the GPS and MPL3115A2...");
#endif
  digitalWrite(GPS_EN, GPS_ON); // Enable the GPS and MPL3115A2

  delay(2000); // Allow time for both to start

  GPS_SERIAL.begin(9600);
  delay(1000);

  // Configure GPS
#ifdef DEBUG
  DEBUG_SERIAL.println("[INFO : start_GPS] Configuring GPS.");
#endif

  // Disable all messages except GGA and RMC
  GPS_SERIAL.println("$PUBX,40,GLL,0,0,0,0*5C"); // Disable GLL
  delay(1100);
  GPS_SERIAL.println("$PUBX,40,ZDA,0,0,0,0*44"); // Disable ZDA
  delay(1100);
  GPS_SERIAL.println("$PUBX,40,VTG,0,0,0,0*5E"); // Disable VTG
  delay(1100);
  GPS_SERIAL.println("$PUBX,40,GSV,0,0,0,0*59"); // Disable GSV
  delay(1100);
  GPS_SERIAL.println("$PUBX,40,GSA,0,0,0,0*4E"); // Disable GSA
  delay(1100);

  //sendUBX(setNavPortable, len_setNav); // Set Portable Navigation Mode
  //sendUBX(setNavPedestrian, len_setNav); // Set Pedestrian Navigation Mode
  //sendUBX(setNavAutomotive, len_setNav); // Set Automotive Navigation Mode
  //sendUBX(setNavSea, len_setNav); // Set Sea Navigation Mode
  sendUBX(setNavAir, len_setNav); // Set Airborne <1G Navigation Mode
  delay(1100);

  sendUBX(setNMEA, len_setNMEA); // Set NMEA: to always output COG; and set main talker to GP (instead of GN)
  delay(1100);

#ifdef GALILEO
  sendUBX(setGNSS, len_setGNSS); // Set GNSS - causes M8 to restart!
  delay(3000); // Wait an extra time for GNSS to restart
#endif

  return ok;  // assumed all is good

}


/*
  This function starts the super capacitor charger.
*/
bool start_LTC3225()
{

  bool  ok = true;

#ifndef NoLED
  LED_cyan(); // Set LED to Cyan
#endif

  // Power up the LTC3225EDDB super capacitor charger
#ifdef DEBUG
  DEBUG_SERIAL.println("[INFO : start_LTC3225] Powering up the LTC3225EDDB");
  DEBUG_SERIAL.println("[INFO : start_LTC3225] Waiting for PGOOD to go HIGH...");
#endif

  digitalWrite(LTC3225shutdown, HIGH); // Enable the LTC3225EDDB supercapacitor charger
  delay(1000); // Let PGOOD stabilise

  // Allow 10 mins for LTC3225 to achieve PGOOD
  PGOOD = digitalRead(LTC3225PGOOD);
  for (tnow = millis(); !PGOOD && millis() - tnow < 10UL * 60UL * 1000UL;)
  {
#ifndef NoLED
    // 'Flash' the LED
    if ((millis() / 1000) % 2 == 1) {
      LED_dim_cyan();
    }
    else {
      LED_cyan();
    }
#endif
    PGOOD = digitalRead(LTC3225PGOOD);
  }

  if (PGOOD == LOW) {
    Serial.println("[ERROR : start_LTC3225] LTC3225 !PGOOD - Supercapacitors did NOT charge");
    ok = false;
  } else {
    // Allow extra time for the super capacitors to charge
#ifdef DEBUG
    DEBUG_SERIAL.println("[INFO : start_LTC3225] PGOOD has gone HIGH");
    DEBUG_SERIAL.println("[INFO : start_LTC3225] Allowing extra time (20 seconds) to make sure capacitors are charged...");
#endif
    // Allow 20s for extra charging
    PGOOD = digitalRead(LTC3225PGOOD);
    for (tnow = millis(); PGOOD && millis() - tnow < 1UL * 20UL * 1000UL;)
    {
#ifndef NoLED
      // 'Flash' the LED
      if ((millis() / 1000) % 2 == 1) {
        LED_dim_cyan();
      }
      else {
        LED_cyan();
      }
#endif
      PGOOD = digitalRead(LTC3225PGOOD);
    }
  }

  if (PGOOD == LOW) {
    Serial.println("[ERROR : start_LTC3225] LTC3225 !PGOOD - Supercapacitors did NOT charge");
    ok = false;
  } else {
    // Allow extra time for the super capacitors to charge
#ifdef DEBUG
    DEBUG_SERIAL.println("[INFO : start_LTC3225] Supercapacitors are CHARGED");
#endif
  }

  return ok;
}


/*
   Powers ON the Iridium 9603N modem and initiates serial communication.
*/

bool start_Iridium() {

  bool ok = true;

#ifndef NoLED
  LED_white(); // Set LED to White
#endif

  // Start talking to the 9603 and power it up
#ifdef DEBUG
  DEBUG_SERIAL.println("[INFO : start_Iridium] Powering Up 9603 modem.");
#endif

  digitalWrite(Enable_9603N, HIGH); // Enable the 9603N
  delay(2000);

  IRIDIUM_SERIAL.begin(19200);
  delay(1000);

  isbd.setPowerProfile(IridiumSBD::DEFAULT_POWER_PROFILE);

  // Setup the Iridium modem
  if (isbd.begin() != ISBD_SUCCESS)
  {
    Serial.println("[ERROR : start_Iridium] Could not begin modem operations.");
    ok = false;
  }

  return ok;
}

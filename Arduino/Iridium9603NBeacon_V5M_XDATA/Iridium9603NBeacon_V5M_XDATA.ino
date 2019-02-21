#include <TinyGPS++.h>
#include <IridiumSBD.h>

/* Uncomment this to allow debug prints to console */
#define DEBUG

/* Define the number of XDATA packets to accumulate for each iridium packet.  
 * Note the Iridium SBD is limited to 340 Char and about 10s between packets
 * so don't exceed these limits 
 */
#define PACKETS_TO_SEND 7 
#define SEND_TIMEOUT 12000 

#define DEBUG_SERIAL Serial //USB serial for debug
#define IMET_SERIAL Serial1 //The hardware serial port that the iMet is attached to.
#define XDATA_SERIAL Serial2 //The hardware serial port that the XData instrument is attached to.
#define IRIDIUM_SERIAL Serial3
#define GPS_SERIAL Serial4


/* Incoming data flags */
bool newXData = false;
bool newIMetData = false;
bool newGPSData = false;

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
String XDataInput;
String IMET_Buff;

/*Iridium Global Variables */
uint8_t IridiumBuffer[340];
int IridiumBufferIndex = 0;
uint8_t SendBuffer[340];
int SendBufferIndex = 0;
int XdataCnt = 0;
unsigned long  SendStartTime = 0;
float BeaconBatteryV = 4.7;
float BeaconT = 23.2;

float lat, lon, alt;

IridiumSBD modem(IRIDIUM_SERIAL);
TinyGPSPlus gps;

void setup() {
IMET_SERIAL.begin(9600);
XDATA_SERIAL.begin(9600);
IRIDIUM_SERIAL.begin(19200);
GPS_SERIAL.begin(9600);

#ifdef DEBUG
  DEBUG_SERIAL.begin(9600);
#endif

  
  modem.setPowerProfile(IridiumSBD::DEFAULT_POWER_PROFILE);

  // Setup the Iridium modem
  if (modem.begin() != ISBD_SUCCESS)
  {
    Serial.println("Couldn't begin modem operations.");
    exit(0);
  }


}

void loop() {
 checkForSerial();
 
 if(newXData)
 {
  parseXDATA();         //Parse the incoming string
  IridiumBufferIndex = addXDataToIridium(IridiumBufferIndex);  //Add the values to the string to send via Iridium
  XdataCnt++;            //Increment the counter
 }

 checkForSerial();
 
 if (XdataCnt >= PACKETS_TO_SEND)
 {
  IridiumBuffer[IridiumBufferIndex + 1] = uint8_t(BeaconT);
  IridiumBuffer[IridiumBufferIndex + 2] = uint8_t(BeaconBatteryV*10);
  SendBufferIndex = IridiumBufferIndex + 2;
  memcpy(IridiumBuffer,SendBuffer, SendBufferIndex); //copy the buffer/indx to a new one so we can start refilling the old one
  XdataCnt = 0;
  IridiumBufferIndex = 0;
  SendStartTime =  millis();
  int err = modem.sendSBDBinary(SendBuffer, SendBufferIndex);
  if (err != ISBD_SUCCESS)
  {
    Serial.print("sendSBDBinary failed, moving on: error ");
    Serial.println(err);
  }
  
 }

}

/*
 * This function checks for new serial data on hardware serial ports, and creates strings
 * using the terminating characters (either \r or \n).  Once it has a complete string on a given port 
 * it sets a flag to show the string is available and/or calls a parsing function to process the string.
 * It is *CRITICAL* that this function is called all the time, or launched from an ISR to make sure
 * we are capturing all the incoming characters. 
 */

void checkForSerial(void)
{  
   if (GPS_SERIAL.available()) {
        gps.encode(GPS_SERIAL.read()); //this sends incoming NEMA data to a parser, need to do this regularly
   }
   
   if(IMET_SERIAL.available()){
    char IMET_Char = IMET_SERIAL.read();
     XDATA_SERIAL.write(IMET_Char); //Pass through any iMet chars to the XData instruments.
     if(IMET_Char == '\n')
     {
      newIMetData = true; //Raise the flag.
      parseIMET();
      IMET_Buff = "";
     } else
     {
      IMET_Buff += IMET_Char;
     }   
  }

  if(XDATA_SERIAL.available()){
    char XDATA_Char = XDATA_SERIAL.read();
    IMET_SERIAL.write(XDATA_Char);  //Pass through any xdata chars to the iMet.
     if(XDATA_Char == '\r')
     {
      #ifdef DEBUG 
      DEBUG_SERIAL.print("XDATA String: ");
      DEBUG_SERIAL.println(XDataInput);
      #endif
      newXData = true;
     } else
     {
      XDataInput += XDATA_Char;
     }   
  }
}

/*
 * This function parses the ASCII Imet strings to extract the p, t and u values
 * from these strings and save them in global variables. For now it ignores the iMet GPS 
 * assuming the instuments are relying on their own GPS
 */

void parseIMET()
{
  char * strtokIndx; // this is used by strtok() as an index
  char IMETArray[64];
  if (IMET_Buff.startsWith("PTUX")) //we have a PTU string
  {
   (IMET_Buff.substring(6)).toCharArray(IMETArray,64);
   strtokIndx = strtok(IMETArray,", ");
   iMet_p = atof(strtokIndx);
   strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
   iMet_t = atof(strtokIndx); 
   strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
   iMet_u = atof(strtokIndx);   
  }
  #ifdef DEBUG
  DEBUG_SERIAL.print("iMET Variables: ");
  DEBUG_SERIAL.print(iMet_p);
  DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.print(iMet_t);
  DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.println(iMet_u);
  #endif
}

/*
 * This function parses incoming XData strings to their component values.  
 * It decides which parser to use based on the 'instrument ID' field in the Xdata.  It puts the component
 * values into global variables that can then be packaged up to send via Iridium. 
 */
 
void parseXDATA()
{
  char XDATAArray[128];
  if (XDataInput.startsWith("xdata=41")) //we have a CNC string
  {
   (XDataInput.substring(10)).toCharArray(XDATAArray,128); //drop the first 10 chars ('xdata=4102') and process the rest
   sscanf(XDATAArray, "%4hX %4hX %4hX %2hhX %2hhX %4hX %2hhX %2hhX", &CNC_300, &CNC_500, &CNC_700, &Pump1_PWM, &Pump2_PWM, &TempCN, &TempIce, &TempPCB);  
  }

  #ifdef DEBUG
  DEBUG_SERIAL.print("CNC Variables: ");
  DEBUG_SERIAL.print(CNC_300); DEBUG_SERIAL.print(' ');
  DEBUG_SERIAL.print(CNC_500); DEBUG_SERIAL.print(' ');
  DEBUG_SERIAL.print(CNC_700); DEBUG_SERIAL.print(' ');
  DEBUG_SERIAL.print(Pump1_PWM); DEBUG_SERIAL.print(' ');
  DEBUG_SERIAL.print(Pump2_PWM); DEBUG_SERIAL.print(' ');
  DEBUG_SERIAL.print(float(TempCN)/100.0); DEBUG_SERIAL.print(' '); // Convert this back to a float
  DEBUG_SERIAL.print(float(TempIce) - 100.0); DEBUG_SERIAL.print(' '); // Convert this back to a float
  DEBUG_SERIAL.print(float(TempPCB) - 100.0); DEBUG_SERIAL.println(); // Convert this back to a float
  #endif
   
}

/*
 * Creates and array of bytes (uint8_t) to send via iridium.   The function is passed the exisitng 
 * array index, and returns the new array index with bytes added to it.   First is checks that there is enough space 
 * in the array for the new bytes, and returns the existing index if there is not enough space.
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

  IridiumBuffer[start_index + 13] += highByte(uint16_t(iMet_p*100.0));
  IridiumBuffer[start_index + 14] += lowByte(uint16_t(iMet_p*100.0));
  IridiumBuffer[start_index + 15] += highByte(uint16_t((iMet_t+273.15)*100.0));
  IridiumBuffer[start_index + 16] += lowByte(uint16_t((iMet_t+273.15)*100.0));
  IridiumBuffer[start_index + 17] += lowByte(uint8_t(iMet_u));

  /*Add the most recent lat/lon/alt */
  /* Latitude + 90.0 then shifted 1e6 to the left and sent as 4 byte int 
   *  The horizontal resolution > 1m
  */
  lat = gps.location.lat();
  lon = gps.location.lng();
  alt = gps.altitude.meters();
  
  IridiumBuffer[start_index + 18] += highByte(uint32_t( (lat + 90.0)*1000000.0) & 0xFFFF0000 >> 16);
  IridiumBuffer[start_index + 19] += lowByte(uint32_t( (lat + 90.0)*1000000.0) & 0xFFFF0000 >> 16);
  IridiumBuffer[start_index + 20] += highByte(uint32_t( (lat + 90.0)*1000000.0));
  IridiumBuffer[start_index + 21] += lowByte(uint32_t( (lat + 90.0)*1000000.0));
  /* Longitude + 180.0 then shifted 1e6 to the left and sent as 4 byte int */
  IridiumBuffer[start_index + 22] += highByte(uint32_t( (lon + 180.0)*1000000.0) & 0xFFFF0000 >> 16);
  IridiumBuffer[start_index + 23] += lowByte(uint32_t( (lon + 180.0)*1000000.0) & 0xFFFF0000 >> 16);
  IridiumBuffer[start_index + 24] += highByte(uint32_t( (lon + 180.0)*1000000.0));
  IridiumBuffer[start_index + 25] += lowByte(uint32_t( (lon + 180.0)*1000000.0));
  /*Altitude in meters sent as a 2 byte in 0 - 65535 meters */
  IridiumBuffer[start_index + 26] += highByte(uint16_t( alt));
  IridiumBuffer[start_index + 27] += lowByte(uint16_t(alt));

  #ifdef DEBUG
  DEBUG_SERIAL.print("Iridium Payload: ");
  DEBUG_SERIAL.write(IridiumBuffer, start_index + 27);
  #endif

  return start_index + 27;
}

/*
 * This is the ISBD call back function that continues to check serial ports for
 * new data and proces that data while we wait for the SBD to send. After 12 seconds
 * it gives up, clears the output buffer and moves on to the next sample. 
 */
bool ISBDCallback()
{
 if ( millis() >  SendStartTime + SEND_TIMEOUT)
 return false;
 
 checkForSerial();
 
 if(newXData)
 {
  parseXDATA();         //Parse the incoming string
  
  IridiumBufferIndex = addXDataToIridium(IridiumBufferIndex);
  XdataCnt++;            //Increment the counter
 }

 return true;
}

/*
 * in Debug mode print the SBD Modem Traffic. 
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

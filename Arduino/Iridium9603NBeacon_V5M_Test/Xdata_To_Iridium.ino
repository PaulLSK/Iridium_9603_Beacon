
/* Uncomment this to allow debug prints to console */
#define DEBUG

/* Define the number of XDATA packets to accumulate for each iridium packet.  
 * Note the Iridium SBD is limited to 340 Char and about 10s between packets
 * so don't exceed these limits 
 */
#define PACKETS_TO_SEND 7 

#define DEBUG_SERIAL Serial //USB serial for debug
#define IMET_SERIAL Serial1 //The hardware serial port that the iMet is attached to.
#define XDATA_SERIAL Serial1 //The hardware serial port that the XData instrument is attached to.

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
String IridiumString = "";
int XdataCnt = 0;
float BeaconBatteryV = 4.7;
float BeaconT = 23.2;

float lat, lon, alt;

void setup() {
IMET_SERIAL.begin(9600);
XDATA_SERIAL.begin(9600);

#ifdef DEBUG
DEBUG_SERIAL.begin(9600);
#endif

}

void loop() {
 checkForSerial();
 
 if(newXData)
 {
  parseXDATA();         //Parse the incoming string
  addXDataToIridium();  //Add the values to the string to send via Iridium
  XdataCnt++;            //Increment the counter
 }

 checkForSerial();
 
 if (XdataCnt >= PACKETS_TO_SEND)
 {
  SendIridium();
  XdataCnt = 0;
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

void addXDataToIridium()
{
  /* Add the formatted XData */
  IridiumString += highByte(CNC_300);
  IridiumString += lowByte(CNC_300);
  IridiumString += highByte(CNC_500);
  IridiumString += lowByte(CNC_500);
  IridiumString += highByte(CNC_700);
  IridiumString += lowByte(CNC_700);
  IridiumString += lowByte(Pump1_PWM);
  IridiumString += lowByte(Pump2_PWM);
  IridiumString += highByte(TempCN);
  IridiumString += lowByte(TempCN);
  IridiumString += lowByte(TempIce);
  IridiumString += lowByte(TempPCB);
  /*Add the most recent p,t,u */

  IridiumString += highByte(uint16_t(iMet_p*100.0));
  IridiumString += lowByte(uint16_t(iMet_p*100.0));
  IridiumString += highByte(uint16_t((iMet_t+273.15)*100.0));
  IridiumString += lowByte(uint16_t((iMet_t+273.15)*100.0));
  IridiumString += lowByte(uint8_t(iMet_u));

  /*Add the most recent lat/lon/alt */
  /* Latitude + 90.0 then shifted 1e6 to the left and sent as 4 byte int */
  IridiumString += highByte(uint32_t( (lat + 90.0)*1000000.0) & 0xFFFF0000 >> 16);
  IridiumString += lowByte(uint32_t( (lat + 90.0)*1000000.0) & 0xFFFF0000 >> 16);
  IridiumString += highByte(uint32_t( (lat + 90.0)*1000000.0));
  IridiumString += lowByte(uint32_t( (lat + 90.0)*1000000.0));
  /* Longitude + 180.0 then shifted 1e6 to the left and sent as 4 byte int */
  IridiumString += highByte(uint32_t( (lon + 180.0)*1000000.0) & 0xFFFF0000 >> 16);
  IridiumString += lowByte(uint32_t( (lon + 180.0)*1000000.0) & 0xFFFF0000 >> 16);
  IridiumString += highByte(uint32_t( (lon + 180.0)*1000000.0));
  IridiumString += lowByte(uint32_t( (lon + 180.0)*1000000.0));
  /*Altitude in meters sent as a 2 byte in 0 - 65535 meters */
  IridiumString += highByte(uint16_t( alt));
  IridiumString += lowByte(uint16_t(alt));

  #ifdef DEBUG
  DEBUG_SERIAL.print("Iridium String: ");
  DEBUG_SERIAL.println(IridiumString);
  #endif
}

/*
 * Generic send iridium function - for now it just prints to console.
 * Need to make sure this is either a non-blocking function or include
 * checkForSerial() within this function so we don't miss serial packets.
 * Also want a timeout for this - if we go more than (PACKETS_TO_SEND - 1) * 2.0
 * seconds, give up on that packet and move to the next one. 
 */
int SendIridium()
{
  IridiumString += uint8_t(BeaconBatteryV *10.0); //Iridium beacon battery voltage 0 - 25.5V
  IridiumString += uint8_t(BeaconT + 100.0);       //Iridium beacon internal temp -100 - 155C 
  #ifdef DEBUG
  DEBUG_SERIAL.print("Sending Iridium String: ");
  DEBUG_SERIAL.println(IridiumString);
  #endif

  IridiumString = ""; //Clear the iridium string. 
  return 0;
}


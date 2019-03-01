#include <SD.h>
#define INSTRUMENT_SERIAL Serial1
#define DEBUG_SERIAL Serial
String Version = "Fake Instrument V0.1";
String DataToTX; //string for outgoing data transmit through INSTRUMENT_SERIAL
String DataToRX; //string for Incoming data to write to SD card

bool RXDataAvailable = false;
const int chipSelect = BUILTIN_SDCARD; 
long StartTime = 0;
int recNum = 0;

File SendFile; //File pointer for data to send

void setup() {
DEBUG_SERIAL.begin(9600); //USB serial
INSTRUMENT_SERIAL.begin(9600);  //Hardware serial at 9600baud
delay(1000);
DEBUG_SERIAL.println(Version);

 if (!SD.begin(chipSelect)) {
    DEBUG_SERIAL.println("Card failed, or not present");
    // don't do anything more:
    
  }
 
  DEBUG_SERIAL.println("card initialized.");
  StartTime = millis();
  OpenFile("OUTGOING.txt");
}

void loop() {
  if(millis() - StartTime > 1000)  //1 second has elapsed
    {
      StartTime = millis();
      WriteToSD(DataToRX); // Write whatever data we have to SD Card
      DataToRX = ""; //Clear the String 
      ReadFromSDandSend();
    }
 checkForSerial();

}

void WriteToSD(String toWrite)
{
  /* Writes a string to the SD Card*/
  DEBUG_SERIAL.print("To Write to SD: ");
  DEBUG_SERIAL.println(toWrite);
  
  checkForSerial();
  File dataFile = SD.open("INCOMING.txt", FILE_WRITE);
  checkForSerial();
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.print(toWrite);
    dataFile.close();
  }  
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening INCOMING.txt");
  } 
}

int ReadFromSDandSend(void)
{
  String list;
  if (SendFile.available())
 {
    list = SendFile.readStringUntil('\n');
    DEBUG_SERIAL.print("Sending to Instrument Serial: ");
    DEBUG_SERIAL.print(list);
    recNum++; // Count the record
    INSTRUMENT_SERIAL.println(list);
    return 1;
 }
  else{
      DEBUG_SERIAL.println("Nothing to send");
      DEBUG_SERIAL.println(list.length());
      SendFile.close();
      OpenFile("OUTGOING.txt");
      return 0;
  }

}


int OpenFile(char Filename[])
{
  SendFile = SD.open(Filename, FILE_READ);
  delay(500);
  if(SendFile)
  {
    Serial.println("File is open");
    return 1;
  }else{
    Serial.println("Error opening file");
    return 0;
  }
}

void checkForSerial()
{
  /* Checks for any new bytes on the various serial ports
   *  If there is a complete data string 
   */
   
  if (INSTRUMENT_SERIAL.available()) {
   char OPC_Char = INSTRUMENT_SERIAL.read();
   //Serial.write(OPC_Char);
   
   
    DataToRX += OPC_Char;
  }


}

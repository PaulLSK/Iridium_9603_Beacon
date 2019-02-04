// Serial2 pin and pad definitions (in Arduino files Variant.h & Variant.cpp)
#define PIN_SERIAL2_RX       (34ul)               // Pin description number for PIO_SERCOM on D12 (Physical Pin 28)
#define PIN_SERIAL2_TX       (36ul)               // Pin description number for PIO_SERCOM on D10 (Physical Pin 27)
#define PAD_SERIAL2_TX       (UART_TX_PAD_2)      // SERCOM1 pad 2 (SC1PAD2)
#define PAD_SERIAL2_RX       (SERCOM_RX_PAD_3)    // SERCOM1 pad 3 (SC1PAD3)
// Instantiate the Serial2 class
Uart Serial2(&sercom1, PIN_SERIAL2_RX, PIN_SERIAL2_TX, PAD_SERIAL2_RX, PAD_SERIAL2_TX);
HardwareSerial &ssFakeGPS(Serial2);


// SERCOM4 -> Serial3 -> iMET
// Serial3 pin and pad definitions (in Arduino files Variant.h & Variant.cpp)
// iMET Tx (input) is connected to MOSI (Digital Pin 23, Port B Pin 10, SERCOM4 Pad 2, Serial3 Tx)
// iMET Rx (output) is connected to SCK (Digital Pin 24, Port B Pin 11, SERCOM4 Pad 3, Serial3 Rx)
#define PIN_SERIAL3_RX       (24ul)               // Pin description number for PIO_SERCOM on D24
#define PIN_SERIAL3_TX       (23ul)               // Pin description number for PIO_SERCOM on D23
#define PAD_SERIAL3_TX       (UART_TX_PAD_2)      // SERCOM4 Pad 2 (SC4PAD2)
#define PAD_SERIAL3_RX       (SERCOM_RX_PAD_3)    // SERCOM4 Pad 3 (SC4PAD3)
// Instantiate the Serial3 class
Uart Serial3(&sercom4, PIN_SERIAL3_RX, PIN_SERIAL3_TX, PAD_SERIAL3_RX, PAD_SERIAL3_TX);
HardwareSerial &ssInst2(Serial3);


// Interrupt handler for SERCOM1 (essential for Serial2 comms)
void SERCOM1_Handler()
{
  Serial2.IrqHandler();
}

// Interrupt handler for SERCOM4 (essential for Serial3 comms)
void SERCOM4_Handler()
{
  Serial3.IrqHandler();
}


void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);
  ssInst2.begin(9600);
  ssFakeGPS.begin(9600);

}

int count = 0;

void loop() {
  // put your main code here, to run repeatedly:

#define MAX_MILLIS_TO_WAIT 1000  //or whatever

  char outBuffer[120];
  char outBuffer1[120];
  unsigned long starttime;
  String iMET_Xdata = "";
  //char iMET_Xdata[120] = "";
  //int char_loc = 0;
  //bool datafound = false;
  
  starttime = millis();

/*
  while (Serial1.available() > 0) {
    iMET_Xdata[char_loc] = Serial1.read();
    char_loc++;
    datafound = true;
  }

  if (datafound) {
    Serial.print("[INFO : start_iMET] RECEIVED from iMET: "); Serial.println(iMET_Xdata);
    memset(iMET_Xdata, 0, sizeof(iMET_Xdata));
    datafound = false;
  } else {
    Serial.println("[ERROR : start_iMET] iMET instrument NOT connected");
  }
*/

  if (Serial1.available() > 0) {
    iMET_Xdata = Serial1.readStringUntil('\n');
    Serial.print("[INFO : start_iMET] RECEIVED from iMET: "); Serial.println(iMET_Xdata);
  } else {
    Serial.println("[ERROR : start_iMET] iMET instrument NOT connected");
  }  



  sprintf(outBuffer, "xdata=0101%013d\r\n", count);
  //Serial.print("SENDING to iMET: "); Serial.println(outBuffer);
  Serial1.write(outBuffer);

  
  sprintf(outBuffer1, "xdata=0202%013d\r\n", count);
  //Serial.print("SENDING to Inst2: "); Serial.println(outBuffer1);
  ssInst2.write(outBuffer1);


  char GGAString[120] = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
  //Serial.print("SENDING to FakeGPS: "); Serial.println(GGAString);
  ssFakeGPS.write(GGAString);

   
  count++;

  delay(1000);
/*  
  while ( (Serial.available() < 9) && ((millis() - starttime) < MAX_MILLIS_TO_WAIT) )
  {
    // hang in this loop until we either get 9 bytes of data or 1 second
    // has gone by
  }
  if (Serial.available() < 9)
  {
    // the data didn't come in - handle that problem here
    Serial.println("ERROR - Didn't get 9 bytes of data!");
  }
  else
  {
    for (int n = 0; n < 9; n++)
      RFin_bytes[n] = Serial.read(); // Then: Get them.
  }
*/
}

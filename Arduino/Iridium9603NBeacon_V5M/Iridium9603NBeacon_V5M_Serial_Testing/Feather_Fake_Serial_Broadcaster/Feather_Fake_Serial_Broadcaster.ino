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


// Interrupt handler for SERCOM4 (essential for Serial3 comms)
void SERCOM4_Handler()
{
  Serial3.IrqHandler();
}


void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);
  ssInst2.begin(9600);

}

int count = 0;

void loop() {
  // put your main code here, to run repeatedly:

#define MAX_MILLIS_TO_WAIT 1000  //or whatever

  char outBuffer[120];
  char outBuffer1[120];
  unsigned long starttime;

  starttime = millis();
  
  sprintf(outBuffer, "xdata=0101%013d\n", count);
  Serial.print("SENDING to iMET: "); Serial.println(outBuffer);
  Serial1.write(outBuffer);

  
  sprintf(outBuffer1, "xdata=0202%013d\n", count);
  Serial.print("SENDING to Inst2: "); Serial.println(outBuffer1);
  ssInst2.write(outBuffer1);

  
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

// Feather9x_RX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (receiver)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Feather9x_TX
//We have made some simple range tests under the following conditions:
//
//    rf95_client base station connected to a VHF discone antenna at 8m height above ground
//    rf95_server mobile connected to 17.3cm 1/4 wavelength antenna at 1m height, no ground plane.
//    Both configured for 13dBm, 434MHz, Bw = 125 kHz, Cr = 4/8, Sf = 4096chips/symbol, CRC on. Slow+long range
//    Minimum reported RSSI seen for successful comms was about -91
//    Range over flat ground through heavy trees and vegetation approx 2km.
//    At 20dBm (100mW) otherwise identical conditions approx 3km.
//    At 20dBm, along salt water flat sandy beach, 3.2km.





#include <SPI.h>
#include <RH_RF95.h>



// for Feather32u4 RFM9x
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7


// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Blinky on receipt
#define LED 13

void setup()
{
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

//  Serial.begin(115200);
//  while (!Serial) {
//   delay(1);
// }
// delay(100);  

//  Serial.println("Feather LoRa RX Test!");

//  gps.begin(9600); // GPS NMEA
  Serial1.begin(115200);
 
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
//    Serial.println("LoRa radio init failed");
//    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
//  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
 //   Serial.println("setFrequency failed");
    while (1);
  }
//  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
  rf95.setModemConfig(RH_RF95::Bw125Cr48Sf4096);
 // rf95.setModemConfig(RH_RF95::Bw125Cr45Sf128);
}

void loop()
{

     
  if (rf95.available())
  {
    // Should be a message for us now
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len))
    {
      digitalWrite(LED, HIGH);
 //     RH_RF95::printBuffer("Received: ", buf, len);

      Serial1.print((char*)buf);
      Serial1.print(",");
      Serial1.print(rf95.lastRssi(),DEC);      

      Serial1.print(",");
      Serial1.print("@");
      digitalWrite(LED, LOW);


      
    }
    else
    {
//      Serial.println("Receive failed");
    }
  }
}

// Feather9x_TX
//
// due to deep sleep in code  usb not work so canot update code via arduino IDE
// so for update need to push 2 time quicly on reset buoton for enter bootlaoder mode 
//
#include <SPI.h>
#include <RH_RF95.h>
#include "ArduinoLowPower.h"


 // Feather M0 w/Radio
  #define RFM95_CS          6
  #define RFM95_INT         13   //19-
  #define RFM95_RST         12

 
  #define VBAT_read         A2
  #define maxbotix_Annalog  A6
  #define Temperature_Read  A7  
  
  #define maxbotix_ON       1
  #define maxbotix_Trig     4
  #define maxbotix_Pulse    10 

  #define Temperature_SHDN  3 
  #define RS485_DIR         18       
  #define reed_SW           17
  #define RS485_POWER       15  
  #define no_sleep          13          
  


// Change to 440.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

long distance = 0;
long duration = 0;
float measuredvbat = 0;
float measuredvtemp = 0;

void setup() 
{
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  pinMode(maxbotix_Pulse, INPUT);
  pinMode(maxbotix_Trig, OUTPUT);
  pinMode(maxbotix_ON, OUTPUT);
  pinMode(RS485_POWER, OUTPUT);

  delay(100);

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  rf95.init();  
  rf95.setFrequency(RF95_FREQ); 
  rf95.setTxPower(23, false); 
  rf95.setModemConfig(RH_RF95::Bw125Cr48Sf4096); //slow burst but long range  
 // rf95.setModemConfig(RH_RF95::Bw125Cr45Sf128);  //fast burst but low range

 delay(20000);
}

int16_t packetnum = 0;  // packet counter, we increment per xmission


void read_sensor() {
  digitalWrite(maxbotix_ON, HIGH);  //put maxbotix on
  delay(200);
  digitalWrite(maxbotix_Trig, HIGH); // pulse trigger pin 
  delayMicroseconds(20);
  digitalWrite(maxbotix_Trig, LOW);
  duration = pulseIn(maxbotix_Pulse, HIGH); // read pulse
//  distance = duration / 58; // sensor > 7.5m
  distance = duration;   // sensor 1mm
  delay(150);
  digitalWrite(maxbotix_ON, LOW);  //put maxbotix off
}

void read_temperature() {
  digitalWrite(Temperature_SHDN, HIGH);  //put TMP36 chip on
  delay(10);
  measuredvtemp = analogRead(Temperature_Read);
  measuredvtemp *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvtemp /= 1024; // convert to voltage
 measuredvtemp = (measuredvtemp - 0.50) * 100; // convert to temperature 
//    measuredvtemp = 500;
  digitalWrite(Temperature_SHDN, LOW);  //put TMP36 chip off
}

void read_VBatt() {
  digitalWrite(RS485_POWER, HIGH);  //put TMP36 chip on
  delay(20);
  measuredvbat = analogRead(VBAT_read );
//  measuredvbat = 503; 
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  digitalWrite(RS485_POWER, LOW);  //put TMP36 chip off
}





void loop()
{

 read_sensor(); 
 read_VBatt();
 read_temperature(); 
 
 
 String 
  radiopacket = String(distance);
  radiopacket += ","; 
  radiopacket += String(measuredvtemp);
  radiopacket += ",";
  radiopacket += String(measuredvbat);
  radiopacket += ",";  
  radiopacket += ",";  // spare turbidity
  radiopacket += ",";  // spare presure kpa
  radiopacket += ",";  // spare mag_sw 
  radiopacket += "SensorID,";
  radiopacket += String(packetnum++);
  
  delay(10);
  rf95.send((uint8_t*)radiopacket.c_str(), radiopacket.length()+1); 
  delay(10);
  rf95.waitPacketSent(); // packet send so now go sleep 
    
  rf95.sleep();
 // LowPower.deepSleep(900000); //900000 = 15 minute
 LowPower.deepSleep(900000); //300000 = 5 minute
//  LowPower.deepSleep(5000); //5000 = 5 segonde
// delay(2000);
// digitalWrite(maxbotix_ON, LOW);  //put maxbotix on
// digitalWrite(RS485_POWER, HIGH);  //put maxbotix on
// delay(2000);
// digitalWrite(maxbotix_ON, HIGH); // pulse trigger pin 
// digitalWrite(RS485_POWER, LOW);  //put maxbotix on

}

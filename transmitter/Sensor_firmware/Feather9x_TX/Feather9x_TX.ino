/**
 * @file Feather9x_TX.ino
 * @author Sebastien Leclerc 
 * @brief This program is used to take measurements (including one of a distance sensor )
 *        and transmit it using a Lora Radio
 * @version 0.1
 * @date 2024-06-04
 * 
 * @copyright Copyright (c) 2024
 *
 * MIT License
 * 
 * Copyright (c) 2024 Sebastien Leclerc
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ***/


/*
 * Prerequisite libraries to be installed:
 *   Arduino Low Power by Adruino, >= v1.2.2 
 *   RadioHead  by Mike McCauley, >= v1.122.1 
 * 
 * Board manager:
 *   Adafruit Feather M0 (SAMD21) 
 *
 * Review the settings in the "Configuration.h" file and make sure that the 
 * radio settings (frequency, modulation, etc) match with the receiver module's
 * compile definitions.
 *
 * This version is a work in progress, including several portions used for troubleshooting
 * as well as debugging with the objective of reducing the power usage to the minimum achievable
 * by the electronics of the board. 
 * 
 *
 * Uploading:
 *   By default, the usb serial device used for uploading the code is disabled and the Arduino
 * IDE cannot flash binaries. The bootloader used in this project requires double-pressing
 * the button on the board to enable the upload port.
 * 
 **/

#include "ArduinoLowPower.h"
#include "Configuration.h"
#include "Definitions.h"
#include <RH_RF95.h>
#include <SPI.h>
#include <LoraEncoder.h>
#include <FlashStorage_SAMD.h>

// #ifdef SERIAL_OFF
// #include <Analog2DigitalConverter.h>
// #include <System.h>
// #include <TimerCounter.h>
// using namespace SAMD21LPE;
// Analog2DigitalConverter& adcTemperature = Analog2DigitalConverter::instance();
// Analog2DigitalConverter& adcVoltage = Analog2DigitalConverter::instance();
// Analog2DigitalConverter& adcInternalVoltage = Analog2DigitalConverter::instance();
// Analog2DigitalConverter& adcInternalTemp = Analog2DigitalConverter::instance();
// const byte GCLKGEN_ID_1K = 6;
// const byte GCLKGEN_ID_ADC_1MHz = 4;
// #define GCLKGEN_ID_ADC_1MHz_FREQ 1000000
// TimerCounter timer;
// #endif


globals_t ctx = { { 0, 0, 0.0f, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0 }, false, 0, 0, false, false, false, BOD33_SLEEP_MULTIPLIER };

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);



// Define your reference voltage array
float vref[] = {
  3.3f,   // AR_DEFAULT
  1.0f,   // AR_INTERNAL1V0
  1.1f,   // AR_INTERNAL1V1
  1.2f,   // AR_INTERNAL1V2
  1.25f,  // AR_INTERNAL1V25
  2.0f,   // AR_INTERNAL2V0
  2.2f,   // AR_INTERNAL2V2
  2.23f,  // AR_INTERNAL2V23
  2.4f,   // AR_INTERNAL2V4
  2.5f,   // AR_INTERNAL2V5
  1.65f,  // AR_INTERNAL1V65
  -1.0f   // AR_EXTERNAL Placeholder (you need to set this based on your external Vref)
};

void read_vbat() {

  // if (ctx.next_volt_read < millis()) {
  Serial.println("Reading VBAT");
  // throttle voltage reading

  pinMode(VBAT_PIN, INPUT);
  pinMode(RS485_POWER, OUTPUT);     // Set RS485_POWER pin as digital input
  digitalWrite(RS485_POWER, HIGH);  // put TMP36 chip on
  wait_ms(100);
  // #ifdef SERIAL_OFF
  //   ctx.measures.bat_voltage = adcVoltage.read(g_APinDescription[VBAT_PIN].ulADCChannelNumber);

  //   // ctx.measures.rawVbat = static_cast<uint32_t>(tot_vbat / vbat_reads);
  //   digitalWrite(RS485_POWER, LOW);  // put TMP36 chip off
  //                                    // voltage divitor divides by 2
  //                                    // ctx.measures.bat_voltage =
  //   //     static_cast<float>(ctx.measures.rawVbat) * 2.0f * 3.3f / 1024.0f;

  //   ctx.measures.internal_voltage = adcInternalVoltage.read(ADC_INPUTCTRL_MUXPOS_BANDGAP_Val);
  //   adcVoltage.disable();
  //   ctx.measures.core_voltage = adcInternalVoltage.read(ADC_INPUTCTRL_MUXPOS_SCALEDCOREVCC_Val);
  //   adcInternalVoltage.disable();
  // #endif

  // #ifndef SERIAL_OFF
  analogReference(AR_DEFAULT);
  ctx.measures.bat_voltage = (float)analogRead(VBAT_PIN) * 2.0f * 3.3f / 1024;
  Serial.print("VBAT: ");
  Serial.println(ctx.measures.bat_voltage, 3);
  Serial.print("CORE V: ");
  Serial.println(ctx.measures.internal_voltage, 3);
  Serial.print("INT V: ");
  Serial.println(ctx.measures.core_voltage, 3);

  // #endif


  // flash_vbat_value();
  //  todo: fix under power
  //          if (ctx.measures.bat_voltage < VOLTAGE_MIN_THRESHOLD) {
  //              ctx.insufficient_power = true;
  //              flash_D3(2, 200, 100);
  //      Serial.println("Insufficient voltage detected");

  //         } else {
  //     Serial.println("Sufficient voltage detected");
  //             ctx.insufficient_power = false;
  //             flash_D3(3, 200, 100);
  //         }
  //     ctx.next_volt_read = millis() + VOLTAGE_READ_DELAY_MS;
  // }
}
uint64_t readUniqueID64() {
  uint32_t* uid = (uint32_t*)0x0080A00C;
  return ((uint64_t)uid[0] << 32) | uid[1];
}

// void setupADC() {
// #ifdef SERIAL_OFF
//   adcTemperature.enable(
//     GCLKGEN_ID_ADC_1MHz, GCLKGEN_ID_ADC_1MHz_FREQ, Analog2DigitalConverter::DIV4);
//   adcTemperature.setReference(Analog2DigitalConverter::Reference::REF_INTVCC_DIV_2, 1.65, false);
//   adcTemperature.setGain(Analog2DigitalConverter::Gain::GAIN_DIV2);
//   adcTemperature.setSampling(0, 3);
//   adcVoltage.enable(GCLKGEN_ID_ADC_1MHz, GCLKGEN_ID_ADC_1MHz_FREQ, Analog2DigitalConverter::DIV4);
//   adcVoltage.setReference(Analog2DigitalConverter::Reference::REF_INTVCC_DIV_2, 3.3);
//   adcVoltage.setGain(Analog2DigitalConverter::Gain::GAIN_DIV2);
//   adcVoltage.setSampling(0, 3);
//   adcInternalTemp.enable(
//     GCLKGEN_ID_ADC_1MHz, GCLKGEN_ID_ADC_1MHz_FREQ, Analog2DigitalConverter::DIV4);
//   // configure continuous hardware averaging (2^3 = 8)
//   adcInternalTemp.setSampling(0, 3);
//   // disable ADC to save power until start of conversion
//   adcInternalTemp.disable();
//   adcInternalVoltage.enable(
//     GCLKGEN_ID_ADC_1MHz, GCLKGEN_ID_ADC_1MHz_FREQ, Analog2DigitalConverter::DIV4);
//   // configure continuous hardware averaging (2^3 = 8)
//   adcInternalVoltage.setSampling(0, 3);
//   // disable ADC to save power until start of conversion
//   adcInternalVoltage.disable();

//   // disable to save power until we are ready
//   adcVoltage.disable();
//   adcTemperature.disable();

// #endif
// }
// void setupClocks() {
//   // configure low power clock generator
// #ifdef SERIAL_OFF
//   // flash_D3(2, 100, 100);
//   System::setupClockGenOSCULP32K(GCLKGEN_ID_1K, 4);  // 2^(4+1) = 32 -> 1 kHz
//   System::setupClockGenOSC8M(GCLKGEN_ID_ADC_1MHz, 8, false);

//   // configure timer counter to run at 1 kHz and to continue in standby
//   // flash_D3(3, 100, 100);
//   timer.enable(4, GCLKGEN_ID_1K, 1024, TimerCounter::DIV1, TimerCounter::RES16, 1000, true);

//   System::setSleepMode(System::IDLE0);
//   // select sleep mode STANDBY between ISR calls and enable sleep-on-exit mode
//   // flash_D3(2, 100, 100);
//   System::setSleepMode(System::STANDBY);
//   // flash_D3(3, 100, 100);
// #endif
// }
void wait_ms(uint32_t ms) {
  // #ifndef SERIAL_OFF
#pragma message("Can we put to light sleep here?")
  Serial.println("Delay ");
  delay(ms);
  // #else
  //   System::setSleepMode(System::IDLE0);
  //   timer.wait(ms);
  //   System::setSleepMode(System::STANDBY);
  // #endif

  Serial.println("Done delay");
}
void setUsPerCM(double scale) {
  // Save into emulated-EEPROM the number increased by 1 for the next run of the sketch
  Serial.print("Storing scale (us/cm): ");
  Serial.println(scale);
  EEPROM.put(SCALE_ADDRESS, WRITTEN_SIGNATURE);
  EEPROM.put(SCALE_ADDRESS + sizeof(WRITTEN_SIGNATURE), scale);
  if (!EEPROM.getCommitASAP()) {
    Serial.println("CommitASAP not set. Need commit()");
    EEPROM.commit();
  }
}
double getUsPerCM() {

  int signature;
  double result = PULSE_SCALE_MS_PER_CM;
  EEPROM.get(SCALE_ADDRESS, signature);
  if (signature != WRITTEN_SIGNATURE) {
    Serial.println("EEPROM is empty, Setting default" );
    setUsPerCM(PULSE_SCALE_MS_PER_CM);
  } else {
    EEPROM.get(SCALE_ADDRESS + sizeof(WRITTEN_SIGNATURE), result);
  }
  return result;
}


void checkReset() {
    unsigned long startTime = millis();
    pinMode(JP1, INPUT_PULLUP);
    Serial.print("JP1: ");
    Serial.println(digitalRead(JP1)==HIGH?"OFF":"ON");
    while (digitalRead(JP1) == LOW) {  // Wait while the pin is held low
        if (millis() - startTime > 500) {  // Check if the pin is low for more than 500ms
            Serial.println("JP1 held low for 500ms, resetting scale value.");
            setUsPerCM(-1);
            break;
        }
    }
    
}


void setup() {
  // flash_D3(7, 100, 100);
  // #ifndef SERIAL_OFF
  Serial.begin(19200);
  ctx.measures.uuid = readUniqueID64();
  checkReset();
  Serial.print("Starting sensor. UUID: ");
  Serial.println(bytesToHexString((uint8_t*)&ctx.measures.uuid, sizeof(ctx.measures.uuid) / sizeof(uint8_t)));
  // while (!Serial) {
  // }
  // #endif
  //   Serial.print(F("SYST: "));
  //   Serial.println(PM->RCAUSE.bit.SYST);   // System (software) reset
  //   Serial.print(F("WDT: "));
  //   Serial.println(PM->RCAUSE.reg & PM_RCAUSE_WDT);   // Watchdog timer reset (use of WDT
  //   bit causes compilation error) Serial.print(F("EXT: "));
  //   Serial.println(PM->RCAUSE.bit.EXT);    // External (reset button) reset
  //   Serial.print(F("POR: ")); Serial.println(PM->RCAUSE.bit.POR); // Power-on reset
  //   Serial.println();
  // #ifdef SERIAL_OFF
  //   System::reducePowerConsumption();
  //   System::enablePORT();
  //   noInterrupts();
  //   System::enableEIC();
  //   interrupts();
  //   // setupClocks();
  //   // setupADC();
  // #endif
  // set_gpios(true);
  // disable non essential MCU modules (incl. USB)

  // todo: fix Brown out
  // if (PM->RCAUSE.reg & PM_RCAUSE_BOD33) {
  //     while (ctx.bod_sleep_remain-- > 0) {
  //         flash_D3(ctx.bod_sleep_remain, 50, 150);
  //         do_sleep(SLEEP_TIME_LOW_POWER_MS);
  //     }
  //     // reset sleep multiplier
  //     ctx.bod_sleep_remain = BOD33_SLEEP_MULTIPLIER;
  // }
  // read_distance();

  // init radio and put it to sleep
  init_radio(true);
  // todo: fix brownout
  // set_brownout(false, SYSCTRL_BOD33_LEVEL_3V07);

  // todo: fix brown-out
  // while (is_under_voltage(200)) {
  //     flash_D3(10, 100, 100);
  //     do_sleep(SLEEP_TIME_LOW_POWER_MS);
  // }
  // activate brownout so it resets the system if a power
  // surge pushes the voltage below a certain value

  // todo: fix brownout
  // set_brownout(true, SYSCTRL_BOD33_LEVEL_3V07);
  ctx.keep_awake_until = millis() + KEEP_AWAKE_TOTAL_TIME_MS;
}

void loop() {
  read_vbat();
  read_temperature();
  read_distance();
  if (!ctx.insufficient_power && !ctx.measures_sent) {
    // flash_D3(4, 200, 50);

    send_data();
    // flash_D3(5, 200, 50);
  }
  // todo: fix under voltage behavior
  // if (ctx.insufficient_power) {
  //     do_sleep(SLEEP_TIME_LOW_POWER_MS);
  // } else if (isBoardReadyToSleep()) {
  //     do_sleep(SLEEP_TIME_REGULAR_MS);
  // }
  // flash_D3(6, 100, 100);

  // todo: fix under voltage behavior
  // if (ctx.insufficient_power) {
  //     do_sleep(SLEEP_TIME_LOW_POWER_MS);
  // } else if (isBoardReadyToSleep()) {
  do_sleep(SLEEP_TIME_SEC * 1000);
}

/************************************************************
 * Power handling
 ************************************************************/
void wait_for_power(uint32_t timeout_ms) {
  /* Disable the brown-out detector during configuration,
   otherwise it might misbehave and reset the
   microcontroller. */
  SYSCTRL->BOD33.bit.ENABLE = 0;
  while (!SYSCTRL->PCLKSR.bit.B33SRDY) {
  };

  /* Configure the brown-out detector so that the
       program can use it to watch the power supply
       voltage */
  SYSCTRL->BOD33.reg = (
    /* This sets the minimum voltage level to 3.0v - 3.2v.
           See datasheet table 37-21. */
    SYSCTRL_BOD33_LEVEL(39) |
    /* Since the program is waiting for the voltage to rise,
           don't reset the microcontroller if the voltage is too
           low. */
    SYSCTRL_BOD33_ACTION_NONE |
    /* Enable hysteresis to better deal with noisy power
           supplies and voltage transients. */
    SYSCTRL_BOD33_HYST);

  /* Enable the brown-out detector and then wait for
       the voltage level to settle. */
  SYSCTRL->BOD33.bit.ENABLE = 1;
}
bool is_under_voltage(uint32_t timeout_ms) {
  uint32_t wait_until = timeout_ms + millis();

  /* BOD33DET is set when the voltage is *too low* */
  while (millis() > wait_until && SYSCTRL->PCLKSR.bit.BOD33DET) {
  }
  // if BOD33 is set, then we're still under voltage
  return SYSCTRL->PCLKSR.bit.BOD33DET;
}
void set_brownout(bool enable, int level) {
  /* Let the brown-out detector automatically reset the microcontroller
   if the voltage drops too low. */
  SYSCTRL->BOD33.bit.ENABLE = 0;
  while (!SYSCTRL->PCLKSR.bit.B33SRDY) {
  };

  SYSCTRL->BOD33.reg =
    (SYSCTRL_BOD33_LEVEL(level) | enable ? SYSCTRL_BOD33_ACTION_RESET
                                         : SYSCTRL_BOD33_ACTION_NONE | SYSCTRL_BOD33_HYST);
  SYSCTRL->BOD33.bit.ENABLE = 1;

  // wait_ms until brown-out is ready
  while (!SYSCTRL->PCLKSR.bit.BOD33RDY) {
  }
}
void do_sleep(int sleep_time) {
#ifdef DEEP_SLEEP_ENABLE
  Serial.println("Peripheral sleep");
  SPI.end();  // shutdown the SPI interface
  set_gpios(false);
  init_radio(false);
  Serial.print("All ready for sleep for ");
  Serial.print(sleep_time);
  Serial.println("ms");
  Serial.flush();


  LowPower.deepSleep(sleep_time);  // 5000 = 5 Secondes
                                   // #else
                                   //   timer.wait(sleep_time);
                                   // #endif
  // Re-enable USB (should never get there because no wakeup interrupt is configured)
  Serial.println("Waking up from sleep");
  set_gpios(true);
  init_radio(true);
  // don't bring back serial
  Serial.println("Done sleeping");


#else
  Serial.printf("Sleep disabled. Pausing for %ums", sleep_time);
  Serial.println("\n\n\n===============");
  delay(sleep_time);
#endif
  // flash_D3(8, 100, 100);
  ctx.next_volt_read = 0;
  ctx.measures_sent = false;
}

bool isBoardReadyToSleep() {

  if (!ctx.first_boot_complete && millis() > ctx.keep_awake_until) {
    ctx.first_boot_complete = true;
    Serial.println("Board Initialized");
  }
  return ctx.first_boot_complete;
}

/************************************************************
 * Utilities
 ************************************************************/
void flash_power(int pin, int times, int on_time, int ratio_pct, bool invert) {

  int state = digitalRead(pin);
  pinMode(pin, OUTPUT);
  digitalWrite(pin, invert ? HIGH : LOW);
  delay(700);
  for (int i = 0; i < times; i++) {
    digitalWrite(pin, invert ? LOW : HIGH);  // put TMP36 chip on
    // LowPower.idle((on_time);
    delay(on_time);
    digitalWrite(pin, invert ? HIGH : LOW);  // put TMP36 chip on
    delay(on_time * ratio_pct / 100);

    // LowPower.idle((on_time*ratio_pct/100);
  }
  delay(on_time * 1.5);
  // digitalWrite(pin, state);
}
void flash_D3(int times, int on_time, int ratio_pct, bool invert) {
  flash_power(RS485_POWER, times, on_time, ratio_pct, false);
}
// void flash_vbat_value() {
//     flash_D3(1, 1500, 0);
//     flash_D3(trunc(ctx.measures.bat_voltage), 500, 100, false);
//     flash_D3(
//         (int)((ctx.measures.bat_voltage - trunc(ctx.measures.bat_voltage)) * 10), 500, 50, false);
//     flash_D3(1, 1500, 0);
// }

/************************************************************
 * Peripheral config
 *************************************************************/
bool init_radio(bool enable) {
  static bool initialized = false;
  Serial.println("Initializing Radio");
  if (enable && !initialized) {
    initialized = true;
    Serial.println("Activating radio power and resetting");
    Serial.flush();
    pinMode(RFM95_RST, INPUT);
    digitalWrite(RFM95_RST, HIGH);
    wait_ms(100);

    // manual reset
    digitalWrite(RFM95_RST, LOW);
    wait_ms(10);
    digitalWrite(RFM95_RST, HIGH);
    wait_ms(10);
    Serial.println("calling init");
    Serial.flush();
    rf95.init();
    rf95.setFrequency(RF95_FREQ);
    rf95.setTxPower(23, false);
    rf95.setModemConfig(RF95_MODE);
    Serial.println("Setting up Radio GPIOs");
    Serial.flush();
    rf95.spiWrite(
      RH_RF95_REG_41_DIO_MAPPING2, RH_RF95_REG_41_DIO_4(RH_RF95_REG_41_DIO_4_DISABLE) | RH_RF95_REG_41_DIO_5(RH_RF95_REG_41_DIO_5_DISABLE));
    rf95.spiWrite(
      RH_RF95_REG_40_DIO_MAPPING1, RH_RF95_REG_40_DIO_1(RH_RF95_REG_40_DIO_1_DISABLE) | RH_RF95_REG_40_DIO_2(RH_RF95_REG_40_DIO_2_DISABLE) | RH_RF95_REG_40_DIO_3(RH_RF95_REG_40_DIO_3_DISABLE));
    Serial.println("Putting radio to sleep");
    Serial.flush();

    rf95.sleep();
    // rf95.setModemConfig(RH_RF95::Bw125Cr45Sf128);  //fast burst but low range
    Serial.println("Done Initializing Radio");
  } else if (!enable && initialized) {
    initialized = false;
    pinMode(SCK, INPUT);
    digitalWrite(SCK, LOW);  // shut off pullup resistor
    pinMode(MOSI, INPUT_PULLDOWN);
    digitalWrite(MOSI, LOW);  // shut off pullup resistor
    pinMode(MISO, INPUT);
    digitalWrite(MISO, LOW);  // shut off pullup resistor

    pinMode(RFM95_CS, INPUT);
    digitalWrite(RFM95_CS, LOW);  // shut off pullup resistor
    pinMode(RFM95_INT, INPUT);
    digitalWrite(RFM95_INT, LOW);  // shut off pullup resistor
    pinMode(RFM95_RST, INPUT);
    digitalWrite(RFM95_RST, LOW);  // shut off pullup resistor
  }
  Serial.println("Done Initializing Radio");
  return initialized;
}

void set_gpios(bool enable) {
  ctx.gpios_enabled = configurePins(enable);
  if (enable) {
    SPI.begin();  // this restarts SPI
  }
}
String bytesToHexString(uint8_t* buf, unsigned int len) {
  String str = "";
  for (unsigned int i = 0; i < len; ++i) {
    if (buf[i] < 0x10) str += "0";  // Pad with zero for single digit hex values
    str += String(buf[i], HEX) + ":";
  }
  return str.substring(0, str.length() - 1);  // Remove the last colon
}
/************************************************************
 * Sensor read
 ************************************************************/
void send_data() {

  if (ctx.measures_sent) {
    return;
  }

  Serial.print("Measuring and sending data. Data Length: ");
  Serial.println(sizeof(ctx.measures));
  ctx.measures.uuid = readUniqueID64();

  if (!rf95.send((uint8_t*)&ctx.measures, sizeof(ctx.measures))) {
  } else {
  }
  ctx.measures.packetnum++;

  wait_ms(10);
  Serial.println("Waiting for send");
  rf95.waitPacketSent();  // packet send so now go sleep
  rf95.sleep();
  Serial.println("Done Sending data");

  // flash_D3(4, 100, 50);
  Serial.println("Done Sending data. Radio sleep");
  ctx.measures_sent = true;
}

/***
 * Distance
 */
void read_distance() {
  Serial.println("Reading Distance");
  pinMode(MBX_ANALOG, INPUT);
  pinMode(MBX_PULSE, INPUT);
  pinMode(MBX_TRIG, OUTPUT);
  digitalWrite(MBX_TRIG, LOW);
  pinMode(MBX_PWR_PL, OUTPUT);
  digitalWrite(MBX_PWR_PL, HIGH);  // put maxbotix on
  delay(200);                      //  wait for capacitor to charge
  digitalWrite(MBX_TRIG, HIGH);    // pulse trigger pin
  // read a single pulse to check that we're all good
  uint32_t wait_count = 0;
  uint32_t pulse_count = 0;
  while (digitalRead(MBX_PULSE) == LOW) { wait_count++; }
  while (digitalRead(MBX_PULSE) == HIGH) { pulse_count++; }

  // now do the measurement
  ctx.measures.duration = pulseIn(MBX_PULSE, HIGH, 2 * (1000 * 1000));
  digitalWrite(MBX_TRIG, LOW);
  // Serial.printf("Wait count: %u. Pulse count: %u\r\n", wait_count,pulse_count);

  double scale_per_cm = getUsPerCM();
  if (scale_per_cm < 0) {
    // calibrating for <= 1M
    if (DISTANCE_FROM_PULSE_MM(ctx.measures.duration, 10.0) <= 1200) {
      scale_per_cm = 10.0;
      setUsPerCM(scale_per_cm);
    } else if (DISTANCE_FROM_PULSE_MM(ctx.measures.duration, 58.0) <= 1200) {
      scale_per_cm = 58.0;
      setUsPerCM(scale_per_cm);
    } else if (DISTANCE_FROM_PULSE_MM(ctx.measures.duration, (147.0 / 2.54)) <= 1200) {
      scale_per_cm = 147.0 / 2.54;
      setUsPerCM(scale_per_cm);
    }
  }
  ctx.measures.distance = DISTANCE_FROM_PULSE_MM(ctx.measures.duration, scale_per_cm);
  digitalWrite(MBX_PWR_PL, LOW);  // put maxbotix off

  Serial.printf("Duration: %uuS. Distance: %umm", ctx.measures.duration, ctx.measures.distance);
  Serial.println();
}

void read_temperature() {
  Serial.println("Reading Temperature");
  // flash_D3(7, 100, 50);
  pinMode(TEMP_READ, INPUT);
  pinMode(TEMP_SHDN, OUTPUT);
  digitalWrite(TEMP_SHDN, HIGH);  // put TMP36 chip on

  wait_ms(10);
  ctx.measures.temperature = (((float)analogRead(TEMP_READ) * 3.3f / 1024.0f) - 0.5f) * 100.0f;
  //   #ifdef SERIAL_OFF
  //   ctx.measures.temperature = adcTemperature.read(g_APinDescription[TEMP_READ].ulADCChannelNumber);
  //   // ctx.measures.temperature = analogRead(TEMP_READ);
  //   // ctx.measures.temperature *= 3.3;  // Multiply by 3.3V, our reference voltage
  //   // ctx.measures.temperature /= 1024; // convert to voltage
  //   ctx.measures.temperature = (ctx.measures.temperature - 0.50) * 100;  // convert to temperature
  //   adcTemperature.disable();

  //   ctx.measures.internal_temp = adcInternalTemp.read(ADC_INPUTCTRL_MUXPOS_TEMP_Val);
  //   adcInternalTemp.disable();
  // #endif
  //    temperature = 500;
  digitalWrite(TEMP_SHDN, LOW);  // put TMP36 chip off
  // flash_D3(8, 100, 50);

  Serial.print("TEMP: ");
  Serial.print(ctx.measures.temperature, 2);
  Serial.println();

  Serial.print("INT TEMP: ");
  Serial.print(ctx.measures.internal_temp, 2);
  Serial.println();

  Serial.println("Done Reading Temperature");
}
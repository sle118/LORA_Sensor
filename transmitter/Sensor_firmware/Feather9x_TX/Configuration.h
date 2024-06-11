/**
 * @file Configuration.h
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
#pragma once

// uncomment to enable serial debug output. This also sets the sleep time to 
// SLEEP_TIME_DEBUGGIG which should in theory be smaller than the non-debugging
// sleep time in the field
// #define SERIAL_OFF

// sleep time when SERIAL_OFF is commented out for debugging 
#define SLEEP_TIME_DEBUGGING_SEC 30


// Sleep time when battery level is sufficient to wake up and
// take measurements
#define SLEEP_TIME_REGULAR_SECONDS 15*60



#ifdef DEEP_SLEEP_ENABLE
#define SLEEP_TIME_SEC SLEEP_TIME_REGULAR_SECONDS
#else
#define SLEEP_TIME_SEC SLEEP_TIME_DEBUGGING_SEC
#endif


// Sleep time when battery level is below threshold
#define SLEEP_TIME_LOW_POWER_MS 20000
#define VOLTAGE_MIN_THRESHOLD 2.7

// Delay between each voltage reads 
#define VOLTAGE_READ_DELAY_MS 500

// Change to 440.0 or other frequency, must match RX's freq!
// #define RF95_FREQ 915.0
#define RF95_FREQ 433.0
// Bw125Cr48Sf4096 has slow burst but long range
#define RF95_MODE RH_RF95::Bw125Cr48Sf4096


// Sleep delay multiplier on BOD 
#define BOD33_SLEEP_MULTIPLIER 10



// RF95 registers

#define RH_RF95_REG_40_DIO_0_pos 6
#define RH_RF95_REG_40_DIO_1_pos 4
#define RH_RF95_REG_40_DIO_2_pos 2
#define RH_RF95_REG_40_DIO_3_pos 0
#define RH_RF95_REG_41_DIO_4_pos 6
#define RH_RF95_REG_41_DIO_5_pos 4

#define RH_RF95_REG_40_DIO_0_RXDONE 0b00
#define RH_RF95_REG_40_DIO_1_RXTIMEOUT 0b00
#define RH_RF95_REG_40_DIO_2_FhssChangeChannel_1 0b00
#define RH_RF95_REG_40_DIO_3_CadDone 0b00
#define RH_RF95_REG_41_DIO_4_CadDetected 0b00
#define RH_RF95_REG_41_DIO_5_ModeReady 0b00

#define RH_RF95_REG_40_DIO_0_TXDONE 0b01
#define RH_RF95_REG_40_DIO_1_FhssChangeChannel 0b01
#define RH_RF95_REG_40_DIO_2_FhssChangeChannel_2 0b01
#define RH_RF95_REG_40_DIO_3_ValidHeader 0b01
#define RH_RF95_REG_41_DIO_4_PllLock 0b01
#define RH_RF95_REG_41_DIO_5_ClkOut 0b01

#define RH_RF95_REG_40_DIO_0_CADDONE 0b10
#define RH_RF95_REG_40_DIO_1_CadDetected 0b10
#define RH_RF95_REG_40_DIO_2_FhssChangeChannel_3 0b10
#define RH_RF95_REG_40_DIO_3_PayloadCrcError 0b10
#define RH_RF95_REG_41_DIO_4_PllLock2 0b10
#define RH_RF95_REG_41_DIO_5_ClkOut2 0b10

#define RH_RF95_REG_40_DIO_0_DISABLE 0b11
#define RH_RF95_REG_40_DIO_1_DISABLE 0b11
#define RH_RF95_REG_40_DIO_2_DISABLE 0b11
#define RH_RF95_REG_40_DIO_3_DISABLE 0b11
#define RH_RF95_REG_41_DIO_4_DISABLE 0b11
#define RH_RF95_REG_41_DIO_5_DISABLE 0b11

#define RH_RF95_REG_40_DIO_0(val) (val << RH_RF95_REG_40_DIO_0_pos)
#define RH_RF95_REG_40_DIO_1(val) (val << RH_RF95_REG_40_DIO_1_pos)
#define RH_RF95_REG_40_DIO_2(val) (val << RH_RF95_REG_40_DIO_2_pos)
#define RH_RF95_REG_40_DIO_3(val) (val << RH_RF95_REG_40_DIO_3_pos)
#define RH_RF95_REG_41_DIO_4(val) (val << RH_RF95_REG_41_DIO_4_pos)
#define RH_RF95_REG_41_DIO_5(val) (val << RH_RF95_REG_41_DIO_5_pos)



// Define the address in SRAM for triple-tap detection
#define SCALE_ADDRESS                0
#define WRITTEN_SIGNATURE            0xBEEFDEED
#define DISTANCE_FROM_PULSE_MM(pulse_width,scale_us_per_cm) ((uint32_t)((double)pulse_width / scale_us_per_cm) * 10)
#define BOOT_WAIT_FOR_SERIAL_MS      2000


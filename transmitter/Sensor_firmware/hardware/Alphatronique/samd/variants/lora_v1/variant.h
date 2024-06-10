/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#pragma once
#include "stdbool.h"
#ifdef __cplusplus
#include "Stream.h"
#endif
// The definitions here needs a SAMD core >=1.6.10
#define ARDUINO_SAMD_VARIANT_COMPLIANCE 10610

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

/** Frequency of the board main oscillator */
#define VARIANT_MAINOSC		(32768ul)

/** Master clock frequency */
#define VARIANT_MCK	(F_CPU)

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "WVariant.h"

#ifdef __cplusplus
#include "SERCOM.h"
#include "Uart.h"
#endif // __cplusplus

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/



enum BoardPins {
    // 0..13 - Digital pins
    // ----------------------
    // 0/1 - SERCOM/UART (Serial1)
    BOARD_PA11_SERCOM,
    BOARD_PA10_SERCOM,
    // 2..12
    // Digital Low
    BOARD_PA14_DIGITAL,
    BOARD_PA9_TIMER,
    BOARD_PA8_TIMER,
    BOARD_PA15_TIMER,
    BOARD_PA20_TIMER_ALT,
    BOARD_PA21_TIMER_ALT,
    // Digital High
    BOARD_PA6_TIMER,
    BOARD_PA7_TIMER,
    BOARD_PA18_TIMER,
    BOARD_PA16_TIMER,
    BOARD_PA19_TIMER_ALT,
    // JP1
    BOARD_PA17_PWM,

    // 14..19 - Analog pins
    // --------------------
    BOARD_PA2_ANALOG,
    BOARD_PB8_ANALOG,
    BOARD_PB9_ANALOG,
    BOARD_PA4_ANALOG,
    BOARD_PA5_ANALOG,
    BOARD_PB2_ANALOG,

    // 20..21 I2C pins (SDA/SCL and also EDBG:SDA/SCL)
    // ----------------------
    BOARD_PA22_SERCOM,
    BOARD_PA23_SERCOM,
    BOARD_PA12_SERCOM_ALT,
    BOARD_PB10_SERCOM_ALT,
    BOARD_PB11_SERCOM_ALT,
    BOARD_PB3_OUTPUT,
    BOARD_PA27_OUTPUT,

    // 27..29 - USB
    // --------------------
    BOARD_PA28_COM,
    BOARD_PA24_COM,
    BOARD_PA25_COM,
    // 30..41 - EDBG
    // ----------------------
    // 30/31 - EDBG/UART
    BOARD_PB22_SERCOM_ALT,
    BOARD_PB23_SERCOM_ALT,
    // 32/33 I2C (SDA/SCL and also EDBG:SDA/SCL)

    BOARD_PA22_SERCOM_EDBG,
    BOARD_PA23_SERCOM_EDBG,
    // 34..37 - EDBG/SPI
    BOARD_PA19_SERCOM,
    BOARD_PA16_SERCOM,
    BOARD_PA18_SERCOM,
    BOARD_PA17_SERCOM,
    // 38..41 - EDBG/Digital

    BOARD_PA13_PWM,
    BOARD_PA21_PWM_ALT,
    BOARD_PA6_PWM,
    BOARD_PA7_PWM

};

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



// Number of pins defined in PinDescription array
#define PINS_COUNT           (26u)
#define NUM_DIGITAL_PINS     (20u)
#define NUM_ANALOG_INPUTS    (8u)
#define NUM_ANALOG_OUTPUTS   (1u)
#define analogInputToDigitalPin(p)  ((p < 6u) ? (p) + 14u : -1)

#define digitalPinToPort(P)        ( &(PORT->Group[g_APinDescription[P].ulPort]) )
#define digitalPinToBitMask(P)     ( 1 << g_APinDescription[P].ulPin )
//#define analogInPinToBit(P)        ( )
#define portOutputRegister(port)   ( &(port->OUT.reg) )
#define portInputRegister(port)    ( &(port->IN.reg) )
#define portModeRegister(port)     ( &(port->DIR.reg) )
#define digitalPinHasPWM(P)        ( g_APinDescription[P].ulPWMChannel != NOT_ON_PWM || g_APinDescription[P].ulTCChannel != NOT_ON_TIMER )

/*
 * digitalPinToTimer(..) is AVR-specific and is not defined for SAMD
 * architecture. If you need to check if a pin supports PWM you must
 * use digitalPinHasPWM(..).
 *
 * https://github.com/arduino/Arduino/issues/1833
 */
// #define digitalPinToTimer(P)

/*
 * Analog pins
 */
#define PIN_A0               (BOARD_PA2_ANALOG)
#define PIN_A1               (BOARD_PB8_ANALOG)
#define PIN_A2               (BOARD_PB9_ANALOG)
#define PIN_A3               (BOARD_PA4_ANALOG)
#define PIN_A4               (BOARD_PA5_ANALOG)
#define PIN_A5               (BOARD_PB2_ANALOG)
#define PIN_A6               (44ul)
#define PIN_A7               (45ul)
#define PIN_DAC0             (BOARD_PA2_ANALOG)






static const uint8_t A0  = PIN_A0;
static const uint8_t A1  = PIN_A1;
static const uint8_t A2  = PIN_A2;
static const uint8_t A3  = PIN_A3;
static const uint8_t A4  = PIN_A4;
static const uint8_t A5  = PIN_A5;
static const uint8_t A6  = PIN_A6 ;
static const uint8_t A7  = PIN_A7 ;
static const uint8_t DAC0 = PIN_DAC0;

#define ADC_RESOLUTION		12

/*
 * Serial interfaces
 */
// Serial (EDBG)
#define PIN_SERIAL_RX       (31ul)
#define PIN_SERIAL_TX       (30ul)
#define PAD_SERIAL_TX       (UART_TX_PAD_2)
#define PAD_SERIAL_RX       (SERCOM_RX_PAD_3)

// Serial1
#define PIN_SERIAL1_RX       (0ul)
#define PIN_SERIAL1_TX       (1ul)
#define PAD_SERIAL1_TX       (UART_TX_PAD_2)
#define PAD_SERIAL1_RX       (SERCOM_RX_PAD_3)

#define LORA_MISO                  (BOARD_PA12_SERCOM_ALT)
#define LORA_MOSI                  (BOARD_PB10_SERCOM_ALT)
#define LORA_SCK                   (BOARD_PB11_SERCOM_ALT)
#define LORA_SS	                   (BOARD_PB9_ANALOG)


/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 1

#define PIN_SPI_MISO         (LORA_MISO)
#define PIN_SPI_MOSI         (LORA_MOSI)
#define PIN_SPI_SCK          (LORA_SCK)
#define PERIPH_SPI           sercom4
#define PAD_SPI_TX           SPI_PAD_2_SCK_3
#define PAD_SPI_RX           SERCOM_RX_PAD_0

static const uint8_t SS	  = PIN_A2 ;	// SERCOM4 last PAD is present on A2 but HW SS isn't used. Set here only for reference.
static const uint8_t MOSI = PIN_SPI_MOSI ;
static const uint8_t MISO = PIN_SPI_MISO ;
static const uint8_t SCK  = PIN_SPI_SCK ;


// Hardware pins setup for the transmitter module
#define RFM95_CS        (BOARD_PA20_TIMER_ALT)
#define RFM95_RST       (BOARD_PA19_TIMER_ALT)
#define RFM95_INT       (BOARD_PB2_ANALOG)
#define MBX_ANALOG      (BOARD_PA6_TIMER)
#define VBAT_PIN        (BOARD_PB9_ANALOG)
#define MBX_PWR_PL      (BOARD_PA10_SERCOM)
#define MBX_TRIG        (BOARD_PA8_TIMER)
#define MBX_PULSE       (BOARD_PA18_TIMER)
#define TEMP_SHDN       (BOARD_PA9_TIMER)
#define TEMP_READ       (BOARD_PA7_TIMER)
#define REED_SW_PH      (BOARD_PA4_ANALOG)
#define RS485_DIR       (BOARD_PA5_ANALOG)
#define RS485_POWER     (BOARD_PB8_ANALOG)
#define RS485_TX        (BOARD_PB22_SERCOM_ALT)
#define RS485_RX        (BOARD_PB23_SERCOM_ALT)
#define no_sleep        (BOARD_PA16_TIMER)
#define JP1             (BOARD_PA17_PWM)
#define JP2_A           (BOARD_PA14_DIGITAL)
#define JP2_B           (BOARD_PA15_TIMER)



/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA         (BOARD_PA22_SERCOM)
#define PIN_WIRE_SCL         (BOARD_PA23_SERCOM)
#define PERIPH_WIRE          sercom3
#define WIRE_IT_HANDLER      SERCOM3_Handler

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

/*
 * USB
 */
// PA28 - Not connected on Lora_V1.0A
#define PIN_USB_HOST_ENABLE -1

#define PIN_USB_DM          (28ul)
#define PIN_USB_DP          (29ul)

// /*
//  * I2S Interfaces
//  */
#define I2S_INTERFACES_COUNT 0

// #define I2S_DEVICE          0
// #define I2S_CLOCK_GENERATOR 3
// #define PIN_I2S_SD          (9u)
// #define PIN_I2S_SCK         (1u)
// #define PIN_I2S_FS          (0u)

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus

/*	=========================
 *	===== SERCOM DEFINITION
 *	=========================
*/
extern SERCOM sercom0;
extern SERCOM sercom1;
extern SERCOM sercom2;
extern SERCOM sercom3;
extern SERCOM sercom4;
extern SERCOM sercom5;
extern Uart Serial5;
extern Uart Serial1;
#endif

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.


// #define SERIAL_PORT_USBVIRTUAL      Serial
// #define SERIAL_PORT_MONITOR         Serial
// // Serial has no physical pins broken out, so it's not listed as HARDWARE port
// #define SERIAL_PORT_HARDWARE        Serial1
// #define SERIAL_PORT_HARDWARE_OPEN   Serial1

#define SERIAL_PORT_USBVIRTUAL      Serial
#define SERIAL_PORT_MONITOR         Serial
// Serial has no physical pins broken out, so it's not listed as HARDWARE port
#define SERIAL_PORT_HARDWARE        Serial1
#define SERIAL_PORT_HARDWARE_OPEN   Serial1


typedef struct  {
  int pullmode;
  int level;
} StateDetails;

typedef struct  {
  int pin;
  StateDetails disabled;
  StateDetails enabled;
} PinState;

extern const PinState pins[];
extern bool configurePins(bool enable);
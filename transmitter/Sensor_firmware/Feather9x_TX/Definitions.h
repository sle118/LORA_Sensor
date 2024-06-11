/**
 * @file Definitions.h
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
#ifdef __cplusplus
// Measurement data structure
typedef struct __attribute__((packed)) {
  uint32_t distance;
  uint32_t duration;
  float bat_voltage;
  uint32_t rawVbat;
  float temperature;
  float internal_temp;
  int16_t packetnum;
  uint64_t uuid;
} measurements_t;

typedef struct __attribute__((packed)) {
    measurements_t measures;
    bool insufficient_power;
    unsigned long next_volt_read;
    bool gpios_enabled;
    bool measures_sent;
    uint16_t bod_sleep_remain;
} globals_t;

void flash_D3(int times, int on_time, int ratio_pct = 100, bool invert = false);
void flash_vbat_value();
void flash_power(int pin, int times, int on_time, int ratio_pct = 100, bool invert = false);
bool init_radio(bool enable = true);
void set_gpios(bool enable);
void send_data();
void read_distance();
void read_temperature();
void wait_for_power(uint32_t timeout_ms);
bool is_under_voltage(uint32_t timeout_ms);
// available brown out voltage levels
#define SYSCTRL_BOD33_LEVEL_1V64 6  /* ~1.64 volts */
#define SYSCTRL_BOD33_LEVEL_1V675 7 /* ~1.675 volts */
#define SYSCTRL_BOD33_LEVEL_2V77 39 /* ~2.77 volts */
#define SYSCTRL_BOD33_LEVEL_3V07 48 /* ~3.07 volts */


#endif
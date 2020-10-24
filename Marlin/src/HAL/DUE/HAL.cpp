/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

/**
 * HAL for Arduino Due and compatible (SAM3X8E)
 */

#ifdef ARDUINO_ARCH_SAM

#include "../../inc/MarlinConfig.h"
#include "HAL.h"

#include <Wire.h>
#include "usb/usb_task.h"

// ------------------------
// Public Variables
// ------------------------

uint16_t HAL_adc_result;

// ------------------------
// Public functions
// ------------------------

// HAL initialization task
void HAL_init() {
  // Initialize the USB stack
  #if ENABLED(SDSUPPORT)
    OUT_WRITE(SDSS, HIGH);  // Try to set SDSS inactive before any other SPI users start up
  #endif

  #if PIN_EXISTS(LED_ESTOP)
    SET_OUTPUT(LED_ESTOP_PIN);
    WRITE(LED_ESTOP_PIN, 0);
  #endif

  #if PIN_EXISTS(LED_BUSY)
    SET_OUTPUT(LED_BUSY_PIN);
    WRITE(LED_BUSY_PIN, 0);
  #endif

  #if PIN_EXISTS(LED_STOP)
    SET_OUTPUT(LED_STOP_PIN);
    WRITE(LED_STOP_PIN, 0);
  #endif

  #if PIN_EXISTS(LED_CLKOK)
    SET_OUTPUT(LED_CLKOK_PIN);
    WRITE(LED_CLKOK_PIN, 1);
  #endif

  #if PIN_EXISTS(LED_I2COK)
    SET_OUTPUT(LED_I2COK_PIN);
    WRITE(LED_I2COK_PIN, 0);
  #endif

  #if PIN_EXISTS(LED_BOOTOK)
    SET_OUTPUT(LED_BOOTOK_PIN);
    WRITE(LED_BOOTOK_PIN, 1);
  #endif

  /* TEMP HACK */
  SET_OUTPUT(FAN_ON);
  WRITE(FAN_ON, 1);

  SET_OUTPUT(TMC_SW_SCK);
  SET_OUTPUT(TMC_SW_MOSI);

  #if DISABLED(PINS_DEBUGGING) && PIN_EXISTS(LED)
    SET_OUTPUT(LED_PIN);  // heartbeat indicator
  #endif

  

  usb_task_init();
}

// HAL idle task
void HAL_idletask() {
  // Perform USB stack housekeeping
  usb_task_idle();
}

// Disable interrupts
void cli() { noInterrupts(); }

// Enable interrupts
void sei() { interrupts(); }

void HAL_clear_reset_source() { }

uint8_t HAL_get_reset_source() {
  switch ((RSTC->RSTC_SR >> 8) & 0x07) {
    case 0: return RST_POWER_ON;
    case 1: return RST_BACKUP;
    case 2: return RST_WATCHDOG;
    case 3: return RST_SOFTWARE;
    case 4: return RST_EXTERNAL;
    default: return 0;
  }
}

void _delay_ms(const int delay_ms) {
  // Todo: port for Due?
  delay(delay_ms);
}

extern "C" {
  extern unsigned int _ebss; // end of bss section
}

// Return free memory between end of heap (or end bss) and whatever is current
int freeMemory() {
  int free_memory, heap_end = (int)_sbrk(0);
  return (int)&free_memory - (heap_end ?: (int)&_ebss);
}

// ------------------------
// ADC
// ------------------------

void HAL_adc_start_conversion(const uint8_t ch) {
  HAL_adc_result = analogRead(ch);
}

uint16_t HAL_adc_get_result() {
  // nop
  return HAL_adc_result;
}

#endif // ARDUINO_ARCH_SAM

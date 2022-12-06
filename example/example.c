/*
  example.c - Iono RP D16 usage example

    Copyright (C) 2022 Sfera Labs S.r.l. - All rights reserved.

    For information, see:
    http://www.sferalabs.cc/

  This code is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  See file LICENSE.txt for further informations on licensing terms.
*/

#include <stdio.h>

#include "iono_rp_d16.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

bool flip;

void main_core1() {
  if (!iono_init()) {
    printf("init error\n");
  }

  while (true) {
    iono_process();
    sleep_ms(1);
  }
}

void on_debounce(int pin, int val) { printf("D%d debounce = %d\n", pin, val); }

int main() {
  // Init USB for logging/debugging (see CMakeLists.txt)
  stdio_usb_init();
  sleep_ms(3000);

  multicore_launch_core1(main_core1);

  // Make sure iono_init() is complete
  while (!iono_is_ready()) {
    tight_loop_contents();
  }

  printf("Ready!\n");

  // Setup D1 as push-pull output
  if (!iono_set_pin_mode(D1, IONO_PIN_MODE_OUT_PP, false)) {
    printf("D1 setup error\n");
  }

  // Setup D2 as high-side output with open-load detection enabled
  if (!iono_set_pin_mode(D2, IONO_PIN_MODE_OUT_HS, true)) {
    printf("D2 setup error\n");
  }

  // Setup D3 as input with wire-break detection enabled
  if (!iono_set_pin_mode(D3, IONO_PIN_MODE_IN, true)) {
    printf("D3 setup error\n");
  }

  // Setup D5 as high-side output
  if (!iono_set_pin_mode(D5, IONO_PIN_MODE_OUT_HS, false)) {
    printf("D5 setup error\n");
  }

  // Setup D6 as high-side output
  if (!iono_set_pin_mode(D6, IONO_PIN_MODE_OUT_HS, false)) {
    printf("D6 setup error\n");
  }

  // Setup D7 as input (required for joining D5-D6 below)
  if (!iono_set_pin_mode(D7, IONO_PIN_MODE_IN, false)) {
    printf("D7 setup error\n");
  }

  // Setup D8 as input (required for joining D5-D6 below)
  if (!iono_set_pin_mode(D8, IONO_PIN_MODE_IN, false)) {
    printf("D8 setup error\n");
  }

  // Join D5 and D6 to be used as single output
  if (!iono_join_outputs(D5, true)) {
    printf("D5-D6 join error\n");
  }

  // Setup DT1 as output
  if (!iono_set_pin_mode(DT1, IONO_PIN_MODE_OUT, false)) {
    printf("DT1 setup error\n");
  }

  // Setup DT2 as input
  if (!iono_set_pin_mode(DT2, IONO_PIN_MODE_IN, false)) {
    printf("DT2 setup error\n");
  }

  // Setup D4 as push-pull output
  if (!iono_set_pin_mode(D4, IONO_PIN_MODE_OUT_PP, false)) {
    printf("D4 setup error\n");
  }
  // with 1Hz frequency, 50% duty-cycle PWM
  if (!iono_set_pwm(D4, 1, 65535 / 2)) {
    printf("D4 PWM setup error\n");
  }

  // Set callback function on D3 state change
  // with 100ms debounce
  iono_subscribe(D3, 100, on_debounce);

  // Flip D2 on every low-to-high transition of D3
  // after a 50ms debounce
  iono_link(D3, D2, LINK_FLIP_H, 50);

  printf("I/O setup done.\n");

  // Init RS-485 interface
  uart_init(IONO_RS485, 115200);
  uart_set_format(IONO_RS485, 8, 1, UART_PARITY_NONE);

  while (true) {
    sleep_ms(500);

    flip = !flip;

    printf("---------- %d\n", flip);

    iono_put_led(flip);

    if (!iono_put(D1, flip ? 1 : 0)) {
      printf("D1 write error\n");
    }
    // Setting D5 will drive D6 too
    if (!iono_put(D5, flip ? 1 : 0)) {
      printf("D5-D6 write error\n");
    }
    if (!iono_put(DT1, flip ? 1 : 0)) {
      printf("DT1 write error\n");
    }

    sleep_ms(10);

    for (int d = D1; d <= D6; d++) {
      printf(
          "D%d = %d\tWB = %d\tOL = %d\tOV = %d\tOVL = %d\tTS = %d\tTSL = "
          "%d\tAT1 = %d\tAT2 = %d\n",
          d, iono_get(d), iono_get_wire_break(d), iono_get_open_load(d),
          iono_get_over_voltage(d), iono_get_over_voltage_lock(d),
          iono_get_thermal_shutdown(d), iono_get_thermal_shutdown_lock(d),
          iono_get_alarm_t1(d), iono_get_alarm_t2(d));
    }

    printf("DT1 = %d\n", iono_get(DT1));
    printf("DT2 = %d\n", iono_get(DT2));
  }

  return 0;
}

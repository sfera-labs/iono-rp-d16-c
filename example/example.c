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

#include <pico/stdlib.h>
#include <stdio.h>

#include "iono_rp_d16.h"

// Main loop

int main() {
  iono_init();

  // Init RS-485 interface
  uart_init(IONO_RS485, 115200);
  uart_set_format(IONO_RS485, 8, 1, UART_PARITY_NONE);

  // Init USB for logging/debugging (see CMakeLists.txt)
  stdio_usb_init();

  sleep_ms(3000);

  printf("Ready!\n");

  while (true) {
    // Blink LED
    iono_put_led(true);
    iono_process();
    sleep_ms(250);
    iono_put_led(false);

    printf("loop\n");

    sleep_ms(2000);
  }

  iono_process();

  return 0;
}

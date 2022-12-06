/*
  iono_rp_d16.h - Iono RP D16 library

    Copyright (C) 2022 Sfera Labs S.r.l. - All rights reserved.

    For information, see:
    http://www.sferalabs.cc/

  This code is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  See file LICENSE.txt for further informations on licensing terms.
*/

#include "hardware/uart.h"

#ifndef IONO_RP_D16_H
#define IONO_RP_D16_H

#define IONO_PIN_DT1 26
#define IONO_PIN_DT2 27
#define IONO_PIN_DT3 28
#define IONO_PIN_DT4 29

#define IONO_PIN_RS485_TX 16
#define IONO_PIN_RS485_RX 17
#define IONO_PIN_RS485_TXEN_N 14

#define IONO_PIN_I2C_SDA 0
#define IONO_PIN_I2C_SCL 1

#define IONO_PIN_SPI_SCK 2
#define IONO_PIN_SPI_TX 3
#define IONO_PIN_SPI_RX 4

#define IONO_PIN_CS_DOL 6
#define IONO_PIN_CS_DOH 5
#define IONO_PIN_CS_DIL 8
#define IONO_PIN_CS_DIH 7

#define IONO_PIN_MAX14912_WD_EN 18

#define IONO_PIN_MAX22190_LATCH 9

#define D1 1
#define D2 2
#define D3 3
#define D4 4
#define D5 5
#define D6 6
#define D7 7
#define D8 8
#define D9 9
#define D10 10
#define D11 11
#define D12 12
#define D13 13
#define D14 14
#define D15 15
#define D16 16
#define DT1 IONO_PIN_DT1
#define DT2 IONO_PIN_DT2
#define DT3 IONO_PIN_DT3
#define DT4 IONO_PIN_DT4

#define LINK_NONE 0
#define LINK_FOLLOW 1
#define LINK_INVERT 2
#define LINK_FLIP_H 3
#define LINK_FLIP_L 4
#define LINK_FLIP_T 5

#define IONO_PIN_MODE_IN 1
#define IONO_PIN_MODE_OUT_HS 2
#define IONO_PIN_MODE_OUT_PP 3
#define IONO_PIN_MODE_OUT 4

#define _MAX22190_NUM 2
#define _MAX14912_NUM 2

#define IONO_RS485 uart0

bool iono_init();
bool iono_is_ready();
void iono_rs485_tx_en(bool);
void iono_process();
int iono_get(int);
bool iono_put(int, int);
bool iono_flip(int);
int iono_get_wire_break(int);
int iono_get_open_load(int);
int iono_get_over_voltage(int);
int iono_get_over_voltage_lock(int);
int iono_get_thermal_shutdown(int);
int iono_get_thermal_shutdown_lock(int);
int iono_get_alarm_t1(int);
int iono_get_alarm_t2(int);
bool iono_set_pin_mode(int, int, bool);
bool iono_join_outputs(int, bool);
bool iono_clear_outputs_faults(int);
void iono_subscribe(int, unsigned long, void (*)(int, int));
void iono_link(int, int, int, unsigned long);
void iono_put_led(bool);
bool iono_set_pwm(int, int, uint16_t);

#endif

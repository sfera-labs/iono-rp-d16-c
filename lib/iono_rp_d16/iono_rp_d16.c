/*
  iono_rp_d16.c - Iono RP D16 library

    Copyright (C) 2022 Sfera Labs S.r.l. - All rights reserved.

    For information, see:
    http://www.sferalabs.cc/

  This code is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  See file LICENSE.txt for further informations on licensing terms.
*/

#include "iono_rp_d16.h"

#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/mutex.h"
#include "pico/stdlib.h"
#include "pico/time.h"

#if defined(IONO_DEBUG)
#include <stdio.h>
#define DEBUG(fmt, ...)         \
  do {                          \
    printf(fmt, ##__VA_ARGS__); \
  } while (0)
#else
#define DEBUG(...) \
  do {             \
  } while (0)
#endif

#define MAX22190_REG_WB 0x00
#define MAX22190_REG_FAULT1 0x04
#define MAX22190_REG_FAULT2 0x1C
#define MAX22190_REG_FLT1 0x06
#define MAX22190_REG_FAULT2EN 0x1E

#define MAX14912_REG_IN 0
#define MAX14912_REG_PP 1
#define MAX14912_REG_OL_EN 2
#define MAX14912_REG_WD_JN 3
#define MAX14912_REG_OL 4
#define MAX14912_REG_THSD 5
#define MAX14912_REG_FAULTS 6
#define MAX14912_REG_OV 7

#define _MAX22190_IDX_L 0
#define _MAX22190_IDX_H 1
#define _MAX14912_IDX_L 0
#define _MAX14912_IDX_H 1

#define _MAX14912_CMD_SET_STATE 0b0
#define _MAX14912_CMD_SET_MODE 0b1
#define _MAX14912_CMD_SET_OL_DET 0b10
#define _MAX14912_CMD_SET_CONFIG 0b11
#define _MAX14912_CMD_READ_REG 0b100000
#define _MAX14912_CMD_READ_RT_STAT 0b110000

#define _PROT_OV_LOCK_MS 10000
#define _PROT_THSD_LOCK_MS 30000

static bool _setupDone;
static int _pinMode[16];
static mutex_t _spiMtx;
static uint8_t _max14912ReadStatCrc;
static bool _ledSet;
static bool _ledVal;
static unsigned long _processTs;
static int _processStep;
static struct max22190Str {
  int pinCs;
  bool error;
  uint8_t inputs;
  uint8_t wb;
  uint8_t fault1;
  uint8_t fault2;
  uint8_t cfgFlt[8];
  uint8_t faultMemWb;
  uint8_t faultMemAlrmT1;
  uint8_t faultMemAlrmT2;
  uint8_t faultMemOtshdn;
} _max22190[_MAX22190_NUM];
static struct max14912Str {
  int pinCs;
  bool error;
  uint8_t outputs;
  uint8_t outputsUser;
  bool clearFaults;
  bool ovProtEn;
  uint8_t ol;
  uint8_t olRT;
  uint8_t ovRT;
  uint8_t thsd;
  uint8_t thsdRT;
  uint8_t cfgModePP;
  uint8_t cfgModePPUser;
  uint8_t cfgOlDet;
  uint8_t cfgJoin;
  uint8_t ovLock;
  uint8_t thsdLock;
  uint8_t faultMemOl;
  uint8_t faultMemOv;
  uint8_t faultMemThsd;
  unsigned long lockTs[8];
} _max14912[_MAX14912_NUM];
static struct subscribeStr {
  int pin;
  void (*cb)(int, int);
  unsigned long debounceMs;
  int value;
  unsigned long lastTs;
} _subscribeD[16], _subscribeDT[4];
static struct linkStr {
  int inPin;
  int outPin;
  int mode;
  unsigned long debounceMs;
  int value;
  unsigned long lastTs;
} _linkD[16][16];
static struct pwmStr {
  unsigned long periodUs;
  unsigned long dutyUs;
  unsigned long startTs;
  bool on;
} _pwm[16];

static int _getBit(uint8_t source, int bitIdx) {
  return (source >> bitIdx) & 1;
}

static void _setBit(uint8_t* target, int bitIdx, bool val) {
  uint8_t mask = 1 << bitIdx;
  *target = (*target & ~mask) | (val ? mask : 0);
}

static uint32_t _millis() { return to_ms_since_boot(get_absolute_time()); }

static uint32_t _micros() { return to_us_since_boot(get_absolute_time()); }

static void _spiTransaction(int cs, uint8_t d2, uint8_t d1, uint8_t d0,
                            uint8_t* r2, uint8_t* r1, uint8_t* r0) {
  DEBUG(">>> %d: %x %x %x\n", cs, d2, d1, d0);
  gpio_put(cs, 0);
  sleep_us(1);
  spi_write_read_blocking(spi0, &d2, r2, 1);
  spi_write_read_blocking(spi0, &d1, r1, 1);
  spi_write_read_blocking(spi0, &d0, r0, 1);
  sleep_us(1);
  gpio_put(cs, 1);
  DEBUG("<<< %d: %x %x %x\n", cs, *r2, *r1, *r0);
}

// MAX22190 =======================

static uint8_t _max22190Crc(uint8_t data2, uint8_t data1, uint8_t data0) {
  int length = 19;          // 19-bit data
  uint8_t crc_init = 0x07;  // 5-bit init word, constant, 00111
  uint8_t crc_poly = 0x35;  // 6-bit polynomial, constant, 110101
  uint8_t crc_step;
  uint8_t tmp;

  uint32_t datainput = (unsigned long)((data2 << 16) + (data1 << 8) + data0);
  datainput = (datainput & 0xffffe0) + crc_init;
  tmp = (uint8_t)((datainput & 0xfc0000) >> 18);
  if ((tmp & 0x20) == 0x20)
    crc_step = (uint8_t)(tmp ^ crc_poly);
  else
    crc_step = tmp;

  for (int i = 0; i < length - 1; i++) {
    tmp = (uint8_t)(((crc_step & 0x1f) << 1) +
                    ((datainput >> (length - 2 - i)) & 0x01));
    if ((tmp & 0x20) == 0x20)
      crc_step = (uint8_t)(tmp ^ crc_poly);
    else
      crc_step = tmp;
  }

  return (uint8_t)(crc_step & 0x1f);
}

static bool _max22190SpiTransaction(struct max22190Str* m, uint8_t* data1,
                                    uint8_t* data0) {
  uint8_t r1, r0, rcrc;
  uint8_t crc = _max22190Crc(*data1, *data0, 0);
  for (int i = 0; i < 3; i++) {
#ifdef IONO_DEBUG
    if (i != 0) {
      DEBUG("max22190 repeat\n");
    }
#endif
    mutex_enter_blocking(&_spiMtx);
    _spiTransaction(m->pinCs, *data1, *data0, crc, &r1, &r0, &rcrc);
    mutex_exit(&_spiMtx);
    if ((rcrc & 0x1f) == _max22190Crc(r1, r0, rcrc)) {
      *data1 = r1;
      *data0 = r0;
      return true;
    }
  }
  DEBUG("max22190 i2c error\n");
  return false;
}

static bool _max22190ReadReg(uint8_t regAddr, struct max22190Str* m,
                             uint8_t* data) {
  uint8_t data1 = regAddr;
  uint8_t data0 = 0;
  DEBUG("max22190 %d read reg=%d\n", m->pinCs, regAddr);
  if (_max22190SpiTransaction(m, &data1, &data0)) {
    m->error = false;
    m->inputs = data1;
    *data = data0;
    return true;
  }
  m->error = true;
  return false;
}

static bool _max22190WriteReg(uint8_t regAddr, struct max22190Str* m,
                              uint8_t val) {
  uint8_t data1 = 0x80 | regAddr;
  uint8_t data0 = val;
  DEBUG("max22190 %d write reg=%d val=0x%x\n", m->pinCs, regAddr, val);
  if (!_max22190SpiTransaction(m, &data1, &data0)) {
    return false;
  }
  if (!_max22190ReadReg(regAddr, m, &data0)) {
    return false;
  }
  return data0 == val;
}

static bool _max22190GetByPin(int pin, struct max22190Str** m, int* inIdx) {
  if (pin >= D1 && pin <= D8) {
    *m = &_max22190[_MAX22190_IDX_L];
    if (inIdx) {
      *inIdx = 8 - pin;
    }
    return true;
  } else if (pin >= D9 && pin <= D16) {
    *m = &_max22190[_MAX22190_IDX_H];
    if (inIdx) {
      *inIdx = 16 - pin;
    }
    return true;
  }
  return false;
}

// MAX14912 =======================

static uint8_t _max14912CrcLoop(uint8_t crc, uint8_t byte1) {
  for (int i = 0; i < 8; i++) {
    crc <<= 1;
    if (crc & 0x80) crc ^= 0xB7;  // 0x37 with MSBit on purpose
    if (byte1 & 0x80) crc ^= 1;
    byte1 <<= 1;
  }
  return crc;
}

static uint8_t _max14912Crc(uint8_t byte1, uint8_t byte2) {
  uint8_t synd;
  synd = _max14912CrcLoop(0x7f, byte1);
  synd = _max14912CrcLoop(synd, byte2);
  return _max14912CrcLoop(synd, 0x80) & 0x7f;
}

static bool _max14912SpiTransaction(bool wr, struct max14912Str* m,
                                    uint8_t* data1, uint8_t* data0) {
  uint8_t r1, r0, rcrc;
  uint8_t zBit = m->clearFaults ? 0x80 : 0x00;
  uint8_t crc = _max14912Crc(zBit | *data1, *data0);
  for (int i = 0; i < 3; i++) {
#ifdef IONO_DEBUG
    if (i != 0) {
      DEBUG("max14912 repeat\n");
    }
#endif
    mutex_enter_blocking(&_spiMtx);
    _spiTransaction(m->pinCs, zBit | *data1, *data0, crc, &r1, &r0, &rcrc);
#ifdef IONO_DEBUG
    if ((rcrc & 0x80) == 0x80) {
      // CRC check of previous transaction
      DEBUG("max14912 CRC prev err\n");
    }
#endif
    if (wr) {
      _spiTransaction(m->pinCs, _MAX14912_CMD_READ_RT_STAT, 0,
                      _max14912ReadStatCrc, &r1, &r0, &rcrc);
      mutex_exit(&_spiMtx);
      if ((rcrc & 0x80) == 0x80) {
        DEBUG("max14912 CRC err\n");
        continue;
      }
    } else {
      mutex_exit(&_spiMtx);
    }
    if ((rcrc & 0x7f) == _max14912Crc(r1, r0)) {
      *data1 = r1;
      *data0 = r0;
      if (zBit == 0x80) {
        m->clearFaults = false;
      }
      return true;
    }
  }
  DEBUG("max14912 i2c error\n");
  return false;
}

static bool _max14912ReadReg(uint8_t regAddr, struct max14912Str* m,
                             uint8_t* dataA, uint8_t* dataQ) {
  uint8_t data1 = _MAX14912_CMD_READ_REG;
  uint8_t data0 = regAddr;
  DEBUG("max14912 %d read reg=%d\n", m->pinCs, regAddr);
  if (_max14912SpiTransaction(true, m, &data1, &data0)) {
    m->error = false;
    if (dataA) {
      *dataA = data1;
    }
    if (dataQ) {
      *dataQ = data0;
    }
    return true;
  }
  m->error = true;
  return false;
}

static bool _max14912Cmd(uint8_t cmd, struct max14912Str* m, uint8_t data) {
  uint8_t data1 = cmd;
  uint8_t data0 = data;
  DEBUG("max14912 %d cmd=%d, data=0x%x\n", m->pinCs, cmd, data);
  return _max14912SpiTransaction(false, m, &data1, &data0);
}

static bool _max14912Config(struct max14912Str* m, uint8_t cmd,
                            uint8_t checkRegAddr, uint8_t val) {
  uint8_t cfg;
  DEBUG("max14912 %d cfg cmd=%d chReg=%d val=0x%x\n", m->pinCs, cmd,
        checkRegAddr, val);
  if (!_max14912Cmd(cmd, m, val)) {
    return false;
  }
  if (!_max14912ReadReg(checkRegAddr, m, NULL, &cfg)) {
    return false;
  }
  if (cfg != val) {
    return false;
  }
  return true;
}

static bool _max14912GetByPin(int pin, struct max14912Str** m, int* outIdx) {
  if (pin >= D1 && pin <= D8) {
    *m = &_max14912[_MAX14912_IDX_L];
    if (outIdx) {
      *outIdx = pin - 1;
    }
    return true;
  } else if (pin >= D9 && pin <= D16) {
    *m = &_max14912[_MAX14912_IDX_H];
    if (outIdx) {
      *outIdx = pin - 9;
    }
    return true;
  }
  return false;
}

static bool _max14912OutputSet(struct max14912Str* m, int outIdx, bool val) {
  _setBit(&m->outputs, outIdx, val);
  return _max14912Cmd(_MAX14912_CMD_SET_STATE, m, m->outputs);
}

static bool _max14912ModePPSet(struct max14912Str* m, int outIdx, bool val) {
  _setBit(&m->cfgModePP, outIdx, val);
  return _max14912Config(m, _MAX14912_CMD_SET_MODE, MAX14912_REG_PP,
                         m->cfgModePP);
}

static void _max14912OverVoltProt(struct max14912Str* m) {
  for (int oi = 0; oi < 8; oi++) {
    if (_getBit(m->ovRT, oi) && !_getBit(m->cfgModePP, oi)) {
      _setBit(&m->ovLock, oi, true);
      if (!_getBit(m->outputs, oi) && !_getBit(m->thsdLock, oi)) {
        _max14912OutputSet(m, oi, true);
      }
      m->lockTs[oi] = _millis();
    } else if (_getBit(m->ovLock, oi)) {
      if (_millis() - m->lockTs[oi] > _PROT_OV_LOCK_MS) {
        if (!_getBit(m->thsdLock, oi)) {
          _max14912OutputSet(m, oi, _getBit(m->outputsUser, oi));
        }
        _setBit(&m->ovLock, oi, false);
      }
    }
  }
}

static void _max14912ThermalProt(struct max14912Str* m) {
  for (int oi = 0; oi < 8; oi++) {
    if (_getBit(m->thsdRT, oi)) {
      _setBit(&m->thsdLock, oi, true);
      if (_getBit(m->cfgModePP, oi)) {
        _max14912ModePPSet(m, oi, false);
      }
      if (_getBit(m->outputs, oi)) {
        _max14912OutputSet(m, oi, false);
      }
      m->lockTs[oi] = _millis();
    } else if (_getBit(m->thsdLock, oi)) {
      if (_millis() - m->lockTs[oi] > _PROT_THSD_LOCK_MS) {
        _max14912ModePPSet(m, oi, _getBit(m->cfgModePPUser, oi));
        _max14912OutputSet(m, oi, _getBit(m->outputsUser, oi));
        _setBit(&m->thsdLock, oi, false);
      }
    }
  }
}

// ==================================

static bool _pinModeInput(int pin, bool wbol) {
  struct max22190Str* m;
  int inIdx;
  if (!_max22190GetByPin(pin, &m, &inIdx)) {
    return false;
  }
  uint8_t regAddr = MAX22190_REG_FLT1 + (inIdx * 2);
  m->cfgFlt[inIdx] = (m->cfgFlt[inIdx] & 0x0f) | (wbol ? 0x10 : 0x00);
  return _max22190WriteReg(regAddr, m, m->cfgFlt[inIdx]);
}

static bool _pinModeOutputProtected(int pin, int mode, bool wbol) {
  struct max14912Str* m;
  int outIdx;
  if (!_max14912GetByPin(pin, &m, &outIdx)) {
    return false;
  }
  _setBit(&m->cfgOlDet, outIdx, wbol);
  if (!_max14912Config(m, _MAX14912_CMD_SET_OL_DET, MAX14912_REG_OL_EN,
                       m->cfgOlDet)) {
    return false;
  }
  bool modePP = mode == IONO_PIN_MODE_OUT_PP;
  _setBit(&m->cfgModePPUser, outIdx, modePP);
  if (_getBit(m->ovLock, outIdx) || _getBit(m->thsdLock, outIdx)) {
    return false;
  }
  return _max14912ModePPSet(m, outIdx, modePP);
}

static bool _writeOutputProtected(int pin, int val) {
  struct max14912Str* m;
  int outIdx;
  if (!_max14912GetByPin(pin, &m, &outIdx)) {
    return false;
  }
  _setBit(&m->outputsUser, outIdx, val);
  if (_getBit(m->ovLock, outIdx) || _getBit(m->thsdLock, outIdx)) {
    return false;
  }
  return _max14912OutputSet(m, outIdx, val);
}

static bool _outputsJoinable(int pin) {
  int idx = pin - 1;
  int base4 = (idx / 4) * 4;
  int mod4 = idx % 4;
  int idxHs1;
  int idxHs2;
  int idxHsOrIn1;
  int idxHsOrIn2;
  if (mod4 <= 1) {
    idxHs1 = base4;
    idxHs2 = base4 + 1;
    idxHsOrIn1 = base4 + 2;
    idxHsOrIn2 = base4 + 3;
  } else {
    idxHsOrIn1 = base4;
    idxHsOrIn2 = base4 + 1;
    idxHs1 = base4 + 2;
    idxHs2 = base4 + 3;
  }
  if (_pinMode[idxHs1] != IONO_PIN_MODE_OUT_HS) {
    DEBUG("outs join %d err 1\n", pin);
    return false;
  }
  if (_pinMode[idxHs2] != IONO_PIN_MODE_OUT_HS) {
    DEBUG("outs join %d err 2\n", pin);
    return false;
  }
  if (_pinMode[idxHsOrIn1] != IONO_PIN_MODE_OUT_HS &&
      _pinMode[idxHsOrIn1] != IONO_PIN_MODE_IN) {
    DEBUG("outs join %d err 3\n", pin);
    return false;
  }
  if (_pinMode[idxHsOrIn2] != IONO_PIN_MODE_OUT_HS &&
      _pinMode[idxHsOrIn2] != IONO_PIN_MODE_IN) {
    DEBUG("outs join %d err 4\n", pin);
    return false;
  }
  return true;
}

static void _subscribeProcess(struct subscribeStr* s) {
  int val = iono_get(s->pin);
  unsigned long ts = _millis();
  if (s->value != val) {
    if ((ts - s->lastTs) >= s->debounceMs) {
      s->value = val;
      s->lastTs = ts;
      s->cb(s->pin, val);
    }
  } else {
    s->lastTs = ts;
  }
}

static void _linkProcess(struct linkStr* l) {
  if (l->inPin == 0 || l->outPin == 0 || l->mode == LINK_NONE) {
    return;
  }
  int val = iono_get(l->inPin);
  unsigned long ts = _millis();
  if (l->value != val) {
    if ((ts - l->lastTs) >= l->debounceMs) {
      l->value = val;
      l->lastTs = ts;
      switch (l->mode) {
        case LINK_FOLLOW:
          iono_put(l->outPin, val);
          break;
        case LINK_INVERT:
          iono_put(l->outPin, val == 1 ? 0 : 1);
          break;
        case LINK_FLIP_T:
          iono_flip(l->outPin);
          break;
        case LINK_FLIP_H:
          if (val == 1) {
            iono_flip(l->outPin);
          }
          break;
        case LINK_FLIP_L:
          if (val == 0) {
            iono_flip(l->outPin);
          }
          break;
      }
    }
  } else {
    l->lastTs = ts;
  }
}

static void _ledCtrl(bool on) {
  uint8_t x;
  mutex_enter_blocking(&_spiMtx);
  gpio_put(IONO_PIN_CS_DOL, on ? 0 : 1);
  gpio_put(IONO_PIN_CS_DIH, 0);
  _spiTransaction(IONO_PIN_CS_DIL, 0, 0, 0, &x, &x, &x);
  gpio_put(IONO_PIN_CS_DIH, 1);
  gpio_put(IONO_PIN_CS_DOL, 1);
  mutex_exit(&_spiMtx);
}

// Public ==========================

bool iono_init() {
  bool ok;

  gpio_init(IONO_PIN_CS_DOL);
  gpio_init(IONO_PIN_CS_DOH);
  gpio_init(IONO_PIN_CS_DIL);
  gpio_init(IONO_PIN_CS_DIH);

  gpio_set_dir(IONO_PIN_CS_DOL, GPIO_OUT);
  gpio_set_dir(IONO_PIN_CS_DOH, GPIO_OUT);
  gpio_set_dir(IONO_PIN_CS_DIL, GPIO_OUT);
  gpio_set_dir(IONO_PIN_CS_DIH, GPIO_OUT);

  gpio_put(IONO_PIN_CS_DOL, 1);
  gpio_put(IONO_PIN_CS_DOH, 1);
  gpio_put(IONO_PIN_CS_DIL, 1);
  gpio_put(IONO_PIN_CS_DIH, 1);

  gpio_init(IONO_PIN_MAX14912_WD_EN);
  gpio_set_dir(IONO_PIN_MAX14912_WD_EN, GPIO_OUT);
  gpio_put(IONO_PIN_MAX14912_WD_EN, 1);

  gpio_init(IONO_PIN_RS485_TXEN_N);
  gpio_set_dir(IONO_PIN_RS485_TXEN_N, GPIO_OUT);
  iono_rs485_tx_en(false);

  gpio_set_function(IONO_PIN_RS485_RX, GPIO_FUNC_UART);
  gpio_set_function(IONO_PIN_RS485_TX, GPIO_FUNC_UART);
  uart_init(IONO_RS485, 9600);
  uart_set_hw_flow(IONO_RS485, false, false);
  uart_deinit(IONO_RS485);

  gpio_set_function(IONO_PIN_SPI_RX, GPIO_FUNC_SPI);
  gpio_set_function(IONO_PIN_SPI_TX, GPIO_FUNC_SPI);
  gpio_set_function(IONO_PIN_SPI_SCK, GPIO_FUNC_SPI);
  spi_init(spi0, 1000000);
  spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

  _max22190[_MAX22190_IDX_L].pinCs = IONO_PIN_CS_DIL;
  _max22190[_MAX22190_IDX_H].pinCs = IONO_PIN_CS_DIH;
  _max14912[_MAX14912_IDX_L].pinCs = IONO_PIN_CS_DOL;
  _max14912[_MAX14912_IDX_H].pinCs = IONO_PIN_CS_DOH;

  _max14912ReadStatCrc = _max14912Crc(_MAX14912_CMD_READ_RT_STAT, 0);

  mutex_init(&_spiMtx);

  ok = _max22190WriteReg(MAX22190_REG_FAULT2EN, &_max22190[_MAX22190_IDX_L],
                         0x3f);
  ok &= _max22190WriteReg(MAX22190_REG_FAULT2EN, &_max22190[_MAX22190_IDX_H],
                          0x3f);

  _ledSet = true;
  _ledVal = false;

  _setupDone = true;

  iono_process();

  return ok;
}

bool iono_is_ready() { return _setupDone; }

void iono_process() {
  int i, j;
  unsigned long ts, dts;
  struct max22190Str* mi;
  struct max14912Str* mo;

  if (!_setupDone) {
    return;
  }

  // Devices are read alternately to avoid delays between
  // subsequent SPI cycles

  // WB is read always to update the inputs state
  for (i = 0; i < _MAX22190_NUM; i++) {
    mi = &_max22190[i];
    _max22190ReadReg(MAX22190_REG_WB, mi, &mi->wb);
    mi->faultMemWb |= mi->wb;
  }

  if (_millis() - _processTs > 20) {
    switch (_processStep++) {
      case 0:
        for (i = 0; i < _MAX22190_NUM; i++) {
          mi = &_max22190[i];
          _max22190ReadReg(MAX22190_REG_FAULT1, mi, &mi->fault1);
          mi->faultMemAlrmT1 |= _getBit(mi->fault1, 3) ? 0xff : 0x00;
          mi->faultMemAlrmT2 |= _getBit(mi->fault1, 4) ? 0xff : 0x00;
        }
        for (i = 0; i < _MAX22190_NUM; i++) {
          mi = &_max22190[i];
          if (_getBit(mi->fault1, 5)) {
            _max22190ReadReg(MAX22190_REG_FAULT2, mi, &mi->fault2);
            mi->faultMemOtshdn |= _getBit(mi->fault2, 4) ? 0xff : 0x00;
          }
        }
        break;

      case 1:
        for (i = 0; i < _MAX14912_NUM; i++) {
          mo = &_max14912[i];
          _max14912ReadReg(MAX14912_REG_OL, mo, &mo->olRT, &mo->ol);
          mo->faultMemOl |= mo->olRT;
        }
        break;

      case 2:
        for (i = 0; i < _MAX14912_NUM; i++) {
          mo = &_max14912[i];
          _max14912ReadReg(MAX14912_REG_OV, mo, &mo->ovRT, NULL);
          mo->faultMemOv |= mo->ovRT;
          if (mo->ovProtEn) {
            _max14912OverVoltProt(mo);
          }
        }
        break;

      case 3:
        for (i = 0; i < _MAX14912_NUM; i++) {
          mo = &_max14912[i];
          _max14912Cmd(_MAX14912_CMD_SET_STATE, mo, mo->outputs);
        }
        break;

      default:
        for (i = 0; i < _MAX14912_NUM; i++) {
          mo = &_max14912[i];
          _max14912ReadReg(MAX14912_REG_THSD, mo, &mo->thsdRT, &mo->thsd);
          mo->faultMemThsd |= mo->thsdRT;
          _max14912ThermalProt(mo);
        }
        _processStep = 0;
        break;
    }

    _processTs = _millis();
  }

  for (i = 0; i < 16; i++) {
    if (_subscribeD[i].cb != NULL) {
      _subscribeProcess(&_subscribeD[i]);
    }
    for (j = 0; j < 16; j++) {
      _linkProcess(&_linkD[i][j]);
    }
  }
  for (i = 0; i < 4; i++) {
    if (_subscribeDT[i].cb != NULL) {
      _subscribeProcess(&_subscribeDT[i]);
    }
  }

  ts = _micros();
  for (i = 0; i < 16; i++) {
    if (_pwm[i].periodUs > 0) {
      dts = ts - _pwm[i].startTs;
      if (dts > _pwm[i].periodUs) {
        _writeOutputProtected(i + 1, 1);
        _pwm[i].startTs = _micros();
        _pwm[i].on = true;
      } else if (_pwm[i].on && dts > _pwm[i].dutyUs) {
        _writeOutputProtected(i + 1, 0);
        _pwm[i].on = false;
      }
    }
  }

  if (_ledVal != _ledSet) {
    _ledCtrl(_ledSet);
    _ledVal = _ledSet;
  }
}

bool iono_set_pin_mode(int pin, int mode, bool wbol) {
  struct max14912Str* mo;
  int outIdx;

  if (pin >= DT1 && pin <= DT4) {
    if (mode == IONO_PIN_MODE_IN) {
      gpio_init(pin);
      gpio_set_dir(pin, GPIO_IN);
      return true;
    }
    if (mode == IONO_PIN_MODE_OUT) {
      gpio_init(pin);
      gpio_set_dir(pin, GPIO_OUT);
      return true;
    }
    return false;
  }

  bool ok = false;
  if (mode == IONO_PIN_MODE_IN) {
    if (!_pinModeOutputProtected(pin, IONO_PIN_MODE_OUT_HS, false)) {
      return false;
    }
    if (!_writeOutputProtected(pin, 0)) {
      return false;
    }
    ok = _pinModeInput(pin, wbol);
  } else if (mode == IONO_PIN_MODE_OUT_HS || mode == IONO_PIN_MODE_OUT_PP) {
    if (mode == IONO_PIN_MODE_OUT_PP && wbol) {
      //  Open-load detection works in high-side mode only
      return false;
    }
    if (!_pinModeInput(pin, false)) {
      return false;
    }
    if (!_max14912GetByPin(pin, &mo, &outIdx)) {
      return false;
    }
    mo->ovProtEn = true;
    ok = _pinModeOutputProtected(pin, mode, wbol);
  }
  if (ok) {
    _pinMode[pin - 1] = mode;
  }
  return ok;
}

bool iono_join_outputs(int pin, bool join) {
  struct max14912Str* m;
  int outIdx;
  if (!_max14912GetByPin(pin, &m, &outIdx)) {
    return false;
  }
  if (join && !_outputsJoinable(pin)) {
    return false;
  }
  int bitIdx = (outIdx <= 3) ? 2 : 3;
  _setBit(&m->cfgJoin, bitIdx, join);
  if (!_max14912Config(m, _MAX14912_CMD_SET_CONFIG, MAX14912_REG_WD_JN,
                       m->cfgJoin)) {
    return false;
  }
  return true;
}

int iono_get(int pin) {
  if (pin >= DT1 && pin <= DT4) {
    return gpio_get(pin);
  }
  struct max22190Str* m;
  int inIdx;
  if (!_max22190GetByPin(pin, &m, &inIdx)) {
    return -1;
  }
  return (m->inputs >> inIdx) & 1;
}

bool iono_put(int pin, int val) {
  if (pin >= DT1 && pin <= DT4) {
    gpio_put(pin, val);
    return true;
  }
  if (pin < D1 || pin > D16) {
    return false;
  }
  if (_pinMode[pin - 1] != IONO_PIN_MODE_OUT_HS &&
      _pinMode[pin - 1] != IONO_PIN_MODE_OUT_PP) {
    return false;
  }
  return _writeOutputProtected(pin, val);
}

bool iono_flip(int pin) {
  int val = iono_get(pin);
  if (val < 0) {
    return false;
  }
  return iono_put(pin, val == 1 ? 0 : 1);
}

int iono_get_wire_break(int pin) {
  struct max22190Str* m;
  int ret, inIdx;
  if (!_max22190GetByPin(pin, &m, &inIdx)) {
    return -1;
  }
  ret = _getBit(m->faultMemWb, inIdx);
  _setBit(&m->faultMemWb, inIdx, false);
  return ret;
}

int iono_get_open_load(int pin) {
  struct max14912Str* m;
  int ret, outIdx;
  if (!_max14912GetByPin(pin, &m, &outIdx)) {
    return -1;
  }
  ret = _getBit(m->faultMemOl, outIdx);
  _setBit(&m->faultMemOl, outIdx, false);
  return ret;
}

int iono_get_over_voltage(int pin) {
  struct max14912Str* m;
  int ret, outIdx;
  if (!_max14912GetByPin(pin, &m, &outIdx)) {
    return -1;
  }
  ret = _getBit(m->faultMemOv, outIdx);
  _setBit(&m->faultMemOv, outIdx, false);
  return ret;
}

int iono_get_over_voltage_lock(int pin) {
  struct max14912Str* m;
  int outIdx;
  if (!_max14912GetByPin(pin, &m, &outIdx)) {
    return -1;
  }
  return _getBit(m->ovLock, outIdx);
}

int iono_get_thermal_shutdown(int pin) {
  struct max14912Str* mo;
  struct max22190Str* mi;
  int ret, outIdx, inIdx;
  if (!_max14912GetByPin(pin, &mo, &outIdx)) {
    return -1;
  }
  if (!_max22190GetByPin(pin, &mi, &inIdx)) {
    return -1;
  }
  ret =
      (_getBit(mo->faultMemThsd, outIdx) || _getBit(mi->faultMemOtshdn, inIdx))
          ? 1
          : 0;
  _setBit(&mo->faultMemThsd, outIdx, false);
  _setBit(&mi->faultMemOtshdn, inIdx, false);
  return ret;
}

int iono_get_thermal_shutdown_lock(int pin) {
  struct max14912Str* m;
  int outIdx;
  if (!_max14912GetByPin(pin, &m, &outIdx)) {
    return -1;
  }
  return _getBit(m->thsdLock, outIdx);
}

int iono_get_alarm_t1(int pin) {
  struct max22190Str* m;
  int ret, inIdx;
  if (!_max22190GetByPin(pin, &m, &inIdx)) {
    return -1;
  }
  ret = _getBit(m->faultMemAlrmT1, inIdx);
  _setBit(&m->faultMemAlrmT1, inIdx, false);
  return ret;
}

int iono_get_alarm_t2(int pin) {
  struct max22190Str* m;
  int ret, inIdx;
  if (!_max22190GetByPin(pin, &m, &inIdx)) {
    return -1;
  }
  ret = _getBit(m->faultMemAlrmT2, inIdx);
  _setBit(&m->faultMemAlrmT2, inIdx, false);
  return ret;
}

bool iono_clear_outputs_faults(int pin) {
  struct max14912Str* m;
  if (!_max14912GetByPin(pin, &m, NULL)) {
    return false;
  }
  m->clearFaults = true;
  return true;
}

void iono_subscribe(int pin, unsigned long debounceMs, void (*cb)(int, int)) {
  struct subscribeStr* s;
  if (pin >= DT1 && pin <= DT4) {
    s = &_subscribeDT[pin - DT1];
  } else if (pin >= D1 && pin <= D16) {
    s = &_subscribeD[pin - D1];
  } else {
    return;
  }
  s->pin = pin;
  s->cb = cb;
  s->debounceMs = debounceMs;
  s->value = -1;
  s->lastTs = _millis();
}

void iono_link(int inPin, int outPin, int mode, unsigned long debounceMs) {
  struct linkStr* l;
  if (inPin >= D1 && inPin <= D16 && outPin >= D1 && outPin <= D16) {
    l = &_linkD[inPin - D1][outPin - D1];
  } else {
    return;
  }
  l->inPin = inPin;
  l->outPin = outPin;
  l->mode = mode;
  l->debounceMs = debounceMs;
  l->value = -1;
  l->lastTs = _millis();
}

void iono_rs485_tx_en(bool enabled) {
  gpio_put(IONO_PIN_RS485_TXEN_N, enabled ? 0 : 1);
}

void iono_put_led(bool on) { _ledSet = on; }

bool iono_set_pwm(int pin, int freqHz, uint16_t dutyU16) {
  if (pin < D1 || pin > D16) {
    return false;
  }
  if (_pinMode[pin - 1] != IONO_PIN_MODE_OUT_PP) {
    return false;
  }
  _pwm[pin - 1].periodUs = 0;
  if (dutyU16 == 0) {
    _writeOutputProtected(pin, 0);
    return true;
  }
  _pwm[pin - 1].dutyUs = (1000000ull / freqHz) * dutyU16 / 65535ull;
  _writeOutputProtected(pin, 1);
  _pwm[pin - 1].startTs = _micros();
  _pwm[pin - 1].on = true;
  _pwm[pin - 1].periodUs = 1000000ull / freqHz;
  return true;
}

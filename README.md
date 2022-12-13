# Iono RP D16 - Raspberry Pi Pico C/C++ SDK

Raspberry Pi Pico C/C++ SDK libraries and examples for [Iono RP D16](https://www.sferalabs.cc/product/iono-rp-d16/) - the multi-purpose digital I/O module with a Raspberry Pi RP2040 (Pico) computing core.

This repository is structured as a standard project for the [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk). Download it and open it in your IDE of choice to start programming Iono RP D16.

## Library usage

For a quick start check out the [**example project**](./example).

Include the `iono_rp_d16` library as shown in the [example's CMakeLists.txt file](./example/CMakeLists.txt).

The library defines the constants `D1` ... `D16` and `DT1` ... `DT4`, corresponding to Iono RP D16's I/Os, to be used as the `pin` parameter for the below functions.

A reference to the UART connected to the RS-485 interface is available as `IONO_RS485`.     
Usage example:

```C
uart_init(IONO_RS485, 115200);
```

The TX-enable pin should be controlled when sending or receiving data on the RS-485 interface, use the `iono_rs485_tx_en()` method.

### Functions

### `bool iono_init()`
Call this function before using any other functionality of the library. It initializes the used pins and peripherals.
#### Returns
`true` upon success.

<br/>

### `bool iono_is_ready()`
Useful for synchronization between the two cores.
#### Returns
whether or not the `iono_init()` method has been executed.
#### Example
```C
void main_core1() {
  iono_init();
  // ...
}

int main() {
  multicore_launch_core1(main_core1);
  // wait for iono_init() to complete on the other core
  while (!iono_is_ready()) {
    tight_loop_contents();
  }
  // ...
}
```

<br/>

### `void iono_process()`
Call this function periodically, with a maximum interval of 20ms. It performs the reading of the input/output peripherals' state, moreover it checks for fault conditions, enables safety routines and updates the outputs' watchdog. It is recommended to reserve one core of the RP2040 for calling this function, while performing your custom logic on the other core.

<br/>

### `bool iono_set_pin_mode(int pin, int mode, bool wbol)`
Initializes a pin as input or output. To be called before any other operation on the same pin.
#### Parameters
**`pin`**: `D1` ... `D16`, `DT1` ... `DT4`

**`mode`**:
- `IONO_PIN_MODE_IN`: use pin as input
- `IONO_PIN_MODE_OUT_HS`: use pin as high-side output (only for `D1` ... `D16`)
- `IONO_PIN_MODE_OUT_PP`: use pin as push-pull output (only for `D1` ... `D16`)
- `IONO_PIN_MODE_OUT`: use pin as output (only for `DT1` ... `DT4`)

**`wbol`**: enable (`true`) or disable (`false`) wire-break (for inputs) or open-load (for high-side outputs) detection (only for `D1` ... `D16`)
#### Returns
`true` upon success.

<br/>

### `bool iono_join_outputs(int pin, bool join)`
Joins two high-side outputs to be used as a single output. Outputs can be joined in specific pairs: `D1`-`D2`, `D3`-`D4`, ..., `D15`-`D16`.
When a pair is joined, the other pair of the same group of four is also joined, e.g. joining `D3`-`D4` results in `D1`-`D2` being joined too if used as outputs.
Therefore, joining a pair requires the pins of the other pair of the same group of four to be initialized as `IONO_PIN_MODE_OUT_HS` or `IONO_PIN_MODE_IN`. In the latter case the two pins can be used as independent inputs.
#### Parameters
**`pin`**: one pin of the pair: `D1` ... `D16`

**`join`**: `true` to join, `false` to un-join
#### Returns
`true` upon success.

<br/>

### `int iono_get(int pin)`
Returns the state of a pin.    
For `D1` ... `D16` pins the returned value corresponds to the reading performed during the latest `process()` call.    
For `DT1` ... `DT4` pins the returned value corresponds to the instant reading of the corresponding GPIO.
#### Parameters
**`pin`**: `D1` ... `D16`, `DT1` ... `DT4`
#### Returns
`1`, `0`, or `-1` upon error.

<br/>

### `bool iono_put(int pin, int val)`
Sets the value of an output pin.
#### Parameters
**`pin`**: `D1` ... `D16`, `DT1` ... `DT4`

**`val`**: `1` or `0`
#### Returns
`true` upon success.

<br/>

### `bool flip(int pin)`
Flips the value of an output pin.
#### Parameters
**`pin`**: `D1` ... `D16`, `DT1` ... `DT4`
#### Returns
`true` upon success.

<br/>

### `int iono_get_wire_break(int pin)`
Returns the wire-break fault state of an input pin with wire-break detection enabled.    
The fault state is updated on each `iono_process()` call and set to `1` when detected. It is cleared (set to `0`) only after calling this method.
#### Parameters
**`pin`**: `D1` ... `D16`
#### Returns
`1` if wire-break detected, `0` if wire-break not detected, or `-1` upon error.

<br/>

### `int iono_get_open_load(int pin)`
Returns the open-load fault state of an input pin with open-load detection enabled.    
The fault state is updated on each `iono_process()` call and set to `1` when detected. It is cleared (set to `0`) only after calling this method.
#### Parameters
**`pin`**: `D1` ... `D16`
#### Returns
`1` if open-load detected, `0` if open-load not detected, or `-1` upon error.

<br/>

### `int iono_get_over_voltage(int pin)`
Returns the over-voltage fault state of a pin.    
The fault state is updated on each `iono_process()` call and set to `1` when detected. It is cleared (set to `0`) only after calling this method.
#### Parameters
**`pin`**: `D1` ... `D16`
#### Returns
`1` if over-voltage detected, `0` if over-voltage not detected, or `-1` upon error.

<br/>

### `int iono_get_over_voltage_lock(int pin)`
Returns whether or not an output pin is temporarily locked due to an over-voltage condition.    
The output cannot be set when locked.    
The fault state is updated on each `iono_process()` call and set to `1` when detected. It is cleared (set to `0`) only after calling this method.
#### Parameters
**`pin`**: `D1` ... `D16`
#### Returns
`1` if locked, `0` if not locked, or `-1` upon error.

<br/>

### `int iono_get_thermal_shutdown(int pin)`
Returns the thermal shutdown fault state of a pin.    
The fault state is updated on each `iono_process()` call and set to `1` when detected. It is cleared (set to `0`) only after calling this method.
#### Parameters
**`pin`**: `D1` ... `D16`
#### Returns
`1` if thermal shutdown active, `0` if thermal shutdown not active, or `-1` upon error.

<br/>

### `int iono_get_thermal_shutdown_lock(int pin)`
Returns whether or not an output pin is temporarily locked due to a thermal shutdown condition.    
The output cannot be set when locked.    
The fault state is updated on each `iono_process()` call and set to `1` when detected. It is cleared (set to `0`) only after calling this method.
#### Parameters
**`pin`**: `D1` ... `D16`
#### Returns
`1` if locked, `0` if not locked, or `-1` upon error.

<br/>

### `int iono_get_alarm_t1(int pin)`
Returns whether or not the temperature alarm 1 threshold has been exceeded on the input peripheral the pin belongs to.    
The fault state is updated on each `iono_process()` call and set to `1` when detected. It is cleared (set to `0`) only after calling this method.
#### Parameters
**`pin`**: `D1` ... `D16`
#### Returns
`1` if threshold exceeded, `0` if threshold not exceeded, or `-1` upon error.

<br/>

### `int iono_get_alarm_t2(int pin)`
Returns whether or not the temperature alarm 2 threshold has been exceeded on the input peripheral the pin belongs to.    
The fault state is updated on each `iono_process()` call and set to `1` when detected. It is cleared (set to `0`) only after calling this method.
#### Parameters
**`pin`**: `D1` ... `D16`
#### Returns
`1` if threshold exceeded, `0` if threshold not exceeded, or `-1` upon error.

<br/>

### `void iono_subscribe(int pin, unsigned long debounceMs, void (*cb)(int, int))`
Set a callback function to be called upon input state change with a debounce filter.    
The callback function is called within `iono_process()` execution, it is therefore recommended to execute only quick operations.
#### Parameters
**`pin`**: `D1` ... `D16`, `DT1` ... `DT4`

**`debounceMs`**: debounce time in milliseconds

**`cb`**: callback function

<br/>

### `void iono_link(int inPin, int outPin, int mode, unsigned long debounceMs)`
Links the state of two pins. when the `inPin` pin changes state (with the specified debounce filter), the `outPin` output is set according to the specified mode.
#### Parameters
**`pin`**: `D1` ... `D16`

**`mode`**:
- `LINK_FOLLOW`: `outPin` is set to the same state of `inPin`
- `LINK_INVERT`: `outPin` is set to the opposite state of `inPin`
- `LINK_FLIP_H`: `outPin` is flipped upon each low-to-high transition of `inPin`
- `LINK_FLIP_L`: `outPin` is flipped upon each high-to-low transition of `inPin`
- `LINK_FLIP_T`: `outPin` is flipped upon each state transition of `inPin`

**`debounceMs`**: debounce time in milliseconds

<br/>

### `void iono_put_led(bool on)`
Sets the blue 'ON' LED state.
#### Parameters
**`on`**: `true` LED on, `false` LED off

<br/>

### `bool iono_set_pwm(int pin, int freqHz, uint16_t dutyU16)`
Sets a soft-PWM on a push-pull output.    
The maximum frequency is determined by the frequency of `iono_process()` calls.
#### Parameters
**`pin`**: `D1` ... `D16`

**`freqHz`**: frequency in Hz

**`dutyU16`**: duty cycle as a ratio `dutyU16 / 65535`
#### Returns
`true` upon success.

<br/>

### `void iono_rs485_tx_en(bool enabled)`
Controls the TX-enable line of the RS-485 interface.    
Call `iono_rs485_tx_en(true)` before writing to the IONO_RS485 UART. When incoming data is expected, call `iono_rs485_tx_en(false)` before. Good practice is to call `iono_rs485_tx_en(false)` as soon as data has been written and flushed to the UART.
#### Parameters
**`enabled`**: `true` to enable the TX-enable line, `false` to disable it
#### Example
```C++
iono_rs485_tx_en(true);
uart_putc_raw(IONO_RS485, data);
uart_putc_raw(IONO_RS485, more_data);
uart_tx_wait_blocking(IONO_RS485);
iono_rs485_tx_en(false);
c = uart_getc(IONO_RS485);
```

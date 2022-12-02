# Exo Sense RP - Raspberry Pi Pico C/C++ SDK

This repository contains the resources for low-level C/C++ development on [Exo Sense RP](https://www.sferalabs.cc/product/exo-sense-rp/).

For other ways of programming Exo Sense RP go to:
https://github.com/sfera-labs/exo-sense-rp

This repository is structured as a standard project for the [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk), which provides all the resources needed to program Exo Sense RP in C/C++.

The [exo_sense_rp library](./lib/exo_sense_rp) consists of a simple header file you can include in your project. It contains the #defines to easily map Exo's I/Os to the corresponding RP2040 GPIOs, as well as I2C addresses and other constants for Exo's sensors, to be used as parameters for the standard SDK functions.

Check the [example project](./example) for a quick start.

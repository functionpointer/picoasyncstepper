# Picoasyncstepper

Asynchronous stepper motor library
Run stepper motors with quite loose timing constraints from your cpu!
Published on PlatformIO: https://registry.platformio.org/libraries/functionpointer/picoasyncstepper

Features:

- Speed mode
- Position mode
- Smooth acceleration and deceleration
- Adjustable acceleration rate
- Adjustable max speed
- Hardware-based generation of step pulses

Supported microcontrollers:

- RP2040 (with [**Earle Philhower's arduino-pico core**](https://github.com/earlephilhower/arduino-pico))

## How it works

A hardware timer (PWM slice) is used in phase correct mode.
The actual pwm value is fixed. Instead the wrap point of the timer is controlled (TOP register).
A lower wrap value causes the timer to wrap more often, emitting more pulses, causing the motor to run faster.

This concept is combined with the DMA engine:
The library pre-calculates many wrap values and stores them in an array.
The DMA engine is used to stream the array into the TOP register.
As a result the will smoothly and perfectly execute all pre-calculated steps without any cpu involvement.
To maximize smoothness, the library uses two arrays and alternates between them, using two DMA requests chained to one another.
Regular CPU attention is required to check if the arrays have switched, and to fill the newly finished one with fresh values.

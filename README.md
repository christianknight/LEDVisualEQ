# LightRamp

This repository contains source code and design resources for building firmware to do real-time digital signal processing of line-level audio signals using an STM32F4 microcontroller with floating-point hardware. Its current application is to control the brightness of 4 LEDs using PWM output, with the duty cycle dependent on the intensity of the audio signals within dedicated frequency bands for each LED.

Included is:
- A project for building firmware using the "System Workbench for STM32" (SW4STM32) IDE.
- Matlab filter design files to generate biquad IIR filter coefficients to use in the firmware for isolating frequency bands in the sampled audio signal.
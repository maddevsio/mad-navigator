# Mad navigator

## Description

Mad navigator is simple compass-like device shows direction to predefined geo point.
This repository contains firmware source code and schematics for device. 

[![Developed by Mad Devs](https://maddevs.io/badge-light.svg)](https://maddevs.io)

## Technologies and modules

- Microcontroller [STM32F100RBT6](https://www.st.com/content/ccc/resource/technical/document/reference_manual/a2/2d/02/4b/78/57/41/a3/CD00246267.pdf/files/CD00246267.pdf/jcr:content/translations/en.CD00246267.pdf)
- LCD display [Nokia 5110](https://www.sparkfun.com/datasheets/LCD/Monochrome/Nokia5110.pdf)
- Accelerometer + gyroscope [MPU6050](https://store.invensense.com/datasheets/invensense/MPU-6050_DataSheet_V3%204.pdf)
- GPS receiver [Neo-6M](https://www.u-blox.com/sites/default/files/products/documents/NEO-6_DataSheet_%28GPS.G6-HW-09005%29.pdf)
- Magnetometer [QMC5883L](https://nettigo.pl/attachments/440)

## Prerequisites

- make
- arm-none-eabi-* (gcc, gdb, ar etc.)
- st-link utils

## Folder structure

root: Makefile, qtcreator project file (not neccessary to use it), LinkerScript

inc: All include files

src: All firmware source files

startup: Standard startup code

kicad_schema: Schematic

## How to run/debug

### RUN

Just use `make clean all program` . Make sure that your device connected via USB and you have all needed permissions.
You don't need to `make clean` every time. 

### DEBUG

1. `make clean all program`
2. run `st-util`. It will launch gdb server on localhost:4242. 
3. use arm-none-eabi-gdb

## Extended documentation

!TODO add link to medium


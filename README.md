# Gascounter-module
Firmware for *Gascounter Modules*, that monitor the flow through gasmeters. They are meant to be used in conjunction with [Helium Management](https://github.com/SampleEnvironment/Helium-Management) application.

## Getting started
### Requisities
In order to start developing/testing/flashing Firmware you will need: 
- [Microchip Studio](https://www.microchip.com/en-us/tools-resources/develop/microchip-studio)
- A debugger for example *Atmel Power debugger*
- Gascounter Module
- Adapter from 6-Pin connector on Powerdebugger to 10-Pin on *gascounter module* specified [here](https://user-images.githubusercontent.com/85115389/188656912-2c82639d-4e30-490b-87a6-de65de5cbdc6.jpg)



### Preparations
Before first compiling the project in Microchip Studio the submodule of the avr-library needs to be initialized:
```
git submodule uptate --init
```


For further Information please refer to the [Wiki](https://github.com/SampleEnvironment/gascounter-module/wiki) pages


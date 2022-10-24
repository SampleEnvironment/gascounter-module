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
git submodule update --init
```

### Build configurations:

Buildconfigurations have the following Format:
```'DisplayType'-'NetworkInterface'```

| Display Type | Description                                            |
|--------------|--------------------------------------------------------|
| GCM_old_disp | old bw LCD display with low resolution                 |
| ili9341      | Newer 2.2" rgb Display with a resolution of 240x320px  |

| Network Interface | Description                                                                                             |
|-------------------|---------------------------------------------------------------------------------------------------------|
| XBEE              | Gascounter-module communicates via an xbee module with the server                                       |
| LAN               | Gascountermodule emulates an Xbee module and sends messages directly over Ethernet via an X-Port Module |




For further Information please refer to the [Wiki](https://github.com/SampleEnvironment/gascounter-module/wiki) pages


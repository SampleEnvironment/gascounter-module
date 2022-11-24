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

### Choosing a Release
In order to generate a executable Firmware of a specific release, `git tag` will list all available releases and 
```git checkout --recurse-submodules vX.XXX``` 
will load the release specified by `vX.XXX` (for example `v2.213`).

### Build configurations (from v2.212):
Build configurations are used to generate firmware binarys that are meant for the same device Type, but with varying Hardware, for example the display that is used. Build configuration names for gascounter modules have the following Format `'DisplayType'-'NetworkInterface'`, and can be set via the dropdown menu depicted in the screenshot below.

![build_configurations](https://user-images.githubusercontent.com/85115389/198265129-1811f6dd-a333-4995-9eb3-8800d6aded16.png)



| Display Type | Description                                            |
|--------------|--------------------------------------------------------|
| GCM_old_disp | old LC-display with low resolution                     |
| ili9341      | Newer 2.2" rgb Display with a resolution of 240x320px  |

| Network Interface | Description                                                                                             |
|-------------------|---------------------------------------------------------------------------------------------------------|
| XBEE              | Gascounter-module communicates via an xbee module with the server                                       |
| LAN               | Gascountermodule emulates an Xbee module and sends messages directly over Ethernet via an X-Port Module |




For further Information please refer to the [Wiki](https://github.com/SampleEnvironment/gascounter-module/wiki) pages


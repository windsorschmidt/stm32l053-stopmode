# About

Demonstrate low power "stop" mode on a STM32L0538-DISCO development board.

- The green user LED flashes while device is normal power mode
- Status messages are printed on USART1, with PA9=TXD and PA10=RXD
- Press the user pushbutton in normal mode to light the red LED and enter stop mode
- Press the user pushbutton in stop mode to wake the device and clear the red LED 

# Requirements

- STM32L0538-DISCO board connected via USB (at CN4 "USB USER")
- USB permissions to access the Discovery board's embedded ST-LINK debugger
- [GNU ARM embedded toolchain](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm)
- [OpenOCD](http://openocd.org)

# Building & Flashing Firmware

```sh
make flash
```

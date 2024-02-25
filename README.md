This article presents the hardware used for the coarse position of a motorized telescope.
![Telescope IMU Top](https://github.com/gvi70000/Telescope-coarse-position-system-using-BNO086-and-NEO-M9N-GNSS-module/assets/248221/7c301ff4-45a4-4cc1-9c5a-1c2d9c8a8eda)

![Telescope IMU Bottom](https://github.com/gvi70000/Telescope-coarse-position-system-using-BNO086-and-NEO-M9N-GNSS-module/assets/248221/b362e21c-a2c6-4972-a74f-7bb803ad52ed)

The position is given by the onboard 9 axis IMU BNO086 on I2C bus, the position on Earth is determined using NEO-M9N GNSS module on UART.

For astronomical calculation the pressure and temperature of the environment is read using BMP581 pressure and temperature sensor.

The communication with the telescope main controller is done on RS485 using ST485EBDR RS232 to RS485 converter and the level shifting between

3.3V and 5V is done using TXS0104E level shifter.

This device comes equipped also with 64kbyte SPI EEPROM to store user data.

The KiCAD files can be downloaded here.

The C library software for each IC:

BNO086 9 axis IMU- Link

BMP581 Temperature and pressure sensor- Link

NEO-M9N GNSS Module- Link

M95512 SPI EEPROM- Link

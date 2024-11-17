This article presents the hardware used for the coarse position of a motorized telescope.

![Telescope IMU Top](https://github.com/user-attachments/assets/c7920368-15aa-4253-a5ae-ce42f280257d)

![Telescope IMU Bottom](https://github.com/user-attachments/assets/cee1513d-d4e4-4df5-be2c-7e78b7368e82)

![Temperature-RH](https://github.com/user-attachments/assets/696bbb27-d3af-480f-b906-e289fd2123bc)

The position is given by the onboard 9 axis IMU BNO086 on I2C bus, the position on Earth is determined using NEO-M9N GNSS module on UART.

For astronomical calculation the pressure and temperature of the environment is read using BMP581 pressure and temperature sensor.

The communication with the telescope main controller is done on RS485 using ST485EBDR RS232 to RS485 converter and the level shifting between

3.3V and 5V is done using TXS0104E level shifter.

This device comes equipped also with 64kbyte SPI EEPROM to store user data.

The KiCAD files can be downloaded here.

The C library software for each IC are uploaded in this repository.

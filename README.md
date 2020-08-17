# mavlink-display
Arduino/OLED display for Mavlink telemetry

video for russian users: https://youtu.be/RXbNGYmCdYs

[![Watch the video](https://github.com/whoim2/mavlink-display/blob/master/photo_title.png?raw=true)](https://youtu.be/RXbNGYmCdYs)

Used Arduino Nano / Mini / Micro or other with h/w Serial and OLED i2c display 128x32 or another.

Arduino RX pin connected to LRS / RC controller telemetry output (TX), e.g., TX pin QCZEK LRS.

Device display current telemetry data: gps coords, battery data, heading, alt and other.
Store last data to eeprom and display on powered, if no fresh telemetry data.
If telemetry lost, display last data.

Build: connect OLED display to Arduino on gnd, vcc, scl, sda pins; install libs from used_libs.zip; upload sketch; connect GND and RX pin arduino to GND and TX pins rc/qczek telemetry output
Power on device. Enjoy, this is easy way to see last telemetry data on crash / lost your aircraft.

# mavlink-display
Arduino/OLED display for Mavlink telemetry

video for russian users: https://youtu.be/RXbNGYmCdYs

Used Arduino Nano / Mini / Micro or other with h/w Serial and OLED i2c display 128x32 or another.

Arduino RX pin connected to LRS / RC controller telemetry output (TX), e.g., TX pin QCZEK LRS.

Device display current telemetry data: gps coords, battery data, heading, alt and other.
Store last data to eeprom and display on power on, if no fresh telemetry.
If telemetry lost, display last data.

Build: connect display to Arduino on gnd, vcc, scl, sda pins; upload sketch; connect GND and RX pin arduino to GND and TX pins rc/qczek telemetry output
Power on device. Enjoy, this is easy way to see last telemetry data on crash / lost your aircraft.

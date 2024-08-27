# Project A.L.I.E.N. All Parts Test Code

<!-- Harry Boyd - 27/08/2024 - github.com/hboyd255 -->

This repository contains code that allows me to test all the parts of the
Project A.L.I.E.N. robot working at the same time.

This code combines the functionality of the 5 separate test codes that I wrote;

- [Motors and Encoders](https://github.com/HBoyd255/project-alien-motor-test-code)
- [Bumper and NES Shift Registers](https://github.com/HBoyd255/project-alien-shift-register-test-code)
- [LED Strip and Level Shifter](https://github.com/HBoyd255/project-alien-led-strip-test-code)
- [Ultrasonic Sensor](https://github.com/HBoyd255/project-alien-ultrasonic-test-code)
- [Infrared Sensors and I2C Multiplexer](https://github.com/HBoyd255/project-alien-infrared-test-code)

The program is designed to take a reading from the NES controller and drive the
robot in that direction.

The 8 bumpers will be read and displayed in green on the LED strip.

The ultrasonic sensor will be read and displayed by increasing the brightness of
the red LEDs under the sensor.

The ultrasonic sensor will be read and displayed by increasing the brightness of
the blue LEDs above each sensors.

The robot will also provide these values over serial.


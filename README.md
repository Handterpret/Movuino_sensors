# Movuino_sensors
## Introduction
This sofware aims to use MPU9250 sensors of the movuino to detect forearm movements.
# Hardware needed
I use the MPU6050 sensor.\
I developped it on a custom ESP32 card called movuino, which includes this sensor.
# Prerequisites
- Arduino Software
- MPU6050 library by Electronic cats

NB: I had to change some files of the MPU6050 library because there were creating declaration functions conflicts. If you have error logs like ```multiple definition of MPU6050 functions```, try to remove the `MPU6050_6Axis_MotionApps20.h` file located in your Arduino/Libraries folder.
# Installation

- Download this repository and prerequisites.
- Plug the movuino/arduino with MPU to your computer. Put the movuino/sensor on your forearm.
- Upload code, open Serial display with 115200 Baud rate.
- ! If the movement displayed are not correct, try to reinstall the sensor by turning it 90° !
# Documentation
All functions and class are documented [>here<](https://handterpret.github.io/Movuino_sensors/)

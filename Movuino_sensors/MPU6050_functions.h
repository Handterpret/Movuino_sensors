#ifndef MPUFUNCTIONS_H
#define MPUFUNCTIONS_H

#ifndef _MPU6050_6AXIS_MOTIONAPPS20_H_
    #include "MPU6050_6Axis_MotionApps20.h"
#endif
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"


void dmpDataReady();
// initial setup of mpu sensors
MPU6050 mpu_init(MPU6050 mpu);
float * get_mpu_data(MPU6050 mpu);
bool check_packet(MPU6050 mpu);
float * get_metrics(MPU6050 mpu);

#endif /* MPUFUNCTIONS_H */

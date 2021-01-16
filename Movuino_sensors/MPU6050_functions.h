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


/** \file
 * Utility functions to use MPU sensors.\n
 * Code was taken from the `MPU6050_DMP6` exemple script.
 */

//! Change `mpuInterrupt` to TRUE
void dmpDataReady();
/*!
 * Init mpu, its dependancies and Serial.
 * Calibrate MPU sensors.
 * @arg mpu previously instanciate
 * @return MPU6050 objet
 */
MPU6050 mpu_init(MPU6050 mpu);
/*!
 * Check if package is available to harvest. Then return metrics
 * @arg mpu
 * @return metrics array of values measured by MPU sensors
 */
float * get_mpu_data(MPU6050 mpu);
/*!
 * Check size of the packet send by mpu:
 * \n - If it is too big (overflow), reset the package tram and return false.
 * \n - Also return false if the packet is not full yet.
 * \n - Else, return true : packet is availabe
 * \n exit program if mpu initialisation failed
 * @arg mpu
 * @return true : packet available, false : packet unavailable
 */
bool check_packet(MPU6050 mpu);
/*!
 * Get metrics from sensor package and reset packetsize to reset packet.
 * \n Format metrics (gyroscope values in degrees and remove gravity influence for accelerometer)
 * and send them back.
 * @arg mpu
 * @return metrics array of values measured by MPU sensors
 */
float * get_metrics(MPU6050 mpu);

#endif /* MPUFUNCTIONS_H */

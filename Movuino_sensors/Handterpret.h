#ifndef HANDTERPRET_H
#define HANDTERPRET_H
#include "Arduino.h"

/** \file
 * Contain Handterpret class
 */

//!  Handterpret class.
/*!
  This class stores MPU sensor values.
  It aims to process these metrics to detect forearm movements.
*/
class Handterpret
{
    private:
        //variables
        //!  Accelerometer values, axis = 0:x, 1:y, 2:z
        int16_t Acc[3];
        //! Gyroscope values,  axis = 0:x, 1:y, 2:z
        float Gyro[3];
        //! Store period rotation measures
        float hist_Gyro[3][HISTORY_SIZE];
        //!  store previous average gyroscope
        float prev_av_Gyro[3];
        //!  Rotation state, 0 = not moving, 1 = negative rotation; 2 = positive rotation
        byte rot_state[3];
        //! Translation state, 0 = not moving, 1 = negative translation; 2 = positive translation
        byte trans_state[3];
        //! Index of the current position in the hist_gyro array
        byte hist_index;

        //functions
        //! Display detected rotation movement (exemple : supination, extension...)
        /**
         * @arg rot Rotation state
         * @arg axis = 0:x, 1:y, 2:z
         */
        void print_rotation(byte rot, byte axis);
        //! Display detected translation movement (exemple : forward, left...)
        /**
         * @arg acc Acceleration state
         * @arg axis = 0:x, 1:y, 2:z
         */
        void print_translation(byte acc, byte axis);
        //! Update rot and trans status and display when movement is detected
        /**
         * Calculate average rotation from historic,
         * detect if rotation and gyroscope status have significatively changed since last period
         * and update their status.
         * Display movement detected with print_translation() and discrimine_rotation() if a movement status changed
         */
        void check_movement();
        //! Compute the average of last period measured values.
        /**
         * @arg hist 1d array of sensor values
         */
        float average_historic(float hist[HISTORY_SIZE]);
        //! Determine what is the actual state of the rotation movement.
        /**
         * Compare previous and current average for one gyroscope axis. If it exceeds defined threshold ROTATION_SENSIBILITY
         * it returns 1 for a negative rotation, 2 for a positive rotation.
         * @arg av_gyro for the current period : average gyroscope value
         * @arg prev_av_gyro for the last period : average gyroscope value
         */
        byte detect_rotation_mov(float av_gyro, float prev_av_gyro);
        //! Determine if there is a translation movement.
        /**
         * Detect if `acc_val` exceeds ACCELERATION_SENSIBILITY for one accelerometer axis.
         * It returns 1 for a negative translation, 2 for a positive translation.
         * @arg acc_val current accelaration value
         */
        byte detect_translation_mov(int16_t acc_val);
        //! Determine the most likely movement made
        /**
         * Due to physical restriction and sensor noise, the movement made is not always easy to determine.
         * Some movements (especially flexion and extension) triggers sometimes multiple axis.
         * This function prioritize some of the axis , order was was done empirically.
         * Order : supination/pronation > flexion/extension > adduction/abduction
         * @arg current_rot_state array of current rotation status
         */
        void discrimine_rotation(byte current_rot_state[3]);

    public:
        //variables
        //! Number of measures stored per period
        const byte HISTORY_SIZE = 20;
        //! Time in ms, each period : rotation and translation status update
        const byte PERIOD_DURATION = 100;
        //! Rotation differentiel which define rotation movement
        const byte ROTATION_SENSIBILITY = 35;
        //! Acceleration differentiel which define translation movement
        const uint16_t ACCELERATION_SENSIBILITY = 2000;
        //! Time in ms, monitor time between each period
        unsigned long StartTime;

        //functions
        //! Default constructor
        Handterpret();
        //! Default destructor
        ~Handterpret();
        //! Fill metrics with current accelerometer and gyroscope measured values.
        /**
         * @param metrics the array filled with acc and gyro values
         */
        void get_metrics(float (&metrics)[6]);
        //! Update metrics with accelerometer and gyroscope and handle movement detection.
        /**
         * This function updates acc and gyro of handterpret class with metrics harvested from MPU6050.
         * Then for each period, it launchs the check_movement() function that updates rotation and translation status.
         * @param metrics the new metrics measured.
         */
        void update(float metrics[6]);
        //! Display Accelerometer and Gyroscope values
        void display_metrics();


};
#endif /* HANDTERPRET_H */

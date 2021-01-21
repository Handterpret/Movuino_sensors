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
    public:
        //variables
        //! Number of measures harvested per period
        static const byte MEASURES_PER_PERIOD = 20;
        //! Time in ms, each period : rotation and translation status update
        static const byte PERIOD_DURATION = 100;
        //! Rotation differentiel which define rotation movement
        static const byte ROTATION_SENSIBILITY = 20;
        //! Acceleration differentiel which define translation movement
        static const uint16_t ACCELERATION_SENSIBILITY = 2000;
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
        //! Display translation speeds detected each period
        void display_speeds();

    private:
        //variables
        //!  Accelerometer values, axis = 0:x, 1:y, 2:z
        int16_t Acc[3];
        //! Gyroscope values,  axis = 0:x, 1:y, 2:z
        float Gyro[3];
        //! Store period rotation measures
        float hist_Gyro[3];
        //! store speed calculated for each period per axis
        float trans_speed[3];
        //!  store previous average gyroscope
        float prev_av_Gyro[3];
        //!  Rotation state, 0 = not moving, 1 = negative rotation; 2 = positive rotation
        byte rot_state[3];
        //! Translation state, 0 = not moving, 1 = negative translation; 2 = positive translation
        byte trans_state[3];
        //! Index of the current number of measures harvested array
        byte measure_index;

        //functions
        //! Display detected rotation movement (exemple : supination, extension...)
        /**
         * @param rot Rotation state
         * @param axis = 0:x, 1:y, 2:z
         */
        void print_rotation(byte rot, byte axis);
        //! Display detected translation movement (exemple : forward, left...)
        /**
         * @param acc Acceleration state
         * @param axis = 0:x, 1:y, 2:z
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
         * @param hist 1d array of sensor values
         */
        float average_historic(float hist);
        //! Determine what is the actual state of the rotation movement.
        /**
         * Compare previous and current average for one gyroscope axis. If it exceeds defined threshold ROTATION_SENSIBILITY
         * it returns 1 for a negative rotation, 2 for a positive rotation.
         * @param av_gyro for the current period : average gyroscope value
         * @param prev_av_gyro for the last period : average gyroscope value
         */
        byte detect_rotation_mov(float av_gyro, float prev_av_gyro);
        //! Determine if there is a translation movement.
        /**
         * Detect if `acc_val` exceeds ACCELERATION_SENSIBILITY for one accelerometer axis.
         * It returns 1 for a negative translation, 2 for a positive translation.
         * @param acc_val current accelaration value
         */
        byte detect_translation_mov(float speed);
        //! Determine the most likely movement made
        /**
         * Due to physical restriction and sensor noise, the movement made is not always easy to determine.
         * Some movements (especially flexion and extension) triggers sometimes multiple axis.
         * This function prioritize some of the axis , order was was done empirically.
         * Order : supination/pronation > flexion/extension > adduction/abduction\n\n
         * NOTA BENE : Currently, as the translantion detection performance are better than the rotation detection,
         * we only use one axis to mesure rotation of the wrist on himself. The other axis movement can be detected
         * as translation movement.
         * @param current_rot_state array of current rotation status
         */
        void discrimine_rotation(byte current_rot_state[3]);
        //! Return an acc value if it is greater (or lower) than (-) `ACCELERATION_SENSIBILITY`.
        /**
         * @param acc raw accelerometer value
         * @return filtered value (0 or acc)
         */
        int16_t filter_acc(int16_t acc);
        //! Detect if a translation movement really happened
        /**
         * A "real" translation movement is defined here as a acceleration measure followed by a decceleration measure.
         * The function aims to detect the transition between those states two,
         * and print then the appropriate translation movement
         * @param current_trans_state the current state of the translation
         * @param post_trans_state the state measured at the last period
         * @param axis the translation axis
         */
        void find_tranlation_movement(byte current_trans_state, byte post_trans_state, byte axis);



};
#endif /* HANDTERPRET_H */

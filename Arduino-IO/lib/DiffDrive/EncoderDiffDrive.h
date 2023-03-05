#pragma once

//#include "DiffDrive.h"
#include "EncodedMotor.h"
#include <Arduino.h>

// typedef struct pose
typedef struct Pose{
    // x and y are in mm
    float x = 0;
    float y = 0;
    // theta is in radians
    float theta = 0;
    // d_x, d_y, and d_theta are the change in position and theta in mm/s and radians/s
    float d_x = 0;
    float d_y = 0;
    float d_theta = 0;
} Pose;
class EncoderDiffDrive{
    public:
        /**
         * @brief Construct a new EncoderDiffDrive object
         * @param leftMotor The left motor of the differential drive
         * @param rightMotor The right motor of the differential drive
         * @return None.
         */
        EncoderDiffDrive(EncodedMotor leftMotor, EncodedMotor rightMotor, float wheelSeparation);
        ~EncoderDiffDrive() = default;

        /**
         * @brief initialize the pins. MUST be called before any other function and during or after the setup() function in main.cpp.
         * @return None.
         */
        //void setup();

        /**
         * @brief Update motors
         * @param leftIncriment A pointer to the number of steps to incriment the left encoder count by. (positive or negative)
         * @param rightIncriment A pointer to the number of steps to incriment the right encoder count by. (positive or negative)
         * @post The incriment will be set to 0;
         * @return None.
         */
        void update(volatile int &leftIncriment, volatile int &rightIncriment);

        /**
         * @brief set the wheel radius for each motor
         * @param, wheelRadius The wheel radius of the motor in mm
         * @return None.
         */
        void setWheelRadius(float wheelRadius);

        /**
         * @brief Get the pose of the robot
         * @return Pose* A pointer to the current pose of the robot
         */
        Pose* getCurrentPose();

        /**
         * @brief Get the target pose of the robot. You can use this pointer to set the target pose of the robot.
         * @return Pose* A pointer to the target pose of the robot
         */
        Pose* getTargetPose();

        /**
         * @ brief set PID constants for each motor
         * @param kp The proportional constant
         * @param ki The integral constant
         * @param kd The derivative constant
         * @return None.
         */
        void setPID(float kp, float ki, float kd);

        /**
         * @brief Print out information about the encoder diff drive
         * @return None.
         */
        void print();

        /**
         * @brief Setup the motors
        */
        void setup();

        protected:
            /**
             * @brief Update the pose of the robot
             * @param leftVelocity The velocity of the left motor in mm/s
             * @param rightVelocity The velocity of the right motor in mm/s
             */
            void updatePose(float leftVelocity, float rightVelocity);

            EncodedMotor encodedLeftMotor;
            EncodedMotor encodedRightMotor;

            unsigned long lastTime = 0;
            Pose current_pose;
            Pose target_pose;
            float wheelSeparation = 0;
};
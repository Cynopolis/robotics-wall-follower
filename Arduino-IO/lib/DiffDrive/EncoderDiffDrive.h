#pragma once

//#include "DiffDrive.h"
#include "EncodedMotor.h"
#include "Pose.h"
#include <Arduino.h>

// TODO: Move the control loop from EncodedMotor to here

class EncoderDiffDrive{
    public:
        /**
         * @brief Construct a new EncoderDiffDrive object
         * @param leftMotor The left motor of the differential drive
         * @param rightMotor The right motor of the differential drive
         * @return None.
         */
        EncoderDiffDrive(EncodedMotor leftMotor, EncodedMotor rightMotor, double wheelSeperation);
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
        void setPID(double kp, double ki, double kd);

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
            EncodedMotor leftMotor;
            EncodedMotor rightMotor;

            unsigned long lastUpdateTime = 0;
            Pose targetPose;
            Pose currentPose;
            Pose sumError = {0, 0, 0, 0, 0, 0};
            Pose pastError = {0, 0, 0, 0, 0, 0};
            double wheelSeperation = 0;

            double kp = 1;
            double ki = 1;
            double kd = 1;

            unsigned long timer = 0;

};
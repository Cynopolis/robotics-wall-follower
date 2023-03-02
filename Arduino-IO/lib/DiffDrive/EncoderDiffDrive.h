#pragma once

//#include "DiffDrive.h"
#include "EncodedMotor.h"
#include <Arduino.h>

class EncoderDiffDrive{
    public:
        /**
         * @brief Construct a new EncoderDiffDrive object
         * @param leftMotor The left motor of the differential drive
         * @param rightMotor The right motor of the differential drive
         * @return None.
         */
        EncoderDiffDrive(EncodedMotor leftMotor, EncodedMotor rightMotor);
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
         * @brief Get the current angular velocity for the left motor
         * @return float The angular velocity of the left motor
         */
        float getLeftAngularVelocity();

        /**
         * @brief Get the current angular velocity for the right motor
         * @return float The angular velocity of the right motor
         */
        float getRightAngularVelocity();

        /**
         * @brief Get the linear velocity of the left wheel
         * @return a float for how far the left wheel has travelled in mm.
         */
        float getLeftVel();

        /**
         * @brief Get the linear velocity of the right wheel
         * @return a float for how far the right wheel has travelled in mm.
         */
        float getRightVel();

        /**
         * @brief Set the distance travelled by the left and right wheels in mm
         * @param leftDistance The distance travelled by the left wheel in mm
         * @param rightDistance The distance travelled by the right wheel in mm
         * @return None.
        */
        void setVelocity(int leftVelocity, int rightVelocity);

        /**
         * @brief Get the pose of the robot
         * @return a float array of the pose of the robot
         */
        float* getPose();

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
            EncodedMotor encodedLeftMotor;
            EncodedMotor encodedRightMotor;
};
#pragma once

#include "DiffDrive.h"
#include "EncodedMotor.h"

class EncoderDiffDrive : public DiffDrive {
    public:
        /**
         * @brief Construct a new EncoderDiffDrive object
         * @param leftMotor The left motor of the differential drive
         * @param rightMotor The right motor of the differential drive
         * @return None.
         */
        // TODO: This is a question for the prof. To use Encoded motor or do I have to use Motor?
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
        void update(long *leftIncriment, long *rightIncriment);

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
         * @brief Get the distance travelled by the left wheel
         * @return a float for how far the left wheel has travelled in mm.
         */
        float getLeftDistance();

        /**
         * @brief Get the distance travelled by the right wheel
         * @return a float for how far the right wheel has travelled in mm.
         */
        float getRightDistance();


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

        protected:
            EncodedMotor* encodedLeftMotor;
            EncodedMotor* encodedRightMotor;
};
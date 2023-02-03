#pragma once
#include "Motor.h"

class EncodedMotor : public Motor {
    public:
        EncodedMotor(int forwardPin, int backwardPin, int pwmPin, int encoderPinA, int encoderPinB);
        ~EncodedMotor() = default;

        /**
         * @brief initialize the pins. MUST be called before any other function and during or after the setup() function in main.cpp.
         */
        void setup() override;
        
        /**
         * @brief set the wheel radius
         * @param wheelRadius The wheel radius of the motor in mm
        */
        void setWheelRadius(float wheelRadius);

        /**
         * @brief Get the current actual velocity of the motor in mm/s
         * @return int The velocity of the motor
         */
        int getVelocity() override;
        
        /**
         * @brief Get the current actual angle of the motor
         * @return float The angle of the motor
         */
        float getAngularVelocity();

        /**
         * @brief Get the current actual distance traveled by the motor in mm
         * @return float The distance traveled by the motor
         */
        float getDistance();

        /**
         * @brief Set the PID constants of the motor
         * @param kp The proportional constant
         * @param ki The integral constant
         * @param kd The derivative constant
         * @pre before this function is called the first time, P = 1, I = 0, D = 0
         * @return None.
         */
        void setPID(float kp, float ki, float kd);

        /**
         * @brief updates the motor's state with current sensor data
         * @param incriment The number of steps to incriment the encoder count by. (positive or negative)
         * @return None.
         */
        void update(long incriment);

        /**
         * @brief print out the current state of the motor
         * @return None.
         */
        void print() override;

        /**
         * @brief Set the wheel radius of the motor
         * @param wheelRadius The wheel radius of the motor
        */
        void setWheelRadius(float wheelRadius);

    
    protected:
        int encoderPinA;
        int encoderPinB;
        long lastEncoderSteps = 0;
        long encoderSteps = 0;
        long targetEncoderSteps = 0;
        long sumError = 0;
        float wheelAngle = 0;
        float wheelRadius = 0.432;
        float kp = 1;
        float ki = 0;
        float kd = 0;

        // wheel diameter is 66 mm
        // The grear ratio is 120 motor turns : 1 wheel turn
        // the encoder has 16 steps per motor revolution
        // or 8*120 = 960 steps per wheel revolution
        // 66*PI = 207.345 mm per wheel revolution
        // 207.345/960 = 0.216 mm per step
        float stepsToMM = 0.432;
        float stepsPerRevolution = 8*120;
};
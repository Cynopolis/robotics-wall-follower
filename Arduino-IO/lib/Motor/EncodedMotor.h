#pragma once
#include "Motor.h"
#include <Arduino.h>

class EncodedMotor{
    public:
        EncodedMotor(uint8_t forwardPin, uint8_t backwardPin, uint8_t pwmPin, uint8_t encoderPinA, uint8_t encoderPinB);
        ~EncodedMotor() = default;
        
        /**
         * @brief set the wheel radius
         * @param wheelRadius The wheel radius of the motor in mm
        */
        void setWheelRadius(float wheelRadius);

        /**
         * @brief Get the current linear velocity of the motor in mm/s
         * @return int The velocity of the motor
         */
        float getVelocity();

        /**
         * @brief Get the current actual distance traveled by the motor in mm
         * @return float The distance traveled by the motor
         */
        float getDistance();

        /**
         * @brief updates the motor's state with current sensor data
         * @param incriment The number of steps to incriment the encoder count by. (positive or negative)
         * @post The incriment will be set to 0;
         * @return None.
         */
        void update(volatile int &incriment);

        /**
         * @brief Set the target distance of the motor in mm
         * @param targetVel The target velocity of the motor from 0-255 (unitless)
         * @return None.
         */
        void setTargetVelocity(int targetVelocity);

        /**
         * @brief print out the current state of the motor
         * @return None.
         */
        void print();

        /**
         * @brief Setup the motor
         * @return None.
         */
        void setup();
    
    protected:
        // pins
        uint8_t forwardPin;
        uint8_t backwardPin;
        uint8_t pwmPin;
        uint8_t encoderPinA;
        uint8_t encoderPinB;

        long lastEncoderSteps = 0;
        long encoderSteps = 0;
        long targetEncoderSteps = 0;
        long lastVelocity = 0;
        long sumError = 0;
        float wheelAngle = 0;
        float wheelRadius = 0.432;
        int kp = 100;
        int ki = 1;
        int kd = 10;

        // wheel diameter is 66 mm
        // The grear ratio is 120 motor turns : 1 wheel turn
        // the encoder has 16 steps per motor revolution
        // or 8*120 = 960 steps per wheel revolution
        // 66*PI = 207.345 mm per wheel revolution
        // 207.345/960 = 0.216 mm per step
        float stepsToMM = 0.432;
        static constexpr float stepsPerRevolution = 8*120; // 8*120 = 960;

        int target_velocity = 0;
        int past_velocity = 0;
        int maxVelocity = 255;
        float acceleration = 0.01;

        unsigned long lastTime = 0;

        /**
         * @brief Set the velocity of the motor
         * @param velocity The velocity to set the motor to
         * @return None.
         */
        void setVelocity(int velocity);
};
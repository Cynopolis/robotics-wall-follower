#pragma once
#include "Motor.h"
#include <Arduino.h>

class EncodedMotor{
    public:
        EncodedMotor(int forwardPin, int backwardPin, int pwmPin, int encoderPinA, int encoderPinB);
        ~EncodedMotor() = default;
        
        /**
         * @brief set the wheel radius
         * @param wheelRadius The wheel radius of the motor in mm
        */
        void setWheelRadius(float wheelRadius);

        /**
         * @brief Get the current actual velocity of the motor in mm/s
         * @return int The velocity of the motor
         */
        int getVelocity();
        
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
         * @post The incriment will be set to 0;
         * @return None.
         */
        void update(volatile int8_t &incriment);

        /**
         * @brief Set the target distance of the motor in mm
         * @param targetDistance The target distance of the motor in mm
         * @return None.
         */
        void setTargetDistance(float targetDistance);

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
        int encoderPinA;
        int encoderPinB;
        int lastEncoderSteps = 0;
        int encoderSteps = 0;
        int targetEncoderSteps = 0;
        float sumError = 0;
        float wheelAngle = 0;
        float wheelRadius = 0.432;
        float kp = 10;
        float ki = 0.1;
        float kd = 1;

        // wheel diameter is 66 mm
        // The grear ratio is 120 motor turns : 1 wheel turn
        // the encoder has 16 steps per motor revolution
        // or 8*120 = 960 steps per wheel revolution
        // 66*PI = 207.345 mm per wheel revolution
        // 207.345/960 = 0.216 mm per step
        float stepsToMM = 0.432;
        float stepsPerRevolution = 8*120;

        int target_velocity = 0;
        float current_velocity = 0;
        int maxVelocity = 100;
        float acceleration = 0.01;
        int forwardPin;
        int backwardPin;
        int pwmPin;
        unsigned long lastTime = 0;

        /**
         * @brief Set the velocity of the motor
         * @param velocity The velocity to set the motor to
         * @return None.
         */
        void setVelocity(float velocity);

        /**
         * @brief Accelerate the motor gradually to the target velocity
         * @return None.
         * @post The motor velocity will increase or decrease.
         */
        void accelerate(float dt);
};
#pragma once
#include "Motor.h"

class PositionMotor : public Motor{
    public:
        /**
         * @brief Construct a new Servo Motor object
         * @param pwmPin The pin to control the speed of the motor
         * @param pwmChannel The channel to control the speed of the motor
         * @param encoderCount The pointer to the encoder counter
         * @param minAngle The minimum angle of the servo in degrees
         * @param maxAngle The maximum angle of the servo in degrees
        */
        PositionMotor(uint8_t pwmPin, uint8_t pwmChannel, volatile int * encoderCount, uint16_t minAngle, uint16_t maxAngle);

        /**
         * @brief Construct a new Servo Motor object
         * @param forwardPin The pin to control the forward direction of the motor
         * @param backwardPin The pin to control the backward direction of the motor
         * @param pwmPin The pin to control the speed of the motor
         * @param pwmChannel The channel to control the speed of the motor
         * @param encoderCount The pointer to the encoder counter
         * @param minAngle The minimum angle of the servo in degrees
         * @param maxAngle The maximum angle of the servo in degrees
        */
        PositionMotor(uint8_t forwardPin, uint8_t backwardPin, uint8_t pwmPin, uint8_t pwmChannel, volatile int * encoderCount);
        ~PositionMotor() = default;

        /**
         * @brief Set the units per encoder pulse for this motor. IE: mm per pulse or degrees per pulse
        */
        void setUnitPerPulse(float unitPerPulse);

        /**
         * @brief Get the units per encoder pulse for this motor. IE: mm per pulse or degrees per pulse
        */
        float getUnitPerPulse();

        /**
         * @brief Set the target position of the motor
         * @param targetPosition The target position of the motor in units
        */
        void setTargetPosition(float targetPosition);

        // These are here as a compatibility layer for the parent motor class
        void setTargetVelocity(float targetVelocity) override {setTargetPosition(targetVelocity);};
        float getTargetVelocity() override {return getTargetPosition();};


        /**
         * @brief Get the target position of the motor
         * @return float The target position of the motor in units
        */
        float getTargetPosition();

        /**
         * @brief Get the current position of the motor
         * @return float The current position of the motor in units
        */
        float getCurrentPosition();

        /**
         * @brief return the last change in position
         * @return float The change in position
        */
        float getPositionChange(){return positionChange;};

        float getVelocity() override;

        /**
         * @brief Set the PID constants for the motor
         * @param kp The proportional constant
         * @param ki The integral constant
         * @param kd The derivative constant
         */
        void setPID(float kp, float ki, float kd);

        /**
         * @brief run any updates which need to be done continuously
         * @return float The current velocity of the motor
        */
        void update();
    
    protected:
        uint16_t minAngle;
        uint16_t maxAngle;

        float unitPerPulse = 1; // the units per number of encoder pulses. IE: mm per pulse or degrees per pulse
        volatile int * incriment;
        long lastEncoderCount = 0;
        long encoderCount = 0;
        float targetPosition;


        unsigned long lastTime = 0;
        float kp = 1;
        float ki = 0;
        float kd = 0;
        float lastError = 0;
        float integral = 0;
        float dt = 0.001;
        float currentVelocity = 0;
        float positionChange = 0;

        virtual float getError();
};
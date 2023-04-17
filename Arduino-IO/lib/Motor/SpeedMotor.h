#pragma once
#include "PositionMotor.h"

class SpeedMotor : public PositionMotor{
    public:
        /**
         * @brief Construct a new Speed Motor object
         * @param pwmPin The pin to control the speed of the motor
         * @param pwmChannel The channel to control the speed of the motor
         * @param encoderCount The pointer to the encoder counter
         * @param minAngle The minimum angle of the servo in degrees
         * @param maxAngle The maximum angle of the servo in degrees
        */
        SpeedMotor(uint8_t pwmPin, uint8_t pwmChannel, volatile int * encoderCount, uint16_t minAngle, uint16_t maxAngle);

        /**
         * @brief Construct a new Speed Motor object
         * @param forwardPin The pin to control the forward direction of the motor
         * @param backwardPin The pin to control the backward direction of the motor
         * @param pwmPin The pin to control the speed of the motor
         * @param pwmChannel The channel to control the speed of the motor
         * @param encoderCount The pointer to the encoder counter
         * @param minAngle The minimum angle of the servo in degrees
         * @param maxAngle The maximum angle of the servo in degrees
        */
        SpeedMotor(uint8_t forwardPin, uint8_t backwardPin, uint8_t pwmPin, uint8_t pwmChannel, volatile int * encoderCount);
        ~SpeedMotor() = default;

    protected:
        float getError() override;
};
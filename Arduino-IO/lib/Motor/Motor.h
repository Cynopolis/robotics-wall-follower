#pragma once
#include <Arduino.h>
#include <Servo.h>

class Motor{
    public:
        /**
         * @brief Construct a new motor object that uses a motor drive like the LM298N
         * @param forwardPin The pin to control the forward direction of the motor
         * @param backwardPin The pin to control the backward direction of the motor
         * @param pwmPin The pin to control the speed of the motor
         * @param incriment The number of steps to incriment the encoder count by. (positive or negative)
        */
        Motor(uint8_t forwardPin, uint8_t backwardPin, uint8_t pwmPin, uint8_t pwmChannel);

        /**
         * @brief Construct a new motor object that uses a servo-signal motor driver
         * @param pwmPin The pin to control the speed of the motor
         * @param pwmChannel the pwm channel to use. on the esp32 they are 0-15 and each one can control one motor.
        */
        Motor(uint8_t pwmPin, uint8_t pwmChannel);
        ~Motor() = default;

        /**
         * @brief Get the current linear velocity of the motor
         * @return float The velocity of the motor
         */
        virtual float getVelocity();

        /**
         * @brief This is here as a forward compatibility function for children classes to override
         */
        virtual float getTargetVelocity(){return currentVelocity;};

        /**
         * @brief this is here as a forward compatibility function for children classes to override
         */
        virtual void setTargetVelocity(float targetVelocity){setVelocity(targetVelocity);};

        /**
         * @brief run any updates which need to be done continuously
        */
        virtual float update();

        /**
         * @brief Setup the motor
         * @return None.
         */
        virtual void begin();

        /**
         * @brief Set the velocity of the motor
         * @param velocity The velocity to set the motor to. (Range: -255 to 255)
         * @return None.
         */
        virtual void setVelocity(int velocity);
    
    protected:
        // pins
        uint8_t forwardPin;
        uint8_t backwardPin;
        uint8_t pwmPin;
        uint8_t pwmChannel;
        Servo servo;

        float currentVelocity = 0;

    private:
        void setServoVelocity(int velocity);
        void setPWMVelocity(int velocity);
};
#pragma once
#include <Arduino.h>

class Motor{
    public:
        /**
         * @brief Construct a new Encoded Motor object
         * @param forwardPin The pin to control the forward direction of the motor
         * @param backwardPin The pin to control the backward direction of the motor
         * @param pwmPin The pin to control the speed of the motor
         * @param incriment The number of steps to incriment the encoder count by. (positive or negative)
        */
        Motor(uint8_t forwardPin, uint8_t backwardPin, uint8_t pwmPin, volatile int* incriment);
        ~Motor() = default;
        
        /**
         * @brief set the wheel radius
         * @param wheelRadius The wheel radius of the motor in mm
        */
        void setWheelRadius(float wheelRadius);
        
        /**
         * @brief get the current wheel radius
         * @return float The wheel radius of the motor in mm
        */
        float getWheelRadius();

        /**
         * @brief Get the current linear velocity of the motor in mm/s
         * @return int The velocity of the motor
         */
        float getVelocity();

        /**
         * @brief updates the motor's state with current sensor data
         * @param incriment The number of steps to incriment the encoder count by. (positive or negative)
         * @post The incriment will be set to 0;
         * @return None.
         */
        float update();

        /**
         * @brief Setup the motor
         * @return None.
         */
        void begin();

        /**
         * @brief Set the velocity of the motor
         * @param velocity The velocity to set the motor to
         * @return None.
         */
        void setVelocity(int velocity);

        /**
         * @brief Get the distance the motor has travelled since the last update
         * @return float The distance the motor has travelled in mm
         */
        float getDistanceSinceLastUpdate();
    
    protected:
        // pins
        uint8_t forwardPin;
        uint8_t backwardPin;
        uint8_t pwmPin;

        long encoderSteps = 0;
        long lastEncoderSteps = 0;
        float wheelRadius = 33;
        float currentVelocity = 0;
        float distanceSinceLastUpdate = 0;
        unsigned long lastTime = 0;
        volatile int* incriment;

        // wheel diameter is 66 mm
        // The grear ratio is 120 motor turns : 1 wheel turn
        // the encoder has 16 steps per motor revolution
        // or 8*120 = 960 steps per wheel revolution
        // 66*PI = 207.345 mm per wheel revolution
        // 207.345/960 = 0.216 mm per step
        float stepsToMM = 0.432;
        static constexpr float stepsPerRevolution = 8*120; // 8*120 = 960;
        int targetVelocity = 0;

        unsigned long timer = 0;
};
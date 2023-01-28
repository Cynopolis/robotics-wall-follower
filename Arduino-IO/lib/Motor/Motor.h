#pragma once

class Motor {
    public:
        /**
         * @brief Construct a new Motor object
         * @param forwardPin The pin to use for forward direction
         * @param backwardPin The pin to use for backward direction
         * @param pwmPin The pin to use for PWM
         * @return None.
         */
        Motor(int forwardPin, int backwardPin, int pwmPin);
        ~Motor() = default;

        /**
         * @brief Set the target velocity of the motor
         * @param targetVelocity The target velocity of the motor
         * @return None.
         */
        void setTargetVelocity(int targetVelocity);

        /**
         * @brief Get the target velocity of the motor
         * @return int The target velocity of the motor
         */
        int getTargetVelocity();

        /**
         * @brief Get the velocity of the motor
         * @return int The velocity of the motor
         */
        int getVelocity();

        /**
         * @brief Set the maximum velocity of the motor
         * @param maxVelocity The maximum velocity of the motor
         * @return None.
         */
        void setMaxVelocity(int maxVelocity);

        /**
         * @brief Get the maximum velocity of the motor
         * @return int The maximum velocity of the motor
         */
        int getMaxVelocity();

        /**
         * @brief Set the acceleration of the motor
         * @param acceleration The acceleration of the motor
         * @return None.
         */
        void setAcceleration(int acceleration);

        /**
         * @brief Get the acceleration of the motor
         * @return int The acceleration of the motor
         */
        int getAcceleration();

        /**
         * @brief updates the current velocity of the motor
         * @param time The current time in milliseconds
         * @return None.
         */
        void update();

    protected:
        int target_velocity = 0;
        int current_velocity = 0;
        int maxVelocity = 1000;
        int acceleration = 100;
        int forwardPin;
        int backwardPin;
        int pwmPin;
        unsigned long lastTime = 0;

        /**
         * @brief Set the velocity of the motor
         * @param velocity The velocity to set the motor to
         * @return None.
         */
        void setVelocity(int velocity);

};
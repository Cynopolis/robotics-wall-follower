#pragma once
#include "Motor.h"

class DiffDrive{
    public:
        /**
         * @brief Construct a new DiffDrive object
         * @param leftMotor The left motor of the differential drive
         * @param rightMotor The right motor of the differential drive
         * @return None.
         */
        DiffDrive(Motor leftMotor, Motor rightMotor);
        ~DiffDrive() = default;

        /**
         * @brief Set the target velocity of the differential drive
         * @param leftTargetVelocity The target velocity of the left motor in mm/s
         * @param rightTargetVelocity The target velocity of the right motor in mm/s
         * @return None.
         */
        void setTargetVelocity(int leftTargetVelocity, int rightTargetVelocity);
        
        /**
         * @brief Set the target velocity of the differential drive
         * @param maxVelocity The maximum velocity of the differential drive
         * @return None.
         */
        void setMaxVelocity(int maxVelocity);

        /**
         * @brief Set the acceleration of the differential drive
         * @param maxAcceleration The maximum acceleration of the differential drive
         * @return None.
         */
        void setAcceleration(int maxAcceleration);

        /**
         * @brief Set the direction vector of the differential drive
         * @param magnitude The magnitude of the direction vector 0-100
         * @param angle The angle of the direction vector 0-360 degrees
         * @return None.
         */
        void setDirectionVector(float magnitude, float angle);

        /**
         * @brief Update motors
         * @return None.
         */
        virtual void update();

        /**
         * @brief Get the acceleration constant for the motors
         * @return The acceleration constant for the motors
         */
        int getAcceleration();
        
        /**
         * @brief Get the maximum velocity of the motors
         * @return The maximum velocity of the motors
         */
        int getMaxVelocity();

        /**
         * @brief Get the left motor velocity
         * @return The current velocity of the left motor
         */
        int getLeftCurrentVelocity();

        /**
         * @brief Get the right motor velocity
         * @return The current velocity of the right motor
         */
        int getRightCurrentVelocity();

        /**
         * @brief Get the left target motor velocity
         * @return The target velocity of the left motor
         */
        int getLeftTargetVelocity();

        /**
         * @brief Get the right target motor velocity
         * @return The target velocity of the right motor
         */
        int getRightTargetVelocity();

    protected:
        Motor leftMotor;
        Motor rightMotor;

};
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
         * @param magnitude The magnitude of the direction vector
         * @param angle The angle of the direction vector
         * @return None.
         */
        void setDirectionVector(float magnitude, float angle);

        /**
         * @brief Update motors
         * @return None.
         */
        virtual void update();

    protected:
        Motor leftMotor;
        Motor rightMotor;

};
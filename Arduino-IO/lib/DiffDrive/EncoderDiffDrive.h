#pragma once

#include "DiffDrive.h"
#include "EncodedMotor.h"

class EncoderDiffDrive : public DiffDrive {
    public:
        /**
         * @brief Construct a new EncoderDiffDrive object
         * @param leftMotor The left motor of the differential drive
         * @param rightMotor The right motor of the differential drive
         * @return None.
         */
        // TODO: This is a question for the prof. To use Encoded motor or do I have to use Motor?
        EncoderDiffDrive(EncodedMotor leftMotor, EncodedMotor rightMotor);
        ~EncoderDiffDrive() = default;

        /**
         * @brief Update motors
         * @param leftIncriment The number of steps to incriment the left encoder count by. (positive or negative)
         * @param rightIncriment The number of steps to incriment the right encoder count by. (positive or negative)
         * @return None.
         */
        void update(long leftIncriment, long rightIncriment);

    protected:
        EncodedMotor leftMotor;
        EncodedMotor rightMotor;
};
#include "DiffDrive.h"
#include "Encoder.h"

class DiffDriveFeedback : public DiffDrive {
    public:
        /**
         * @brief Construct a new DiffDriveFeedback object
         * @param leftMotor The left motor of the differential drive
         * @param rightMotor The right motor of the differential drive
         * @param leftEncoder The left encoder of the differential drive
         * @param rightEncoder The right encoder of the differential drive
         * @return None.
         */
        DiffDriveFeedback(Motor leftMotor, Motor rightMotor, Encoder leftEncoder, Encoder rightEncoder);
        ~DiffDriveFeedback() = default;

        /**
         * @brief Update motors
         * @return None.
         */
        void update();

        /**
         * @brief Set the PID constants
         * @param kp The proportional constant
         * @param ki The integral constant
         * @param kd The derivative constant
         * @return None.
         */
        void setPID(float kp, float ki, float kd);

        /**
         * @brief Set the target distance and target angle of the differential drive
         * @param targetDistance The target distance of the differential drive
         * @param targetAngle The target angle of the differential drive
         * @return None.
         */
        void setTarget(float targetDistance, float targetAngle);

        /**
         * @brief Set the current position of the differential drive
         * @param angle The current angle of the differential drive
         * @param distance The current distance of the differential drive
         * @return None.
         */
        void setCurrentPosition(double angle, double distance);

        /**
         * @brief Get the current distance of the differential drive
         * @return double The current distance of the differential drive
         */
        double getDistance();

        /**
         * @brief Get the current angle of the differential drive
         * @return double The current angle of the differential drive
         */
        double getAngle();

    private:
        void getOrientation();
    protected:
        Encoder leftEncoder;
        Encoder rightEncoder;

        float kp = 1;
        float ki = 0.005;
        float kd = 0.2;

        unsigned long lastTime = 0;

        double lastDistance = 0;
        double distance = 0;
        double targetDistance = 0;
        double sumDistanceError = 0;

        double lastAngle = 0;
        double angle = 0;
        double targetAngle = 0;
        double sumAngleError = 0;

        // wheel diameter is 66 mm
        // The grear ratio is 120 motor turns : 1 wheel turn
        // the encoder has 8 steps per motor revolution
        // or 8*120 = 960 steps per wheel revolution
        // 66*PI = 207.345 mm per wheel revolution
        // 207.345/960 = 0.216 mm per step
        const float stepsToMM = 0.216;
        const float wheelBase = 145; // in mm
};
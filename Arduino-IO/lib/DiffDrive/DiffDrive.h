#pragma once
#include "Pose.h"
#include "Motor.h"

class DiffDrive{
    public:

        /**
         * @brief Create a new DiffDrive object
         * @param leftMotor A pointer to the left motor
         * @param rightMotor A pointer to the right motor
         * @param wheelSeparation The distance between the wheels in mm
        */
        DiffDrive(Motor* leftMotor, Motor* rightMotor, float wheelSeparation);

        /**
         * @brief Set the PID constants for the controller
         * @param kp The proportional constant
         * @param ki The integral constant
         * @param kd The derivative constant
        */
        void setPID(float kp, float ki, float kd);

        /**
         * @brief Get the current pose of the robot. This is how you will set and read the current pose
         * @return A pointer to the current pose
        */
        Pose* getCurrentPose();

        /**
         * @brief Get the target pose of the robot. This is how you will set the target pose
         * @return A pointer to the target pose
        */
        Pose* getTargetPose();

        /**
         * @brief Begin the controller
        */
        void begin();

        /**
         * @brief Update the controller
        */
        void update();


    private:
        Motor* leftMotor;
        Motor* rightMotor;
        float wheelSeparation;

        float kp = 1;
        float ki = 0;
        float kd = 0;

        Pose currentPose;
        Pose targetPose;

        unsigned long lastTime;
};
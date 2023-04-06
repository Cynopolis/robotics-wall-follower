#pragma once
#include "Motor.h"
#include "IMU.h"
#include "xyzData.h"
#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))
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
         * @brief Get the current pose of the robot
         * @return float* A pointer to an array of floats containing the current pose of the robot
        */
       xyzData getCurrentPose();

        /**
         * @brief Set the current pose of the robot
         * @param x The x coordinate of the robot in mm
         * @param y The y coordinate of the robot in mm
         * @param theta The angle of the robot in radians
        */
       void setCurrentPose(float x, float y, float theta);

        /**
         * @brief Get the target pose of the robot
         * @return float* A pointer to an array of floats containing the target pose of the robot
        */
        xyzData getTargetPose();

        /**
         * @brief Set the target pose of the robot
         * @param x The x coordinate of the robot in mm
         * @param y The y coordinate of the robot in mm
         * @param theta The angle of the robot in radians
        */
        void setTargetPose(float x, float y, float theta);


        /**
         * @brief Begin the controller
        */
        void begin();

        /**
         * @brief Update the controller
         * @param imu The IMU to use

        */
        void update(IMU* imu = nullptr);


    private:

        /**
         * @brief Wrap an angle to be between -PI and PI
         * @param angle The angle to wrap
         * @return float The wrapped angle
        */
        float wrap_angle(float angle);

        /**
         * @brief Calculate the shortest difference between two angles
         * @param a The first angle
         * @param b The second angle
         * @return float The difference between the two angles
        */
        float angleDiff(float a, float b);

        Motor* leftMotor;
        Motor* rightMotor;
        float wheelSeparation;

        float k_rho = 1;
        float k_alpha = 0;
        float k_beta = 0;
        
        // The current pose of the robot. X position, y position in mm, and z stores the angle in radians
        xyzData currentPose;
        float wheelAngle = 0;
        xyzData targetPose;

        float lastVel = 0;

        unsigned long lastTime;
        bool isReversed = false;
        constexpr static float TAU = 2*PI;
        
};
#pragma once
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
         * @brief Get the current pose of the robot
         * @return float* A pointer to an array of floats containing the current pose of the robot
        */
       float* getCurrentPose();

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
        float* getTargetPose();

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
        */
        void update();


    private:

        /**
         * @brief update the current pose of the robot
         * @return none
        */
       void updatePose(float dt, float leftDis, float rightDis);

       /**
        * @brief calculate the new velocities for the motors based on the current and target poses.
        * @return none
       */
      void calcMotorVels(float dt, float leftDis, float rightDis);

        Motor* leftMotor;
        Motor* rightMotor;
        float wheelSeparation;

        float k_rho = 1;
        float k_alpha = 0;
        float k_beta = 0;

        float current_x = 0;
        float current_y = 0;
        float current_theta = 0;
        
        float target_x = 0;
        float target_y = 0;
        float target_theta = 0;

        float last_error_x = 0;
        float last_error_y = 0;
        float last_error_theta = 0;

        float sum_error_x = 0;
        float sum_error_y = 0;
        float sum_error_theta = 0;

        float currentPose[3] = {current_x, current_y, current_theta};
        float targetPose[3] = {target_x, target_y, target_theta};

        unsigned long lastTime;

        constexpr static float TAU = 2*PI;

        float angleDiff(float a, float b);

        /**
         * @brief Wrap an angle to be between -PI and PI
         * @param angle The angle to wrap
         * @return float The wrapped angle
        */
        float wrap_angle(float angle);
};
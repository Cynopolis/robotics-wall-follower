#include "DiffDrive.h"
#include <Arduino.h>

DiffDrive::DiffDrive(SpeedMotor* leftMotor, SpeedMotor* rightMotor, float wheelSeparation) :
    leftMotor(leftMotor), rightMotor(rightMotor), wheelSeparation(wheelSeparation) {}

void DiffDrive::setVelocityPID(float kp, float ki, float kd) {
    this->velPID.x = kp;
    this->velPID.y = ki;
    this->velPID.z = kd;
}

void DiffDrive::setAnglePID(float kp, float ki, float kd) {
    this->anglePID.x = kp;
    this->anglePID.y = ki;
    this->anglePID.z = kd;
}

xyzData DiffDrive::getCurrentPose() {
    return currentPose;
}

void DiffDrive::setCurrentPose(float x, float y, float theta) {
    this->currentPose = xyzData(x, y, theta);
}

xyzData DiffDrive::getTargetPose() {
    return targetPose;
}

void DiffDrive::setTargetPose(float velocity, float theta) {
    theta = wrap_angle(theta);
    this->targetPose = xyzData(velocity, 0, theta);
}

void DiffDrive::begin() {
    lastTime = millis();
    leftMotor->begin();
    rightMotor->begin();
}

// bound between -pi and pi
float DiffDrive::wrap_angle(float angle) {
    // First, wrap the angle between -2*pi and 2*pi radians
    angle = fmod(angle + PI, TAU) - PI;

    // Then, bound the angle between -pi and pi radians
    if (angle < -PI) {
        angle += TAU;
    } else if (angle > PI) {
        angle -= TAU;
    }

    return angle;
}

void DiffDrive::update(IMU* imu) {
    /**
     * This section calculates the time since the last update
    */
    unsigned long now = millis();
    long time_diff = now - lastTime;
    if(time_diff < 1) return;
    float dt = float(time_diff) / 1000.0;
    lastTime = now;

    /**
     * This section updates the current pose of the robot
    */
    float leftDis = leftMotor->update();
    float rightDis = rightMotor->update();

    float d_pos = (leftDis + rightDis) / 2.0;
    float d_theta = (rightDis - leftDis) / wheelSeparation;
    
    wheelAngle += d_theta;
    wheelAngle = wrap_angle(wheelAngle);

    
    if(imu != nullptr) {
        if(abs(d_theta) < 0.001 || abs(d_pos) < 0.001){
            imu->freezeGyro(true);
        }
        else{
            imu->freezeGyro(false);
        }
        imu->update();

        currentPose.z = imu->getOrientation().z;

        if(abs(imu->getOrientationChange().z - d_theta) < 0.0065){
            // no slip detected
            currentPose.x += d_pos * cos(currentPose.z);
            currentPose.y += d_pos * sin(currentPose.z);
        }
    } else {
        currentPose.z = wheelAngle;
        currentPose.x += d_pos * cos(currentPose.z);
        currentPose.y += d_pos * sin(currentPose.z);
    }
    

    

    /**
     * This section calculates the new velocities for the motors
    */
    // calculate the error between the current pose and the target pose
    xyzData error = targetPose - currentPose;
    sumError = sumError + (error * dt);
    xyzData dError = (error - lastError) / dt;
    lastError = error;

    // calculate the pid values
    float vel = velPID.x * error.x + velPID.y * sumError.x + velPID.z * dError.x;
    float angle = anglePID.x * error.z + anglePID.y * sumError.z + anglePID.z * dError.z;

    // calculate the left and right velocities
    float leftVel = vel - angle * wheelSeparation / 2.0;
    float rightVel = vel + angle * wheelSeparation / 2.0;

    // cap the velocities
    float maxVel = 200;
    if(abs(leftVel) > maxVel) {
        float percent = maxVel / abs(leftVel);
        leftVel *= percent;
        rightVel *= percent;
    }
    if(abs(rightVel) > maxVel) {
        float percent = maxVel / abs(rightVel);
        leftVel *= percent;
        rightVel *= percent;
    }

    // set the new velocities
    leftMotor->setVelocity(leftVel);
    rightMotor->setVelocity(rightVel);
}

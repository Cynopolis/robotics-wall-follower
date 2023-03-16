#include "DiffDrive.h"

DiffDrive::DiffDrive(Motor* leftMotor, Motor* rightMotor, float wheelSeparation) : 
leftMotor(leftMotor), rightMotor(rightMotor), wheelSeparation(wheelSeparation){}

void DiffDrive::setPID(float kp, float ki, float kd){
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

Pose* DiffDrive::getCurrentPose(){
    return &currentPose;
}

Pose* DiffDrive::getTargetPose(){
    return &targetPose;
}

void DiffDrive::begin(){
    lastTime = millis();
    leftMotor->begin();
    rightMotor->begin();
}

void DiffDrive::update(){
    // calculate dt
    unsigned long now = millis();
    int time_diff = int(now - lastTime);
    if(time_diff < 1) return;
    lastTime = now;
    float dt = float(time_diff)*0.001;

    // calculate the error
    float xError = targetPose.x - currentPose.x;
    float yError = targetPose.y - currentPose.y;
    float thetaError = targetPose.theta - currentPose.theta;

    // calculate the velocity
    float v = kp * sqrt(xError*xError + yError*yError);
    float w = kp * thetaError;

    // calculate the wheel velocities
    float leftWheelVelocity = (v - (wheelSeparation/2)*w) / leftMotor->getWheelRadius();
    float rightWheelVelocity = (v + (wheelSeparation/2)*w) / rightMotor->getWheelRadius();

    // set the wheel velocities
    leftMotor->setVelocity(leftWheelVelocity);
    rightMotor->setVelocity(rightWheelVelocity);

    // update the motors
    leftMotor->update();
    rightMotor->update();

    // update the current pose
    currentPose.x += leftMotor->getVelocity() * dt;
    currentPose.y += rightMotor->getVelocity() * dt;
    currentPose.theta += (rightMotor->getVelocity() - leftMotor->getVelocity()) * dt / wheelSeparation;
}
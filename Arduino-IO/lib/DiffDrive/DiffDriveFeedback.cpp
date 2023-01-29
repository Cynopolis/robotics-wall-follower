#include "DiffDriveFeedback.h"
#include "Arduino.h"

DiffDriveFeedback::DiffDriveFeedback(Motor leftMotor, Motor rightMotor, Encoder leftEncoder, Encoder rightEncoder)
 : DiffDrive(leftMotor, rightMotor), leftEncoder(leftEncoder), rightEncoder(rightEncoder) {
    this->lastTime = millis();
 }

 void DiffDriveFeedback::setPID(float kp, float ki, float kd){
     this->kp = kp;
     this->ki = ki;
     this->kd = kd;
 }

void DiffDriveFeedback::getOrientation() {
    double leftDistance = double(leftEncoder.read()) * stepsToMM;
    double rightDistance = double(rightEncoder.read()) * stepsToMM;
    
    this->lastDistance = distance;
    this->distance = (leftDistance + rightDistance) / 2;
    //TODO: check if this is correct
    this->lastAngle = angle;
    this->angle = (rightDistance - leftDistance) / wheelBase;
}

 void DiffDriveFeedback::update(){
    int dt = millis() - lastTime;
    this->lastTime = millis();

    getOrientation();

    double distanceError = targetDistance - distance;
    double angleError = targetAngle - angle;

    // calculate new velocity PID
    this->sumDistanceError += distanceError * dt;
    double diff = kd* (distance - lastDistance) / dt;
    double newVelocity = kp * distanceError + diff + ki * sumDistanceError;

    // calculate new angle PID
    this->sumAngleError += angleError * dt;
    diff = kd* (angle - lastAngle) / dt;
    double newAngleVelocity = kp * angleError + diff + ki * sumAngleError;

    // calculate left and right velocity
    //TODO: check if this is correct
    double leftVelocity = newVelocity - newAngleVelocity * wheelBase / 2;
    double rightVelocity = newVelocity + newAngleVelocity * wheelBase / 2;

    // set motor velocity
    leftMotor.setTargetVelocity(leftVelocity);
    rightMotor.setTargetVelocity(rightVelocity);

    // update motors
    leftMotor.update();
    rightMotor.update();
 }

void DiffDriveFeedback::setTarget(float targetDistance, float targetAngle) {
    this->targetDistance = targetDistance;
    this->targetAngle = targetAngle;
}

void DiffDriveFeedback::setCurrentPosition(double angle, double distance) {
    this->angle = angle;
    this->distance = distance;

    //TODO: check if this is correct. It's unlikely that it is
    long leftEncoderSteps = distance / stepsToMM + angle * wheelBase / 2 / stepsToMM;
    long rightEncoderSteps = distance / stepsToMM - angle * wheelBase / 2 / stepsToMM;
    leftEncoder.write(leftEncoderSteps);
    rightEncoder.write(rightEncoderSteps);
}
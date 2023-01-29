#include "DiffDrive.h"
#include <Arduino.h>

DiffDrive::DiffDrive(Motor leftMotor, Motor rightMotor) 
    : leftMotor(leftMotor), rightMotor(rightMotor)
{
}

void DiffDrive::setTargetVelocity(int leftTargetVelocity, int rightTargetVelocity) {
    leftMotor.setTargetVelocity(leftTargetVelocity);
    rightMotor.setTargetVelocity(rightTargetVelocity);
}

void DiffDrive::setMaxVelocity(int maxVelocity) {
    leftMotor.setMaxVelocity(maxVelocity);
    rightMotor.setMaxVelocity(maxVelocity);
}

void DiffDrive::setAcceleration(int maxAcceleration) {
    leftMotor.setAcceleration(maxAcceleration);
    rightMotor.setAcceleration(maxAcceleration);
}

void DiffDrive::setDirectionVector(float magnitude, float angle) {
    angle = angle * PI / 180;
    int leftTargetVelocity = int(magnitude * cos(angle));
    int rightTargetVelocity = int(magnitude * sin(angle));
    setTargetVelocity(leftTargetVelocity, rightTargetVelocity);
}

void DiffDrive::update() {
    leftMotor.update();
    rightMotor.update();
}


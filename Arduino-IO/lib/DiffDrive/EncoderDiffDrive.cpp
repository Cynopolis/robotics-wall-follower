#include "EncoderDiffDrive.h"

EncoderDiffDrive::EncoderDiffDrive(EncodedMotor leftMotor, EncodedMotor rightMotor):
    DiffDrive(leftMotor, rightMotor){
        this->encodedLeftMotor = &leftMotor;
        this->encodedRightMotor = &rightMotor;
    }

void EncoderDiffDrive::update(long *leftIncriment, long *rightIncriment){
    // some funky pointer magic to keep things going
    this->encodedLeftMotor->update(leftIncriment);
    this->encodedRightMotor->update(rightIncriment);
}

void EncoderDiffDrive::setWheelRadius(float wheelRadius){
    this->encodedLeftMotor->setWheelRadius(wheelRadius);
    this->encodedRightMotor->setWheelRadius(wheelRadius);
}

float EncoderDiffDrive::getLeftAngularVelocity(){
    return this->encodedLeftMotor->getAngularVelocity();
}

float EncoderDiffDrive::getRightAngularVelocity(){
    return this->encodedRightMotor->getAngularVelocity();
}

void EncoderDiffDrive::setPID(float kp, float ki, float kd){
    this->encodedLeftMotor->setPID(kp, ki, kd);
    this->encodedRightMotor->setPID(kp, ki, kd);
}

void EncoderDiffDrive::print(){
    this->encodedLeftMotor->print();
    this->encodedRightMotor->print();
}
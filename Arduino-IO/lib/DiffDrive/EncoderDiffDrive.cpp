#include "EncoderDiffDrive.h"

EncoderDiffDrive::EncoderDiffDrive(EncodedMotor leftMotor, EncodedMotor rightMotor):
    encodedLeftMotor(leftMotor), encodedRightMotor(rightMotor){}

void EncoderDiffDrive::update(volatile int &leftIncriment, volatile int &rightIncriment){
    // some funky pointer magic to keep things going
    this->encodedLeftMotor.update(leftIncriment);
    this->encodedRightMotor.update(rightIncriment);

    //TODO: sum velocities and update pose
}

void EncoderDiffDrive::setWheelRadius(float wheelRadius){
    this->encodedLeftMotor.setWheelRadius(wheelRadius);
    this->encodedRightMotor.setWheelRadius(wheelRadius);
}

float EncoderDiffDrive::getLeftAngularVelocity(){
    return this->encodedLeftMotor.getAngularVelocity();
}

float EncoderDiffDrive::getRightAngularVelocity(){
    return this->encodedRightMotor.getAngularVelocity();
}

void EncoderDiffDrive::setPID(float kp, float ki, float kd){
    this->encodedLeftMotor.setPID(kp, ki, kd);
    this->encodedRightMotor.setPID(kp, ki, kd);
}

void EncoderDiffDrive::print(){
    this->encodedLeftMotor.print();
    this->encodedRightMotor.print();
}

float EncoderDiffDrive::getLeftVel(){
    return this->encodedLeftMotor.getVelocity();
}

float EncoderDiffDrive::getRightVel(){
    return this->encodedRightMotor.getVelocity();
}

void EncoderDiffDrive::setup(){
    this->encodedLeftMotor.setup();
    this->encodedRightMotor.setup();
}

void EncoderDiffDrive::setVelocity(int leftVelocity, int rightVelocity){
    this->encodedLeftMotor.setTargetVelocity(leftVelocity);
    this->encodedRightMotor.setTargetVelocity(rightVelocity);
}

//TODO: Impliment pose


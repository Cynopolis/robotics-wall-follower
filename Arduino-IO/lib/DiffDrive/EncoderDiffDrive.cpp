#include "EncoderDiffDrive.h"

EncoderDiffDrive::EncoderDiffDrive(EncodedMotor leftMotor, EncodedMotor rightMotor, float wheelSeparation):
    encodedLeftMotor(leftMotor), encodedRightMotor(rightMotor), wheelSeparation(wheelSeparation){}

void EncoderDiffDrive::update(volatile int &leftIncriment, volatile int &rightIncriment){
    this->encodedLeftMotor.update(leftIncriment);
    this->encodedRightMotor.update(rightIncriment);

    this->updatePose(this->encodedLeftMotor.getVelocity(), this->encodedRightMotor.getVelocity());
}

void EncoderDiffDrive::setWheelRadius(float wheelRadius){
    this->encodedLeftMotor.setWheelRadius(wheelRadius);
    this->encodedRightMotor.setWheelRadius(wheelRadius);
}


void EncoderDiffDrive::setPID(float kp, float ki, float kd){
    this->encodedLeftMotor.setPID(kp, ki, kd);
    this->encodedRightMotor.setPID(kp, ki, kd);
}

void EncoderDiffDrive::print(){
    this->encodedLeftMotor.print();
    this->encodedRightMotor.print();
}

void EncoderDiffDrive::setup(){
    this->encodedLeftMotor.setup();
    this->encodedRightMotor.setup();
}

void EncoderDiffDrive::setTargetPose(Pose *targetPose){
    this->target_pose = *targetPose;
}

Pose* EncoderDiffDrive::getCurrentPose(){
    return &this->current_pose;
}

void EncoderDiffDrive::updatePose(float leftVelocity, float rightVelocity){
    unsigned long dt = millis() - this->lastTime;
    this->lastTime = millis();
    
    // calculate the angle velocity
    // TODO: check if this is degrees or radians
    this->current_pose.d_theta = (rightVelocity - leftVelocity) / this->wheelSeparation;

    // calculate the linear velocity
    float linearVelocity = (rightVelocity + leftVelocity) / 2;

    // calculate the x and y angle velocity
    this->current_pose.d_x = linearVelocity * cos(this->current_pose.d_theta);
    this->current_pose.d_y = linearVelocity * sin(this->current_pose.d_theta);

    // calculate the change in x, y, and theta
    current_pose.x += current_pose.d_x * dt;
    current_pose.y += current_pose.d_y * dt;
    current_pose.theta += current_pose.d_theta * dt;
}
    



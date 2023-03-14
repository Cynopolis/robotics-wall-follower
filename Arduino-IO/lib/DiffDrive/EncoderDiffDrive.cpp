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
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

void EncoderDiffDrive::print(){
    // print current pose
    Serial.print("Current Pose: ");
    Serial.print("x: ");
    Serial.print(this->current_pose.x);
    Serial.print(" y: ");
    Serial.print(this->current_pose.y);
    Serial.print(" theta: ");
    Serial.println(this->current_pose.theta);
    // print derivatives too
    Serial.print("d_x: ");
    Serial.print(this->current_pose.d_x);
    Serial.print(" d_y: ");
    Serial.print(this->current_pose.d_y);
    Serial.print(" d_theta: ");
    Serial.println(this->current_pose.d_theta);

    // print target pose
    Serial.print("Target Pose: ");
    Serial.print("x: ");
    Serial.print(this->target_pose.x);
    Serial.print(" y: ");
    Serial.print(this->target_pose.y);
    Serial.print(" theta: ");
    Serial.println(this->target_pose.theta);
}

void EncoderDiffDrive::setup(){
    this->encodedLeftMotor.setup();
    this->encodedRightMotor.setup();
}

Pose* EncoderDiffDrive::getCurrentPose(){
    return &this->current_pose;
}

Pose* EncoderDiffDrive::getTargetPose(){
    return &this->target_pose;
}

// TODO: Verify this function
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

    this->calculateMotorVelocities(dt);
}

// TODO: Verify this function
void EncoderDiffDrive::calculateMotorVelocities(unsigned long dt){
    // difference
    Pose error = this->target_pose - this->current_pose;

    // integral
    // this->sum_error = this->sum_error + (error * dt);

    // // derivative
    // Pose dedt = (error - this->last_error) / dt;
    // this->last_error = error;
    

    // // TODO: Check if the signs are correct on the derivative term
    // dedt.d_theta -= this->current_pose.d_theta;
    // dedt.d_x -= this->current_pose.d_x;
    // dedt.d_y -= this->current_pose.d_y;

    Pose pid = (error * this->kp);// + (this->sum_error * this->ki);// + (dedt * this->kd);

    float vel_mag = sqrt(pow(pid.x, 2) + pow(pid.y, 2));

    float leftVelocity = vel_mag - (pid.theta * this->wheelSeparation / 2);
    float rightVelocity = vel_mag + (pid.theta * this->wheelSeparation / 2);

    this->encodedLeftMotor.setTargetVelocity(leftVelocity);
    this->encodedRightMotor.setTargetVelocity(rightVelocity);
}
    



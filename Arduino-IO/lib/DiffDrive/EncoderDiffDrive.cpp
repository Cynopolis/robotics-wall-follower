#include "Pose.h"
#include "EncoderDiffDrive.h"


EncoderDiffDrive::EncoderDiffDrive(EncodedMotor leftMotor, EncodedMotor rightMotor, float wheelSeperation) :
    leftMotor(leftMotor), rightMotor(rightMotor), wheelSeperation(wheelSeperation) {
    targetPose = {0, 0, 0, 0, 0, 0};
    currentPose = {0, 0, 0, 0, 0, 0};
    lastUpdateTime = 0;
}

// Get the current pose
Pose* EncoderDiffDrive::getCurrentPose(){
    return &(this->currentPose);
}

// Get the target pose
Pose* EncoderDiffDrive::getTargetPose(){
    return &(this->targetPose);
}

// Set the PID constants
void EncoderDiffDrive::setPID(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

// Print out information about the encoder diff drive
void EncoderDiffDrive::print() {
    Serial.print("Current Pose: ");
    currentPose.print();
    Serial.print("Target Pose: ");
    targetPose.print();
}

// Setup the motors
void EncoderDiffDrive::setup() {
    leftMotor.setup();
    rightMotor.setup();
}

// Update the motor speeds based on the current and target poses
void EncoderDiffDrive::update(volatile long &leftIncriment, volatile long &rightIncriment) {
    // Calculate elapsed time since last update
    unsigned long now = millis();
    float dt = (now - lastUpdateTime);
    // if the time is less than 1ms, don't update. We use 0.5 to account for rounding errors
    if(dt < 0.5) return;
    dt *= 0.001;
    lastUpdateTime = now;

    // update the motors with the latest encoder incriment
    leftMotor.update(leftIncriment);
    rightMotor.update(rightIncriment);

    // Calculate current velocity using encoders
    float leftVelocity = leftMotor.getVelocity();
    float rightVelocity = rightMotor.getVelocity();
    float v = (leftVelocity + rightVelocity) / 2.0;
    float omega = (leftVelocity - rightVelocity) / wheelSeperation;

    // Calculate current position using current velocity and elapsed time
    float theta = currentPose.theta + omega * dt;
    float x = currentPose.x + v * cos(theta) * dt;
    float y = currentPose.y + v * sin(theta) * dt;
    Pose currentPosition(x, y, theta, 0, 0, 0);

    // Calculate error between current and target pose
    Pose error = targetPose - currentPosition;

    // calculate the desired velocity based on the error
    float v_x = kp * error.x + ki * currentPose.d_x + kd * (error.x - currentPose.d_x) / dt;
    float v_y = kp * error.y + ki * currentPose.d_y + kd * (error.y - currentPose.d_y) / dt;
    float v_theta = kp * error.theta + ki * currentPose.d_theta + kd * (error.theta - currentPose.d_theta) / dt;

    // Calculate motor speeds based on the desired velocities
    float leftSpeed = (v_x - v_theta * wheelSeperation / 2.0);
    float rightSpeed = (v_x + v_theta * wheelSeperation / 2.0);

    // Set motor speeds
    leftMotor.setTargetVelocity(leftSpeed);
    rightMotor.setTargetVelocity(rightSpeed);

    // Update current pose
    currentPose = currentPosition;
    currentPose.d_x = v_x;
    currentPose.d_y = v_y;
    currentPose.d_theta = v_theta;
}
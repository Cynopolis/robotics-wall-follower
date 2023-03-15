#include "Pose.h"
#include "EncoderDiffDrive.h"


EncoderDiffDrive::EncoderDiffDrive(EncodedMotor leftMotor, EncodedMotor rightMotor, double wheelSeperation) :
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
void EncoderDiffDrive::setPID(double kp, double ki, double kd) {
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
void EncoderDiffDrive::update(volatile int &leftIncriment, volatile int &rightIncriment) {
    // Calculate elapsed time since last update
    unsigned long now = millis();
    int time_diff = (now - lastUpdateTime);
    // if the time is less than 1ms, don't update. We use 0.5 to account for rounding errors
    if(time_diff < 1) return;
    double dt = double(time_diff) / 1000.0;
    lastUpdateTime = now;

    bool print = false;
    if(millis() - timer > 3000 && leftIncriment != 0 && rightIncriment != 0) {
        timer = millis();
        Serial.print("Current Pose: ");
        currentPose.print();
        Serial.print("Target Pose: ");
        targetPose.print();
        Serial.print("dt: ");
        Serial.println(dt, 5);
        print = true;
    }

    // update the motors with the latest encoder incriment
    leftMotor.update(leftIncriment);
    rightMotor.update(rightIncriment);

    // Calculate current velocity using encoders
    double leftVelocity = leftMotor.getVelocity();
    double rightVelocity = rightMotor.getVelocity();
    double v = (leftVelocity + rightVelocity) / 2;
    double d_theta = 2 * PI * (leftVelocity - rightVelocity) / wheelSeperation;
    if(print){
        Serial.print("Total Velocity: ");
        Serial.println(v);
    }
    // Calculate current position using current velocity and elapsed time
    double theta = currentPose.theta + d_theta * dt;

    // Make sure theta is between 0 and 2*PI
    while(theta > 2 * PI) theta -= 2 * PI;
    while(theta < 0) theta += 2 * PI;

    // update the current pose of the robot
    currentPose.d_x = v * cos(theta);
    currentPose.d_y = v * sin(theta);
    currentPose.d_theta = d_theta;
    currentPose.x = currentPose.x + currentPose.d_x * dt;
    currentPose.y = currentPose.y + currentPose.d_y * dt;
    currentPose.theta = theta;

    // Calculate error between current and target pose
    Pose error = targetPose - currentPose;
    sumError = sumError + error * dt;

    // calculate the desired velocity based on the error
    Pose v_pid = error * kp + sumError * ki + (error - pastError) * kd / dt;

    // update the past error
    pastError = error;

    // Calculate motor speeds based on the desired velocities
    double v_out = sqrt(v_pid.x * v_pid.x + v_pid.y * v_pid.y);
    int leftSpeed = int(v_out - v_pid.theta * wheelSeperation / 2.0);
    int rightSpeed = int(v_out + v_pid.theta * wheelSeperation / 2.0);

    // Set motor speeds
    leftMotor.setTargetVelocity(leftSpeed);
    rightMotor.setTargetVelocity(rightSpeed);
}
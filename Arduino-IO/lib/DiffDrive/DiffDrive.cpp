#include "DiffDrive.h"
#include <Arduino.h>

DiffDrive::DiffDrive(Motor* leftMotor, Motor* rightMotor, float wheelSeparation) :
    leftMotor(leftMotor), rightMotor(rightMotor), wheelSeparation(wheelSeparation) {}

void DiffDrive::setPID(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

float* DiffDrive::getCurrentPose() {
    currentPose[0] = current_x;
    currentPose[1] = current_y;
    currentPose[2] = current_theta;
    return currentPose;
}

void DiffDrive::setCurrentPose(float x, float y, float theta) {
    current_x = x;
    current_y = y;
    current_theta = theta;
}

float* DiffDrive::getTargetPose() {
    targetPose[0] = target_x;
    targetPose[1] = target_y;
    targetPose[2] = target_theta;
    return targetPose;
}

void DiffDrive::setTargetPose(float x, float y, float theta) {
    target_x = x;
    target_y = y;
    target_theta = theta;
}

void DiffDrive::begin() {
    lastTime = millis();
}

void DiffDrive::update() {
    unsigned long now = millis();
    long time_diff = now - lastTime;
    if(time_diff < 1) return;
    float dt = float(time_diff) / 1000.0;
    lastTime = now;

    // update current pose
    // leftMotor->update();
    // rightMotor->update();

    float leftDis = leftMotor->update();// leftMotor->getDistanceSinceLastUpdate();
    float rightDis = rightMotor->update(); //rightMotor->getDistanceSinceLastUpdate();
    float d_pos = (leftDis + rightDis) / 2.0;
    float d_theta = (rightDis - leftDis) / wheelSeparation;


    current_theta += d_theta;
    current_x += d_pos * cos(current_theta);
    current_y += d_pos * sin(current_theta);

    current_theta = fmod(current_theta + TAU, TAU);

    // calculate new motor velocities

    float error_x = target_x - current_x;
    float error_y = target_y - current_y;
    float error_theta = angleDiff(target_theta, current_theta);

    sum_error_x += error_x * dt;
    sum_error_y += error_y * dt;
    sum_error_theta += error_theta * dt;

    float vel_x = kp * error_x + ki * sum_error_x;
    float vel_y = kp * error_y + ki * sum_error_y;

    float rot_vel = kp * error_theta + ki * sum_error_theta;

    float vel_left = (vel_x - rot_vel * wheelSeparation / 2.0);
    float vel_right = (vel_x + rot_vel * wheelSeparation / 2.0);

    leftMotor->setVelocity(vel_left);
    rightMotor->setVelocity(vel_right);
}

float DiffDrive::angleDiff(float a, float b) {
    float d1 = fmod(b - a + TAU, TAU);
    float d2 = fmod(a - b + TAU, TAU);
    return d1 < d2 ? -d1 : d2;
}
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

    updatePose(dt);
}

void DiffDrive::updatePose(float dt){
    float leftDis = leftMotor->update();
    float rightDis = rightMotor->update();
    float d_pos = (leftDis + rightDis) / 2.0;
    float d_theta = (rightDis - leftDis) / wheelSeparation;
    

    current_theta += d_theta;
    current_x += d_pos * cos(current_theta);
    current_y += d_pos * sin(current_theta);

    current_theta = fmod(current_theta + TAU, TAU);

    if(d_theta != 0){
        Serial.print("x: ");
        Serial.print(current_x);
        Serial.print(" y: ");
        Serial.print(current_y);
        Serial.print(" theta: ");
        Serial.println(current_theta);
    }
}

float DiffDrive::angleDiff(float a, float b) {
    float d1 = fmod(b - a + TAU, TAU);
    float d2 = fmod(a - b + TAU, TAU);
    return d1 < d2 ? -d1 : d2;
}
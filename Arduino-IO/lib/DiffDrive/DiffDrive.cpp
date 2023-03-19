#include "DiffDrive.h"
#include <Arduino.h>

DiffDrive::DiffDrive(Motor* leftMotor, Motor* rightMotor, float wheelSeparation) :
    leftMotor(leftMotor), rightMotor(rightMotor), wheelSeparation(wheelSeparation) {}

void DiffDrive::setPID(float kp, float ki, float kd) {
    this->k_rho = kp;
    this->k_alpha = ki;
    this->k_beta = kd;
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
    leftMotor->begin();
    rightMotor->begin();
}

void DiffDrive::update() {
    unsigned long now = millis();
    long time_diff = now - lastTime;
    if(time_diff < 1) return;
    float dt = float(time_diff) / 1000.0;
    lastTime = now;

    /**
     * This section updates the current pose of the robot
    */
    float leftDis = leftMotor->update();
    float rightDis = rightMotor->update();

    float d_pos = (leftDis + rightDis) / 2.0;
    float d_theta = (rightDis - leftDis) / wheelSeparation;
    

    current_theta += d_theta;
    current_x += d_pos * cos(current_theta);
    current_y += d_pos * sin(current_theta);

    // wrap the angle to be between -pi and pi
    current_theta = wrap_angle(current_theta);

    /**
     * This section calculates the new velocities for the motors
    */
    // calculate osme of the variables needed for the controller
    float delta_x = target_x - current_x;
    float delta_y = target_y - current_y;
    float rho = sqrt(delta_x * delta_x + delta_y * delta_y);
    float alpha = atan2(delta_y, delta_x) - current_theta;
    float beta = -current_theta - alpha;

    // float v = d_pos / dt; // v stands for velocity
    // float w = d_theta / dt; // w stands for omega (angular velocity)

    float d_rho = wrap_angle(-k_rho * rho * cos(alpha));
    float d_alpha = wrap_angle(k_rho * sin(alpha) - k_alpha * alpha - k_beta * beta);
    float d_beta = wrap_angle(-k_rho * sin(alpha));

    // calculate the new velocities for the motors
    float v_r = k_rho * (rho);
    float w_r = k_alpha * (alpha) + k_beta * (beta);

    float phi_right = -127*(v_r + w_r)/wheelSeparation;
    float phi_left = -127*(v_r - w_r)/wheelSeparation;

    if(d_theta != 0){
        Serial.print("x: ");
        Serial.print(current_x);
        Serial.print(" y: ");
        Serial.print(current_y);
        Serial.print(" theta: ");
        Serial.println(current_theta);

        if(phi_right != 0 || phi_left != 0){
            Serial.print("phi_right: ");
            Serial.print(phi_right);
            Serial.print(" phi_left: ");
            Serial.println(phi_left);
        }
    }
    
    leftMotor->setVelocity(50*(4+current_theta));
    rightMotor->setVelocity(50*(4-current_theta));

    // leftMotor->setVelocity(int(phi_left));
    // rightMotor->setVelocity(int(phi_right));

}

float DiffDrive::angleDiff(float a, float b) {
    float d1 = fmod(b - a + TAU, TAU);
    float d2 = fmod(a - b + TAU, TAU);
    return d1 < d2 ? -d1 : d2;
}

float DiffDrive::wrap_angle(float angle) {
    // wrap the angle to be between -pi and pi
    return fmod(angle + PI, TAU) - PI;
}
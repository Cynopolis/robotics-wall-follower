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

// bound between -pi and pi
float DiffDrive::wrap_angle(float angle) {
    // First, wrap the angle between -2*pi and 2*pi radians
    angle = fmod(angle + PI, TAU) - PI;

    // Then, bound the angle between -pi and pi radians
    if (angle < -PI) {
        angle += TAU;
    } else if (angle > PI) {
        angle -= TAU;
    }

    return angle;
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
    current_theta = this->wrap_angle(current_theta);
    current_x += d_pos * cos(current_theta);
    current_y += d_pos * sin(current_theta);

    /**
     * This section calculates the new velocities for the motors
    */
    // calculate osme of the variables needed for the controller
    float delta_x = target_x - current_x;
    float delta_y = target_y - current_y;
    float rho = sqrt(delta_x * delta_x + delta_y * delta_y);
    float alpha = wrap_angle(atan2(delta_y, delta_x) - current_theta);
    float beta = -current_theta - alpha;

    if(isReversed && rho < 50){
        isReversed = false;
    }

    // if alpha is in the left half plane, make rho negative
    if(alpha > PI/2 || alpha < -PI/2){
        isReversed = true;
    }

    if(isReversed) rho = -rho;

    // float v = d_pos / dt; // v stands for velocity
    // float w = d_theta / dt; // w stands for omega (angular velocity)

    // float d_rho = -k_rho * rho * cos(alpha);
    // float d_alpha = k_rho * sin(alpha) - k_alpha * alpha - k_beta * beta;
    // float d_beta = -k_rho * sin(alpha);

    // calculate the new target velocity and angle for the motors to be drive at
    float v_r = k_rho * (rho);
    float w_r = k_alpha * (alpha) + k_beta * (beta);
    lastVel = v_r;

    float phi_right = 30*(v_r + w_r)/wheelSeparation;
    float phi_left = 30*(v_r - w_r)/wheelSeparation;
    // calculate the new velocities for the motors using the a matrix
    if(isReversed){
        phi_right = 30*(v_r - w_r)/wheelSeparation;
        phi_left = 30*(v_r + w_r)/wheelSeparation;
    }

    float max_vel = 160;
    if(abs(phi_left) > max_vel){
        float percent = abs(max_vel / phi_left);
        phi_left *= percent;
        phi_right *= percent;
    }
    if(abs(phi_right) > max_vel){
        float percent = abs(max_vel / phi_right);
        phi_left *= percent;
        phi_right *= percent;
    }

    // Print the current pose of the robot
    if(d_theta != 0){
        Serial.print("x: ");
        Serial.print(current_x,4);
        Serial.print(" y: ");
        Serial.print(current_y,4);
        Serial.print(" theta: ");
        Serial.println(current_theta,4);

        Serial.print("d_x: ");
        Serial.print(delta_x,4);
        Serial.print(" d_y: ");
        Serial.print(delta_y,4);
        Serial.print(" rho: ");
        Serial.print(rho,4);
        Serial.print(" alpha: ");
        Serial.print(alpha,4);
        Serial.print(" beta: ");
        Serial.println(beta,4);

        if(phi_left != 0 || phi_right != 0){
            Serial.print("left_speed: ");
            Serial.print(phi_left,0);
            Serial.print(" right_speed: ");
            Serial.println(phi_right,0);
        }
    }
    
    // leftMotor->setVelocity(50*(4+current_theta));
    // rightMotor->setVelocity(50*(4-current_theta));

    leftMotor->setVelocity(phi_left);
    rightMotor->setVelocity(phi_right);

}



// bound between -pi/2 and pi/2
// float wrap_angle(float angle) {
//     while (angle > PI/2) {
//         angle -= PI;
//     }
//     while (angle <= -PI/2) {
//         angle += PI;
//     }
//     return angle;
// }

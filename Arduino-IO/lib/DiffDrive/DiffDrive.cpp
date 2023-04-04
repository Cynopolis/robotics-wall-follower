#include "DiffDrive.h"
#include <Arduino.h>

DiffDrive::DiffDrive(Motor* leftMotor, Motor* rightMotor, float wheelSeparation) :
    leftMotor(leftMotor), rightMotor(rightMotor), wheelSeparation(wheelSeparation) {}

void DiffDrive::setPID(float kp, float ki, float kd) {
    this->k_rho = kp;
    this->k_alpha = ki;
    this->k_beta = kd;
}

xyzData DiffDrive::getCurrentPose() {
    return currentPose;
}

void DiffDrive::setCurrentPose(float x, float y, float theta) {
    this->currentPose = xyzData(x, y, theta);
}

xyzData DiffDrive::getTargetPose() {
    return targetPose;
}

void DiffDrive::setTargetPose(float x, float y, float theta) {
    this->targetPose = xyzData(x, y, theta);
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

void DiffDrive::update(IMU* imu) {
    /**
     * This section calculates the time since the last update
    */
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
    
    wheelAngle += d_theta;
    wheelAngle = wrap_angle(wheelAngle);

    
    if(imu != nullptr) {
        imu->update();
        if(abs(d_theta) < 0.001 || abs(d_pos) < 0.001){
            imu->freezeGyro(true);
        }
        else{
            imu->freezeGyro(false);
        }
        currentPose.z = imu->getOrientation().z;

        if(abs(imu->getOrientationChange().z - d_theta) < 0.0065){
            // no slip detected
            currentPose.x += d_pos * cos(currentPose.z);
            currentPose.y += d_pos * sin(currentPose.z);
        }
        else{
            Serial.print("Slip detected: ");
            Serial.print(imu->getOrientationChange().z - d_theta, 5);
            Serial.print(" ");
            Serial.println(imu->getOrientation().z);
        }
    } else {
        currentPose.z = wheelAngle;
        currentPose.x += d_pos * cos(currentPose.z);
        currentPose.y += d_pos * sin(currentPose.z);
    }
    

    

    /**
     * This section calculates the new velocities for the motors
    */
    // calculate osme of the variables needed for the controller
    xyzData delta = targetPose - currentPose;
    float rho = sqrt(delta.x * delta.x + delta.y * delta.y);
    float alpha = wrap_angle(atan2(delta.y, delta.x) - currentPose.z);
    float beta = -currentPose.z - alpha;

    // if alpha is in the left half plane, make rho negative
    if(alpha > PI/2 || alpha < -PI/2){
        isReversed = false;
    }

    if(isReversed) rho = -rho;

    // float v = d_pos / dt; // v stands for velocity
    // float w = d_theta / dt; // w stands for omega (angular velocity)

    // float d_rho = -k_rho * rho * cos(alpha);
    // float d_alpha = k_rho * sin(alpha) - k_alpha * alpha - k_beta * beta;
    // float d_beta = -k_rho * sin(alpha);

    // calculate the new target velocity and angle for the motors to be drive at

    if(abs(rho) < 10){
        rho = 0;
        alpha = 0;
    }

    float v_r = k_rho * (rho);
    float w_r = k_alpha * (alpha) + k_beta * (beta);

    float phi_right = (v_r + w_r)/30;
    float phi_left = (v_r - w_r)/30;

    float left_motor_speed = (0.25*120*phi_left)*255/7.4;
    float right_motor_speed = (0.25*120*phi_right)*255/7.4;

    float max_vel = 180;
    if(abs(left_motor_speed) > max_vel){
        float percent = abs(max_vel / left_motor_speed);
        left_motor_speed *= percent;
        right_motor_speed *= percent;
    }
    if(abs(right_motor_speed) > max_vel){
        float percent = abs(max_vel / right_motor_speed);
        left_motor_speed *= percent;
        right_motor_speed *= percent;
    }

    // Print the current pose of the robot
    bool debug = false;
    if(d_theta != 0 && debug){
        Serial.print("x: ");
        Serial.print(currentPose.x,4);
        Serial.print(" y: ");
        Serial.print(currentPose.y,4);
        Serial.print(" theta: ");
        Serial.println(currentPose.z,4);

        Serial.print("d_x: ");
        Serial.print(delta.x,4);
        Serial.print(" d_y: ");
        Serial.print(delta.y,4);
        Serial.print(" rho: ");
        Serial.print(rho,4);
        Serial.print(" alpha: ");
        Serial.print(alpha,4);
        Serial.print(" beta: ");
        Serial.println(beta,4);
        Serial.print("v_r: ");
        Serial.print(v_r,4);
        Serial.print(" w_r: ");
        Serial.println(w_r,4);

        if(left_motor_speed != 0 || right_motor_speed != 0){
            Serial.print("left_speed: ");
            Serial.print(left_motor_speed,0);
            Serial.print(" right_speed: ");
            Serial.println(right_motor_speed,0);
        }

        if(imu != nullptr) {
            Serial.print("IMU Angle:");
            Serial.print(imu->getOrientation().z, 4);
            Serial.print("Wheel Angle:");
            Serial.println(wheelAngle, 4);
            // Serial.print(" IMU Accel:");
            // Serial.println(imu->getAccel().magnitude(),4);
        }
    }
    
    leftMotor->setVelocity(left_motor_speed);
    rightMotor->setVelocity(right_motor_speed);

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

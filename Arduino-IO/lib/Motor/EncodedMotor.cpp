#include "EncodedMotor.h"
#include "Arduino.h"

EncodedMotor::EncodedMotor(int forwardPin, int backwardPin, int pwmPin, int encoderPinA, int encoderPinB)
: Motor(forwardPin, backwardPin, pwmPin), encoderPinA(encoderPinA), encoderPinB(encoderPinB){}

void EncodedMotor::setup() {
    Motor::setup();
    pinMode(encoderPinA, INPUT);
    pinMode(encoderPinB, INPUT);

}

void EncodedMotor::setWheelRadius(float wheelRadius) {
    this->wheelRadius = wheelRadius;
    // wheel diameter is 66 mm
    // The grear ratio is 120 motor turns : 1 wheel turn
    // the encoder has 16 steps per motor revolution
    // or 8*120 = 960 steps per wheel revolution
    // 66*PI = 207.345 mm per wheel revolution
    // 207.345/960 = 0.216 mm per step
    this->stepsToMM = (wheelRadius * PI) / stepsPerRevolution;
}

int EncodedMotor::getVelocity() {
    float diffDist = float(encoderSteps - lastEncoderSteps) * stepsToMM;
    float diffTime = float(millis() - lastTime);
    return int(diffDist / diffTime);
}

float EncodedMotor::getAngularVelocity() {
    float diffAngle = 2*PI*float(encoderSteps - lastEncoderSteps) / stepsPerRevolution;
    float diffTime = float(millis() - lastTime);
    return diffAngle / diffTime;
}

float EncodedMotor::getDistance() {
    return encoderSteps * stepsToMM;
}

void EncodedMotor::setPID(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

void EncodedMotor::update(long incriment) {
    // update the encoder count
    this->encoderSteps += incriment;
    float past_error = float(targetEncoderSteps - lastEncoderSteps);
    this->lastEncoderSteps = encoderSteps;

    // update the timer
    float dt = float(millis() - lastTime)*0.001;
    lastTime = millis();

    // update other things
    float error = float(targetEncoderSteps - encoderSteps);
    this->sumError += long(error);

    float velocity = kp * error + ki * sumError * dt + kd * (error - past_error) / dt;
    
    // if the current velocity is out of bounds, stop integral windup
    if(abs(velocity) > maxVelocity) {
        this->sumError -= long(error);
    }

    setVelocity(int(velocity));
}

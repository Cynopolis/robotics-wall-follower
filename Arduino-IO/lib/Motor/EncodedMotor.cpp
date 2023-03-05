#include "EncodedMotor.h"

EncodedMotor::EncodedMotor(int forwardPin, int backwardPin, int pwmPin, int encoderPinA, int encoderPinB)
: forwardPin(forwardPin), backwardPin(backwardPin), pwmPin(pwmPin), encoderPinA(encoderPinA), encoderPinB(encoderPinB){}

void EncodedMotor::setup() {
    // set the pins as outputs:
    pinMode(forwardPin, OUTPUT);
    pinMode(backwardPin, OUTPUT);
    pinMode(pwmPin, OUTPUT);
    this->lastTime = micros();
    pinMode(encoderPinA, INPUT_PULLUP);
    pinMode(encoderPinB, INPUT_PULLUP);

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

void EncodedMotor::setPID(int kp, int ki, int kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

void EncodedMotor::update(volatile int &incriment) {

    // update the current step count with the incriment
    this->lastEncoderSteps = this->encoderSteps;
    this->encoderSteps += incriment;
    // reset incriment to 0
    incriment = 0;
    
    // Bypass PID
    this->setVelocity(this->target_velocity);
    
}

// TODO: Change this to do something else
void EncodedMotor::accelerate(float dt){
    // incriment the velocity based on the acceleration and dt
    dt *= 1000000;
    float a = acceleration*dt;
    int current_velocity = this->getVelocity();
    float diff = target_velocity - current_velocity;
    if(abs(diff) < a){
        current_velocity = target_velocity;
        setVelocity(current_velocity);
        return;
    }
    else if(diff > 0){
        current_velocity += a;
    }
    else if(diff < 0){
        current_velocity -= a;
    }
    
    setVelocity(current_velocity);

}

void EncodedMotor::setTargetVelocity(float targetDistance) {
    this->targetEncoderSteps = long(targetDistance / stepsToMM);
}

void EncodedMotor::print() {
    Serial.print("Encoder Steps: ");
    Serial.print(encoderSteps);
    Serial.print(" Target Encoder Steps: ");
    Serial.print(targetEncoderSteps);
    Serial.print(" Velocity: ");
    Serial.print(getVelocity());
    Serial.print(" Angular Velocity: ");
    Serial.print(getAngularVelocity());
    Serial.print(" Distance: ");
    Serial.print(getDistance());
    Serial.print(" Wheel Angle: ");
    Serial.print(wheelAngle);
    Serial.print(" Wheel Radius: ");
    Serial.print(wheelRadius);
    Serial.print(" Kp: ");
    Serial.print(kp);
    Serial.print(" Ki: ");
    Serial.print(ki);
    Serial.print(" Kd: ");
    Serial.println(kd);
}

void EncodedMotor::setVelocity(int velocity){
    // make sure the requested velocity is within the set bounds
    if (velocity > maxVelocity) {
        velocity = maxVelocity;
    }
    else if (velocity < -maxVelocity) {
        velocity = -maxVelocity;
    }
    // set the new motor direction based on the sign of the velocity
    if (velocity > 0) {
        digitalWrite(forwardPin, HIGH);
        digitalWrite(backwardPin, LOW);
    }
    else if (velocity < 0) {
        digitalWrite(forwardPin, LOW);
        digitalWrite(backwardPin, HIGH);
    }
    else {
        digitalWrite(forwardPin, LOW);
        digitalWrite(backwardPin, LOW);
    }
    // map the velocity to the analog range of the PWM pin
    velocity = map(abs(velocity), 0, maxVelocity, 0, 255);
    // write the new velocity to the PWM pin
    analogWrite(pwmPin, velocity);
}


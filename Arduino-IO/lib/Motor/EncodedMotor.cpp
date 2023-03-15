#include "EncodedMotor.h"

EncodedMotor::EncodedMotor(uint8_t forwardPin, uint8_t backwardPin, uint8_t pwmPin, uint8_t encoderPinA, uint8_t encoderPinB)
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

float EncodedMotor::getVelocity() {
    return this->currentVelocity;
}

float EncodedMotor::getDistance() {
    return encoderSteps * stepsToMM;
}

void EncodedMotor::update(volatile long &incriment) {

    // update the current step count with the incriment
    this->lastEncoderSteps = this->encoderSteps;
    if(millis() - timer > 3000){
        Serial.print("Encoder Steps: ");
        Serial.println(this->encoderSteps);
    }

    this->encoderSteps += incriment;
    // reset incriment to 0
    incriment = 0;

    unsigned long now = millis();
    float dt = float(now - this->lastTime) / 1000;
    float distance = (this->encoderSteps - this->lastEncoderSteps) * stepsToMM;
    this->currentVelocity = distance / dt;

    lastTime = now;

    if(millis() - timer > 3000){
        Serial.print("Current Velocity: ");
        Serial.println(this->currentVelocity);
        timer = millis();
    }
    
    // Bypass PID
    this->setVelocity(this->targetVelocity);
    
}

void EncodedMotor::setTargetVelocity(int targetVelocity) {
    this->targetVelocity = targetVelocity;
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


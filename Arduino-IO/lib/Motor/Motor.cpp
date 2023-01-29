#include "Motor.h"
#include <Arduino.h>

Motor::Motor(int forwardPin, int backwardPin, int pwmPin) {
    this->forwardPin = forwardPin;
    this->backwardPin = backwardPin;
    this->pwmPin = pwmPin;

    // set the pins as outputs:
    pinMode(forwardPin, OUTPUT);
    pinMode(backwardPin, OUTPUT);
    pinMode(pwmPin, OUTPUT);
    this->lastTime = micros();
}

void Motor::setVelocity(float velocity){
    // make sure the requested velocity is within the set bounds
    if (abs(velocity - maxVelocity) < 0.1 ) {
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
    // set the current velocity to the requested velocity
    this->current_velocity = velocity;
    // map the velocity to the analog range of the PWM pin
    velocity = map(abs(velocity), 0, maxVelocity, 0, 255);
    // write the new velocity to the PWM pin
    analogWrite(pwmPin, velocity);
}

void Motor::setTargetVelocity(int targetVelocity) {
    this->target_velocity = targetVelocity;
}

int Motor::getTargetVelocity() {
    return target_velocity;
}

int Motor::getVelocity() {
    return int(current_velocity);
}

void Motor::setMaxVelocity(int maxVelocity) {
    this->maxVelocity = maxVelocity;
}

int Motor::getMaxVelocity() {
    return maxVelocity;
}

void Motor::setAcceleration(int acceleration) {
    this->acceleration = acceleration;
}

int Motor::getAcceleration() {
    return acceleration;
}

void Motor::update(){
    // if the current velocity is equal to the target velocity, do nothing
    if (abs(current_velocity - target_velocity) < 0.1) {
        current_velocity = target_velocity;
        this->lastTime = micros();
        return;
    }
    // calculate the time since the last update
    float dt = float(micros() - lastTime)/1000000;
    Serial.print("dt: ");
    Serial.println(dt, 5);
    this->lastTime = micros();

    float newVel = current_velocity;
    // calculate the new velocity based on the acceleration
    if (current_velocity < target_velocity) {
        newVel = newVel + float(acceleration) * dt;
    }
    else if (current_velocity > target_velocity) {
        newVel = newVel - float(acceleration) * dt;
    }

    Serial.print("New Velocity: ");
    Serial.println(newVel, 5);
    
    setVelocity(newVel);
}

void Motor::print(){
    Serial.print("Target Velocity: ");
    Serial.println(target_velocity);
    Serial.print("Current Velocity: ");
    Serial.println(int(current_velocity));
    Serial.print("Max Velocity: ");
    Serial.println(maxVelocity);
    Serial.print("Acceleration: ");
    Serial.println(acceleration);

    Serial.print("Forward Pin: ");
    Serial.println(forwardPin);
    Serial.print("Backward Pin: ");
    Serial.println(backwardPin);
    Serial.print("PWM Pin: ");
    Serial.println(pwmPin);
}

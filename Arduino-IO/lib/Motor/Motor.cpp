#include "Motor.h"
#include <Arduino.h>

Motor::Motor(int forwardPin, int backwardPin, int pwmPin) :
    forwardPin(forwardPin), backwardPin(backwardPin), pwmPin(pwmPin) {}

void Motor::setup(){
    // set the pins as outputs:
    pinMode(forwardPin, OUTPUT);
    pinMode(backwardPin, OUTPUT);
    pinMode(pwmPin, OUTPUT);
    this->lastTime = millis();
}

void Motor::setVelocity(int velocity){
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
    return current_velocity;
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
    if (current_velocity == target_velocity) {
        this->lastTime = millis();
        return;
    }
    // calculate the time since the last update
    int dt = int(millis() - lastTime);
    this->lastTime = millis();

    int newVel = current_velocity;
    // calculate the new velocity based on the acceleration
    if (current_velocity < target_velocity) {
        int newVel = newVel + acceleration * dt;
    }
    else if (current_velocity > target_velocity) {
        int newVel = newVel - acceleration * dt;
    }

    // If the new velocity is greater than the target velocity, set the new velocity to the target velocity
    if (newVel > target_velocity) {
        newVel = target_velocity;
    }
    else if (newVel < -target_velocity) {
        newVel = -target_velocity;
    }
    
    setVelocity(target_velocity);
}

void Motor::print(){
    Serial.print("Target Velocity: ");
    Serial.println(target_velocity);
    Serial.print("Current Velocity: ");
    Serial.println(current_velocity);
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

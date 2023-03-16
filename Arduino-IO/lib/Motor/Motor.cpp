#include "Motor.h"

Motor::Motor(uint8_t forwardPin, uint8_t backwardPin, uint8_t pwmPin, volatile int* incriment) : 
forwardPin(forwardPin), backwardPin(backwardPin), pwmPin(pwmPin), incriment(incriment){}

void Motor::setWheelRadius(float wheelRadius){
    this->wheelRadius = wheelRadius;
    this->stepsToMM = 2*PI*wheelRadius*stepsPerRevolution;
}

float Motor::getWheelRadius(){
    return this->wheelRadius;
}

float Motor::getVelocity(){
    return this->currentVelocity;
}

void Motor::update(){
    // calculate dt
    unsigned long now = millis();
    int time_diff = int(now - lastTime);
    if(time_diff < 1 || *incriment == 0) return;
    lastTime = now;
    float dt = float(time_diff)*0.001;

    // calculate velocity
    encoderSteps += *incriment;
    currentVelocity = (float(*incriment) * stepsToMM) / dt;
    // reset incriment to 0
    *incriment = 0;
}

void Motor::begin(){
    pinMode(forwardPin, OUTPUT);
    pinMode(backwardPin, OUTPUT);
    pinMode(pwmPin, OUTPUT);
}

void Motor::setVelocity(int velocity){
    // make sure the velocity is in the range of -255 to 255
    if(velocity > 255){
        velocity = 255;
    } else if(velocity < -255){
        velocity = -255;
    }
    this->targetVelocity = velocity;

    if(velocity > 0){
        digitalWrite(forwardPin, HIGH);
        digitalWrite(backwardPin, LOW);
    } else if(velocity < 0){
        digitalWrite(forwardPin, LOW);
        digitalWrite(backwardPin, HIGH);
    } else {
        digitalWrite(forwardPin, LOW);
        digitalWrite(backwardPin, LOW);
    }
    analogWrite(pwmPin, abs(velocity));
}


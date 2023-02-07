#include "Sonar.h"
#include <Arduino.h>

Sonar::Sonar(int trigPin, int echoPin)
: trigPin(trigPin), echoPin(echoPin)
{
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    resetDistances();
}

void Sonar::attachServo(Servo servo) {
    this->servo = servo;
    this->servoAttached = true;
}

void Sonar::resetDistances() {
    for(int i = 0; i < 180; i++){
        this->distances[i] = -1;
    }
}

float Sonar::getDistance(int angle) {
    // calculate the servo angle
    if(angle > 180 || angle < 0) return -1; // invalid angle
    if(!servoAttached) return -1; // servo not attached
    // move the servo and update its angle
    servo.write(angle);
    delay(4*abs(angle - lastAngle));
    this->currentAngle = angle;

    // get the distance
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    float duration = float(pulseIn(echoPin, HIGH));
    float distance = duration * 0.0005627; // convert to feet
    return distance;
}

void Sonar::enableScanMode(bool enable) {
    this->scanMode = enable;
}

void Sonar::setAngleIncrement(int angleIncrement) {
    if(angleIncrement < 0 || angleIncrement > 180) return; // invalid angle increment (must be between 0 and 180
    this->angleIncrement = angleIncrement;
    this->currentAngle = 0;
    resetDistances();
}

int Sonar::getAngleIncrement() {
    return angleIncrement;
}

void Sonar::print() {
    for(int i = 0; i < 180; i++){
        Serial.print(",");
        Serial.print(distances[i]);
    }
}

void Sonar::update(){
    if(!scanMode || !servoAttached) return;
    distances[currentAngle] = getDistance(currentAngle);
    this->lastAngle = currentAngle;
    if(scanLeft){
        this->currentAngle -= angleIncrement;
    }
    else{
        this->currentAngle += angleIncrement;
    }
    if(currentAngle >= 180){
        this->currentAngle = 180 - angleIncrement;
        this->scanLeft = true;
    }
    else if(currentAngle <= 0){
        this->currentAngle = angleIncrement;
        this->scanLeft = false;
    }
}

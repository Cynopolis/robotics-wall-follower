#pragma once
#include <Servo.h>

class Sonar{
    public:
        /**
         * @brief Construct a new Sonar object
         * @param trigPin The pin that the trigger pin of the sonar is connected to
         * @param echoPin The pin that the echo pin of the sonar is connected to
         */
        Sonar(int trigPin, int echoPin);
        
        void attachServo(Servo servo);

        /**
         * @brief Get the distance in feet at the given angle. 0 degrees is left, 90 is forward, 180 is right.
         * @param angle The angle to get the distance at
         * @return The distance in feet
         */
        float getDistance(int angle);

        /**
         * @brief Enable or disable scan mode. When scan mode is enabled, the sonar will scan the area in front of it and store the distances in an array.
         * @param enable True to enable scan mode, false to disable
         */
        void enableScanMode(bool enable);

        /**
         * @brief Set the angle increment for scan mode. The default is 5 degrees.
         * @param angleIncrement The angle increment in degrees
         * @post The distances array will be reset
         */
        void setAngleIncrement(int angleIncrement);

        /**
         * @brief Get the angle increment for scan mode
         * @return The angle increment in degrees
         */
        int getAngleIncrement();

        /**
         * @brief print out all of the initialized distances in the distance array.
         */
        void print();

        /**
         * @brief Update the distances array. This should be called in the loop function if scan mode is enabled.
         */
        void update();

    private:
        /**
         * @brief Reset the distances array to -1
         */
        void resetDistances();
        
        int trigPin;
        int echoPin;
        int servoPin;
        int angleIncrement = 5;
        float distances[180];
        int currentAngle = 0;
        int lastAngle = 0;
        bool scanMode = false;
        Servo servo;
        bool servoAttached = false;
        bool scanLeft = true;
};
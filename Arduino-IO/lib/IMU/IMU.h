#pragma once
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include "Arduino.h"

#define MAG_ADDRESS 0x1C  //Would be 0x1E if SDO_M is HIGH		
#define ACC_ADDRESS 0x6A
#define GYR_ADDRESS 0x6A  //Would be 0x6B if SDO_AG is HIGH
#define GPS_ADDRESS 0x77 // I'm not sure if this is the GPS, but it showed up on the I2C scanner

struct xyzData{
    float x;
    float y;
    float z;

    xyzData(float x, float y, float z){
        this->x = x;
        this->y = y;
        this->z = z;
    }

    xyzData(){
        this->x = 0;
        this->y = 0;
        this->z = 0;
    }

    xyzData operator+(xyzData other){
        return xyzData(this->x + other.x, this->y + other.y, this->z + other.z);
    }

    xyzData operator-(xyzData other){
        return xyzData(this->x - other.x, this->y - other.y, this->z - other.z);
    }

    xyzData operator*(float scalar){
        return xyzData(this->x * scalar, this->y * scalar, this->z * scalar);
    }

    xyzData operator/(float scalar){
        return xyzData(this->x / scalar, this->y / scalar, this->z / scalar);
    }

    void operator=(xyzData other){
        this->x = other.x;
        this->y = other.y;
        this->z = other.z;
    }

    void print(){
        Serial.print(x);
        Serial.print(",");
        Serial.print(y);
        Serial.print(",");
        Serial.print(z);
    }
};

class IMU{
    public:

        /**
         * @brief Construct a new IMU object
        */
        IMU() = default;

        /**
         * @brief Initialize the IMU
         * @param None
        */
        void begin();

        /**
         * @brief Update the IMU data
         * @param None
        */
        void update();

        /**
         * @brief Get the acceleration data
         * @param None
         * @return xyzData
        */
        double * getAngles();

        /**
         * @brief Get the acceleration data
         * @param None
         * @return xyzData
        */
        void calibrate();
        
        /**
         * @brief Print the IMU data
         * @param None
         * @return None
        */
        void print();

    private:
        xyzData accel;
        xyzData gyro;
        xyzData mag;

        xyzData accelOffset;
        xyzData gyroOffset;
        xyzData magOffset;

        xyzData orientation;

        LSM9DS1 imu;

        bool isInitialized = false;
        bool isCalibrated = false;

        unsigned long lastUpdate = 0;

        static constexpr float deg_to_rad = PI / 180;
        static constexpr float TAU = 2 * PI;

        /**
         * @brief Wrap the angle between -pi and pi
         * @param angle
         * @return float contrained between -pi and pi
        */
        float wrap_angle(float angle);
};
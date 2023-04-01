#pragma once
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include "Arduino.h"
#include "xyzData.h"

#define MAG_ADDRESS 0x1C  //Would be 0x1E if SDO_M is HIGH		
#define ACC_ADDRESS 0x6A
#define GYR_ADDRESS 0x6A  //Would be 0x6B if SDO_AG is HIGH
#define GPS_ADDRESS 0x77 // I'm not sure if this is the GPS, but it showed up on the I2C scanner



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
        const xyzData getAngles();

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

        /**
         * @brief Freeze/unfreeze gyro updates
         * @param freeze True to freeze, false to unfreeze
        */
        void freezeGyro(bool freeze){this->isGyroFrozen = freeze;}

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
        bool isGyroFrozen = false;

        unsigned long lastUpdate = 0;
        unsigned long lastGyroUpdate = 0;

        static constexpr float deg_to_rad = PI / 180;
        static constexpr float TAU = 2 * PI;

        /**
         * @brief Wrap the angle between -pi and pi
         * @param angle
         * @return float contrained between -pi and pi
        */
        float wrap_angle(float angle);

        void update_angle();
};
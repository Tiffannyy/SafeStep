#pragma once
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <MAX30105.h>
#include <heartRate.h>
#include <TFT_eSPI.h>

// Sensor initalization
extern Adafruit_MPU6050 mpu;       // gyroscope and accelerometer
extern Adafruit_BME280 bme;        // temperature, humidity, and pressure
extern MAX30105 max30102;          // heart rate and SpO2
extern Adafruit_Sensor *mpu_accel;
extern TFT_eSPI tft;

// MPU variables
extern int steps;                       // step count
// ** acceleration thresholds for step detection **
extern float threshold;                 // general threshold for step detection
extern float upperThreshold;            // threshold for step start
extern float lowerThreshold;            // threshold for reset

extern bool repeatFlag;                 // bool to prevent multiple steps recorded in one step
extern unsigned long lastStep;          // time of last step
extern const unsigned long stepDelay;   // delay between recorded steps

// BME280 variables
extern const int delayTime;             // delay between readings
extern unsigned long startTime;

// Function prototypes
void stepTracker(unsigned long now);
void bmeRead(unsigned long now);
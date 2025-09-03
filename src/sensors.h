#pragma once
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <MAX30105.h>
#include <heartRate.h>
#include <TFT_eSPI.h>

// User data type
struct SensorData {
    float tempC;
    float tempF;
    float humidity;
    int steps;
    int heartRate;  // optional for now, can leave unused

    bool fallDetected = false;  // used to send alerts
    bool lowHRDetected = false;
};

// Sensor initalization
extern Adafruit_MPU6050 mpu;       // gyroscope and accelerometer
extern Adafruit_BME280 bme;        // temperature, humidity, and pressure
extern MAX30105 max30102;          // heart rate and SpO2
extern Adafruit_Sensor *mpu_accel, *mpu_gyro;
extern TFT_eSPI tft;

// MPU variables
// ** acceleration thresholds for step detection **
extern float threshold;                 // general threshold for step detection
extern float upperThreshold;            // threshold for step start
extern float lowerThreshold;            // threshold for reset

extern bool repeatFlag;                 // bool to prevent multiple steps recorded in one step
extern unsigned long lastStep;          // time of last step
extern const unsigned long stepDelay;   // delay between recorded steps

// ** fall detection variables **
// Fall detection parameters
extern const float FREE_FALL_THRESHOLD;         // m/2^2
extern const float IMPACT_THRESHOLD;            // m/s^2   normal force + sudden change in acceleration
extern const unsigned long FALL_TIME_WINDOW;    // ms time window after free fall
extern unsigned long freeFallTime;
extern bool freeFallDetected;

// MAX30102 variables
extern const byte RATE_SIZE;
extern byte rates[];      // array of hb
extern byte rateSpot;

// BME280 variables
extern const int delayTime;             // delay between readings
extern unsigned long startTimeBME;
extern unsigned long startTimeHB;

// Function prototypes
void stepTracker(unsigned long now, SensorData &data);
void bmeRead(unsigned long now, SensorData &data); 
void fallDetector(unsigned long now, SensorData &data);
void heartbeat(unsigned long now, SensorData &data);
float calculatePitch(const sensors_event_t& accel);
float calculateRoll(const sensors_event_t& accel);
bool isOrientationFall(const sensors_event_t& accel);
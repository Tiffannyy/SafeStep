#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <MAX30105.h>
#include <heartRate.h>
#include <Wire.h>
#include <SPI.h>
#include <TFT_eSPI.h>

// definitions
#define SEALEVELPRESSURE_HPA (1013.25)  // bme

// Sensor initialization
Adafruit_MPU6050 mpu;       // gyroscope and accelerometer
Adafruit_BME280 bme;        // temperature, humidity, and pressure
MAX30105 max30102;          // heart rate and SpO2
Adafruit_Sensor *mpu_accel, *mpu_gyro;
TFT_eSPI tft = TFT_eSPI();  // TFT display

// MPU variables
int steps = 0;                        // step count
// ** acceleration thresholds for step detection **
float upperThreshold = 3.0;           // threshold for step start
float lowerThreshold = 1.0;           // threshold for reset

bool repeatFlag = false;              // bool to prevent multiple steps recorded in one step
unsigned long lastStep = 0;           // time of last step
const unsigned long stepDelay = 400;  // delay between recorded steps

// ** fall detection variables **
// Fall detection parameters
const float FREE_FALL_THRESHOLD = 4.9;  // m/2^2
const float IMPACT_THRESHOLD = 18.6;      // m/s^2   normal force + sudden change in acceleration
const unsigned long FALL_TIME_WINDOW = 1000;  // ms time window after free fall
unsigned long freeFallTime = 0;
bool freeFallDetected = false;

// BME280 variables
const int delayTime = 10000; // delay between readings
unsigned long startTime = millis();


// Function prototypes
void stepTracker(unsigned long now);
void fallDetector(unsigned long now);
void bmeRead(unsigned long now);

// Hekper function prototypes
float calculatePitch(const sensors_event_t& accel);
float calculateRoll(const sensors_event_t& accel);
bool isOrientationFall(const sensors_event_t& accel);

// Begin program
void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // wait for serial port to connect. Needed for native USB
  }
  Wire.begin(); // initialize I2C

  // display setup
  tft.init();
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(TL_DATUM);

  // Wait for devices
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");

    while (1) {
      delay(10); // halt if MPU6050 not found
    }
  }

  if (!bme.begin()){
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1) {
      delay(10); // halt if BME280 not found
    }
  }
  // initialize sesnors
  mpu_accel = mpu.getAccelerometerSensor();
  mpu_gyro = mpu.getGyroSensor();

  tft.println("Booting...");
}


void loop() {
  // current time
  unsigned long now = millis();

  // MPU6050 step tracker
  stepTracker(now);
  // MPU6050 fall detection
  fallDetector(now);
  // bme temp and humidity read
  bmeRead(now);

}


// MPU6050 step tracker
void stepTracker(unsigned long now) {
  sensors_event_t accel;        // sensor event
  mpu_accel->getEvent(&accel);

  // acceleration calculation
  float accelMag = accel.acceleration.x * accel.acceleration.x +
                   accel.acceleration.y * accel.acceleration.y +
                   accel.acceleration.z * accel.acceleration.z;
  accelMag = sqrt(accelMag); // magnitude of acceleration vector
  float dynamicAccel = fabs(accelMag - 9.8); // subtract gravity's normal force from magnitude;

  // check accel threshold, if accel exceeds threshold, record step
  if (!repeatFlag && dynamicAccel > upperThreshold && now - lastStep > stepDelay) {
    steps++;                                        // record step, set flag
    lastStep = now;
    repeatFlag = true;
    String msg = "Steps: " + String(steps);           // notify of update
    Serial.println(msg);
  }
  else if (repeatFlag && dynamicAccel < lowerThreshold) { // reset repeat flag
    repeatFlag = false;
  }
}


// MPU6050 fall detection
void fallDetector(unsigned long now) {
  // sensor events
  sensors_event_t accel;
  sensors_event_t gyro;
  mpu_accel->getEvent(&accel);
  mpu_gyro->getEvent(&gyro);

  float accelMag = sqrt(accel.acceleration.x * accel.acceleration.x +
                      accel.acceleration.y * accel.acceleration.y +
                      accel.acceleration.z * accel.acceleration.z);

  if (!freeFallDetected){
    // detect free fall
    if (accelMag < FREE_FALL_THRESHOLD) {
      freeFallDetected = true;
      freeFallTime = now;
      Serial.println("Free fall detected!");
    }
  }
  else{
    // see if an impact occurs
    unsigned long elapsed = now - freeFallTime;
    if (accelMag > IMPACT_THRESHOLD){
      if (isOrientationFall(accel)) {
        Serial.println("** IMPACT DETECTED! **");
        // TODO: add impact handling code (e.g., alert, log, etc.)
      }
      // reset free fall detection
      freeFallDetected = false;
    }
    else if (elapsed > FALL_TIME_WINDOW) {
      // reset free fall detection if time window exceeded
      freeFallDetected = false;
    }
  }
}


// temp and humidity read
void bmeRead(unsigned long now){
  if (now - startTime > delayTime){
    startTime = now;

    float hum   = bme.readHumidity();
    float tempC = bme.readTemperature();
    float tempF = tempC * 9/5 + 32;

    // display temp
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(0, 0);
    tft.printf("Temp:\n%.2f C\n", tempC);
    tft.printf("%.2f F\n", tempF);

    // display humidity
    tft.printf("\nHumidity:\n%.2f %%\n", hum);
  }
}


// Helper functions for fall detection
float calculatePitch(const sensors_event_t& accel) {
  // uses arctangent to calculate pitch angle
  // pitch is nose to tail
  return atan2(-accel.acceleration.x, sqrt(accel.acceleration.y * accel.acceleration.y + accel.acceleration.z * accel.acceleration.z)) * 180.0 / PI;
}

float calculateRoll(const sensors_event_t& accel) {
  // uses arctangent to calculate roll angle
  // roll is wing to wing
  return atan2(accel.acceleration.y, accel.acceleration.z) * 180.0 / PI;
}

// check device orientation
bool isOrientationFall(const sensors_event_t& accel) {
  float pitch = calculatePitch(accel);
  float roll = calculateRoll(accel);

  // 'fallen' thresholds
  if (abs(pitch) > 50 || abs(roll) > 50) {
    return true;
  }
  return false;
}
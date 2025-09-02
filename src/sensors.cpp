#include <sensors.h>

// Sensor initalization
Adafruit_MPU6050 mpu;       // gyroscope and accelerometer
Adafruit_BME280 bme;        // temperature, humidity, and pressure
MAX30105 max30102;          // heart rate and SpO2
Adafruit_Sensor *mpu_accel, *mpu_gyro;

TFT_eSPI tft = TFT_eSPI();;

// MPU variables
// ** acceleration thresholds for step detection **
float upperThreshold;           // threshold for step start
float lowerThreshold;           // threshold for reset
float mean;

bool repeatFlag = false;              // bool to prevent multiple steps recorded in one step
unsigned long lastStep = 0;           // time of last step
const unsigned long stepDelay = 400;  // delay between recorded steps

// ** fall detection variables **
// Fall detection parameters
const float FREE_FALL_THRESHOLD = 4.9;        // m/2^2
const float IMPACT_THRESHOLD = 22.3;          // m/s^2   normal force + sudden change in acceleration
const unsigned long FALL_TIME_WINDOW = 1000;  // ms time window after free fall
unsigned long freeFallTime = 0;
bool freeFallDetected = false;

// BME280 variables
const int delayTime = 10000; // delay between readings
unsigned long startTimeBME = millis();
unsigned long startTimeHB = millis();

// MAX30102 variables
const byte RATE_SIZE = 4;   // can increase for more accurate avg
byte rates[RATE_SIZE];      // array of hb
byte rateSpot = 0;
long lastBeat = 0;          // time at which last hb occured
static unsigned long lastUpdate = 0;
const unsigned long UPDATE_INTERVAL = 1000; // update every 1 second


// MPU6050 step tracker
void stepCalibration(unsigned long now){
  float sum = 0;
  float sumSq = 0;
  int count = 0;

  unsigned int last = now;
  
  // calibrate for 10 s
  int i = 10;
  while(millis() - now < 10000){
    if (millis() - last >= 1000){
      last += 1000;
      i--;
      tft.fillScreen(TFT_BLACK);
      tft.setCursor(0,0);
      tft.println("Calibrating...\n");
      tft.print(i);
      tft.println("s remaining");
    }
  
    sensors_event_t accel;        // sensor event
    mpu_accel->getEvent(&accel);

    float accelMag = accel.acceleration.x * accel.acceleration.x +
                    accel.acceleration.y * accel.acceleration.y +
                    accel.acceleration.z * accel.acceleration.z;
    accelMag = sqrt(accelMag);    // magnitude of acceleration vector

    sum += accelMag;
    sumSq += accelMag * accelMag;
    count++;
  }
  mean = sum / count;
  float variance = (sumSq / count) - (mean * mean);
  float stdDev = sqrt(variance);

  Serial.printf("THRESHOLD:%f", mean);
  upperThreshold = stdDev * 0.94;                     // threshold for step start
  lowerThreshold = stdDev * 0.3;                            // threshold for step reset
}

void stepTracker(unsigned long now, SensorData &data) {
  sensors_event_t accel;        // sensor event
  mpu_accel->getEvent(&accel);

  // acceleration calculation
  float accelMag = sqrt(
    accel.acceleration.x * accel.acceleration.x +
    accel.acceleration.y * accel.acceleration.y +
    accel.acceleration.z * accel.acceleration.z);

  // low pass filter
  const float alpha = 0.3;  // smoothing factor [0,1]
  static float filteredAccel = 0;
  filteredAccel = alpha * accelMag + (1 - alpha) * filteredAccel;
  float dynamicAccel = fabs(filteredAccel - mean);          // subtract gravity from magnitude;

  // check accel threshold, if accel exceeds threshold, record step
  if (!repeatFlag && dynamicAccel > upperThreshold && now - lastStep > stepDelay) {
    data.steps++;                                           // record step, set flag
    lastStep = now;
    repeatFlag = true;
    String msg = "Steps: " + String(data.steps);            // notify of update
    Serial.println(msg);
  }
  else if (repeatFlag && dynamicAccel < lowerThreshold) {   // reset repeat flag
    repeatFlag = false;
  }
}

// MPU6050 fall detection
bool fallDetector(unsigned long now, SensorData &data) {
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
      // NOTE: uncoment for debugging
      // Serial.println("Free fall detected!");
    }
  }
  else{
    // see if an impact occurs
    unsigned long elapsed = now - freeFallTime;
    if (accelMag > IMPACT_THRESHOLD){
      if (isOrientationFall(accel)) {
        return true;
      }
      // reset free fall detection
      freeFallDetected = false;
    }
    else if (elapsed > FALL_TIME_WINDOW) {
      // reset free fall detection if time window exceeded
      freeFallDetected = false;
    }
  }
  return false;
}

// BME temp/humidity
void bmeRead(unsigned long now, SensorData &data){
  // temp
  if (now - startTimeBME > delayTime){
    startTimeBME = now;

    data.humidity = bme.readHumidity();
    data.tempC    = bme.readTemperature();
    data.tempF    = data.tempC * 9/5 + 32;
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

// Helper function for heartbeat
void heartbeat(unsigned long now, SensorData &data){
  long irValue = max30102.getIR();      // infrared
  static int lastValidBPM = 0;        // remember last valid BPM
  
  if (checkForBeat(irValue)){
    // hb sensed
    long delta = now - lastBeat;
    lastBeat = now;

    float BPM = 60 / (delta / 1000.0);

    if (BPM < 255 && BPM > 20){
      rates[rateSpot++] = (byte)BPM;    // store reading in array
      rateSpot %= RATE_SIZE;
    }
  }

  // take avg
  if (now - lastUpdate > 1000) {
    lastUpdate = now;
    int sum = 0;
    byte count = 0;
    for (byte i = 0; i < RATE_SIZE; i++) {
      if (rates[i] > 0){
          sum += rates[i];
          count++;
      }
    }
    if (count > 0){
        data.heartRate = sum / count;
        lastValidBPM = data.heartRate;
    }
  }

  // always keep last valid BPM so webhook can access it
  if (data.heartRate <= 0){
      data.heartRate = lastValidBPM;
  }
}
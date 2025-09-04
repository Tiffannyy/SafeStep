#include <sensors.h>

// Sensor initalization
Adafruit_MPU6050 mpu;       // gyroscope and accelerometer
Adafruit_BME280 bme;        // temperature, humidity, and pressure
MAX30105 max30102;          // heart rate and SpO2
Adafruit_Sensor *mpu_accel, *mpu_gyro;

// MPU variables
// ** acceleration thresholds for step detection **
float threshold = 3.0;                // general threshold for step detection
float upperThreshold = 3.5;           // threshold for step start
float lowerThreshold = 1.0;           // threshold for reset

bool repeatFlag = false;              // bool to prevent multiple steps recorded in one step
unsigned long lastStep = 0;           // time of last step
const unsigned long stepDelay = 400;  // delay between recorded steps

// ** fall detection variables **
// Fall detection parameters
const float FREE_FALL_THRESHOLD = 4.9;        // m/2^2
const float IMPACT_THRESHOLD = 18.6;          // m/s^2   normal force + sudden change in acceleration
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
const int HR_LOW_BPM = 30;                      // alert if sustained below this
const int HR_RECOVER_BPM = 50;                  // reset once above this
const unsigned long HR_LOW_DURATION_MS = 3000;  // must be low for 3s


// MPU6050 step tracker
void stepTracker(unsigned long now, SensorData &data) {
  sensors_event_t accel;        // sensor event
  mpu_accel->getEvent(&accel);

  // acceleration calculation
  float accelMag = accel.acceleration.x * accel.acceleration.x +
                   accel.acceleration.y * accel.acceleration.y +
                   accel.acceleration.z * accel.acceleration.z;
  accelMag = sqrt(accelMag); // magnitude of acceleration vector
  float dynamicAccel = fabs(accelMag - 9.8); // subtract gravity from magnitude;

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
  static unsigned long heartAlertStart = 0;
  static bool heartAlertSent = false;
  
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
    }
  }

  int bpm = data.heartRate;
  if (bpm > 0 && bpm < HR_LOW_BPM) {
    if (heartAlertStart == 0) heartAlertStart = now;
    if (!heartAlertSent && (now - heartAlertStart >= HR_LOW_DURATION_MS)) {
      data.lowHRDetected = true;                  // raise low-HR alert
      heartAlertSent = true;                      // set true until recovery
    }
  } else if (bpm >= HR_RECOVER_BPM) {             // recovered
    heartAlertStart = 0;
    heartAlertSent = false;
  }

}
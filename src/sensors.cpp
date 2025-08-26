#include <sensors.h>

// Sensor initalization
Adafruit_MPU6050 mpu;       // gyroscope and accelerometer
Adafruit_BME280 bme;        // temperature, humidity, and pressure
MAX30105 max30102;          // heart rate and SpO2
Adafruit_Sensor *mpu_accel;

// MPU variables
int steps = 0;                        // step count
// ** acceleration thresholds for step detection **
float threshold = 3.0;                // general threshold for step detection
float upperThreshold = 3.5;           // threshold for step start
float lowerThreshold = 1.0;           // threshold for reset

bool repeatFlag = false;              // bool to prevent multiple steps recorded in one step
unsigned long lastStep = 0;           // time of last step
const unsigned long stepDelay = 400;  // delay between recorded steps

// BME280 variables
const int delayTime = 10000; // delay between readings
unsigned long startTime = millis();


// MPU6050 step tracker
void stepTracker(unsigned long now) {
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

void bmeRead(unsigned long now){
  // temp
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
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <MAX30105.h>
#include <Wire.h>

// Sensor initialization
Adafruit_MPU6050 mpu;     // gyroscope and accelerometer
Adafruit_BME280 bme;      // temperature, humidity, and pressure
MAX30105 max30102;             // heart rate and SpO2
Adafruit_Sensor *mpu_temp, *mpu_accel;

// MPU variables
int steps = 0;                        // step count
float threshold = 1.0;                // acceleration threshold - CHANGE BASED ON TESTING
bool repeatFlag = false;              // bool to prevent multiple steps recorded in one step
unsigned long lastStep = 0;           // time of last step
const unsigned long stepDelay = 300;  // delay between recorded steps


void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // wait for serial port to connect. Needed for native USB
  }
  Wire.begin(); // initialize I2C

  Serial.println("Initializing MPU6050...");

  // Wait for 
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");

    while (1) {
      delay(10); // halt if MPU6050 not found
    }
  }

  Serial.println("MPU6050 found!");
  mpu_temp = mpu.getTemperatureSensor();
  mpu_temp->printSensorDetails();
  mpu_accel = mpu.getAccelerometerSensor();
  mpu_accel->printSensorDetails();

}

void loop() {
  // MPU portion (step counting)
  sensors_event_t temp;
  sensors_event_t accel;
  mpu_temp->getEvent(&temp);
  mpu_accel->getEvent(&accel);

  Serial.print("\t\tTemperature ");
  Serial.print(temp.temperature);
  Serial.println(" deg C");

  float accelZ = accel.acceleration.z;             // read z-axis accel
  unsigned long now = millis();                     // get current time

  if (fabs(accelZ) <= threshold) {                  // check accel threshold
    repeatFlag = false;
    delay(20);
    return;
  }
  if (now - lastStep <= stepDelay || repeatFlag) {  // check stepDelay and repeatFlag
    delay(20);
    return;
  }

  steps++;                                          // record step, set flag
  lastStep = now;
  repeatFlag = true;
  String msg = "Steps: " + String(steps);           // notify of update
  Serial.println(msg);
  delay(20);
}
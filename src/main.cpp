#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <MAX30105.h>
#include <heartRate.h>
#include <Wire.h>
#include <SPI.h>
#include <TFT_eSPI.h>

#include <WiFi.h>
#include <HTTPClient.h>

// definitions
#define SEALEVELPRESSURE_HPA (1013.25)  // bme

// Webhook configuration
const char* WIFI_SSID      = "wifi id"; // CHANGE THIS TO MATCH WIFI <--- !!!
const char* WIFI_PASS      = "wifi password";
const char* WEBHOOK_URL    = "https://safe-step-vercel-webhook-receiver.vercel.app/api/alert"; // vercel URL
const char* WEBHOOK_SECRET = "group5-secret";                              // MUST match Vercel env var WEBHOOK_SECRET

// Button constants
const int BUTTON_PIN = 0;                         // CHANGE THIS to actual button pin. <--- !!!
// NOTE: current implementation uses INPUT_PULLUP. So button is connected to BUTTON_PIN and GND. Reads LOW when pressed. <--- !!!
const unsigned long LONG_PRESS_MS = 3000;         // hold time to trigger webhook
const unsigned long DEBOUNCE_MS    = 40;          // debounce window

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
byte rates(RATE_SIZE);      // array of hb
byte rateSpot = 0;
long lastBeat = 0;          // time at which last hb occured

// Button variables
bool pressedStable = false;             // updated if button is held past debounce length
bool sentForThisHold = false;           // only sends one webhook per button press
unsigned long lastEdgeMs = 0;           // last time raw input toggled
unsigned long pressStartMs = 0;         // when the current press started

float BPM;
int beatAvg;

// Function prototypes
void stepTracker(unsigned long now);
void fallDetector(unsigned long now);
void bmeRead(unsigned long now);

// Helper function prototypes
float calculatePitch(const sensors_event_t& accel);
float calculateRoll(const sensors_event_t& accel);
bool isOrientationFall(const sensors_event_t& accel);
void connectWiFi();
void sendWebhook(const String& event, const String& msg);
void handleButton(unsigned long now);
static inline bool readPressedRaw();

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

  // button setup
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Wait for devices
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050...");

    while (1) delay(1000);
  }

  if (!bme.begin()){
    Serial.println("Failed to find BME280...");
    while (1) delay(1000);
  }

  if (!max30102.begin(Wire, I2C_SPEED_FAST)){
    Serial.println("Failed to find MAX30102");
    while (1) delay(1000);
  }

  // initialize sesnors
  mpu_accel = mpu.getAccelerometerSensor();
  mpu_gyro = mpu.getGyroSensor();

  max30102.setup();
  max30102.setPulseAmplitudeRed(0x0A);      // turn red LED to low to indicate sensor works
  max30102.setPulseAmplitudeGreen(0);       // turn off green LED

  // connect to wifi
  connectWiFi();

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
  // heartbeat
  heartbeat(now);

  // check for button press. If pressed, send webhook
  handleButton(now);

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
  if (now - startTimeBME > delayTime){
    startTimeBME = now;

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

// Helper function for heartbeat
void heartbeat(unsigned long now){
  long irValue = max30102.getIR();      // infrared

  if (checkForBeat(irValue)){
    // hb sensed
    long delta = now - lastBeat;
    lastBeat = now;

    BPM = 60 / (delta / 1000.0);

    if (BPM < 255 && BPM > 20){
      rates[rateSpot++] = (byte)BPM;    // store reading in array
      rateSpot %= RATE_SIZE;

      // take avg
      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++)
        beatAvg += rates[x]
      beatAvg /= RATE_SIZE;
    }

  }
  // TODO: add structs for each sensor so we can move multiple variables btwn functions
  // if (now - startTimeBME > delayTime)
}


// Helper functions for button handling
static inline bool readPressedRaw() {
  return digitalRead(BUTTON_PIN) == LOW;
}

// button press detection and webhook trigger function
void handleButton(unsigned long now) {
  // debounce edge detection
  static bool lastRaw = readPressedRaw();
  bool raw = readPressedRaw();

  if (raw != lastRaw) {
    lastRaw = raw;
    lastEdgeMs = now;
  }

  // after debounce window, accept the new stable state
  if (now - lastEdgeMs > DEBOUNCE_MS) {
    if (pressedStable != lastRaw) {
      pressedStable = lastRaw;
      if (pressedStable) {
        // became pressed
        pressStartMs = now;
        sentForThisHold = false;
        Serial.println("[BUTTON] pressed");
      } else {
        // became released
        if (!sentForThisHold) {
          Serial.println("[BUTTON] released (no long-press)");
        } else {
          Serial.println("[BUTTON] released (alert already sent)");
        }
        sentForThisHold = false; // allow next hold to alert again
      }
    }
  }

  // if held long enough and not yet sent for this hold, send webhook
  if (pressedStable && !sentForThisHold) {
    if (now - pressStartMs >= LONG_PRESS_MS) {
      sentForThisHold = true;
      Serial.println("[BUTTON] long-press detected → sending webhook");

      // read BME280 values
      float hum   = bme.readHumidity();
      float tempC = bme.readTemperature();
      float tempF = tempC * 9.0 / 5.0 + 32.0;

      // steps
      int stepCount = steps;

      // heart rate (placeholder, replace with real MAX30105 readings)
      float heartRate = 0; // TODO: implement MAX30105 heart rate reading

      // TODO: Combine functions
      // build combined message
      String msg = "Help button held for 3s\n";
      msg += "Temp: " + String(tempC, 2) + " C / " + String(tempF, 2) + " F\n";
      msg += "Humidity: " + String(hum, 2) + " %\n";
      msg += "Steps: " + String(stepCount) + "\n";
      msg += "Heart Rate: " + String(heartRate, 1) + " bpm\n";

      // display on TFT
      tft.setCursor(0, 0);
      tft.fillScreen(TFT_BLACK);
      tft.println("ALERT: Long press");

      // send webhook with all sensor data
      sendWebhook("button_long_press", msg);
    }
  }
}

// wifi connection helper function
void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(200);
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[WiFi] Connected. IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("[WiFi] Not connected.");
  }
}

// webhook post helper function
void sendWebhook(const String& event, const String& msg) {
  connectWiFi();
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WEBHOOK] Wi-Fi not connected; cannot send.");
    return;
  }

  HTTPClient http;
  http.setTimeout(6000);
  http.begin(WEBHOOK_URL);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("X-Webhook-Secret", WEBHOOK_SECRET);

  // json payload - add more fields later (bpm/temp/steps/etc.)
  String body = String("{\"event\":\"") + event +
                "\",\"msg\":\"" + msg +
                "\",\"ts\":" + millis() + "}";

  int code = http.POST(body);
  Serial.printf("[WEBHOOK] POST %d %s\n", code, body.c_str());
  http.end();
}

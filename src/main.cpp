#include <sensors.h>
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

// Display initialization
TFT_eSPI tft = TFT_eSPI();  // TFT display

// Button variables
bool pressedStable = false;             // updated if button is held past debounce length
bool sentForThisHold = false;           // only sends one webhook per button press
unsigned long lastEdgeMs = 0;           // last time raw input toggled
unsigned long pressStartMs = 0;         // when the current press started

// Function prototypes
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

  // connect to wifi
  connectWiFi();

  tft.println("Booting...");
}

void loop() {
  // current time
  unsigned long now = millis();

  // MPU6050 step tracker
  stepTracker(now);

  // check for button press. If pressed, send webhook
  handleButton(now);

  bmeRead(now);
}

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

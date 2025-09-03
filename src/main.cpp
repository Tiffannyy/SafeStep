#include <sensors.h>
#include <secrets.h>
#include <Wire.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <string>
#include <ESP_Mail_Client.h>

// --Definitions--
#define SEALEVELPRESSURE_HPA (1013.25)  // bme

// WiFi Credentials
// TODO: add wifi credentials to secrets.h

// Azure IoT Hub config
// TODO: generate a token on IoT Hub, sensitive info
#define SAS_TOKEN "TOKEN"

// Root CA certificate for Azure IoT Hub
// run openssl s_client -showcerts -connect [your hub name].azure-devices.net:443 in azure terminal
// TODO: retrieve root ca, add to secrets.h sensitive info

String iothubName = "hubname"; //Your hub name (replace if needed)
String deviceName = "devicename"; // Your device name (replace if needed)
String url = "https://" + iothubName + ".azure-devices.net/devices/" +
deviceName + "/messages/events?api-version=2021-04-12";

// Telemetry interval
#define TELEMETRY_INTERVAL 5000 // Send data every 5 seconds

uint8_t count = 0;
uint32_t lastTelemetryTime = 0;

// --Gmail STMP config--
// TODO: change AUTHOR_EMAIL to user email, and use an app password (in secrets.h)
#define SMTP_HOST "smtp.gmail.com"
#define SMTP_PORT 465
#define AUTHOR_DOMAIN ""
// TODO: Change recipient info, add email in secrets.h
#define RECIPIENT_NAME "Recipient"

// --Fall detection variables--
bool fallPending = false;
unsigned long fallDetectedMs = 0;
const unsigned long FALL_TIMEOUT_MS = 30000;    // 30s time slot to cancel alert

// --TFT variables--
unsigned long lastUpdate = 0;
const unsigned long UPDATE_INTERVAL = 500;      // ms                            // MUST match Vercel env var WEBHOOK_SECRET

// --Button constants--
const int BUTTON_PIN = 25;                        // CHANGE THIS to actual button pin. <--- !!!
// NOTE: current implementation uses INPUT_PULLUP. So button is connected to BUTTON_PIN and GND. Reads LOW when pressed. <--- !!!
const unsigned long LONG_PRESS_MS = 3000;         // hold time to trigger webhook
const unsigned long DEBOUNCE_MS    = 40;          // debounce window

// --Sensor initialization--
SensorData sensorData;

// --Button variables--
bool pressedStable = false;             // updated if button is held past debounce length
bool sentForThisHold = false;           // only sends one webhook per button press
unsigned long lastEdgeMs = 0;           // last time raw input toggled
unsigned long pressStartMs = 0;         // when the current press started

float BPM;
int beatAvg;

// --Function prototypes--
void displayData(const SensorData& data);
void connectWiFi();
void sendData(unsigned long now, const SensorData& data);
void handleButton(unsigned long now, const SensorData& data);
static inline bool readPressedRaw();
void sendEmailAlert(String subject, String msg);

// --Begin program--
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
    Serial.println("Failed to find MPU6050");

    while (1) delay(1000);
  }

  if (!bme.begin()){
    Serial.println("Failed to find BME280");
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
  delay(500);

  // ---Calibrations---
  stepCalibration(millis());
}


void loop() {
  // current time
  unsigned long now = millis();

  // MPU6050 step tracker
  stepTracker(now, sensorData);
  // MPU6050 fall detection
  bool fall = fallDetector(now, sensorData);
  if (fall && !fallPending){
    fallPending = true;
    fallDetectedMs = now;
    Serial.println("** FALL DETECTED! **");
  }

  // bme temp and humidity read
  bmeRead(now, sensorData);
  // heartbeat
  heartbeat(now, sensorData);
  // tft display
  if (!fallPending && (now - lastUpdate >= UPDATE_INTERVAL)){
    displayData(sensorData);
    lastUpdate = now;
  }

  // fall check
  if (fallPending){
    unsigned long elapsed = now - fallDetectedMs;
    int remaining = (FALL_TIMEOUT_MS - elapsed) / 1000;

    static int lastRemaining = -1;
    if (remaining != lastRemaining){
      lastRemaining = remaining;
      tft.fillScreen(TFT_BLACK);
      tft.setCursor(0,0);
      tft.println("FALL DETECTED!");
      tft.println("Hold button for 3 seconds to cancel alert");
      tft.printf("Auto alert in %d s\n", remaining);
    }
  
  }
  // alert was not cancelled, send an alert
  if (fallPending && (now - fallDetectedMs >= FALL_TIMEOUT_MS)) {
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(0,0);
    tft.println("SENDING ALERT!");

    Serial.println("[FALL] No button press in 30s -> sending automatic webhook");

    String message = "This is a SafeStep alert:\nThe current user of the " \
                    "device has fallen and has not cancelled the alert.\n" \
                    "Assistance may be required!";
    
    sendEmailAlert("[SafeStep Alert] Assistance Requested", message);

    // reset pending state
    fallPending = false;
  }

  // button check
  handleButton(now, sensorData);
  
  // send data
  if (now - lastTelemetryTime >= TELEMETRY_INTERVAL){
    sendData(now, sensorData);
    lastTelemetryTime = now;
  }
}

// Display data
void displayData(const SensorData& data){
    // display temp
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(0, 0);
    tft.printf("Temp:\n%.2f C\n", data.tempC);
    tft.printf("%.2f F\n", data.tempF);

    // display humidity
    tft.printf("\nHumidity:\n%.2f %%\n", data.humidity);

    // display heart rate
    tft.printf("\n\nBPM:%d\n", data.heartRate);

    // display steps
    tft.printf("\nSteps:%d", data.steps);
}

// Helper functions for button handling
static inline bool readPressedRaw() {
  return digitalRead(BUTTON_PIN) == LOW;
}

// button press detection and webhook trigger function
void handleButton(unsigned long now, const SensorData& data) {
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
          Serial.println("[BUTTON] released (long press)");
        }
        sentForThisHold = false; // allow next hold to alert again
      }
    }
  }

  // if held long enough and not yet sent for this hold, send webhook
  if (pressedStable && !sentForThisHold) {
    if (now - pressStartMs >= LONG_PRESS_MS) {
      sentForThisHold = true;

      // fall alert cancelled
      if (fallPending){
        fallPending = false;
        tft.fillScreen(TFT_BLACK);
        tft.setCursor(0, 0);
        tft.println("Fall Dismissed");
      }
    }
  }
}

// wifi connection helper function
void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
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

// Send data to Azure
void sendData(unsigned long now, const SensorData& data) {
  // create JSON
  ArduinoJson::JsonDocument doc;
  doc["temperature"]  = data.tempC;
  doc["humidity"]     = data.humidity;
  doc["heartrate"]    = data.heartRate;
  doc["steps"]        = data.steps;
  doc["timestamp"]    = now;
  char buffer[256];
  serializeJson(doc, buffer, sizeof(buffer));

  // send JSON via HTTPS
  WiFiClientSecure client;
  client.setCACert(root_ca);    // Set root CA certificate

  HTTPClient http;
  http.setTimeout(6000);
  http.begin(client, url);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Authorization", SAS_TOKEN);

  int httpCode = http.POST(buffer);
  if (httpCode != 204) {        // IoT Hub returns 204 No Content for successful telemetry
    Serial.println("Failed to send telemetry. HTTP code: " + String(httpCode));
  }
  // NOTE: Uncomment for debug
  else{
    Serial.println("[Telemetry] Sending JSON:");
    Serial.println(buffer);
  }
  http.end();
}


// --Email alert--
void sendEmailAlert(String subject, String msg) {
  SMTPSession smtp;
  ESP_Mail_Session session;

  session.server.host_name = SMTP_HOST;
  session.server.port = SMTP_PORT;
  session.login.email = AUTHOR_EMAIL;
  session.login.password = AUTHOR_PASSWORD;
  session.login.user_domain = AUTHOR_DOMAIN;

  if (!smtp.connect(&session)) {
    Serial.println("SMTP connection failed.");
    return;
  }

  SMTP_Message emailMsg;
  emailMsg.sender.name = "IoT Alert";
  emailMsg.sender.email = AUTHOR_EMAIL;
  emailMsg.subject = subject;
  emailMsg.addRecipient(RECIPIENT_NAME, RECIPIENT_EMAIL);
  emailMsg.text.content = msg.c_str();

  // send email
  if (!MailClient.sendMail(&smtp, &emailMsg)) {
    Serial.print("Error sending email: ");
    Serial.println(smtp.errorReason());
  } else {
    Serial.println("Email sent");
  }

  smtp.closeSession();
}

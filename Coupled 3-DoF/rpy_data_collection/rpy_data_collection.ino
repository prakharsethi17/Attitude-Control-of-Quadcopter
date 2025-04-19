#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include "driver/ledc.h"

// ------------------------------------
// Wi-Fi and WebSocket Configuration
// ------------------------------------
const char* ssid     = "Captain";
const char* password = "waeg1466";
WebSocketsServer webSocket(81);

// ------------------------------------
// BNO08x IMU Configuration
// ------------------------------------
#define SDA_PIN 21
#define SCL_PIN 22
Adafruit_BNO08x bno08x;

// ------------------------------------
// Motor Pin Configuration
// ------------------------------------
const int in1 = 14; // M1 (right front)
const int in5 = 26; // M2 (left rear)
const int in7 = 32; // M3 (left front)
const int in3 = 15; // M4 (right rear)

// ------------------------------------
// PWM/LEDC v3 Settings
// ------------------------------------
const int freq               = 70000;                 // 70 kHz
const ledc_timer_bit_t bitWidth = LEDC_TIMER_10_BIT;
const ledc_timer_t timer       = LEDC_TIMER_0;
const ledc_mode_t speedMode    = LEDC_HIGH_SPEED_MODE;
const ledc_channel_t chM1      = LEDC_CHANNEL_0;
const ledc_channel_t chM2      = LEDC_CHANNEL_1;
const ledc_channel_t chM3      = LEDC_CHANNEL_2;
const ledc_channel_t chM4      = LEDC_CHANNEL_3;

// ------------------------------------
// Experiment Parameters
// ------------------------------------
const unsigned long experimentDuration = 10UL * 60UL * 1000UL; // 10 min
const unsigned long onDuration         = 10UL * 1000UL;         // 10 s
const unsigned long offDuration        = 3UL  * 1000UL;         // 3 s
const unsigned long cyclePeriod        = onDuration + offDuration;

bool armed = false;
unsigned long experimentStart = 0;
int randM1, randM2, randM3, randM4;

// ------------------------------------
// Telemetry Buffer (5 readings @50 ms)
// ------------------------------------
const int bufferSize = 5;
int idx = -1;
float rollAngles[bufferSize], pitchAngles[bufferSize], yawAngles[bufferSize];
float rollVels[bufferSize], pitchVels[bufferSize], yawVels[bufferSize];
int m1s[bufferSize], m2s[bufferSize], m3s[bufferSize], m4s[bufferSize];
unsigned long ctrlTimes[bufferSize], fullTimes[bufferSize];

// ------------------------------------
// Timing Intervals (microseconds)
// ------------------------------------
const unsigned long controlInterval   = 10000UL;  // 10 ms
const unsigned long telemetryInterval = 50000UL;  // 50 ms
unsigned long lastControlTime   = 0;
unsigned long lastTelemetryTime = 0;
unsigned long lastFullLoopTime  = 0;

// ------------------------------------
// Sensor State
// ------------------------------------
float latestRoll = 0, latestPitch = 0, latestYaw = 0;
float latestGyroRateX = 0, latestGyroRateY = 0, latestGyroRateZ = 0;

// ------------------------------------
// Utility: Quaternion → Euler
// ------------------------------------
void quaternionToEuler(float qr, float qi, float qj, float qk,
                       float &roll, float &pitch, float &yaw) {
  float sqr = qr*qr, sqi = qi*qi, sqj = qj*qj, sqk = qk*qk;
  roll  = atan2(2.0f*(qj*qk + qi*qr), (-sqi - sqj + sqk + sqr));
  pitch = asin(-2.0f*(qi*qk - qj*qr) / (sqi + sqj + sqk + sqr));
  yaw   = atan2(2.0f*(qi*qj + qk*qr), (sqi - sqj - sqk + sqr));
  roll  *= (180.0f / PI);
  pitch *= (180.0f / PI);
  yaw   *= (180.0f / PI);
}

// ------------------------------------
// Motor Setup & Control (LEDC v3)
// ------------------------------------
void setupLEDC() {
  // Timer configuration
  ledc_timer_config_t tcfg;
  tcfg.speed_mode       = speedMode;
  tcfg.duty_resolution  = bitWidth;
  tcfg.timer_num        = timer;
  tcfg.freq_hz          = freq;
  tcfg.clk_cfg          = LEDC_AUTO_CLK;
  ledc_timer_config(&tcfg);

  // Channel configuration template
  ledc_channel_config_t ccfg;
  ccfg.speed_mode = speedMode;
  ccfg.timer_sel  = timer;
  ccfg.intr_type  = LEDC_INTR_DISABLE;
  ccfg.hpoint     = 0;
  ccfg.duty       = 0;

  // M1
  ccfg.channel  = chM1;
  ccfg.gpio_num = in1;
  ledc_channel_config(&ccfg);
  // M2
  ccfg.channel  = chM2;
  ccfg.gpio_num = in5;
  ledc_channel_config(&ccfg);
  // M3
  ccfg.channel  = chM3;
  ccfg.gpio_num = in7;
  ledc_channel_config(&ccfg);
  // M4
  ccfg.channel  = chM4;
  ccfg.gpio_num = in3;
  ledc_channel_config(&ccfg);
}

void setMotor(ledc_channel_t channel, int duty) {
  ledc_set_duty(speedMode, channel, duty);
  ledc_update_duty(speedMode, channel);
}

// ------------------------------------
// WebSocket Commands
// ------------------------------------
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_TEXT) {
    String cmd = (char*)payload;
    if (cmd == "A") {
      armed = true;
      experimentStart = millis();
      randomSeed(experimentStart);
      randM1 = random(150, 1024);
      randM2 = random(150, 1024);
      randM3 = random(150, 1024);
      randM4 = random(150, 1024);
      webSocket.sendTXT(num, "Experiment started");
    } else if (cmd == "D") {
      armed = false;
      webSocket.sendTXT(num, "Experiment stopped");
      setMotor(chM1, 0);
      setMotor(chM2, 0);
      setMotor(chM3, 0);
      setMotor(chM4, 0);
    } else {
      webSocket.sendTXT(num, "Unknown command");
    }
  }
}

// ------------------------------------
// Initialization
// ------------------------------------
void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print('.'); }
  Serial.println("\nWiFi connected");

  ArduinoOTA.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  setupLEDC();

  Wire.begin(SDA_PIN, SCL_PIN);
  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip!");
    while (1) delay(10);
  }
  bno08x.enableReport(SH2_ROTATION_VECTOR, controlInterval);
  bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, controlInterval);

  lastControlTime   = micros();
  lastTelemetryTime = micros();
  lastFullLoopTime  = micros();
}

// ------------------------------------
// Main Loop
// ------------------------------------
void loop() {
  ArduinoOTA.handle();
  webSocket.loop();

  unsigned long nowFull      = micros();
  unsigned long fullLoopTime = nowFull - lastFullLoopTime;
  lastFullLoopTime           = nowFull;

  // Control + Sensor Sampling @ 10 ms
  if (nowFull - lastControlTime >= controlInterval) {
    lastControlTime = nowFull;
    unsigned long ctrlStart = micros();

    sh2_SensorValue_t sensorValue;
    while (bno08x.getSensorEvent(&sensorValue)) {
      if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
        quaternionToEuler(sensorValue.un.rotationVector.real,
                          sensorValue.un.rotationVector.i,
                          sensorValue.un.rotationVector.j,
                          sensorValue.un.rotationVector.k,
                          latestRoll, latestPitch, latestYaw);
      } else if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
        latestGyroRateX = sensorValue.un.gyroscope.x * (180.0f/PI);
        latestGyroRateY = sensorValue.un.gyroscope.y * (180.0f/PI);
        latestGyroRateZ = sensorValue.un.gyroscope.z * (180.0f/PI);
      }
    }

    // Experiment logic
    if (armed && (millis() - experimentStart) < experimentDuration) {
      unsigned long phase = (millis() - experimentStart) % cyclePeriod;
      bool motorOn = (phase < onDuration);
      setMotor(chM1, motorOn ? randM1 : 0);
      setMotor(chM2, motorOn ? randM2 : 0);
      setMotor(chM3, motorOn ? randM3 : 0);
      setMotor(chM4, motorOn ? randM4 : 0);
    } else if (armed) {
      // end experiment
      armed = false;
      setMotor(chM1, 0); setMotor(chM2,0);
      setMotor(chM3, 0); setMotor(chM4,0);
    }

    unsigned long ctrlEnd = micros();
    unsigned long ctrlLoopTime = ctrlEnd - ctrlStart;

    // Store telemetry
    idx = (idx + 1) % bufferSize;
    rollAngles[idx]  = latestRoll;
    rollVels[idx]    = latestGyroRateX;
    pitchAngles[idx] = latestPitch;
    pitchVels[idx]   = latestGyroRateY;
    yawAngles[idx]   = latestYaw;
    yawVels[idx]     = latestGyroRateZ;
    m1s[idx] = armed ? randM1 : 0;
    m2s[idx] = armed ? randM2 : 0;
    m3s[idx] = armed ? randM3 : 0;
    m4s[idx] = armed ? randM4 : 0;
    ctrlTimes[idx]  = ctrlLoopTime;
    fullTimes[idx]  = fullLoopTime;
  }

  // Telemetry @ 50 ms
  if (micros() - lastTelemetryTime >= telemetryInterval) {
    lastTelemetryTime = micros();
    String json = "{";
    // ... (compose JSON as before) ...
    webSocket.broadcastTXT(json);
  }
}

#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

// --- Conditional definition for calibrated gyroscope report ---
// If your Adafruit_BNO08x library does not define SH2_GYROSCOPE,
// then we use SH2_RAW_GYROSCOPE as a fallback.
// For the sensor’s built‐in calibrated angular velocity vector, ensure you update your library.
#ifndef SH2_GYROSCOPE
  #warning "SH2_GYROSCOPE not defined in this library. Using SH2_RAW_GYROSCOPE as fallback, which may be uncalibrated."
  #define SH2_GYROSCOPE SH2_RAW_GYROSCOPE
#endif

// --------------------------
// Configuration
// --------------------------
const char* ssid     = "Captain";
const char* password = "waeg1466";

WebSocketsServer webSocket(81);

// Motor pins
// DRV1:
const int in1 = 14, in2 = 27; // M1: PWM pin and direction pin
const int in3 = 15, in4 = 5;  // M4: PWM pin and direction pin
// DRV2:
const int in5 = 26, in6 = 25; // M2: PWM pin and direction pin
const int in7 = 32, in8 = 33; // M3: PWM pin and direction pin

// PWM settings for LEDC v3 API
const int freq       = 70000;
const int resolution = 10;  // Resolution in bits (e.g. 10 means duty from 0 to 1023)

// BNO08x sensor configuration
#define SDA_PIN 21
#define SCL_PIN 22
Adafruit_BNO08x bno08x;

// --------------------------
// Global Variables
// --------------------------
bool armed = false;
int basePWM1 = 150, basePWM2 = 150, basePWM3 = 150, basePWM4 = 150;

// --------------------------
// Experiment Configuration
// --------------------------
int exciteMode = 0;           
String experimentMode = "";
const int excitationValues[] = {256, 384, 512, 640, 768, 896, 1023};
const int excitationCount = sizeof(excitationValues) / sizeof(excitationValues[0]);

bool experimentActive = false;
int currentImpulseIndex = 0;
unsigned long experimentStartTime = 0;
unsigned long experimentDuration = 0;
const unsigned long impulseDuration = 2000;   // 2 s per step
const unsigned long stepDuration    = 5000;     // 5 s per step
const unsigned long sineDuration    = 10000;    // 10 s per step
bool waitingBetweenExperiments = false;
unsigned long experimentEndTime = 0;
const unsigned long experimentDelay = 3000;     // 3 s delay between sub-experiments

// For random mode sub-experiments:
bool randomCycleActive = false;
int  randomCycleIndex = 0;
unsigned long randomCycleStart = 0;
int randomM1 = 0, randomM2 = 0, randomM3 = 0, randomM4 = 0;
String experimentCSV = ""; 
const unsigned long randomMaxDuration = 780000UL;
const unsigned long randomOnDuration  = 10000UL;
const unsigned long randomOffDuration = 3000UL;
unsigned long randomExperimentStart = 0;

// --------------------------
// Global Variables for Sensor Data
// --------------------------
float latestRoll = 0.0;     // (Degrees) computed from rotation vector
float rollVelocity = 0.0;   // (Degrees per second) from sensor’s calibrated gyroscope
unsigned long lastControlTime = 0;

// --------------------------
// Utility Functions
// --------------------------

// Converts quaternion (rotation vector) into Euler angles.
void quaternionToEuler(float qw, float qx, float qy, float qz,
                       float &roll, float &pitch, float &yaw) {
  double sinr_cosp = 2.0 * (qw * qx + qy * qz);
  double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
  roll = atan2(sinr_cosp, cosr_cosp) * (180.0 / PI);

  double sinp = 2.0 * (qw * qy - qz * qx);
  if (abs(sinp) >= 1)
    pitch = copysign(90.0, sinp);
  else
    pitch = asin(sinp) * (180.0 / PI);

  double siny_cosp = 2.0 * (qw * qz + qx * qy);
  double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  yaw = atan2(siny_cosp, cosy_cosp) * (180.0 / PI);
}

// --------------------------
// Motor Control Functions using LEDC v3 API
// --------------------------
// Setup motor by attaching the PWM pin with the given frequency and resolution.
// LEDC channel assignment is done automatically.
void setupMotor(int pwmPin, int dirPin) {
  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  digitalWrite(pwmPin, LOW);
  digitalWrite(dirPin, LOW);
  if (!ledcAttach(pwmPin, freq, resolution)) {
    Serial.println("Error attaching LEDC to pin " + String(pwmPin));
  }
}

// Set motor duty cycle. The direction pin is set based on the sign of duty.
void setMotor(int pwmPin, int dirPin, int duty) {
  if (duty > 0) {
    digitalWrite(dirPin, HIGH);
  } else {
    digitalWrite(dirPin, LOW);
  }
  // Write PWM value to the pin (using LEDC v3 API)
  ledcWrite(pwmPin, abs(duty));
}

// --------------------------
// Data Logging
// --------------------------
void captureDataSample(float roll, float velocity, int m1, int m2, int m3, int m4) {
  String line = String(roll, 2) + "," +
                String(velocity, 2) + "," +
                String(m1) + "," + String(m2) + "," + String(m3) + "," + String(m4);
  experimentCSV += line + "\n";
}

void sendCsvOverWebSocket(String mode, int amplitudeOrIndex) {
  String header = "START_CSV:" + mode + "_" + String(amplitudeOrIndex);
  webSocket.broadcastTXT(header);
  webSocket.broadcastTXT(experimentCSV);
  String footer = "END_CSV:" + mode + "_" + String(amplitudeOrIndex);
  webSocket.broadcastTXT(footer);
  experimentCSV = "";
}

// --------------------------
// For impulse/step/sine excitation signals
// --------------------------
double getExcitationSignal() {
  if (experimentMode == "sine") {
    unsigned long elapsed = millis() - experimentStartTime;
    int baseVal = excitationValues[currentImpulseIndex];
    double amplitude = baseVal * 0.1;
    double t = elapsed / 1000.0;
    return baseVal + amplitude * sin(2 * PI * 1.0 * t);
  }
  return excitationValues[currentImpulseIndex];
}

// --------------------------
// For random mode sub-experiments
// --------------------------
void startRandomCycle() {
  randomM1 = random(150, 1024);
  randomM2 = random(150, 1024);
  randomM3 = random(150, 1024);
  randomM4 = random(150, 1024);
  experimentCSV = "";
  randomCycleStart = millis();
  randomCycleActive = true;
}

void endRandomCycle() {
  sendCsvOverWebSocket("random", randomCycleIndex);
  randomCycleIndex++;
  randomCycleActive = false;
}

// --------------------------
// WebSocket Event Handler
// --------------------------
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  if (type == WStype_TEXT) {
    String cmd = String((char*)payload);
    Serial.println("Received: " + cmd);
    
    if (cmd == "A") {
      armed = true;
      if (!experimentActive && experimentMode != "random") {
        setMotor(in1, in2, basePWM1);
        setMotor(in5, in6, basePWM2);
        setMotor(in7, in8, basePWM3);
        setMotor(in3, in4, basePWM4);
      }
      webSocket.sendTXT(num, "[✓] Armed");
    }
    else if (cmd == "D") {
      armed = false;
      setMotor(in1, in2, 0);
      setMotor(in5, in6, 0);
      setMotor(in7, in8, 0);
      setMotor(in3, in4, 0);
      webSocket.sendTXT(num, "[✓] Disarmed");
    }
    else if (cmd.startsWith("E:")) {
      String modeCmd = cmd.substring(2);
      if (modeCmd == "impulse" || modeCmd == "step" || modeCmd == "sine" || modeCmd == "random") {
        experimentMode = modeCmd;
        if (modeCmd == "impulse") {
          currentImpulseIndex = 0;
          experimentDuration = impulseDuration;
          experimentActive = true;
        } 
        else if (modeCmd == "step") {
          currentImpulseIndex = 0;
          experimentDuration = stepDuration;
          experimentActive = true;
        }
        else if (modeCmd == "sine") {
          currentImpulseIndex = 0;
          experimentDuration = sineDuration;
          experimentActive = true;
        }
        else if (modeCmd == "random") {
          randomCycleIndex = 0;
          randomExperimentStart = millis();
          startRandomCycle();
        }
        experimentStartTime = millis();
        webSocket.broadcastTXT("Excitation " + modeCmd + " triggered");
      } else {
        webSocket.broadcastTXT("Unknown excitation mode");
      }
    }
    else {
      webSocket.sendTXT(num, "Unknown command");
    }
  }
}

// --------------------------
// Setup
// --------------------------
void setup() {
  Serial.begin(115200);
  Serial.println("Booting...");
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected! IP: " + WiFi.localIP().toString());
  
  ArduinoOTA.setHostname("nano");
  ArduinoOTA.setPassword("prakhar");
  ArduinoOTA.begin();
  
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  
  // Setup motor pins with LEDC attachment (no channel parameter needed)
  setupMotor(in1, in2);
  setupMotor(in5, in6);
  setupMotor(in7, in8);
  setupMotor(in3, in4);
  
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.println("Starting BNO08x IMU...");
  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip!");
    while (1) delay(10);
  }
  Serial.println("BNO08x Found!");
  
  // Enable rotation vector (absolute orientation) report.
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR, 20)) {
    Serial.println("Could not enable rotation vector report");
    while (1) delay(10);
  }
  Serial.println("Rotation Vector report enabled at 20 ms sampling.");
  
  // Enable the calibrated gyroscope report.
  if (!bno08x.enableReport(SH2_GYROSCOPE, 20)) {
    Serial.println("Could not enable calibrated gyroscope report");
    while (1) delay(10);
  }
  Serial.println("Calibrated Gyroscope report enabled at 20 ms sampling.");

  lastControlTime = millis();
}

// --------------------------
// Loop
// --------------------------
void loop() {
  ArduinoOTA.handle();
  webSocket.loop();
  
  unsigned long now = millis();
  
  // Run the control loop approximately every 10 ms.
  if (now - lastControlTime >= 10) {
    lastControlTime = now;
    
    // Process all available sensor events.
    sh2_SensorValue_t sensorValue;
    while (bno08x.getSensorEvent(&sensorValue)) {
      if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
        float qw = sensorValue.un.rotationVector.real;
        float qx = sensorValue.un.rotationVector.i;
        float qy = sensorValue.un.rotationVector.j;
        float qz = sensorValue.un.rotationVector.k;
        
        float roll, pitch, yaw;
        quaternionToEuler(qw, qx, qy, qz, roll, pitch, yaw);
        latestRoll = roll;
      }
      else if (sensorValue.sensorId == SH2_GYROSCOPE) {
        // Retrieve the sensor’s built‐in (calibrated) angular velocity.
        // (Assuming roll velocity is reported on the x-axis.)
        rollVelocity = sensorValue.un.gyroscope.x * (180.0 / PI);
      }
    }
    
    // Print the current roll angle and roll velocity.
    Serial.print("Roll Angle: ");
    Serial.print(latestRoll, 2);
    Serial.print(" deg, Roll Velocity: ");
    Serial.print(rollVelocity, 2);
    Serial.println(" deg/s");
    
    // --- Handle experiments and manual operation ---
    if (experimentMode == "random") {
      unsigned long totalElapsed = now - randomExperimentStart;
      if (totalElapsed >= randomMaxDuration) {
        if (randomCycleActive) { endRandomCycle(); }
        experimentMode = "";
        webSocket.broadcastTXT("Random sequence complete");
      }
      else {
        if (randomCycleActive) {
          unsigned long cycleElapsed = now - randomCycleStart;
          if (cycleElapsed < randomOnDuration) {
            setMotor(in1, in2, randomM1);
            setMotor(in5, in6, randomM2);
            setMotor(in7, in8, randomM3);
            setMotor(in3, in4, randomM4);
          }
          else if (cycleElapsed < (randomOnDuration + randomOffDuration)) {
            setMotor(in1, in2, 0);
            setMotor(in5, in6, 0);
            setMotor(in7, in8, 0);
            setMotor(in3, in4, 0);
          }
          else {
            endRandomCycle();
            if ((now - randomExperimentStart) < randomMaxDuration) {
              startRandomCycle();
            }
          }
          captureDataSample(latestRoll, rollVelocity, randomM1, randomM2, randomM3, randomM4);
        }
      }
    }
    else if (experimentActive) {
      int excitation = 0;
      if (experimentMode == "sine") {
        unsigned long elapsed = now - experimentStartTime;
        if (elapsed < sineDuration) {
          double baseVal = excitationValues[currentImpulseIndex];
          double amplitude = baseVal * 0.1;
          double t = elapsed / 1000.0;
          excitation = baseVal + (int)(amplitude * sin(2 * PI * 1.0 * t));
        }
        else { 
          excitation = 0; 
        }
      }
      else {
        excitation = excitationValues[currentImpulseIndex];
      }
      setMotor(in1, in2, excitation);
      setMotor(in5, in6, excitation);
      setMotor(in7, in8, excitation);
      setMotor(in3, in4, excitation);
      
      captureDataSample(latestRoll, rollVelocity, excitation, excitation, excitation, excitation);
      
      if (now - experimentStartTime >= experimentDuration) {
        sendCsvOverWebSocket(experimentMode, excitationValues[currentImpulseIndex]);
        waitingBetweenExperiments = true;
        experimentEndTime = now;
        experimentActive = false;
      }
    }
    else if (armed) {
      setMotor(in1, in2, basePWM1);
      setMotor(in5, in6, basePWM2);
      setMotor(in7, in8, basePWM3);
      setMotor(in3, in4, basePWM4);
    }
  }
  
  // Wait between sub-experiments for impulse/step/sine modes.
  if (waitingBetweenExperiments && (now - experimentEndTime >= experimentDelay)) {
    waitingBetweenExperiments = false;
    currentImpulseIndex++;
    if (currentImpulseIndex < excitationCount) {
      experimentActive = true;
      experimentStartTime = now;
      if (experimentMode == "impulse") experimentDuration = impulseDuration;
      else if (experimentMode == "step") experimentDuration = stepDuration;
      else if (experimentMode == "sine") experimentDuration = sineDuration;
    }
    else {
      currentImpulseIndex = 0;
      webSocket.broadcastTXT("Sequence complete (" + experimentMode + ")");
      experimentMode = "";
    }
  }
}

#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

// --------------------------
// Configuration
// --------------------------
const char* ssid     = "Captain";
const char* password = "waeg1466";

WebSocketsServer webSocket(81);

// Motor pins
// DRV1:
const int in1 = 14, in2 = 27; // M1
const int in3 = 15, in4 = 5;  // M4
// DRV2:
const int in5 = 26, in6 = 25; // M2
const int in7 = 32, in8 = 33; // M3

// PWM settings
const int freq       = 70000;
const int resolution = 10;
const int chM1 = 0, chM2 = 1, chM3 = 2, chM4 = 3;

// BNO08x sensor configuration
#define SDA_PIN 21
#define SCL_PIN 22
Adafruit_BNO08x bno08x;

// --------------------------
// Global Variables
// --------------------------

// Manual mode (when not experimenting)
bool armed = false;
int basePWM1 = 150, basePWM2 = 150, basePWM3 = 150, basePWM4 = 150;

// --------------------------
// Experiment Configuration
// --------------------------
int exciteMode = 0;           
String experimentMode = "";

// For impulse/step/sine:
const int excitationValues[] = {256, 384, 512, 640, 768, 896, 1023};
const int excitationCount = sizeof(excitationValues) / sizeof(excitationValues[0]);

bool experimentActive = false;
int currentImpulseIndex = 0;
unsigned long experimentStartTime = 0;
unsigned long experimentDuration = 0;

// Durations for each sub-experiment:
const unsigned long impulseDuration = 2000;   // 2s per step
const unsigned long stepDuration    = 5000;   // 5s per step
const unsigned long sineDuration    = 10000;  // 10s per step
bool waitingBetweenExperiments = false;
unsigned long experimentEndTime = 0;
const unsigned long experimentDelay = 3000; // 3s delay between sub-experiments

// For random mode (split into repeated 10s "on" + 3s "off" cycles):
bool randomCycleActive = false;
int  randomCycleIndex = 0;          // which cycle # we are on
unsigned long randomCycleStart = 0; // when the on-phase began
int randomM1 = 0, randomM2 = 0, randomM3 = 0, randomM4 = 0;

// We do sub-experiments for each 10s on-phase, storing CSV data to send later.
String experimentCSV = ""; 

// We run random mode for up to 10 minutes total:
const unsigned long randomMaxDuration = 600000UL; // 10 minutes
const unsigned long randomOnDuration  = 10000UL;  // 10 seconds on
const unsigned long randomOffDuration = 3000UL;   // 3 seconds off
unsigned long randomExperimentStart = 0;          // overall start of random experiment

// --------------------------
// Global Variables for Sensor & Control Timing
// --------------------------
float latestRoll = 0.0;
float latestAngularVelocity = 0.0; // Angular velocity in deg/s (filtered/fused)
int latestM1 = 0, latestM2 = 0, latestM3 = 0, latestM4 = 0;
unsigned long lastControlTime = 0; // 10 ms loop

// --------------------------
// Utility Functions
// --------------------------
void quaternionToEuler(float qw, float qx, float qy, float qz,
                       float &roll, float &pitch, float &yaw) {
  double sinr_cosp = 2 * (qw * qx + qy * qz);
  double cosr_cosp = 1 - 2 * (qx*qx + qy*qy);
  roll = atan2(sinr_cosp, cosr_cosp) * (180.0 / PI);

  double sinp = 2 * (qw*qy - qz*qx);
  if (abs(sinp) >= 1)
    pitch = copysign(90.0, sinp);
  else
    pitch = asin(sinp) * (180.0 / PI);

  double siny_cosp = 2 * (qw*qz + qx*qy);
  double cosy_cosp = 1 - 2 * (qy*qy + qz*qz);
  yaw = atan2(siny_cosp, cosy_cosp) * (180.0 / PI);
}

void setupMotor(int pwmPin, int dirPin, int channel) {
  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  digitalWrite(pwmPin, LOW);
  digitalWrite(dirPin, LOW);
  ledcAttachChannel(pwmPin, freq, resolution, channel);
}

void setMotor(int pwmPin, int dirPin, int duty) {
  if (duty > 0) {
    digitalWrite(pwmPin, HIGH);
    digitalWrite(dirPin, LOW);
  } else {
    digitalWrite(pwmPin, LOW);
    digitalWrite(dirPin, LOW);
  }
  ledcWrite(pwmPin, duty);
}

// --------------------------
// Data Logging
// --------------------------
// Logs in the order: angle, velocity, m1, m2, m3, m4
void captureDataSample(float roll, float angVel, int m1, int m2, int m3, int m4) {
  String line = String(roll, 2) + "," + String(angVel, 2) + "," +
                String(m1) + "," + String(m2) + "," + String(m3) + "," + String(m4);
  experimentCSV += line + "\n";
}

// Send the CSV data over WebSocket
void sendCsvOverWebSocket(String mode, int amplitudeOrIndex) {
  String header = "START_CSV:" + mode + "_" + String(amplitudeOrIndex);
  webSocket.broadcastTXT(header);
  webSocket.broadcastTXT(experimentCSV);
  String footer = "END_CSV:" + mode + "_" + String(amplitudeOrIndex);
  webSocket.broadcastTXT(footer);
  experimentCSV = ""; // Clear buffer
}

// --------------------------
// For impulse/step/sine
// --------------------------
double getExcitationSignal() {
  if (experimentMode == "sine") {
    unsigned long elapsed = millis() - experimentStartTime;
    int baseVal = excitationValues[currentImpulseIndex];
    double amplitude = baseVal * 0.1;
    double t = elapsed / 1000.0;
    return baseVal + amplitude * sin(2 * PI * 1.0 * t);
  } else {
    return excitationValues[currentImpulseIndex];
  }
}

// --------------------------
// For random mode sub-experiments
// --------------------------
void startRandomCycle() {
  // Pick random motor values
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
      // "impulse", "step", "sine", or "random"
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
          startRandomCycle(); // Start the first random cycle
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
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected! IP: " + WiFi.localIP().toString());
  
  ArduinoOTA.setHostname("nano");
  ArduinoOTA.setPassword("prakhar");
  ArduinoOTA.begin();
  
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  
  setupMotor(in1, in2, chM1);
  setupMotor(in5, in6, chM2);
  setupMotor(in7, in8, chM3);
  setupMotor(in3, in4, chM4);
  
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000); // Fast I2C mode
  
  Serial.println("Starting BNO08x IMU...");
  if(!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip!");
    while(1) delay(10);
  }
  Serial.println("BNO08x Found!");
  
  if(!bno08x.enableReport(SH2_ROTATION_VECTOR, 10)) {
    Serial.println("Could not enable rotation vector report");
    while(1) delay(10);
  }
  Serial.println("Rotation Vector at 10 ms sampling.");

  if(!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10)) {
    Serial.println("Could not enable gyroscope calibrated report");
    while(1) delay(10);
  }
  Serial.println("Gyroscope (calibrated) report at 10 ms sampling.");
  
  lastControlTime = millis();
}

// --------------------------
// Loop
// --------------------------
void loop() {
  ArduinoOTA.handle();
  webSocket.loop();
  
  unsigned long now = millis();
  
  // Run control loop every 10 ms
  if(now - lastControlTime >= 10) {
    lastControlTime = now;
    
    sh2_SensorValue_t sensorValue;
    // Process all available sensor events
    while(bno08x.getSensorEvent(&sensorValue)) {
      if(sensorValue.sensorId == SH2_ROTATION_VECTOR) {
        float qw = sensorValue.un.rotationVector.real;
        float qx = sensorValue.un.rotationVector.i;
        float qy = sensorValue.un.rotationVector.j;
        float qz = sensorValue.un.rotationVector.k;
        
        float roll, pitch, yaw;
        quaternionToEuler(qw, qx, qy, qz, roll, pitch, yaw);
        latestRoll = roll;
      }
      else if(sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
        // Use the fused gyroscope reading on the x-axis and convert rad/s to deg/s
        latestAngularVelocity = sensorValue.un.gyroscope.x * (180.0 / PI);
      }
    }
    
    // Update motors and log data based on the current mode
    if(experimentMode == "random") {
      unsigned long totalElapsed = now - randomExperimentStart;
      if(totalElapsed >= randomMaxDuration) {
        if(randomCycleActive) {
          endRandomCycle();
        }
        experimentMode = "";
        webSocket.broadcastTXT("Random sequence complete");
      } else {
        if(randomCycleActive) {
          unsigned long cycleElapsed = now - randomCycleStart;
          if(cycleElapsed < randomOnDuration) {
            latestM1 = randomM1;
            latestM2 = randomM2;
            latestM3 = randomM3;
            latestM4 = randomM4;
            setMotor(in1, in2, latestM1);
            setMotor(in5, in6, latestM2);
            setMotor(in7, in8, latestM3);
            setMotor(in3, in4, latestM4);
          } else if(cycleElapsed < (randomOnDuration + randomOffDuration)) {
            latestM1 = latestM2 = latestM3 = latestM4 = 0;
            setMotor(in1, in2, 0);
            setMotor(in5, in6, 0);
            setMotor(in7, in8, 0);
            setMotor(in3, in4, 0);
          } else {
            endRandomCycle();
            if((now - randomExperimentStart) < randomMaxDuration) {
              startRandomCycle();
            }
          }
          captureDataSample(latestRoll, latestAngularVelocity, latestM1, latestM2, latestM3, latestM4);
        }
      }
    }
    else if(experimentActive) {
      int excitation = 0;
      if (experimentMode == "sine") {
        unsigned long elapsed = now - experimentStartTime;
        if (elapsed < sineDuration) {
          double baseVal = excitationValues[currentImpulseIndex];
          double amplitude = baseVal * 0.1;
          double t = elapsed / 1000.0;
          excitation = baseVal + (int)(amplitude * sin(2 * PI * 1.0 * t));
        } else {
          excitation = 0;
        }
      } else {
        excitation = excitationValues[currentImpulseIndex];
      }
      latestM1 = excitation;
      latestM2 = excitation;
      latestM3 = excitation;
      latestM4 = excitation;
      setMotor(in1, in2, latestM1);
      setMotor(in5, in6, latestM2);
      setMotor(in7, in8, latestM3);
      setMotor(in3, in4, latestM4);

      captureDataSample(latestRoll, latestAngularVelocity, latestM1, latestM2, latestM3, latestM4);
      
      if(now - experimentStartTime >= experimentDuration) {
        int currentPresetValue = excitationValues[currentImpulseIndex];
        sendCsvOverWebSocket(experimentMode, currentPresetValue);
        waitingBetweenExperiments = true;
        experimentEndTime = now;
        experimentActive = false;
      }
    }
    else if(armed) {
      latestM1 = basePWM1;
      latestM2 = basePWM2;
      latestM3 = basePWM3;
      latestM4 = basePWM4;
      setMotor(in1, in2, latestM1);
      setMotor(in5, in6, latestM2);
      setMotor(in7, in8, latestM3);
      setMotor(in3, in4, latestM4);
    }
  }
  
  // Wait 3 sec between sub-experiments for impulse/step/sine
  if(waitingBetweenExperiments && (now - experimentEndTime >= experimentDelay)) {
    waitingBetweenExperiments = false;
    currentImpulseIndex++;
    if(currentImpulseIndex < excitationCount) {
      experimentActive = true;
      experimentStartTime = now;
      if(experimentMode == "impulse") experimentDuration = impulseDuration;
      else if(experimentMode == "step") experimentDuration = stepDuration;
      else if(experimentMode == "sine") experimentDuration = sineDuration;
    } else {
      currentImpulseIndex = 0;
      webSocket.broadcastTXT("Sequence complete (" + experimentMode + ")");
      experimentMode = "";
    }
  }
}

#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

// ------------------------------------
// Wi-Fi and WebSocket Configuration
// ------------------------------------
const char* ssid     = "Captain";
const char* password = "waeg1466";
WebSocketsServer webSocket(81);

// ------------------------------------
// Motor Pin Configuration
// ------------------------------------
// DRV1:
const int in1 = 14, in2 = 27; // M1 (right front)
const int in3 = 15, in4 = 5;  // M4 (right rear)
// DRV2:
const int in5 = 26, in6 = 25; // M2 (left rear)
const int in7 = 32, in8 = 33; // M3 (left front)

// ------------------------------------
// PWM Settings
// ------------------------------------
const int freq       = 70000;       // 70 kHz
const int resolution = 10;          // 10-bit (0-1023)
const int chM1 = 0, chM2 = 1, chM3 = 2, chM4 = 3;

// ------------------------------------
// BNO08x Sensor Configuration
// ------------------------------------
#define SDA_PIN 21
#define SCL_PIN 22
Adafruit_BNO08x bno08x;

// ------------------------------------
// Cascaded PID Parameters for Control
// ------------------------------------
// Outer loop (angle control)
float Kp_angle = 2.0;   
// Inner loop (velocity control)
float Kp_vel = 0.1;     
float Ki_vel = 0.05;    
float Kd_vel = 0.15;    
const double N_vel = 100.0;   // Derivative filter coefficient for inner loop

// Note: Inner loop now uses Ts = 1 for integration and differentiation

// ------------------------------------
// Global Variables for State and Telemetry
// ------------------------------------
static double desiredRoll = 0.0; // Desired roll angle, set via WebSocket command
bool armed = false;
const int baselinePWM = 150;     // Fixed baseline PWM for motor mapping
int M1 = baselinePWM, M2 = baselinePWM, M3 = baselinePWM, M4 = baselinePWM;
float latestRoll = 0.0;  
float latestGyroRate = 0.0; // Angular velocity (deg/s) from the gyroscope

// Telemetry variables for the PID contributions
float currentP = 0.0, currentI = 0.0, currentD = 0.0, currentU = 0.0;

// <<< Added Global Variables for Velocity Control >>>
double velocitySetpoint = 0.0;
double velocityMeasured = 0.0;
double velocityError = 0.0;

// >>> Moved these from inside if(armed) so they can be reset <<<
static double lastError_vel = 0.0; 
static double iTerm_vel     = 0.0; 
static double dFilt_vel     = 0.0; 

// ------------------------------------
// Timing Variables
// ------------------------------------
unsigned long lastControlTime   = 0;  // For control loop (every 10 ms)
unsigned long lastTelemetryTime = 0;  // Telemetry sent every 100 ms

// ------------------------------------
// Utility Functions
// ------------------------------------
void quaternionToEuler(float qw, float qx, float qy, float qz,
                       float &roll, float &pitch, float &yaw) {
  double sinr_cosp = 2.0 * (qw * qx + qy * qz);
  double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
  roll = atan2(sinr_cosp, cosr_cosp) * (180.0 / PI);

  double sinp = 2.0 * (qw * qy - qz * qx);
  if (fabs(sinp) >= 1.0)
    pitch = copysign(90.0, sinp);
  else
    pitch = asin(sinp) * (180.0 / PI);

  double siny_cosp = 2.0 * (qw * qz + qx * qy);
  double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
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

// ------------------------------------
// WebSocket Event Handler
// ------------------------------------
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_TEXT) {
    String cmd = String((char*)payload);
    Serial.println("Received: " + cmd);

    if (cmd == "A") {
      armed = true;
      Serial.println("[✓] Armed");
      webSocket.sendTXT(num, "[✓] Armed");
    }
    else if (cmd == "D") {
      armed = false;
      setMotor(in1, in2, 0);
      setMotor(in5, in6, 0);
      setMotor(in7, in8, 0);
      setMotor(in3, in4, 0);
      Serial.println("[✓] Disarmed");
      webSocket.sendTXT(num, "[✓] Disarmed");
    }
    // Desired Roll command: "R:<angle>"
    else if (cmd.startsWith("R:")) {
      String angleStr = cmd.substring(2);
      desiredRoll = angleStr.toFloat();
      Serial.println("[!] Desired Roll set to " + String(desiredRoll, 2));
      webSocket.sendTXT(num, "Desired Roll set to " + String(desiredRoll, 2));
    }
    // Update inner loop PID gains command: "K:kp,ki,kd"
    else if (cmd.startsWith("K:")) {
      String gainsStr = cmd.substring(2);
      int comma1 = gainsStr.indexOf(',');
      int comma2 = gainsStr.lastIndexOf(',');
      if (comma1 > 0 && comma2 > comma1) {
        double newKp_vel = gainsStr.substring(0, comma1).toFloat();
        double newKi_vel = gainsStr.substring(comma1+1, comma2).toFloat();
        double newKd_vel = gainsStr.substring(comma2+1).toFloat();
        Kp_vel = newKp_vel;
        Ki_vel = newKi_vel;
        Kd_vel = newKd_vel;
        Serial.println("[!] Inner loop PID gains updated: Kp_vel=" + String(Kp_vel,2) +
                       ", Ki_vel=" + String(Ki_vel,2) + ", Kd_vel=" + String(Kd_vel,2));
        webSocket.sendTXT(num, "Inner loop PID gains set: " + String(Kp_vel,2) + "," +
                                  String(Ki_vel,2) + "," + String(Kd_vel,2));
      } else {
        webSocket.sendTXT(num, "Invalid PID gain format. Use K:kp,ki,kd");
      }
    }
    // Reset PID state command: "reset"
    else if (cmd == "reset") {
      // To reset the inner loop PID states, disarm and re-arm manually.
      armed = false;

      // <<< Only added lines relevant to resetting P_vel, I_vel, D_vel >>>
      lastError_vel = 0.0;
      iTerm_vel     = 0.0;
      dFilt_vel     = 0.0;
      currentP      = 0.0;
      currentI      = 0.0;
      currentD      = 0.0;
      // ^^^ End of new reset lines ^^^

      Serial.println("[✓] PID state reset - Disarmed, then re-arm manually.");
      webSocket.sendTXT(num, "[✓] PID state reset - Disarmed, then re-arm manually.");
    }
    else {
      webSocket.sendTXT(num, "Unknown command: " + cmd);
    }
  }
}

// ------------------------------------
// Setup Function
// ------------------------------------
void setup() {
  Serial.begin(921600);
  Serial.println("Booting...");

  // Wi-Fi Connection
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print("\nWiFi connected! IP: ");
  Serial.println(WiFi.localIP());

  ArduinoOTA.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  // Setup Motors
  setupMotor(in1, in2, chM1);
  setupMotor(in5, in6, chM2);
  setupMotor(in7, in8, chM3);
  setupMotor(in3, in4, chM4);

  // Initialize IMU (BNO08x)
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip!");
    while (1) delay(10);
  }
  Serial.println("BNO08x Found!");

  // Enable Rotation Vector Report @ 10 ms sampling
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR, 10)) {
    Serial.println("Could not enable rotation vector report");
    while (1) delay(10);
  }
  Serial.println("Rotation Vector report enabled at 10 ms sampling.");

  // (Optional) Enable Gyroscope Calibrated Report if available
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10)) {
    Serial.println("Could not enable gyroscope calibrated report");
    // Proceed even if gyroscope report is not enabled.
  } else {
    Serial.println("Gyroscope calibrated report enabled at 10 ms sampling.");
  }

  lastControlTime = millis();
  lastTelemetryTime = millis();
}

// ------------------------------------
// Main Loop
// ------------------------------------
void loop() {
  // Record loop start time in microseconds for telemetry measurement
  unsigned long loopStart = micros();

  ArduinoOTA.handle();
  webSocket.loop();

  unsigned long now = millis();

  // Control loop: execute every 10 ms
  if (now - lastControlTime >= 10) {
    lastControlTime = now;

    // Process all available sensor events
    sh2_SensorValue_t sensorValue;
    while (bno08x.getSensorEvent(&sensorValue)) {
      if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
        // Update roll from the rotation vector
        float qw = sensorValue.un.rotationVector.real;
        float qx = sensorValue.un.rotationVector.i;
        float qy = sensorValue.un.rotationVector.j;
        float qz = sensorValue.un.rotationVector.k;
        float roll, pitch, yaw;
        quaternionToEuler(qw, qx, qy, qz, roll, pitch, yaw);
        latestRoll = roll;
      }
      else if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
        // Use the calibrated gyroscope reading (deg/s).
        latestGyroRate = sensorValue.un.gyroscope.x * (180.0 / PI);
      }
    }

    if (armed) {
      // ---------------------------
      // Cascaded PID Control
      // ---------------------------
      double angleError = desiredRoll - latestRoll;
      velocitySetpoint  = Kp_angle * angleError;
      velocityMeasured  = latestGyroRate;
      velocityError     = velocitySetpoint - velocityMeasured;

      // Inner loop (velocity control)
      double error = velocityError;
      float P_vel = Kp_vel * error;

      iTerm_vel += error; // Ts=1
      float I_vel = Ki_vel * iTerm_vel * 1;

      float alpha = (N_vel * 1) / (1.0 + N_vel * 1);
      double dRaw = (error - lastError_vel) / 1; // Ts=1
      lastError_vel = error;
      dFilt_vel = dFilt_vel + alpha * (dRaw - dFilt_vel);
      float D_vel = Kd_vel * dFilt_vel;

      float u = P_vel + I_vel + D_vel;

      M1 = constrain(baselinePWM - (int)(0.5 * u), 150, 1023); // Right front
      M4 = constrain(baselinePWM - (int)(0.5 * u), 150, 1023); // Right rear
      M2 = constrain(baselinePWM + (int)(0.5 * u), 150, 1023); // Left rear
      M3 = constrain(baselinePWM + (int)(0.5 * u), 150, 1023); // Left front

      setMotor(in1, in2, M1);
      setMotor(in5, in6, M2);
      setMotor(in7, in8, M3);
      setMotor(in3, in4, M4);

      // Update PID telemetry
      currentP = P_vel;
      currentI = I_vel;
      currentD = D_vel;
      currentU = u;
    }
    else {
      M1 = M2 = M3 = M4 = 0;
      setMotor(in1, in2, 0);
      setMotor(in5, in6, 0);
      setMotor(in7, in8, 0);
      setMotor(in3, in4, 0);
    }
  }

  // Telemetry: Send every 100 ms
  if (now - lastTelemetryTime >= 100) {
    lastTelemetryTime = now;
    unsigned long LT = micros() - loopStart;
    /*
    String telemetry = "A_ang:" + String(latestRoll, 2) +
                       "A_vel:" + String(velocityMeasured, 2);
    */
    String telemetry = "T_ang:" + String(desiredRoll, 2) +
                       ",A_ang:" + String(latestRoll, 2) +
                       ",E_ang:" + String(desiredRoll - latestRoll, 2) +
                       ",Out_ang:" + String(velocitySetpoint, 2) +
                       ",T_vel:" + String(velocitySetpoint, 2) +
                       ",A_vel:" + String(velocityMeasured, 2) +
                       ",E_vel:" + String(velocityError, 2) +
                       ",P_vel:" + String(currentP, 2) +
                       ",I_vel:" + String(currentI, 2) +
                       ",D_vel:" + String(currentD, 2) +
                       ",M1:" + String(M1) +
                       ",M2:" + String(M2) +
                       ",M3:" + String(M3) +
                       ",M4:" + String(M4) +
                       ",Kp_a:" + String(Kp_angle, 2) +
                       ",Kp_v:" + String(Kp_vel, 2) +
                       ",Ki_v:" + String(Ki_vel, 2) +
                       ",Kd_v:" + String(Kd_vel, 2) +
                       ",LT:" + String(LT);
    webSocket.broadcastTXT(telemetry);
  }


  // -------------------------------------------------------------------------
  //                    ADDED CODE: Test-loop for ±30° in steps
  // -------------------------------------------------------------------------
  /*
     This block automates a sequence of target angles:
       1) 0 -> +30 in steps of 5, then back to 0
       2) 0 -> -30 in steps of 5, then back to 0
       3) 0 -> +10, 0, -10, 0, +20, 0, -20, 0, +30, 0, -30, 0
       4) 0 -> +10 -> -10 -> 0, +20 -> -20 -> 0, +30 -> -30 -> 0
     It moves to each target only after 2 consecutive sensor readings are
     within a threshold of that target angle.
  */

  // 1) Define the entire test-sequence of angles in an array.
  //    (Feel free to expand or rearrange as needed.)
  static const float testAngles[] = {
    // 0 -> +30 (step 5) -> 0
    0, 5, 10, 15, 20, 25, 30, 25, 20, 15, 10, 5, 0,
    // 0 -> -30 (step 5) -> 0
    0, -5, -10, -15, -20, -25, -30, -25, -20, -15, -10, -5, 0,
    // 0 -> +10 -> 0 -> -10 -> 0 -> +20 -> 0 -> -20 -> 0 -> +30 -> 0 -> -30 -> 0
    0, 10, 0, -10, 0, 20, 0, -20, 0, 30, 0, -30, 0,
    // 0 -> +10 -> -10 -> 0, 0 -> +20 -> -20 -> 0, 0 -> +30 -> -30 -> 0
    0, 10, -10, 0,
    0, 20, -20, 0,
    0, 30, -30, 0
  };
  static const int numTestAngles = sizeof(testAngles)/sizeof(testAngles[0]);

  // 2) We need some static variables to track progress:
  static int testIndex = 0;           // which angle in sequence
  static int consecutiveCount = 0;    // how many consecutive times we've been "close enough"
  const float angleThreshold = 1.0;   // how close we must be to say "we've reached it"
  static bool testActive = false;     // indicates we've started the automated test loop

  // 3) If we are armed, automatically run the test routine (set testActive = true).
  //    Alternatively, you could trigger testActive with a command or a button, etc.
  if (armed) {
    // Once armed, begin the automated test if not already started.
    if (!testActive) {
      testActive = true;
      testIndex = 0;
      consecutiveCount = 0;
      Serial.println("Starting automated test sequence...");
    }

    // If we are still within the testAngles array, set the desiredRoll accordingly
    if (testActive && testIndex < numTestAngles) {
      desiredRoll = testAngles[testIndex];

      // Check if we've reached the target angle (within threshold)
      if (fabs(latestRoll - desiredRoll) < angleThreshold) {
        consecutiveCount++;
        // Once we have 2 consecutive readings inside threshold, move to the next angle
        if (consecutiveCount >= 2) {
          testIndex++;
          consecutiveCount = 0;
          Serial.println("Reached target angle: " + String(desiredRoll, 1));
          // If we've finished all angles, test is done
          if (testIndex >= numTestAngles) {
            testActive = false;
            Serial.println("All test angles complete.");
          }
        }
      }
      else {
        // Not close enough, reset the consecutive counter
        consecutiveCount = 0;
      }
    }
  }
  else {
    // If disarmed, reset the test so it can start again next time we arm.
    testActive = false;
    testIndex = 0;
    consecutiveCount = 0;
  }
}

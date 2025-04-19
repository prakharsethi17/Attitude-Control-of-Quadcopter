#include <Wire.h>
#include <Adafruit_BNO08x.h>

// Define I2C pins (adjust if yours are different)
#define SDA_PIN 21
#define SCL_PIN 22

Adafruit_BNO08x bno08x;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("BNO08x Connection Test");

  // Initialize I2C with your custom SDA/SCL pins
  Wire.begin(SDA_PIN, SCL_PIN);

  // Try to initialize the BNO08x sensor
  if (!bno08x.begin_I2C()) {
    Serial.println("❌ BNO08x not detected. Check wiring and power.");
  } else {
    Serial.println("✅ BNO08x detected!");

    // Optional: Enable a basic report to confirm communication
    if (bno08x.enableReport(SH2_ROTATION_VECTOR)) {
      Serial.println("✅ Rotation vector report enabled.");
    } else {
      Serial.println("❌ Failed to enable rotation vector report.");
    }
  }
}

void loop() {
  // Try to get a sensor event
  sh2_SensorValue_t sensorValue;
  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
      Serial.print("Quat: ");
      Serial.print(sensorValue.un.rotationVector.real);
      Serial.print(", ");
      Serial.print(sensorValue.un.rotationVector.i);
      Serial.print(", ");
      Serial.print(sensorValue.un.rotationVector.j);
      Serial.print(", ");
      Serial.println(sensorValue.un.rotationVector.k);
    }
  }
  delay(100);
}

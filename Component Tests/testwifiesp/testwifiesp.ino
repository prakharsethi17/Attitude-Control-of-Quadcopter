#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoOTA.h>

const char* ssid     = "Captain";
const char* password = "waeg1466";

WebSocketsServer webSocket = WebSocketsServer(81);

unsigned long wifiConnectedMillis = 0;  // Time when connected to WiFi

void setup() {
  Serial.begin(115200);
  Serial.println("Booting...");

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("Connecting to WiFi");
  unsigned long startAttemptTime = millis();
  const unsigned long timeout = 10000;

  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < timeout) {
    Serial.print(".");
    delay(500);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    wifiConnectedMillis = millis();  // Save when WiFi was connected

    // Start WebSocket server
    webSocket.begin();
    webSocket.onEvent(onWebSocketEvent);
    Serial.println("WebSocket server started on port 81");

    // OTA Setup
    ArduinoOTA
      .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else
          type = "filesystem";

        Serial.println("Start updating " + type);
      })
      .onEnd([]() {
        Serial.println("\nEnd");
      })
      .onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      })
      .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
      });

    ArduinoOTA.setHostname("ESP32-WebSocket-OTA");
    ArduinoOTA.begin();
    Serial.println("OTA Ready");
  } else {
    Serial.println("\nFailed to connect to WiFi.");
  }
}

void loop() {
  webSocket.loop();
  ArduinoOTA.handle();
}

void onWebSocketEvent(uint8_t client_num, WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.printf("Client [%u] connected\n", client_num);
      break;

    case WStype_DISCONNECTED:
      Serial.printf("Client [%u] disconnected\n", client_num);
      break;

    case WStype_TEXT:
      Serial.printf("Client [%u] sent text: %s\n", client_num, payload);

      if (strcmp((char*)payload, "ping") == 0) {
        webSocket.sendTXT(client_num, "pong");
      } else if (strcmp((char*)payload, "uptime") == 0) {
        unsigned long uptimeMs = millis() - wifiConnectedMillis;
        unsigned long seconds = uptimeMs / 1000;
        unsigned long mins = (seconds / 60) % 60;
        unsigned long hrs = (seconds / 3600);
        char uptimeStr[32];
        sprintf(uptimeStr, "%luh %lum %lus", hrs, mins, seconds % 60);
        webSocket.sendTXT(client_num, uptimeStr);
      } else {
        webSocket.sendTXT(client_num, (char *)payload, length);  // Echo
      }
      break;

    default:
      break;
  }
}

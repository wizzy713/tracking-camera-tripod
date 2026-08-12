/*
 * CamX Tripod Firmware for ESP32
 * Compatible with CamX Android App
 *
 * Dependencies:
 * - ESP32Servo (Install via Arduino Library Manager)
 *
 * Features:
 * - mDNS auto-discovery as "_arduino._tcp"
 * - UDP coordinate parsing (X:value,Y:value)
 * - Proportional servo control with deadzone
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <ESP32Servo.h>

// --- Configuration ---
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
const char* deviceName = "CamX-Tripod";
const int udpPort = 4210;

// Servo Pins (Adjust based on your wiring)
const int PAN_PIN = 18;
const int TILT_PIN = 19;

// Tracker Settings
const int FRAME_WIDTH = 640;  // Match Android camera resolution
const int FRAME_HEIGHT = 480;
const int DEADZONE = 20;      // Pixels from center to ignore to prevent jitter

// --- Global Objects ---
WiFiUDP udp;
Servo panServo;
Servo tiltServo;

int currentPan = 90;
int currentTilt = 90;
char packetBuffer[255];

void setup() {
  Serial.begin(115200);

  // Initialize Servos
  panServo.attach(PAN_PIN);
  tiltServo.attach(TILT_PIN);
  panServo.write(currentPan);
  tiltServo.write(currentTilt);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Start mDNS discovery for the Android app
  if (!MDNS.begin(deviceName)) {
    Serial.println("Error setting up MDNS responder!");
  } else {
    Serial.println("mDNS responder started");
    MDNS.addService("arduino", "tcp", udpPort);
  }

  // Start UDP
  udp.begin(udpPort);
  Serial.printf("Listening on UDP port %d\n", udpPort);
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(packetBuffer, 255);
    if (len > 0) packetBuffer[len] = 0;

    String payload = String(packetBuffer);
    Serial.println("Received: " + payload);

    // Manual Parsing of "X:val,Y:val"
    int xIndex = payload.indexOf("X:");
    int yIndex = payload.indexOf(",Y:");

    if (xIndex != -1 && yIndex != -1) {
      int targetX = payload.substring(xIndex + 2, yIndex).toInt();
      int targetY = payload.substring(yIndex + 3).toInt();

      updateTripod(targetX, targetY);
    }
  }
}

/**
 * Maps pixel coordinates to servo movements.
 * Android sends (320, 240) for center.
 */
void updateTripod(int x, int y) {
  int centerX = FRAME_WIDTH / 2;
  int centerY = FRAME_HEIGHT / 2;

  // Pan Logic (Horizontal)
  if (abs(x - centerX) > DEADZONE) {
    if (x < centerX) currentPan++; // Object is left, turn left
    else currentPan--;             // Object is right, turn right
  }

  // Tilt Logic (Vertical)
  if (abs(y - centerY) > DEADZONE) {
    if (y < centerY) currentTilt--; // Object is up, tilt up
    else currentTilt++;             // Object is down, tilt down
  }

  // Constrain servos to physical limits
  currentPan = constrain(currentPan, 0, 180);
  currentTilt = constrain(currentTilt, 0, 180);

  panServo.write(currentPan);
  tiltServo.write(currentTilt);
}

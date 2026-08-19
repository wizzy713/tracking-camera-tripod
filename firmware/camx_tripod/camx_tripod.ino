/*
 * CamX Tripod Firmware for ESP32
 * Compatible with CamX Android App
 *
 * Dependencies:
 * - ESP32Servo (Install via Arduino Library Manager)
 *
 * Features:
 * - mDNS advertisement as "_arduino._tcp" (for future discovery tooling)
 * - UDP normalized-error parsing (EX:value,EY:value,SEQ:value)
 * - Proportional (P) servo control with deadzone
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <ESP32Servo.h>
#include <string.h>

// --- Configuration ---
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
const char* deviceName = "CamX-Tripod";
const int udpPort = 4210;

// Servo Pins (Adjust based on your wiring)
const int PAN_PIN = 18;
const int TILT_PIN = 19;

// Control tuning. The Android app sends error as a fraction of half-frame in
// [-1, 1], where 0 means the subject is centred -- these constants no longer
// depend on any particular camera resolution.
const float DEADZONE = 0.03f;  // Normalized error below which we hold still, to prevent jitter
const float KP = 6.0f;         // Degrees of servo motion per unit of normalized error
const float MAX_STEP = 4.0f;   // Max degrees of servo motion applied per packet

// --- Global Objects ---
WiFiUDP udp;
Servo panServo;
Servo tiltServo;

float currentPan = 90.0f;
float currentTilt = 90.0f;

uint32_t lastSeq = 0;
bool haveSeq = false;

const int PACKET_BUFFER_SIZE = 255;
char packetBuffer[PACKET_BUFFER_SIZE];

void setup() {
  Serial.begin(115200);

  // Initialize Servos
  panServo.attach(PAN_PIN);
  tiltServo.attach(TILT_PIN);
  panServo.write((int)currentPan);
  tiltServo.write((int)currentTilt);

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

  // Advertise over mDNS for future discovery tooling
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
  // Drain the socket each iteration so we always act on the most recent
  // packet rather than a backlog of stale coordinates built up under load.
  char latestPacket[PACKET_BUFFER_SIZE];
  int latestLen = -1;

  int packetSize;
  while ((packetSize = udp.parsePacket()) > 0) {
    // Read at most PACKET_BUFFER_SIZE-1 bytes so index [len] is always a
    // valid slot for the null terminator.
    int len = udp.read(packetBuffer, PACKET_BUFFER_SIZE - 1);
    if (len <= 0) continue;
    packetBuffer[len] = 0;
    memcpy(latestPacket, packetBuffer, len + 1);
    latestLen = len;
  }

  if (latestLen > 0) {
    String payload = String(latestPacket);

    int exIndex = payload.indexOf("EX:");
    int eyIndex = payload.indexOf(",EY:");
    int seqIndex = payload.indexOf(",SEQ:");

    if (exIndex != -1 && eyIndex != -1) {
      float errX = payload.substring(exIndex + 3, eyIndex).toFloat();
      float errY;

      if (seqIndex != -1) {
        errY = payload.substring(eyIndex + 4, seqIndex).toFloat();
        uint32_t seq = (uint32_t) payload.substring(seqIndex + 5).toInt();
        if (haveSeq && seq < lastSeq) {
          Serial.println("Dropped out-of-order packet");
          return;
        }
        lastSeq = seq;
        haveSeq = true;
      } else {
        errY = payload.substring(eyIndex + 4).toFloat();
      }

      updateTripod(errX, errY);
    }
  }
}

/**
 * Proportional control from normalized error in [-1, 1] (fraction of
 * half-frame from centre). Positive errX means the subject is right of
 * centre; positive errY means the subject is below centre.
 */
void updateTripod(float errX, float errY) {
  if (fabs(errX) > DEADZONE) {
    float step = constrain(KP * errX, -MAX_STEP, MAX_STEP);
    currentPan -= step; // Flip sign if pan direction is inverted for your mounting.
  }

  if (fabs(errY) > DEADZONE) {
    float step = constrain(KP * errY, -MAX_STEP, MAX_STEP);
    currentTilt += step; // Flip sign if tilt direction is inverted for your mounting.
  }

  // Constrain servos to physical limits
  currentPan = constrain(currentPan, 0.0f, 180.0f);
  currentTilt = constrain(currentTilt, 0.0f, 180.0f);

  panServo.write((int)currentPan);
  tiltServo.write((int)currentTilt);
}

/*
 * CamX Tripod Firmware for ESP32-C6 (WiFi 6)
 * Compatible with CamX Android App
 *
 * Board: ESP32-C6 (e.g. ESP32-C6-DevKitC-1 / DevKitM-1). Requires:
 * - arduino-esp32 core >= 3.0 with ESP32-C6 board support installed
 *   (Boards Manager -> esp32 by Espressif Systems)
 * - ESP32Servo >= 3.0 (Install via Arduino Library Manager) -- older
 *   versions predate the core 3.x LEDC API rewrite and will not compile.
 *
 * Features:
 * - UDP normalized-error parsing (EX:value,EY:value,SEQ:value)
 * - PID speed control with deadzone, for CONTINUOUS-ROTATION (360-degree)
 *   servos -- these have no absolute position, so write() commands a
 *   speed/direction rather than an angle to move to and hold. A neutral
 *   pulse (~90, but calibrated per-axis) means stop. See PAN_NEUTRAL /
 *   TILT_NEUTRAL / MAX_SPEED_OFFSET below.
 * - Loss-of-signal failsafe: stops both motors if no UDP packet has been
 *   received recently, so a dropped connection or closed app can't leave a
 *   continuous-rotation servo spinning indefinitely.
 *
 * No code changes are needed to benefit from WiFi 6 -- the C6 negotiates
 * 802.11ax automatically against a WiFi 6 access point via the same
 * WiFi.begin() call used for 802.11n.
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>
#include <string.h>

// --- Configuration ---
const char* ssid = "Tab";
const char* password = "12345678";
const int udpPort = 4210;

// Servo Pins (Adjust based on your wiring).
//
// ESP32-C6 has a different pinout from the classic ESP32 -- pins 18/19
// used on the original ESP32 do not carry the same meaning here. GPIO2/3
// are safe general-purpose pins on both the DevKitC-1 and DevKitM-1.
//
// AVOID these on ESP32-C6:
//   GPIO4, 5, 8, 9, 15  - strapping pins, sampled at boot/reset
//   GPIO8               - also drives the onboard RGB LED on DevKit boards
//   GPIO12, 13          - reserved for native USB (USB_D-/USB_D+)
// Other safe alternatives: GPIO0, 1, 6, 7, 10, 11, 14, 20, 21, 22, 23
// (verify against your specific board's silkscreen/pinout diagram, since
// breakout boards vary in which of these are actually broken out).
const int PAN_PIN = 2;
const int TILT_PIN = 3;

// Control tuning. The Android app sends error as a fraction of half-frame in
// [-1, 1], where 0 means the subject is centred -- these constants no longer
// depend on any particular camera resolution.
//
// PAN_NEUTRAL/TILT_NEUTRAL are the pulse values that stop each continuous-
// rotation servo. 90 is only a nominal starting guess -- cheap continuous-
// rotation servos (e.g. FS90R-type) are frequently trimmed well off from
// true center (drift of 10-20 is common, not just 1-2), and pan/tilt are two
// separate physical units so they will likely need different values. If a
// servo only ever spins one direction and never reverses no matter which
// way the error points, that means its whole commanded range is landing on
// one side of the *true* stop point -- use the serial calibration mode
// below (send e.g. "P70" or "T110" over Serial Monitor) to find each
// servo's true stop point, then set these constants to match.
const int PAN_NEUTRAL = 90;
const int TILT_NEUTRAL = 90;
const float DEADZONE = 0.08f;         // Normalized error below which we command a full stop, to prevent hunting/jitter near center
const float KP = 5.0f;                // Proportional gain: speed-offset per unit of normalized error
const float KI = 2.0f;                // Integral gain: corrects small persistent bias/creep. Set to 0 to disable.
const float KD = 0.0f;                // Derivative gain: dampens overshoot, but AMPLIFIES noise from a jittery
                                       // vision signal. Start at 0. Only raise this, in small steps, if you still
                                       // see overshoot/oscillation after KP and KI are tuned -- if it makes things
                                       // jerkier, that's noise amplification; back it off.
const float MAX_SPEED_OFFSET = 15.0f; // Max offset from NEUTRAL, i.e. max commanded speed either direction
const float MAX_INTEGRAL = 5.0f;      // Anti-windup clamp on the accumulated integral term

// If no UDP packet arrives within this long, stop both motors. Without this,
// losing WiFi or closing the app would leave a continuous-rotation servo
// spinning at its last commanded speed forever.
const unsigned long SIGNAL_TIMEOUT_MS = 500;

// --- Global Objects ---
WiFiUDP udp;
Servo panServo;
Servo tiltServo;

unsigned long lastPacketMillis = 0;
bool motorsStopped = true;

// Per-axis PID state
float panIntegral = 0.0f;
float tiltIntegral = 0.0f;
float lastErrX = 0.0f;
float lastErrY = 0.0f;
unsigned long lastUpdateMillis = 0;

uint32_t lastSeq = 0;
bool haveSeq = false;

const int PACKET_BUFFER_SIZE = 255;
char packetBuffer[PACKET_BUFFER_SIZE];

void setup() {
  Serial.begin(115200);

  // Initialize Servos. setPeriodHertz + explicit pulse-width range before
  // attach() matches ESP32Servo's own recommendation for non-classic-ESP32
  // targets (C3/C6/S3), where the plain attach(pin) default range does not
  // always suit every servo brand.
  panServo.setPeriodHertz(50);
  tiltServo.setPeriodHertz(50);
  panServo.attach(PAN_PIN, 500, 2400);
  tiltServo.attach(TILT_PIN, 500, 2400);

  // attach() can silently fail to claim a PWM/LEDC channel on some
  // core/library version combinations on the C6 -- log it so a bad attach
  // shows up here instead of as "servo just doesn't move."
  Serial.printf("Pan servo attached: %s\n", panServo.attached() ? "yes" : "NO - check wiring/pin/library version");
  Serial.printf("Tilt servo attached: %s\n", tiltServo.attached() ? "yes" : "NO - check wiring/pin/library version");

  panServo.write(PAN_NEUTRAL);
  tiltServo.write(TILT_NEUTRAL);

  Serial.println("Calibration mode: send \"P<0-180>\" or \"T<0-180>\" over Serial to test a raw");
  Serial.println("pulse value on that servo directly (e.g. \"P70\"), to find its true stop point.");
  Serial.println("Note: live UDP tracking will overwrite a calibration write on the next packet.");

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

  // Start UDP
  udp.begin(udpPort);
  Serial.printf("Listening on UDP port %d\n", udpPort);
}

void loop() {
  handleCalibrationInput();

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

    Serial.print("Received: ");
    Serial.println(payload);

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

      Serial.printf("Parsed -> errX: %.2f, errY: %.2f\n", errX, errY);

      lastPacketMillis = millis();
      motorsStopped = false;
      updateTripod(errX, errY);
    }
  }

  // Failsafe: a continuous-rotation servo keeps spinning at its last
  // commanded speed if we simply stop writing to it, unlike a positional
  // servo which just holds still. If the app closes or WiFi drops, force a
  // stop rather than let it run away.
  if (!motorsStopped && (millis() - lastPacketMillis > SIGNAL_TIMEOUT_MS)) {
    panServo.write(PAN_NEUTRAL);
    tiltServo.write(TILT_NEUTRAL);
    motorsStopped = true;
    Serial.println("Signal lost -- motors stopped");
  }
}

/**
 * Reads a line like "P70" or "T110" from Serial and writes that raw pulse
 * value directly to the named servo, to find its true stop point without
 * reflashing. Only intended for manual testing with the app not actively
 * tracking -- a live UDP packet will overwrite the test value on its next
 * update, since updateTripod() runs independently in the loop above.
 */
void handleCalibrationInput() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() < 2) return;

  char axis = line.charAt(0);
  int value = line.substring(1).toInt();
  value = constrain(value, 0, 180);

  if (axis == 'P' || axis == 'p') {
    panServo.write(value);
    Serial.printf("Calibration: pan servo set to %d\n", value);
  } else if (axis == 'T' || axis == 't') {
    tiltServo.write(value);
    Serial.printf("Calibration: tilt servo set to %d\n", value);
  } else {
    Serial.println("Calibration: unrecognized command, use \"P<0-180>\" or \"T<0-180>\"");
  }
}

/**
 * PID speed offset for one axis. Returns 0 (-> NEUTRAL, i.e. stop) whenever
 * the error is within DEADZONE, and resets that axis's integral term at the
 * same time so it doesn't creep once centered.
 */
float computeAxisPID(float error, float &integral, float &lastError, float dt) {
  if (fabs(error) <= DEADZONE) {
    integral = 0.0f;
    lastError = error;
    return 0.0f;
  }

  float derivative = 0.0f;
  if (dt > 0.0f) {
    integral = constrain(integral + error * dt, -MAX_INTEGRAL, MAX_INTEGRAL);
    derivative = (error - lastError) / dt;
  }
  lastError = error;

  float output = KP * error + KI * integral + KD * derivative;
  return constrain(output, -MAX_SPEED_OFFSET, MAX_SPEED_OFFSET);
}

/**
 * PID speed control from normalized error in [-1, 1] (fraction of
 * half-frame from centre). Unlike a positional servo, there is no target
 * angle to move to and hold -- each call computes and writes the current
 * commanded speed directly, and an in-deadzone error means "stop," not
 * "stay put." Positive errX means the subject is right of centre; positive
 * errY means the subject is below centre.
 */
void updateTripod(float errX, float errY) {
  unsigned long now = millis();
  float dt = (lastUpdateMillis == 0) ? 0.0f : (now - lastUpdateMillis) / 1000.0f;
  lastUpdateMillis = now;

  float panOffset = computeAxisPID(errX, panIntegral, lastErrX, dt);
  float tiltOffset = computeAxisPID(errY, tiltIntegral, lastErrY, dt);

  int panSpeed = PAN_NEUTRAL - (int)panOffset;   // Flip sign if pan direction is inverted for your mounting.
  int tiltSpeed = TILT_NEUTRAL + (int)tiltOffset; // Flip sign if tilt direction is inverted for your mounting.

  panServo.write(panSpeed);
  tiltServo.write(tiltSpeed);

  Serial.printf("Servo speed -> pan: %d, tilt: %d\n", panSpeed, tiltSpeed);
}

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
 *   servos -- these have no absolute position, so the pulse we send commands
 *   a speed/direction rather than an angle to move to and hold. A neutral
 *   pulse (1500 us, trimmable per-axis) means stop. Control is done in raw
 *   microseconds via writeMicroseconds(), NOT the 0-180 write() overload:
 *   write(90) against our attach range emits 1450 us, not 1500, and that
 *   50 us bias alone makes a well-trimmed servo creep one way forever. See
 *   PAN_NEUTRAL_US / TILT_NEUTRAL_US / MAX_SPEED_OFFSET_US below.
 * - Loss-of-signal failsafe: stops both motors if no UDP packet has been
 *   received recently, so a dropped connection or closed app can't leave a
 *   continuous-rotation servo spinning indefinitely.
 * - Battery monitoring via an INA219 high-side current/voltage sensor wired
 *   BEFORE the 5 V regulator, so it sees the raw 2S Li-ion pack. A blended
 *   coulomb-count + open-circuit-voltage fuel gauge turns those readings into a
 *   state-of-charge percentage, sent back to the app as a UDP reply so the
 *   camera screen can show remaining battery. See the "Battery" config block.
 *
 * No code changes are needed to benefit from WiFi 6 -- the C6 negotiates
 * 802.11ax automatically against a WiFi 6 access point via the same
 * WiFi.begin() call used for 802.11n.
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <math.h>
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
// PAN_NEUTRAL_US / TILT_NEUTRAL_US are the pulse widths (microseconds) that
// stop each continuous-rotation servo. 1500 us is the true center for
// virtually every hobby servo and is the correct starting point -- but cheap
// continuous-rotation servos (e.g. FS90R-type) are often trimmed a few us
// off, and pan/tilt are two separate physical units, so each may still need
// a small per-axis trim. If a servo only ever spins one direction and never
// reverses no matter which way the error points, its neutral is off: use the
// serial calibration mode below (send e.g. "P1495" or "T1505" over Serial
// Monitor) to find each servo's true stop point, then set these to match.
//
// NOTE: everything downstream now works in microseconds and calls
// writeMicroseconds(). The older code used write(90) as "neutral", which,
// against the 500-2400 us attach range, actually emitted 1450 us -- a
// standing ~50 us bias that made even a well-centered servo drift one way.
const int PAN_NEUTRAL_US  = 1500;
const int TILT_NEUTRAL_US = 1500;

// --- PID gains: model-based, not hand-tuned ---
//
// Plant: servo pulse offset u (us) -> camera angular rate w = Ks*u, and the
// app's error e = angle / half-FOV, so  de/dt = -(Ks/half_FOV) * u.  That is a
// PURE INTEGRATOR, E(s)/U(s) = -Kg/s with Kg = Ks/half_FOV. For an integrator
// the loop delay L is the only thing that caps the gain (phase margin):
//
//   w_c = KP * Kg           (crossover freq = loop gain)
//   PM  = 90deg - w_c*L*(180/pi) - (~10deg from the integral term)
//   KP  = w_c / Kg = w_c * half_FOV / Ks
//   KI  = (w_c / 6) * KP    (integral zero one hexave below crossover)
//   KD  = 0                 (integrator plant needs no D; vision jitter + the
//                            app-side Kalman already supply the lead term)
//
// Inputs (ESTIMATES -- see firmware/README.md "Tuning the PID" for how to
// measure each on your rig, then recompute):
//   Ks       ~= 1.8 deg/s per us   (FS90R-class @ 5V, ~100 RPM no-load, derated
//                                   ~40% for head load; plausible 1.2 - 3.0)
//   half_FOV ~= 26 deg pan, 33 deg tilt   (phone main camera, portrait)
//   L        ~= 0.15 s   (~0.10 s vision+net pipeline + T/2 at 30 Hz + ~50 ms
//                         servo internal speed-loop lag)
//   PM target 50 deg  ->  w_c ~= 3.5 rad/s  (~0.56 Hz BW, ~0.8 s settle)
//
// -> KP_pan ~= 50, KP_tilt ~= 64;  KI ~= (w_c/6)*KP ~= 30..37.  One shared
// pair covers both axes within the modeling error.
//
// The defaults below sit a bit above the PM-50 point (w_c ~= 5-6 rad/s): the
// L ~= 0.15 s estimate is deliberately pessimistic (it double-counts delay the
// app-side Kalman look-ahead already cancels), so the conservative gains felt
// sluggish on the bench -- bumped again (85->110 / 50->65, same KI/KP ratio)
// for the same reason, still UNVERIFIED on hardware past the first bump.
// KP/KI/KD/MAX_SPEED_OFFSET_US are RUNTIME-TUNABLE over Serial -- send
// "KP120", "KI70", "KD0", "MS160" (see handleCalibrationInput) to dial in
// responsiveness without reflashing, then copy the values you like back
// here. Raise KP until the camera just starts to overshoot/hunt, then back
// off ~30%. If tilt lags pan (gravity), it needs the higher end. The
// SATURATION_* guard below catches a fully diverging loop (wrong sign, or
// the servo can't keep up at all) but NOT gain-induced oscillation around a
// correctly-centred target -- watch for hunting/buzzing on first power-up
// and back KP off if you see it.
const float DEADZONE = 0.03f;             // Normalized error for full stop. ~2.5 sigma of the app's
                                          // Kalman-filtered position jitter (~0.013 normalized).
float KP = 110.0f;                        // Proportional gain: pulse offset (us) per unit of normalized error
float KI = 50.0f;                         // Integral gain (us per unit-error-second): cancels steady bias/creep. Set to 0 to disable.
float KD = 5.0f;                          // Derivative gain: kept at 0 by design (see model above). Only raise, in
                                          // small steps, if overshoot/oscillation remains after KP and KI are set --
                                          // if it makes things jerkier that is noise amplification; back it off.
float MAX_SPEED_OFFSET_US = 220.0f;       // Max offset from NEUTRAL (us) ~= 250 deg/s camera slew. The P term alone
                                          // maxes at KP*1 = 110 us, so control stays linear across the whole frame
                                          // and this clamp only bounds integral windup + fast-subject transients.
const float MAX_INTEGRAL = 1.0f;          // Anti-windup clamp on the accumulated integral (unit-error-seconds): ~50 us
                                          // of bias authority at KI above, enough for neutral mistrim + tilt gravity.

// Feedback direction per axis. The pulse written is
//   NEUTRAL_US + DIR * offset
// so DIR sets which way each servo turns for a given error. This is the ONE
// thing that depends on your servo wiring and how the pan/tilt head is
// assembled, and getting it wrong makes the loop DIVERGE: the servo drives the
// subject further off-centre until it spins at full speed. Bring-up procedure:
//   1. Serial-send "P1560" (neutral + 60). Note which way pan turns.
//   2. Aim the camera at a subject to the RIGHT of centre. Pan must turn the
//      camera RIGHT (toward the subject) to track it.
//   3. If "P1560" turned the camera right, PAN_DIR = +1; if left, PAN_DIR = -1.
//   4. Same for tilt with "T1560": a subject BELOW centre needs the camera to
//      tilt DOWN.
// The SATURATION_* guard below will stop the motors (not spin forever) if this
// is still wrong, but fix the sign -- don't rely on the guard.
const int PAN_DIR  = -1;
const int TILT_DIR = +1;

// Divergence guard. A correctly-wired loop pulls |error| back toward 0. If
// |error| instead stays pinned at the frame edge for this long, the loop is
// diverging (wrong *_DIR, or the servo physically can't keep up) -- stop the
// motors and say so, rather than spin at full speed until the app is closed.
const float SATURATION_LEVEL = 0.97f;
const unsigned long SATURATION_TIMEOUT_MS = 1500;

// If no UDP packet arrives within this long, stop both motors. Without this,
// losing WiFi or closing the app would leave a continuous-rotation servo
// spinning at its last commanded speed forever.
const unsigned long SIGNAL_TIMEOUT_MS = 500;

// --- Battery monitor (INA219) ---
//
// Wiring: the INA219 sits on the HIGH SIDE of the pack, BEFORE the 5 V
// regulator, so it measures the full 2S Li-ion voltage (~6.0-8.4 V) and the
// total current the whole rig draws (regulator + servos + ESP32):
//
//   pack + ---> [INA219 IN+]--shunt--[INA219 IN-] ---> regulator IN / Vin
//   pack - ---> common ground (regulator GND, ESP32 GND, servo GND)
//   INA219 VCC ---> ESP32 3V3      INA219 GND ---> common ground
//   INA219 SDA ---> GPIO INA_SDA_PIN   INA219 SCL ---> GPIO INA_SCL_PIN
//
// I2C pins: pick from the ESP32-C6 "safe" list in the servo-pin comment above.
// GPIO6/7 are broken out on both DevKitC-1 and DevKitM-1 and are not strapping
// or USB pins. Default INA219 I2C address is 0x40 (both A0/A1 straps low).
const int INA_SDA_PIN = 6;
const int INA_SCL_PIN = 7;
const uint8_t INA_I2C_ADDR = 0x40;   // INA219_ADDRESS is a macro in the library

// Pack spec (from the cells' datasheet): 2S Li-ion, 2 x 3.7 V nominal, 2600 mAh,
// 9.62 Wh. Full charge is 4.2 V/cell (8.4 V pack); empty is 3.0 V/cell (6.0 V).
const int   BATT_CELLS        = 2;
const float BATT_CAPACITY_MAH = 2600.0f;
const float BATT_ENERGY_WH    = 9.62f;

// Whole-pack internal resistance estimate, used to turn a loaded terminal
// voltage back into open-circuit voltage (OCV) for the voltage-based SoC:
//   OCV ~= V_terminal + I_discharge * R.
// ~75 mOhm/cell is typical for a mid-grade 18650 in a 2S pack; measure yours
// (dV across a known current step) and refine if the gauge drifts under load.
const float BATT_IR_OHMS = 0.15f;

// INA219 current sign. The library reports current POSITIVE when it flows from
// IN+ to IN-. With IN+ on the pack + terminal and IN- toward the regulator,
// discharge is positive -> leave this +1. If the app ever shows a negative
// current while the rig is clearly running, the shunt is reversed: set -1.
const int INA_CURRENT_SIGN = 1;

// Below this |current| the terminal voltage is close enough to OCV to trust the
// voltage-based SoC and pull the coulomb counter toward it (drift correction).
const float BATT_REST_CURRENT_MA = 150.0f;

const unsigned long BATT_SAMPLE_INTERVAL_MS    = 250;   // fuel-gauge update rate
const unsigned long BATT_TELEMETRY_INTERVAL_MS = 2000;  // how often we tell the app
const unsigned long BATT_PRINT_INTERVAL_MS     = 5000;  // Serial debug print rate

// --- Global Objects ---
WiFiUDP udp;
Servo panServo;
Servo tiltServo;

Adafruit_INA219 ina219(INA_I2C_ADDR);
bool inaPresent = false;

// Fuel-gauge state. battChargeMah is the estimated charge left in the pack;
// battSoC is that as a 0..1 fraction of BATT_CAPACITY_MAH.
float battChargeMah = BATT_CAPACITY_MAH;
float battSoC       = 1.0f;
float battPackV     = 0.0f;   // last pack terminal voltage (V)
float battCurrentMa = 0.0f;   // last current, + = discharge
unsigned long lastBattSampleMs    = 0;
unsigned long lastBattTelemetryMs = 0;
unsigned long lastBattPrintMs     = 0;

// Where to send battery telemetry: learned from the source address of the
// app's incoming EX/EY packets, so replies go back on the same UDP flow.
IPAddress appIP;
uint16_t  appPort = 0;
bool      haveApp = false;

unsigned long lastPacketMillis = 0;
bool motorsStopped = true;

// Divergence-guard state (see SATURATION_* above)
unsigned long panSaturatedSince = 0;
unsigned long tiltSaturatedSince = 0;
bool controlDiverged = false;

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

  // Onboard LED on, solid, as a power/alive indicator -- do this first so a
  // board that hangs later still shows it powered up. On the ESP32-C6 DevKit
  // this is the addressable RGB LED on GPIO8; the arduino-esp32 core maps
  // neopixelWrite()/digitalWrite(RGB_BUILTIN, ...) onto its NeoPixel protocol.
#if defined(RGB_BUILTIN)
  neopixelWrite(RGB_BUILTIN, 0, 40, 0);   // dim green
#elif defined(LED_BUILTIN)
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
#endif

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

  panServo.writeMicroseconds(PAN_NEUTRAL_US);
  tiltServo.writeMicroseconds(TILT_NEUTRAL_US);

  Serial.println("Serial commands: P<us>/T<us> = raw servo pulse (find true stop point);");
  Serial.println("KP<v>/KI<v>/KD<v>/MS<v> = live PID tuning; ? = print current values.");
  Serial.println("Note: live UDP tracking overwrites a P/T calibration write on the next packet.");
  printControlValues();

  // Battery monitor. Non-fatal if absent -- the tripod still tracks, it just
  // won't report battery.
  Wire.begin(INA_SDA_PIN, INA_SCL_PIN);
  if (ina219.begin()) {
    inaPresent = true;
    // Default calibration is 32 V / 2 A (higher current resolution than the
    // 32 V / 3.2 A max range). Servo-stall spikes can briefly exceed 2 A and
    // will read clipped, but they're too short to matter to the coulomb count.
    Serial.println("INA219 battery monitor online");
    initBatteryGauge();
  } else {
    Serial.println("INA219 NOT found -- battery telemetry disabled (check SDA/SCL wiring and addr 0x40)");
  }

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

    // Remember who is talking to us so battery telemetry can go back on the
    // same flow (the app receives on the socket it sends from).
    appIP = udp.remoteIP();
    appPort = udp.remotePort();
    haveApp = true;
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
    panServo.writeMicroseconds(PAN_NEUTRAL_US);
    tiltServo.writeMicroseconds(TILT_NEUTRAL_US);
    motorsStopped = true;
    panSaturatedSince = 0;
    tiltSaturatedSince = 0;
    controlDiverged = false;
    haveSeq = false;
    Serial.println("Signal lost -- motors stopped");
  }

  // Battery fuel gauge + telemetry. Both self-throttle, so calling them every
  // loop is cheap.
  updateBatteryGauge();
  sendBatteryTelemetry();
}

/**
 * Reads a line like "P1495" or "T1505" from Serial and writes that raw pulse
 * width (microseconds) directly to the named servo, to find its true stop
 * point without reflashing. Only intended for manual testing with the app not
 * actively tracking -- a live UDP packet will overwrite the test value on its
 * next update, since updateTripod() runs independently in the loop above.
 *
 * Also accepts live PID tuning (applies immediately, survives until reboot):
 *   KP<v> KI<v> KD<v>   PID gains
 *   MS<v>              MAX_SPEED_OFFSET_US (clamped 10..400)
 *   ?                  print current values
 */
void handleCalibrationInput() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() < 1) return;

  String cmd = line;
  cmd.toUpperCase();

  if (cmd == "?") {
    printControlValues();
    printBatteryStatus();
    return;
  }
  if (cmd.startsWith("KP")) { KP = cmd.substring(2).toFloat(); Serial.printf("KP = %.2f\n", KP); return; }
  if (cmd.startsWith("KI")) { KI = cmd.substring(2).toFloat(); panIntegral = tiltIntegral = 0.0f; Serial.printf("KI = %.2f (integrators reset)\n", KI); return; }
  if (cmd.startsWith("KD")) { KD = cmd.substring(2).toFloat(); Serial.printf("KD = %.2f\n", KD); return; }
  if (cmd.startsWith("MS")) { MAX_SPEED_OFFSET_US = constrain(cmd.substring(2).toFloat(), 10.0f, 400.0f); Serial.printf("MAX_SPEED_OFFSET_US = %.0f\n", MAX_SPEED_OFFSET_US); return; }

  if (line.length() < 2) return;
  char axis = line.charAt(0);
  int value = constrain(line.substring(1).toInt(), 1000, 2000);

  if (axis == 'P' || axis == 'p') {
    panServo.writeMicroseconds(value);
    Serial.printf("Calibration: pan servo set to %d us\n", value);
  } else if (axis == 'T' || axis == 't') {
    tiltServo.writeMicroseconds(value);
    Serial.printf("Calibration: tilt servo set to %d us\n", value);
  } else {
    Serial.println("Unrecognized. Use P<us>/T<us>, KP<v>/KI<v>/KD<v>/MS<v>, or ?");
  }
}

void printControlValues() {
  Serial.printf("KP=%.2f  KI=%.2f  KD=%.2f  MAX_SPEED_OFFSET_US=%.0f  DEADZONE=%.3f\n",
                KP, KI, KD, MAX_SPEED_OFFSET_US, DEADZONE);
}

void printBatteryStatus() {
  if (!inaPresent) {
    Serial.println("Battery: INA219 not present");
    return;
  }
  Serial.printf("Battery: %.0f%%  %.2f V  %.0f mA  %.2f/%.2f Wh  (%.0f/%.0f mAh)\n",
                battSoC * 100.0f, battPackV, battCurrentMa,
                battSoC * BATT_ENERGY_WH, BATT_ENERGY_WH,
                battChargeMah, BATT_CAPACITY_MAH);
}

/**
 * Open-circuit-voltage -> state-of-charge for ONE Li-ion cell, as a
 * piecewise-linear curve through resting-voltage / SoC points typical of a
 * generic 18650. The pack curve is this evaluated at OCV / BATT_CELLS. The
 * nominal 3.7 V/cell sits at ~50%, matching the cell datasheet.
 */
float cellOcvToSoC(float v) {
  static const float pts[][2] = {
    {3.00f, 0.00f}, {3.20f, 0.03f}, {3.30f, 0.07f}, {3.40f, 0.12f},
    {3.50f, 0.20f}, {3.60f, 0.35f}, {3.70f, 0.50f}, {3.80f, 0.62f},
    {3.90f, 0.74f}, {4.00f, 0.85f}, {4.10f, 0.94f}, {4.20f, 1.00f}
  };
  const int n = sizeof(pts) / sizeof(pts[0]);
  if (v <= pts[0][0]) return 0.0f;
  if (v >= pts[n - 1][0]) return 1.0f;
  for (int i = 1; i < n; i++) {
    if (v < pts[i][0]) {
      float t = (v - pts[i - 1][0]) / (pts[i][0] - pts[i - 1][0]);
      return pts[i - 1][1] + t * (pts[i][1] - pts[i - 1][1]);
    }
  }
  return 1.0f;
}

/** One INA219 read -> pack terminal voltage (V) and signed current (mA). */
void readBattery(float &packV, float &currentMa) {
  float shuntmV = ina219.getShuntVoltage_mV();
  float busV    = ina219.getBusVoltage_V();   // measured at IN- (regulator side)
  currentMa = ina219.getCurrent_mA() * INA_CURRENT_SIGN;
  // Bus voltage is the load side of the shunt; add the shunt drop back to get
  // the actual pack terminal voltage.
  packV = busV + shuntmV / 1000.0f;
}

/**
 * Seed the fuel gauge from the pack's open-circuit voltage at boot. This is the
 * only absolute reference the coulomb counter gets until the pack next rests, so
 * if you power on under heavy load the initial % can be a few points low until
 * the rest-correction in updateBatteryGauge() catches up.
 */
void initBatteryGauge() {
  readBattery(battPackV, battCurrentMa);
  float ocv = battPackV + (battCurrentMa / 1000.0f) * BATT_IR_OHMS;
  battSoC = cellOcvToSoC(ocv / BATT_CELLS);
  battChargeMah = battSoC * BATT_CAPACITY_MAH;
  lastBattSampleMs = millis();
  Serial.printf("Battery init: pack %.2f V, OCV %.2f V -> %.0f%%\n",
                battPackV, ocv, battSoC * 100.0f);
  updateBatteryLed();
}

/**
 * Blended fuel gauge, run every BATT_SAMPLE_INTERVAL_MS:
 *   1. Coulomb count -- integrate current out of battChargeMah. Precise
 *      short-term, but drifts over hours (sensor offset, unmodelled loads).
 *   2. Voltage estimate -- OCV (IR-compensated) through the cell curve.
 *      Absolute but noisy under load and flat in the mid-SoC plateau.
 *   3. Fuse -- near rest, trust the voltage and pull the counter toward it.
 *      Under load, only let the voltage estimate DRAG THE COUNTER DOWN (never
 *      up), so a stale counter can't mask a nearly-flat pack, but voltage sag
 *      under a servo surge can't make the gauge jump around either.
 */
void updateBatteryGauge() {
  if (!inaPresent) return;
  unsigned long now = millis();
  if (now - lastBattSampleMs < BATT_SAMPLE_INTERVAL_MS) return;
  float dtHours = (now - lastBattSampleMs) / 3600000.0f;
  lastBattSampleMs = now;

  readBattery(battPackV, battCurrentMa);

  // 1. Coulomb counting.
  battChargeMah -= battCurrentMa * dtHours;
  battChargeMah = constrain(battChargeMah, 0.0f, BATT_CAPACITY_MAH);

  // 2. Voltage-based estimate.
  float ocv    = battPackV + (battCurrentMa / 1000.0f) * BATT_IR_OHMS;
  float mahV   = cellOcvToSoC(ocv / BATT_CELLS) * BATT_CAPACITY_MAH;

  // 3. Fuse.
  if (fabs(battCurrentMa) < BATT_REST_CURRENT_MA) {
    battChargeMah += 0.02f * (mahV - battChargeMah);         // converge over ~30 s at rest
  } else if (mahV < battChargeMah) {
    battChargeMah += 0.005f * (mahV - battChargeMah);        // slow one-sided pull-down under load
  }
  battChargeMah = constrain(battChargeMah, 0.0f, BATT_CAPACITY_MAH);
  battSoC = battChargeMah / BATT_CAPACITY_MAH;

  updateBatteryLed();

  if (now - lastBattPrintMs >= BATT_PRINT_INTERVAL_MS) {
    lastBattPrintMs = now;
    printBatteryStatus();
  }
}

/**
 * Encode state of charge on the onboard RGB LED, kept dim so it stays a status
 * light and not a torch: green > 50%, amber 20-50%, steady red 10-20%, blinking
 * red < 10%. Falls back to the plain LED_BUILTIN (on = not critical) on boards
 * with no addressable LED.
 */
void updateBatteryLed() {
#if defined(RGB_BUILTIN)
  uint8_t r = 0, g = 0, b = 0;
  if (battSoC > 0.50f)      { g = 40; }
  else if (battSoC > 0.20f) { r = 40; g = 25; }
  else if (battSoC > 0.10f) { r = 40; }
  else                      { r = ((millis() / 500) % 2 == 0) ? 60 : 0; }
  neopixelWrite(RGB_BUILTIN, r, g, b);
#elif defined(LED_BUILTIN)
  digitalWrite(LED_BUILTIN, battSoC > 0.10f ? HIGH : (((millis() / 500) % 2 == 0) ? HIGH : LOW));
#endif
}

/**
 * Send the current battery state back to the app as a UDP reply on the same
 * flow the EX/EY packets arrive on. Format mirrors the app->firmware protocol
 * (KEY:value, comma-separated):
 *   BATT:<percent 0-100>,MV:<pack millivolts>,MA:<current mA, + = discharge>,WH:<Wh remaining>
 */
void sendBatteryTelemetry() {
  if (!inaPresent || !haveApp) return;
  unsigned long now = millis();
  if (now - lastBattTelemetryMs < BATT_TELEMETRY_INTERVAL_MS) return;
  lastBattTelemetryMs = now;

  int pct = (int) lroundf(battSoC * 100.0f);
  pct = constrain(pct, 0, 100);
  char msg[96];
  int len = snprintf(msg, sizeof(msg), "BATT:%d,MV:%d,MA:%d,WH:%.2f",
                     pct,
                     (int) lroundf(battPackV * 1000.0f),
                     (int) lroundf(battCurrentMa),
                     battSoC * BATT_ENERGY_WH);
  if (len <= 0) return;
  udp.beginPacket(appIP, appPort);
  udp.write((const uint8_t *) msg, len);
  udp.endPacket();
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
  return constrain(output, -MAX_SPEED_OFFSET_US, MAX_SPEED_OFFSET_US);
}

/**
 * Returns true once |error| has sat at/above SATURATION_LEVEL continuously for
 * longer than SATURATION_TIMEOUT_MS -- the sign of a diverging loop. Clears the
 * timer as soon as the error comes back inside that band.
 */
bool axisDiverging(float error, unsigned long &saturatedSince) {
  unsigned long now = millis();
  if (fabs(error) >= SATURATION_LEVEL) {
    if (saturatedSince == 0) saturatedSince = now;
    return (now - saturatedSince) > SATURATION_TIMEOUT_MS;
  }
  saturatedSince = 0;
  return false;
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
  // Divergence guard: if either axis is pinned at the frame edge, the loop is
  // running away (usually a wrong PAN_DIR/TILT_DIR). Stop both, dump the
  // integrators, and say so once -- do not keep commanding speed.
  if (axisDiverging(errX, panSaturatedSince) || axisDiverging(errY, tiltSaturatedSince)) {
    panServo.writeMicroseconds(PAN_NEUTRAL_US);
    tiltServo.writeMicroseconds(TILT_NEUTRAL_US);
    panIntegral = 0.0f;
    tiltIntegral = 0.0f;
    if (!controlDiverged) {
      Serial.println("Control DIVERGING -- |error| pinned at the frame edge. Motors stopped.");
      Serial.println("Fix: check PAN_DIR / TILT_DIR sign (see comment at top), or the servo");
      Serial.println("cannot keep up with the subject. Re-centre the subject to resume.");
      controlDiverged = true;
    }
    return;
  }
  controlDiverged = false;

  unsigned long now = millis();
  float dt = (lastUpdateMillis == 0) ? 0.0f : (now - lastUpdateMillis) / 1000.0f;
  lastUpdateMillis = now;

  float panOffset = computeAxisPID(errX, panIntegral, lastErrX, dt);
  float tiltOffset = computeAxisPID(errY, tiltIntegral, lastErrY, dt);

  int panPulse = PAN_NEUTRAL_US + PAN_DIR * (int)panOffset;
  int tiltPulse = TILT_NEUTRAL_US + TILT_DIR * (int)tiltOffset;

  panServo.writeMicroseconds(panPulse);
  tiltServo.writeMicroseconds(tiltPulse);

  Serial.printf("Servo pulse (us) -> pan: %d, tilt: %d\n", panPulse, tiltPulse);
}

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
 * - Manual joystick drive (JOY:x,y,SEQ:value): open-loop speed command from
 *   the app's on-screen joystick, bypassing the PID. See updateManual().
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
#include <esp_system.h>

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

// --- PID gains: measured on the rig, 2026-10-06 ---
//
// The first gains (KP 110, KI 50, KD 5) came from a model that assumed a linear
// servo (camera rate = Ks * pulse offset) and a loop delay of 0.15 s. The
// pendulum calibration of 2026-10-06 (Tests/calibration.m) measured neither to
// be true, with a still ball and a fixed pulse (KP 0, DEADZONE 0, so the output
// is +-MIN_OFFSET_US and the camera rocks across the ball):
//
//   * Dead band: the pan servo does not move for pulses within ~51-52 us of
//     neutral (52 us: 0.06 / 0.17 half-frames/s in the two directions; the
//     tilt servo not at all).
//   * Steep beyond it: 60 us already gives ~1.2 half-frames/s (about
//     0.14 half-frames/s per us), so the whole useful speed range is ~8 us wide.
//   * Loop delay 0.42 s from ball crossing centre to camera reversing: 0.14 s
//     in the app's Kalman filter at its old settings (R 100, sigma_a 150),
//     ~0.11 s ramping the pulse across the dead band at MAX_SLEW_US_PER_S, and
//     ~0.17 s camera + network + servo.
//
// So a plain PID either did nothing (KP*error inside the dead band) or, once
// KP was high enough to leave it, swept the whole speed range on a small error
// and hunted. The loop that works is: MIN_OFFSET_US at the edge of the dead
// band, a low KP for the narrow range beyond it, the filter lag removed in the
// app (R 5, sigma_a 300) and a 0.20 s prediction horizon for the rest of the
// delay. Three pendulum runs at these values (R14-R16) gave an offset RMS of
// 0.17 with the ball in the central third of the frame 96% of the time,
// against 0.24 and 80% for a camera that did not move.
//
// KI and KD are 0: neither was needed to meet the targets on the pendulum, and
// they have not been tuned. All of these are RUNTIME-TUNABLE over Serial
// ("KP18", "MO52", ... see handleCalibrationInput) and from the app's
// Experiment tab, so re-tune on the running rig and copy the values back here.
// The SATURATION_* guard below catches a fully diverging loop (wrong sign, or
// the servo can't keep up at all) but NOT gain-induced oscillation around a
// correctly-centred target -- watch for hunting on first power-up.
float DEADZONE = 0.03f;                   // Normalized error for full stop. ~2.5 sigma of the app's
                                          // Kalman-filtered position jitter (~0.013 normalized).
                                          // RUNTIME-TUNABLE like KP/KI/KD/MS -- see handleCalibrationInput
                                          // (Serial "DZ<v>") and handleConfigPacket (UDP "CFG:...").
float MIN_OFFSET_US = 52.0f;              // Servo dead-band compensation (us), set to the measured edge of the
                                          // dead band (see above). Without this, any
                                          // KP*error below that does nothing, so low gains never move the camera
                                          // and the first gain that does is already high enough to hunt. Outside
                                          // DEADZONE this is added to the PID output in the direction of the
                                          // error, so the command starts just inside the dead band and KP only has
                                          // to supply the rest. Do not go ABOVE the dead band: the servo could
                                          // then never command a slow speed and would chatter around the deadzone.
                                          // RUNTIME-TUNABLE: Serial "MO<v>", UDP "CFG:...,MO:<v>". 0 disables.
float KP = 18.0f;                         // Proportional gain: pulse offset (us) per unit of normalized error
float KI = 0.0f;                          // Integral gain (us per unit-error-second): cancels steady bias/creep. 0 = disabled (not tuned).
float KD = 0.0f;                          // Derivative gain: 0 (not tuned; the prediction horizon supplies the lead). Only raise, in
                                          // small steps, if overshoot/oscillation remains after KP and KI are set --
                                          // if it makes things jerkier that is noise amplification; back it off.
float MAX_SPEED_OFFSET_US = 300.0f;       // Max offset from NEUTRAL (us). MIN_OFFSET_US + KP*1 = 70 us at the
                                          // defaults, so this clamp only bounds integral windup if KI is raised.
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
int TILT_DIR = -1;         // History: -1 on 2026-10-01, +1 on 2026-10-06 (pendulum calibration, where
                           // -1 drove the camera AWAY from the ball), back to -1 on 2026-10-08 after
                           // auto tracking tilted away from the subject on both cameras. The sign
                           // depends on which way round the tilt servo sits in the mount, so re-check
                           // it (step 4 above) whenever the tilt stage is reassembled.
                           // RUNTIME-TUNABLE so a wrong guess needs no reflash: Serial
                           // "TD1" / "TD-1", UDP "CFG:...,TD:<1|-1>" (not kept across a reboot).

// Manual joystick drive ("JOY:" packets, see updateManual). Full stick commands
// MIN_OFFSET_US + JOY_SPEED_RANGE_US from neutral: the same 52-70 us span the
// tracking loop uses at the default gains, which is where the measured servo
// goes from just moving to ~2 half-frames/s. Inside JOY_DEADZONE the axis stops.
const float JOY_SPEED_RANGE_US = 18.0f;
const float JOY_DEADZONE = 0.08f;

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

// --- Power-safety limits ---
//
// The servos share the 5 V regulator with the ESP32. A continuous-rotation
// servo slammed from full speed one way to full speed the other is close to a
// stall and pulls a large current spike; two of them doing it at 30 Hz (what
// very high KP/KD produce on noisy vision error) sags the rail enough to
// brown out the C6 or drop its WiFi. Two layers keep that from happening:
//
// 1. Slew limit: the commanded pulse may move at most this many microseconds
//    per second, so speed changes and reversals ramp instead of stepping.
//    1000 us/s -> neutral to a typical 220 us offset in ~0.2 s, a full
//    +400 -> -400 reversal in ~0.8 s. Lower it if brownouts persist; raise it
//    if tracking feels laggy on a solid power supply. Failsafe/divergence
//    stops bypass it (they must stop immediately).
float MAX_SLEW_US_PER_S = 1000.0f;   // RUNTIME-TUNABLE: Serial "SL<v>", UDP "CFG:...,SL:<v>" (500..20000).
                                     // Measured 2026-10-06: at 1000 us/s a reversal across the servo dead
                                     // band (+52 to -52 us) takes ~0.1 s, a quarter of the total loop delay.
// Cap on the dt used for the slew step, so a long gap between packets can't
// be spent as one big jump.
const float MAX_SLEW_DT_S = 0.1f;
//
// 2. Gain limits: hard caps on the runtime-tunable gains, applied to both
//    Serial and UDP (Experiment tab) updates, well above the model-based
//    defaults but below where the loop just bang-bangs between extremes.
//    Keep the app's Experiment-tab slider ranges in sync with these.
const float KP_LIMIT = 200.0f;
const float KI_LIMIT = 100.0f;
const float KD_LIMIT = 20.0f;
const float MS_MIN   = 10.0f;
const float MS_LIMIT = 400.0f;
const float DZ_LIMIT = 0.5f;
const float MO_LIMIT = 80.0f;
const float SL_MIN   = 500.0f;
const float SL_LIMIT = 20000.0f;

// --- WiFi resilience ---
// If still disconnected this long after a drop, force a reconnect in case the
// core's auto-reconnect has stalled.
const unsigned long WIFI_RECONNECT_INTERVAL_MS = 10000;

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

// Slew-limited commanded offset from neutral (us, signed, already includes
// *_DIR) -- what is actually being written to each servo right now.
float panCmdOffsetUs = 0.0f;
float tiltCmdOffsetUs = 0.0f;

// WiFi link state (see onWiFiEvent)
volatile bool wifiUp = false;
unsigned long wifiDownSinceMs = 0;
unsigned long lastWifiReconnectMs = 0;

uint32_t lastSeq = 0;
bool haveSeq = false;

const int PACKET_BUFFER_SIZE = 255;
char packetBuffer[PACKET_BUFFER_SIZE];

void setup() {
  Serial.begin(115200);

  // Why did we (re)boot? A BROWNOUT here means the supply sagged -- almost
  // always servo current spikes on the shared 5 V rail.
  esp_reset_reason_t resetReason = esp_reset_reason();
  Serial.printf("\nReset reason: %s\n", resetReasonName(resetReason));

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

  if (resetReason == ESP_RST_BROWNOUT) {
    Serial.println("WARNING: last reset was a BROWNOUT -- the supply sagged (servo current");
    Serial.println("spikes on the shared 5 V rail?). Add bulk capacitance on the servo rail,");
    Serial.println("power the servos separately, or lower MAX_SLEW_US_PER_S / the gains.");
    signalBrownoutOnLed();
  }

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

  // Connect to WiFi. Modem power-save is disabled: on phone hotspots it adds
  // latency, drops packets, and can get the tripod kicked as "idle".
  WiFi.onEvent(onWiFiEvent);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
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
  maintainWiFi();

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

    // Remember who is talking to us so battery telemetry can go back on the
    // same flow (the app receives on the socket it sends from).
    appIP = udp.remoteIP();
    appPort = udp.remotePort();
    haveApp = true;

    // Discovery: the app broadcasts "DISCOVER" when it can't hear us, and we
    // answer it directly so it learns our IP. Handled per packet (not via
    // latestPacket) so a probe mixed in with tracking packets is never dropped.
    if (strncmp(packetBuffer, "DISCOVER", 8) == 0) {
      announceTripod();
      continue;
    }
    // Config is handled per packet for the same reason: with error or joystick
    // packets arriving at 30 Hz, a CFG that shared a drain with a later one was
    // silently dropped (and the app never got its reply).
    if (strncmp(packetBuffer, "CFG", 3) == 0) {
      handleConfigPacket(String(packetBuffer));
      continue;
    }

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

    if (payload.startsWith("JOY:")) {
      int comma = payload.indexOf(',');
      if (comma != -1) {
        float joyX = payload.substring(4, comma).toFloat();
        float joyY = payload.substring(comma + 1).toFloat();   // toFloat() stops at ",SEQ:"
        if (seqIndex != -1 && !acceptSeq((uint32_t) payload.substring(seqIndex + 5).toInt())) return;

        lastPacketMillis = millis();
        motorsStopped = false;
        updateManual(joyX, joyY);
      }
    } else if (exIndex != -1 && eyIndex != -1) {
      float errX = payload.substring(exIndex + 3, eyIndex).toFloat();
      float errY;

      if (seqIndex != -1) {
        errY = payload.substring(eyIndex + 4, seqIndex).toFloat();
        if (!acceptSeq((uint32_t) payload.substring(seqIndex + 5).toInt())) return;
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
    stopServosNow();
    motorsStopped = true;
    lastUpdateMillis = 0;   // next packet starts a fresh dt, not the whole outage
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
 *   DZ<v>              DEADZONE (clamped 0..0.5)
 *   MO<v>              MIN_OFFSET_US, servo dead-band compensation (clamped 0..80)
 *   TD<1|-1>           TILT_DIR, tilt servo direction
 *   SL<v>              MAX_SLEW_US_PER_S, pulse ramp limit (clamped 500..20000)
 *   ?                  print current values
 *
 * The same five gains (KP/KI/KD/MS/DZ) are also live-tunable over UDP from the
 * app's Experiment tab -- see handleConfigPacket() / sendConfigTelemetry() and
 * the "CFG" protocol in firmware/README.md.
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
  if (cmd.startsWith("KP")) { KP = constrain(cmd.substring(2).toFloat(), 0.0f, KP_LIMIT); Serial.printf("KP = %.2f (max %.0f)\n", KP, KP_LIMIT); return; }
  if (cmd.startsWith("KI")) { KI = constrain(cmd.substring(2).toFloat(), 0.0f, KI_LIMIT); panIntegral = tiltIntegral = 0.0f; Serial.printf("KI = %.2f (max %.0f, integrators reset)\n", KI, KI_LIMIT); return; }
  if (cmd.startsWith("KD")) { KD = constrain(cmd.substring(2).toFloat(), 0.0f, KD_LIMIT); Serial.printf("KD = %.2f (max %.0f)\n", KD, KD_LIMIT); return; }
  if (cmd.startsWith("MS")) { MAX_SPEED_OFFSET_US = constrain(cmd.substring(2).toFloat(), MS_MIN, MS_LIMIT); Serial.printf("MAX_SPEED_OFFSET_US = %.0f\n", MAX_SPEED_OFFSET_US); return; }
  if (cmd.startsWith("MO")) { MIN_OFFSET_US = constrain(cmd.substring(2).toFloat(), 0.0f, MO_LIMIT); Serial.printf("MIN_OFFSET_US = %.0f\n", MIN_OFFSET_US); return; }
  if (cmd.startsWith("SL")) { MAX_SLEW_US_PER_S = constrain(cmd.substring(2).toFloat(), SL_MIN, SL_LIMIT); Serial.printf("MAX_SLEW_US_PER_S = %.0f\n", MAX_SLEW_US_PER_S); return; }
  if (cmd.startsWith("TD")) { TILT_DIR = (cmd.substring(2).toFloat() < 0.0f) ? -1 : +1; Serial.printf("TILT_DIR = %d\n", TILT_DIR); return; }
  if (cmd.startsWith("DZ")) { DEADZONE = constrain(cmd.substring(2).toFloat(), 0.0f, DZ_LIMIT); Serial.printf("DEADZONE = %.3f\n", DEADZONE); return; }

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
  Serial.printf("KP=%.2f  KI=%.2f  KD=%.2f  MAX_SPEED_OFFSET_US=%.0f  DEADZONE=%.3f  MIN_OFFSET_US=%.0f\n",
                KP, KI, KD, MAX_SPEED_OFFSET_US, DEADZONE, MIN_OFFSET_US);
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
 * Finds "<key>:" in payload and parses the float that follows, up to the next
 * ',' or end of string. Returns `fallback` (leaves the caller's value alone)
 * if the key isn't present, so a CFG packet can update any subset of gains.
 */
float parseKV(const String &payload, const char *key, float fallback) {
  String needle = String(key) + ":";
  int idx = payload.indexOf(needle);
  if (idx == -1) return fallback;
  int start = idx + needle.length();
  int end = payload.indexOf(',', start);
  String valueStr = (end == -1) ? payload.substring(start) : payload.substring(start, end);
  return valueStr.toFloat();
}

/**
 * Handles a "CFG:..." (set) or "CFG?" (query) packet from the app's Experiment
 * tab -- see "CFG protocol" in firmware/README.md. A set packet carries a ':'
 * after the "CFG" prefix (e.g. "CFG:KP:110.00,KI:50.00,..."); a bare query has
 * none. Either way, always replies with the full current gain set so the app
 * stays in sync even after a Serial-side change.
 */
void handleConfigPacket(const String &payload) {
  if (payload.indexOf(':') != -1) {
    // Clamped to the power-safety limits; the CFG reply below reports the
    // values actually applied, so the app's sliders snap to them on Sync.
    KP = constrain(parseKV(payload, "KP", KP), 0.0f, KP_LIMIT);
    KI = constrain(parseKV(payload, "KI", KI), 0.0f, KI_LIMIT);
    KD = constrain(parseKV(payload, "KD", KD), 0.0f, KD_LIMIT);
    MAX_SPEED_OFFSET_US = constrain(parseKV(payload, "MS", MAX_SPEED_OFFSET_US), MS_MIN, MS_LIMIT);
    DEADZONE = constrain(parseKV(payload, "DZ", DEADZONE), 0.0f, DZ_LIMIT);
    MIN_OFFSET_US = constrain(parseKV(payload, "MO", MIN_OFFSET_US), 0.0f, MO_LIMIT);
    TILT_DIR = (parseKV(payload, "TD", TILT_DIR) < 0.0f) ? -1 : +1;
    MAX_SLEW_US_PER_S = constrain(parseKV(payload, "SL", MAX_SLEW_US_PER_S), SL_MIN, SL_LIMIT);
    panIntegral = 0.0f;
    tiltIntegral = 0.0f;
    Serial.print("Config updated via UDP: ");
    printControlValues();
  }
  sendConfigTelemetry();
}

/**
 * Replies with the current PID gains on the same UDP flow CFG packets arrive
 * on, mirroring sendBatteryTelemetry()'s appIP/appPort reuse -- but unlike
 * battery telemetry this is NOT rate-limited: every CFG/CFG? gets an
 * immediate reply so the Experiment tab's "Apply"/"Sync" feel instant.
 */
/**
 * Reply to an app's "DISCOVER" broadcast with "TRIPOD:<our IP>", sent straight
 * back to the asking socket. The app takes our address from the reply and
 * saves it, so a new DHCP lease on the hotspot doesn't need re-entering.
 */
void announceTripod() {
  char msg[48];
  int len = snprintf(msg, sizeof(msg), "TRIPOD:%s", WiFi.localIP().toString().c_str());
  if (len <= 0) return;
  udp.beginPacket(appIP, appPort);
  udp.write((const uint8_t *) msg, len);
  udp.endPacket();
  Serial.printf("Discovery: announced %s to %s:%u\n", msg, appIP.toString().c_str(), appPort);
}

void sendConfigTelemetry() {
  if (!haveApp) return;
  char msg[128];
  int len = snprintf(msg, sizeof(msg), "CFG:KP:%.2f,KI:%.2f,KD:%.2f,MS:%.1f,DZ:%.3f,MO:%.1f,TD:%d,SL:%.0f",
                     KP, KI, KD, MAX_SPEED_OFFSET_US, DEADZONE, MIN_OFFSET_US, TILT_DIR, MAX_SLEW_US_PER_S);
  if (len <= 0) return;
  udp.beginPacket(appIP, appPort);
  udp.write((const uint8_t *) msg, len);
  udp.endPacket();
}

/**
 * Sequence check shared by the EX/EY and JOY packets (one counter in the app).
 * Returns false, after logging, for a packet older than the last one acted on.
 */
bool acceptSeq(uint32_t seq) {
  if (haveSeq && seq < lastSeq) {
    Serial.println("Dropped out-of-order packet");
    return false;
  }
  lastSeq = seq;
  haveSeq = true;
  return true;
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
  output += (error > 0.0f) ? MIN_OFFSET_US : -MIN_OFFSET_US;   // servo dead-band compensation
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
    stopServosNow();
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

  driveServos(panOffset, tiltOffset, dt);
}

/**
 * Manual drive from the app's on-screen joystick: x and y in [-1, 1], with the
 * same sign convention as the tracking error (positive x pans the way a subject
 * right of centre would, positive y tilts the way a subject below centre would).
 * Open loop -- stick deflection maps straight to a speed, starting at the edge
 * of the servo dead band (MIN_OFFSET_US) so the whole stick travel is usable.
 * The PID and the divergence guard are bypassed (a held stick is not a runaway
 * loop); the slew limit and the loss-of-signal failsafe still apply.
 */
void updateManual(float joyX, float joyY) {
  unsigned long now = millis();
  float dt = (lastUpdateMillis == 0) ? 0.0f : (now - lastUpdateMillis) / 1000.0f;
  lastUpdateMillis = now;

  // Leave nothing behind for the tracking loop to pick up when it resumes.
  panIntegral = 0.0f;
  tiltIntegral = 0.0f;
  lastErrX = 0.0f;
  lastErrY = 0.0f;
  panSaturatedSince = 0;
  tiltSaturatedSince = 0;
  controlDiverged = false;

  driveServos(joyAxisOffset(joyX), joyAxisOffset(joyY), dt);
}

float joyAxisOffset(float v) {
  v = constrain(v, -1.0f, 1.0f);
  if (fabs(v) <= JOY_DEADZONE) return 0.0f;
  float offset = MIN_OFFSET_US + fabs(v) * JOY_SPEED_RANGE_US;
  return (v > 0.0f) ? offset : -offset;
}

/**
 * Writes a pan/tilt speed (offset from neutral in us, before *_DIR) to the
 * servos, ramping toward it instead of stepping to it (see MAX_SLEW_US_PER_S).
 * The first packet after a stop has dt == 0; treat it as one nominal 30 Hz
 * frame so the servo still starts moving.
 */
void driveServos(float panOffset, float tiltOffset, float dt) {
  float slewDt = (dt > 0.0f) ? min(dt, MAX_SLEW_DT_S) : (1.0f / 30.0f);
  float maxStep = MAX_SLEW_US_PER_S * slewDt;
  panCmdOffsetUs  += constrain(PAN_DIR  * panOffset  - panCmdOffsetUs,  -maxStep, maxStep);
  tiltCmdOffsetUs += constrain(TILT_DIR * tiltOffset - tiltCmdOffsetUs, -maxStep, maxStep);

  int panPulse = PAN_NEUTRAL_US + (int)lroundf(panCmdOffsetUs);
  int tiltPulse = TILT_NEUTRAL_US + (int)lroundf(tiltCmdOffsetUs);

  panServo.writeMicroseconds(panPulse);
  tiltServo.writeMicroseconds(tiltPulse);

  Serial.printf("Servo pulse (us) -> pan: %d, tilt: %d\n", panPulse, tiltPulse);
}

/**
 * Immediate stop (bypasses the slew limit) for the failsafe and divergence
 * guard, and resets the slew state so the next move ramps up from neutral.
 */
void stopServosNow() {
  panServo.writeMicroseconds(PAN_NEUTRAL_US);
  tiltServo.writeMicroseconds(TILT_NEUTRAL_US);
  panCmdOffsetUs = 0.0f;
  tiltCmdOffsetUs = 0.0f;
}

/**
 * Logs WiFi link changes with the disconnect reason code, which tells apart a
 * weak signal (BEACON_TIMEOUT), the hotspot dropping us (AUTH_EXPIRE /
 * ASSOC_LEAVE ...), and wrong credentials (AUTH_FAIL / NO_AP_FOUND).
 */
void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      wifiUp = true;
      Serial.printf("WiFi up, IP %s\n", WiFi.localIP().toString().c_str());
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED: {
      uint8_t reason = info.wifi_sta_disconnected.reason;
      if (wifiUp) wifiDownSinceMs = millis();
      wifiUp = false;
      Serial.printf("WiFi DISCONNECTED, reason %u (%s), RSSI was %d dBm\n",
                    reason, WiFi.disconnectReasonName((wifi_err_reason_t) reason),
                    info.wifi_sta_disconnected.rssi);
      break;
    }
    default:
      break;
  }
}

/**
 * Fallback for the core's auto-reconnect: if the link has been down for
 * WIFI_RECONNECT_INTERVAL_MS, kick a reconnect ourselves.
 */
void maintainWiFi() {
  if (wifiUp) return;
  unsigned long now = millis();
  if (now - wifiDownSinceMs < WIFI_RECONNECT_INTERVAL_MS) return;
  if (now - lastWifiReconnectMs < WIFI_RECONNECT_INTERVAL_MS) return;
  lastWifiReconnectMs = now;
  Serial.println("WiFi still down -- forcing reconnect");
  WiFi.reconnect();
}

const char *resetReasonName(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON:   return "POWERON (normal power-up)";
    case ESP_RST_SW:        return "SW (software restart)";
    case ESP_RST_PANIC:     return "PANIC (crash)";
    case ESP_RST_INT_WDT:   return "INT_WDT (interrupt watchdog)";
    case ESP_RST_TASK_WDT:  return "TASK_WDT (task watchdog)";
    case ESP_RST_WDT:       return "WDT (other watchdog)";
    case ESP_RST_BROWNOUT:  return "BROWNOUT (supply voltage sagged)";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_EXT:       return "EXT (reset pin)";
    default:                return "UNKNOWN/OTHER";
  }
}

/**
 * Three red blinks at boot after a brownout, so a power-sag reboot is visible
 * without Serial Monitor. (The battery colour takes over the LED afterwards.)
 */
void signalBrownoutOnLed() {
#if defined(RGB_BUILTIN)
  for (int i = 0; i < 3; i++) {
    neopixelWrite(RGB_BUILTIN, 60, 0, 0);
    delay(200);
    neopixelWrite(RGB_BUILTIN, 0, 0, 0);
    delay(200);
  }
  neopixelWrite(RGB_BUILTIN, 0, 40, 0);
#endif
}

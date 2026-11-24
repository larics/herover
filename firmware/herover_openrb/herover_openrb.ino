// herover_openrb — motor controller for the herover (OpenRB-150 + 4x XL430).
//
// Reads body-frame velocity commands as text lines and drives the four
// mecanum wheels in velocity mode:
//
//   CMD <vx> <vy> <wz>\n    m/s, m/s, rad/s (REP-103: x fwd, y left, z CCW)
//   STOP\n                  all wheels to zero
//
// Lines come from CMD_SERIAL (the ESP32 — or later an RPi — on pin 13 RX)
// and from USB Serial (bench testing from a serial monitor). Every
// valid line is applied, including one identical to the previous. Nothing is
// ever written back to CMD_SERIAL; diagnostics go to USB only.
//
// Watchdog: when no valid line has arrived for CMD_TIMEOUT_MS the wheels are
// stopped, so a sender must repeat its command to keep the robot moving. This
// covers a crashed host, a lost WiFi link and a crashed or unplugged ESP32.

#include <Dynamixel2Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "mecanum.h"

// OpenRB-150: the DYNAMIXEL bus is Serial1 and needs no direction pin.
#define DXL_SERIAL Serial1
const int DXL_DIR_PIN = -1;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

struct Motor {
  uint8_t id;
  bool invert;
};

// Order matches WheelRpm: FL, FR, RL, RR.
const Motor MOTORS[4] = {
  {DXL_ID_FL, INVERT_FL},
  {DXL_ID_FR, INVERT_FR},
  {DXL_ID_RL, INVERT_RL},
  {DXL_ID_RR, INVERT_RR},
};
bool present[4] = {false, false, false, false};

// Line input. Defined up here, ahead of every function, because the Arduino
// builder inserts its generated prototypes before the first function.
#define LINE_LEN 64

struct LineReader {
  Stream* in;
  char buf[LINE_LEN];
  uint8_t len;
  bool overflow;  // current line is too long: drop it up to the newline
};

LineReader cmdLink = {&CMD_SERIAL, {0}, 0, false};
LineReader usbLink = {&Serial, {0}, 0, false};

// Watchdog: set by every valid command, cleared once the timeout stop has
// reached every motor. Off at boot, where the motors are already at zero.
bool watchdogArmed = false;
unsigned long lastCmdMs = 0;

// ---------------- motors ----------------

// Velocity mode, torque on, stopped. False if the motor does not answer.
bool setupMotor(uint8_t id) {
  if (!dxl.ping(id)) return false;
  dxl.torqueOff(id);
  dxl.setOperatingMode(id, OP_VELOCITY);
  dxl.torqueOn(id);
  dxl.setGoalVelocity(id, 0);
  return true;
}

void setupMotors() {
  dxl.begin(DXL_BAUD);
  dxl.setPortProtocolVersion(DXL_PROTOCOL);

  // dxl.begin() switches on the motor power and waits 500 ms. A motor that
  // is still not answering is retried for up to DXL_BOOT_WAIT_MS. With all
  // four answering this returns on the first round.
  const unsigned long start = millis();
  int found = 0;
  while (true) {
    for (int i = 0; i < 4; i++) {
      if (!present[i] && setupMotor(MOTORS[i].id)) {
        present[i] = true;
        found++;
      }
    }
    if (found == 4 || millis() - start >= DXL_BOOT_WAIT_MS) break;
    delay(50);
  }

  for (int i = 0; i < 4; i++) {
    if (present[i]) continue;
    Serial.print("DXL id ");
    Serial.print(MOTORS[i].id);
    Serial.println(" not found");
  }
}

// True if every present motor acknowledged its new speed.
bool applyTwist(float vx, float vy, float wz) {
  const WheelRpm w = mecanumIK(vx, vy, wz, WHEEL_RADIUS_M, HALF_WHEELBASE_M,
                               HALF_TRACK_M, MAX_WHEEL_RPM);
  const float rpm[4] = {w.fl, w.fr, w.rl, w.rr};

  bool ok = true;
  for (int i = 0; i < 4; i++) {
    if (!present[i]) continue;  // skip absent motors: each write would wait out a timeout
    const float v = MOTORS[i].invert ? -rpm[i] : rpm[i];
    if (!dxl.setGoalVelocity(MOTORS[i].id, v, UNIT_RPM)) ok = false;
  }

#if DEBUG_PRINT
  Serial.print("applied ");
  Serial.print(vx, 3); Serial.print(' ');
  Serial.print(vy, 3); Serial.print(' ');
  Serial.print(wz, 3); Serial.print(" -> rpm ");
  Serial.print(w.fl, 1); Serial.print(' ');
  Serial.print(w.fr, 1); Serial.print(' ');
  Serial.print(w.rl, 1); Serial.print(' ');
  Serial.println(w.rr, 1);
#endif
  return ok;
}

// ---------------- watchdog ----------------

void commandReceived() {
  lastCmdMs = millis();
  watchdogArmed = true;
}

// Stop the wheels once the commands have stopped. Stays armed, and so tries
// again on the next loop, while any motor has not acknowledged the stop.
void checkWatchdog() {
  if (!watchdogArmed || millis() - lastCmdMs < CMD_TIMEOUT_MS) return;
  if (!applyTwist(0.0f, 0.0f, 0.0f)) return;
  watchdogArmed = false;
  Serial.print("no command for ");
  Serial.print(CMD_TIMEOUT_MS);
  Serial.println(" ms: stopped");
}

// ---------------- parsing ----------------

// One whole token -> finite float. Rejects trailing junk, NaN, inf and
// absurd magnitudes (no herover command is anywhere near 1000 m/s).
bool parseFloatToken(const char* tok, float& out) {
  if (!tok) return false;
  char* end;
  out = strtof(tok, &end);
  if (end == tok || *end != '\0') return false;
  return fabsf(out) < 1.0e3f;  // false for NaN too
}

void processLine(char* line) {
  const char* SEP = " \t";
  char* cmd = strtok(line, SEP);
  if (!cmd) return;

  if (strcmp(cmd, "CMD") == 0) {
    float vx, vy, wz;
    if (!parseFloatToken(strtok(NULL, SEP), vx)) return;
    if (!parseFloatToken(strtok(NULL, SEP), vy)) return;
    if (!parseFloatToken(strtok(NULL, SEP), wz)) return;
    // Exactly three numbers. This also rejects the WiFi-side
    // "CMD <id> vx vy wz" form, which would otherwise read the id as vx.
    if (strtok(NULL, SEP) != NULL) return;
    commandReceived();
    applyTwist(vx, vy, wz);
  } else if (strcmp(cmd, "STOP") == 0) {
    commandReceived();
    applyTwist(0.0f, 0.0f, 0.0f);
  }
}

// ---------------- line input ----------------

void poll(LineReader& r) {
  while (r.in->available()) {
    const char c = (char)r.in->read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (!r.overflow && r.len > 0) {
        r.buf[r.len] = '\0';
        processLine(r.buf);
      }
      r.len = 0;
      r.overflow = false;
      continue;
    }
    if (r.overflow) continue;
    if (r.len < LINE_LEN - 1) {
      r.buf[r.len++] = c;
    } else {
      r.overflow = true;
    }
  }
}

// ---------------- setup / loop ----------------

void setup() {
  Serial.begin(115200);  // USB: bench commands + diagnostics; never waited on
  setupMotors();
  // Opened only now: commands sent while the motors were being found are
  // dropped instead of queued and applied seconds late.
  CMD_SERIAL.begin(CMD_BAUD);
  Serial.println("herover_openrb ready");
}

void loop() {
  poll(cmdLink);
  poll(usbLink);
  checkWatchdog();
}

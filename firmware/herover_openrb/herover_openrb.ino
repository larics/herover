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

// ---------------- motors ----------------

void setupMotors() {
  dxl.begin(DXL_BAUD);
  dxl.setPortProtocolVersion(DXL_PROTOCOL);

  for (int i = 0; i < 4; i++) {
    const uint8_t id = MOTORS[i].id;
    if (!dxl.ping(id)) {
      Serial.print("DXL id ");
      Serial.print(id);
      Serial.println(" not found");
      continue;
    }
    dxl.torqueOff(id);
    dxl.setOperatingMode(id, OP_VELOCITY);
    dxl.torqueOn(id);
    dxl.setGoalVelocity(id, 0);
    present[i] = true;
  }
}

void applyTwist(float vx, float vy, float wz) {
  const WheelRpm w = mecanumIK(vx, vy, wz, WHEEL_RADIUS_M, HALF_WHEELBASE_M,
                               HALF_TRACK_M, MAX_WHEEL_RPM);
  const float rpm[4] = {w.fl, w.fr, w.rl, w.rr};

  for (int i = 0; i < 4; i++) {
    if (!present[i]) continue;  // skip absent motors: each write would wait out a timeout
    const float v = MOTORS[i].invert ? -rpm[i] : rpm[i];
    dxl.setGoalVelocity(MOTORS[i].id, v, UNIT_RPM);
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
    applyTwist(vx, vy, wz);
  } else if (strcmp(cmd, "STOP") == 0) {
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
  CMD_SERIAL.begin(CMD_BAUD);
  setupMotors();
  Serial.println("herover_openrb ready");
}

void loop() {
  poll(cmdLink);
  poll(usbLink);
}

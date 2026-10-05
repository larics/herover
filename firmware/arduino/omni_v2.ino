#include "Arduino.h"
#include <SoftHalfDuplexSerial.h>
#include <DynamixelAx.h>

softHalfDuplexSerial port(8);
dxlAx dxlCom(&port);

#define DXL_BAUD 57600

// Set this to a unique value on each robot. 0 is reserved for broadcast.
#define ROBOT_ID 1

// Tune this for turning contribution
float K_ROT = 0.5;

// Serial buffer
#define BUF_LEN 64
char buf[BUF_LEN];
uint8_t pos = 0;

// ---------------- helpers ----------------

void setWheelMode(uint8_t id) {
  dxlCom.setCWAngleLimit(id, 0);
  dxlCom.setCCWAngleLimit(id, 0);
}

uint16_t toDxlSpeed(float v) {
  if (v > 1.0) v = 1.0;
  if (v < -1.0) v = -1.0;

  int speed = (int)(v * 1023.0);

  if (speed == 0) return 0;

  if (speed > 0) {
    return (uint16_t)speed;
  } else {
    return (uint16_t)(1024 + (-speed));
  }
}

void setMotorRaw(uint8_t id, float cmd, bool invert) {
  if (invert) cmd = -cmd;
  dxlCom.setMovingSpeed(id, toDxlSpeed(cmd));
}

void setOmniMotion(float vx, float vy, float wz) {
  // IDs:
  // 3 = front-left
  // 4 = front-right
  // 1 = rear-left
  // 2 = rear-right

  float m3 = vx - vy - K_ROT * wz;  // front-left
  float m4 = vx + vy + K_ROT * wz;  // front-right
  float m1 = vx + vy - K_ROT * wz;  // rear-left
  float m2 = vx - vy + K_ROT * wz;  // rear-right

  // Normalize so no wheel exceeds 1.0 (preserves direction at saturation)
  float m = fabs(m1);
  if (fabs(m2) > m) m = fabs(m2);
  if (fabs(m3) > m) m = fabs(m3);
  if (fabs(m4) > m) m = fabs(m4);
  if (m > 1.0) {
    m1 /= m; m2 /= m; m3 /= m; m4 /= m;
  }

  // Left side normal, right side inverted
  setMotorRaw(1, m1, false);
  setMotorRaw(3, m3, false);
  setMotorRaw(2, m2, true);
  setMotorRaw(4, m4, true);
}

void stopAll() {
  for (int i = 1; i <= 4; i++) {
    dxlCom.setMovingSpeed(i, 0);
  }
}

// ---------------- parsing ----------------

void processLine(char* line) {
  char* cmd = strtok(line, " ");
  if (!cmd) return;

  if (strcmp(cmd, "CMD") == 0) {
    char* idStr = strtok(NULL, " ");
    char* vxStr = strtok(NULL, " ");
    char* vyStr = strtok(NULL, " ");
    char* wzStr = strtok(NULL, " ");

    if (!idStr || !vxStr || !vyStr || !wzStr) return;

    int id = atoi(idStr);
    if (id != ROBOT_ID && id != 0) return;

    float vx = atof(vxStr);
    float vy = atof(vyStr);
    float wz = atof(wzStr);

    setOmniMotion(vx, vy, wz);

    Serial.print("CMD OK ");
    Serial.print(vx);
    Serial.print(" ");
    Serial.print(vy);
    Serial.print(" ");
    Serial.println(wz);
  }

  if (strcmp(cmd, "STOP") == 0) {
    stopAll();
    Serial.println("STOP OK");
  }
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\r') continue;

    if (c == '\n') {
      buf[pos] = '\0';
      if (pos > 0) processLine(buf);
      pos = 0;
      return;
    }

    if (pos < BUF_LEN - 1) {
      buf[pos++] = c;
    } else {
      pos = 0;
    }
  }
}

// ---------------- setup / loop ----------------

void setup() {
  Serial.begin(115200);
  dxlCom.begin(DXL_BAUD);

  delay(500);

  for (int i = 1; i <= 4; i++) {
    setWheelMode(i);
  }

  stopAll();
  Serial.println("READY");
}

void loop() {
  readSerial();
}

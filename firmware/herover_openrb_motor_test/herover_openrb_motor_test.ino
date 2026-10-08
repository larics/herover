// herover_openrb_motor_test — bench test for the four XL430 wheels, no ESP32
// or serial commands needed.
//
// Once after boot, each wheel in turn (FL, FR, RL, RR) runs forward, stops,
// runs backward, stops; then everything stops and the sketch idles. Press
// reset to run it again. Put the robot on blocks first.
//
// "Forward" applies the same INVERT_* as the main firmware, so every wheel
// should roll the robot forward in its forward phase. That checks the IDs
// (the right wheel moves) and the INVERT_* flags (it moves the right way).
//
// Progress and the measured rpm go to USB serial at 115200. The onboard LED
// blinks fast while the test runs (wheels may move) and slowly once it is done.

#include <Dynamixel2Arduino.h>

// Copied from firmware/herover_openrb/config.h; keep in sync. (An Arduino
// sketch cannot include files from another sketch folder.)
#define DXL_ID_FL 3
#define DXL_ID_FR 4
#define DXL_ID_RL 1
#define DXL_ID_RR 2
#define INVERT_FL false
#define INVERT_FR true
#define INVERT_RL false
#define INVERT_RR true
#define DXL_PROTOCOL 2.0f
#define DXL_BAUD     1000000

// ---------------- test settings ----------------
#define TEST_RPM       40.0f  // well below the 55 rpm limit of the main
#define RUN_MS         2000   // each direction
#define PAUSE_MS       1000   // stopped between phases
#define START_DELAY_MS 3000   // time to step back after reset
#define BLINK_TEST_MS  100    // LED toggle period while the test runs
#define BLINK_DONE_MS  1000   // LED toggle period once it is done

// OpenRB-150: the DYNAMIXEL bus is Serial1 and needs no direction pin.
#define DXL_SERIAL Serial1
const int DXL_DIR_PIN = -1;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

struct Motor {
  const char* name;
  uint8_t id;
  bool invert;
};

const Motor MOTORS[4] = {
  {"FL", DXL_ID_FL, INVERT_FL},
  {"FR", DXL_ID_FR, INVERT_FR},
  {"RL", DXL_ID_RL, INVERT_RL},
  {"RR", DXL_ID_RR, INVERT_RR},
};
bool present[4] = {false, false, false, false};

unsigned long blinkMs = BLINK_TEST_MS;
unsigned long lastToggleMs = 0;
bool ledOn = false;

// Toggles the LED every blinkMs; call it often.
void updateLed() {
  if (millis() - lastToggleMs < blinkMs) return;
  lastToggleMs = millis();
  ledOn = !ledOn;
  digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);
}

// delay() that keeps the LED blinking.
void waitBlinking(unsigned long ms) {
  const unsigned long start = millis();
  while (millis() - start < ms) updateLed();
}

// Wheel-forward rpm -> motor rpm.
float motorRpm(const Motor& m, float rpm) {
  return m.invert ? -rpm : rpm;
}

void stopAll() {
  for (int i = 0; i < 4; i++) {
    if (present[i]) dxl.setGoalVelocity(MOTORS[i].id, 0);
  }
}

// Runs one motor at rpm (wheel-forward positive) for RUN_MS, prints what the
// motor reports near the end, then stops it and pauses.
void runPhase(const Motor& m, float rpm, const char* label) {
  Serial.print("  ");
  Serial.print(label);
  Serial.print(" at ");
  Serial.print(rpm, 1);
  Serial.print(" rpm ... ");

  dxl.setGoalVelocity(m.id, motorRpm(m, rpm), UNIT_RPM);
  waitBlinking(RUN_MS);
  const float measured = motorRpm(m, dxl.getPresentVelocity(m.id, UNIT_RPM));
  dxl.setGoalVelocity(m.id, 0);

  Serial.print("measured ");
  Serial.print(measured, 1);
  Serial.println(" rpm");
  waitBlinking(PAUSE_MS);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  // Native USB: wait briefly for a monitor so the first lines are not lost,
  // but still run on battery with nothing attached.
  while (!Serial && millis() < 5000) updateLed();

  dxl.begin(DXL_BAUD);
  dxl.setPortProtocolVersion(DXL_PROTOCOL);

  Serial.println("herover_openrb_motor_test");
  for (int i = 0; i < 4; i++) {
    const Motor& m = MOTORS[i];
    Serial.print(m.name);
    Serial.print(" (id ");
    Serial.print(m.id);
    Serial.print("): ");
    if (!dxl.ping(m.id)) {
      Serial.println("not found, skipped");
      continue;
    }
    dxl.torqueOff(m.id);
    dxl.setOperatingMode(m.id, OP_VELOCITY);
    dxl.torqueOn(m.id);
    dxl.setGoalVelocity(m.id, 0);
    present[i] = true;
    Serial.println("ok");
  }

  Serial.print("starting in ");
  Serial.print(START_DELAY_MS / 1000);
  Serial.println(" s");
  waitBlinking(START_DELAY_MS);

  for (int i = 0; i < 4; i++) {
    if (!present[i]) continue;
    const Motor& m = MOTORS[i];
    Serial.print(m.name);
    Serial.print(" (id ");
    Serial.print(m.id);
    Serial.println(")");
    runPhase(m, TEST_RPM, "forward");
    runPhase(m, -TEST_RPM, "backward");
  }

  stopAll();
  Serial.println("done, all stopped. Reset to run again.");
  blinkMs = BLINK_DONE_MS;
}

void loop() {
  updateLed();
}

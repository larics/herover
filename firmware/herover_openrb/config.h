// herover_openrb configuration. Identical on every herover: the OpenRB has no
// robot id — addressing is done upstream (ESP32 / RPi), so one build fits all.
#pragma once

// ---------------- geometry (SI) ----------------
// Used by the mecanum inverse kinematics to turn a body-frame twist (m/s,
// rad/s) into wheel speeds. Measure on the robot.
#define WHEEL_RADIUS_M    0.030f  // TODO measure: wheel radius (m)
#define HALF_WHEELBASE_M  0.100f  // TODO measure: lx, centre -> front/rear axle (m)
#define HALF_TRACK_M      0.100f  // TODO measure: ly, centre -> left/right wheel (m)

// Wheel speed ceiling (rpm). Any command needing more is scaled down on all
// four wheels together, so the direction of travel is kept. XL430-W250 no-load
// speed is ~57 rpm at 11.1 V (~61 rpm at 12 V). Must stay below the motors'
// Velocity Limit register (factory 265 x 0.229 = 60.7 rpm): an X-series motor
// rejects a Goal Velocity above it. TODO: set from your supply voltage.
#define MAX_WHEEL_RPM     55.0f

// ---------------- motors ----------------
// Same ids as the old Arduino/AX-12A build. Right side is mounted mirrored,
// so its sign is flipped.
#define DXL_ID_FL 3
#define DXL_ID_FR 4
#define DXL_ID_RL 1
#define DXL_ID_RR 2
#define INVERT_FL false
#define INVERT_FR true
#define INVERT_RL false
#define INVERT_RR true

#define DXL_PROTOCOL 2.0f   // X-series (XL430, XL330, XM430, ...)
#define DXL_BAUD     57600  // XL430 factory default; 1000000 also works if set in Dynamixel Wizard

// ---------------- command link ----------------
// Serial2 is the OpenRB-150's dedicated 3.3 V UART header. Serial3 (pins
// 13 RX / 14 TX) is an alternative if the header is taken.
#define CMD_SERIAL   Serial2
#define CMD_BAUD     115200

// 1 = echo every applied command on USB Serial (never on CMD_SERIAL).
#define DEBUG_PRINT  0

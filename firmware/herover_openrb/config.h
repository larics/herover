// herover_openrb configuration. Identical on every herover: the OpenRB has no
// robot id — addressing is done upstream (ESP32 / RPi), so one build fits all.
#pragma once

// ---------------- geometry (SI) ----------------
// Used by the mecanum inverse kinematics to turn a body-frame twist (m/s,
// rad/s) into wheel speeds. Measured on the robot (README: Robot dimensions).
#define WHEEL_RADIUS_M    0.0395f  // wheel diameter 79 mm / 2
#define HALF_WHEELBASE_M  0.061f   // lx, centre -> front/rear axle: 122 mm / 2
#define HALF_TRACK_M      0.100f   // ly, centre -> left/right wheel: 200 mm / 2

// Wheel speed ceiling (rpm). Any command needing more is scaled down on all
// four wheels together, so the direction of travel is kept. XL430-W250 no-load
// speed is ~57 rpm at 11.1 V (~61 rpm at 12 V). Must stay below the motors'
// Velocity Limit register (factory 265 x 0.229 = 60.7 rpm): an X-series motor
// rejects a Goal Velocity above it. 45 is meant to stay reachable under load
// on a nearly empty 3S battery (estimated, not measured), so that no wheel
// falls behind the others.
#define MAX_WHEEL_RPM     45.0f

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
#define DXL_BAUD     1000000  // must match the motors' Baud Rate (XL430 factory default: 57600)

// At boot, how long to keep retrying motors that do not answer yet. A motor
// still missing after this is skipped until the next reset.
#define DXL_BOOT_WAIT_MS 3000

// ---------------- command link ----------------
// Serial3 is edge pins 13 RX / 14 TX; only pin 13 (RX) is wired. Serial2,
// the separate 4-pin UART header, is an alternative.
#define CMD_SERIAL   Serial3
#define CMD_BAUD     115200

// Watchdog: with no valid command (from CMD_SERIAL or USB) for this long, the
// wheels are stopped. Keep it many times the sender's repeat period (the
// cmd_vel_bridge re-sends at 10 Hz), so a few lost datagrams don't stop the
// robot.
#define CMD_TIMEOUT_MS 2000

// 1 = echo every applied command on USB Serial (never on CMD_SERIAL).
#define DEBUG_PRINT  0

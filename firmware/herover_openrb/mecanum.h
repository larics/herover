// Mecanum inverse kinematics in SI units. Pure (no Arduino deps) so it can be
// compiled and checked on a host.
#pragma once

#include <math.h>

struct WheelRpm {
  float fl, fr, rl, rr;
};

// Body-frame twist (REP-103: x forward, y left, z counter-clockwise; m/s and
// rad/s) -> wheel speeds in rpm, positive = wheel rolling the robot forward.
// X-configuration rollers; k = lx + ly. If any wheel would exceed max_rpm,
// all four are scaled by the same factor, so the direction of travel is kept.
inline WheelRpm mecanumIK(float vx, float vy, float wz,
                          float wheel_radius, float lx, float ly,
                          float max_rpm) {
  const float k = lx + ly;
  const float to_rpm = 60.0f / (2.0f * (float)M_PI * wheel_radius);

  WheelRpm w;
  w.fl = (vx - vy - k * wz) * to_rpm;
  w.fr = (vx + vy + k * wz) * to_rpm;
  w.rl = (vx + vy - k * wz) * to_rpm;
  w.rr = (vx - vy + k * wz) * to_rpm;

  float m = fabsf(w.fl);
  if (fabsf(w.fr) > m) m = fabsf(w.fr);
  if (fabsf(w.rl) > m) m = fabsf(w.rl);
  if (fabsf(w.rr) > m) m = fabsf(w.rr);
  if (m > max_rpm) {
    const float s = max_rpm / m;
    w.fl *= s; w.fr *= s; w.rl *= s; w.rr *= s;
  }
  return w;
}

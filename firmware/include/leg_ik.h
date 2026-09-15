// Inverse kinematics for one 3-DoF leg: coxa (yaw), femur (pitch), tibia (pitch).
//
// A line-for-line port of hexapod_gait/leg_ik.py. That file is the reference;
// if the two ever disagree, the Python is right and this is the bug. The native
// test (test/test_kinematics.cpp) checks them against shared fixtures.
//
// Pure functions, no Arduino, no globals - so it compiles for the host as
// happily as for the ESP32, which is what makes it testable at all.
//
// Leg frame: origin at the coxa pivot, +X outward along the leg's rest
// direction, +Y to its left, +Z up. Joint angle zero is a straight leg;
// positive femur and tibia angles lift the link.
#pragma once

#include <math.h>

#include "robot_geometry.h"

namespace robot {

struct Vec3 {
  float x, y, z;
};

// M_PI is a POSIX extension, not standard C++. Arduino defines it, a host
// compiler need not - and the whole point of these headers is that they build
// in both places.
constexpr float kPi = 3.14159265358979323846f;

// Everything is float, not double: the ESP32-S3's FPU is single precision, and
// one double in an expression pulls the lot into software emulation.
inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

/// Joint angles (coxa, femur, tibia) in radians that place the foot at `target`.
///
/// Targets outside the reachable annulus are clamped to the nearest reachable
/// point rather than returning NaN, so a bad gait parameter shows up as a
/// visibly wrong pose instead of a leg full of quiet NaNs.
inline Vec3 ik_solve(const Vec3& target,
                     float coxa = COXA, float femur = FEMUR, float tibia = TIBIA) {
  const float coxa_angle = atan2f(target.y, target.x);

  // Reduce to a planar 2-link problem in the vertical plane the leg now
  // occupies: `horiz` outward from the femur pivot, `vert` up.
  const float horiz = sqrtf(target.x * target.x + target.y * target.y) - coxa;
  const float vert = target.z;
  float dist = sqrtf(horiz * horiz + vert * vert);
  dist = clampf(dist, fabsf(femur - tibia) + 1e-3f, femur + tibia - 1e-3f);

  // Law of cosines. The +acos branch picks the knee-up solution, the one a
  // hexapod stands in.
  const float bearing = atan2f(vert, horiz);
  const float femur_angle =
      bearing + acosf(clampf((femur * femur + dist * dist - tibia * tibia) /
                             (2.0f * femur * dist), -1.0f, 1.0f));

  // Interior angle at the knee, re-expressed relative to the femur: a straight
  // leg reads zero and the knee folds negative.
  const float knee =
      acosf(clampf((femur * femur + tibia * tibia - dist * dist) /
                   (2.0f * femur * tibia), -1.0f, 1.0f));

  return Vec3{coxa_angle, femur_angle, knee - kPi};
}

/// Forward kinematics - where a given pose actually puts the foot.
/// Used by the tests to confirm ik_solve hit what it was asked for.
inline Vec3 ik_foot_position(const Vec3& a,
                             float coxa = COXA, float femur = FEMUR, float tibia = TIBIA) {
  const float horiz = coxa + femur * cosf(a.y) + tibia * cosf(a.y + a.z);
  const float vert = femur * sinf(a.y) + tibia * sinf(a.y + a.z);
  return Vec3{horiz * cosf(a.x), horiz * sinf(a.x), vert};
}

/// Joint angles converted to servo angles, measured from each servo's centre.
/// This is the number that becomes a pulse width.
inline Vec3 to_servo(const Vec3& a) {
  const float d2r = kPi / 180.0f;
  return Vec3{a.x - CENTER_DEG[0] * d2r,
              a.y - CENTER_DEG[1] * d2r,
              a.z - CENTER_DEG[2] * d2r};
}

/// True when every joint sits inside the servo's usable travel.
inline bool within_limits(const Vec3& a) {
  const Vec3 s = to_servo(a);
  const float t = TRAVEL_DEG * kPi / 180.0f;
  return fabsf(s.x) <= t && fabsf(s.y) <= t && fabsf(s.z) <= t;
}

}  // namespace robot

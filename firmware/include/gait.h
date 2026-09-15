// Tripod gait generator - a port of hexapod_gait/gait.py.
//
// The whole gait is one function. Given where a foot rests when standing and
// how the body wants to move, it returns where that foot should be right now.
// Each leg calls it independently; the coordination falls out of the half-cycle
// phase offset between the two tripods rather than any shared state.
//
// Pure, like leg_ik.h, and for the same reason.
#pragma once

#include <math.h>

#include "leg_ik.h"
#include "robot_geometry.h"

namespace robot {

/// Displacement from a foot's neutral stance position, in the BODY frame.
///
/// neutral  - the foot's resting position relative to the body centre
/// vx, vy   - commanded body velocity, m/s (+X forward, +Y left)
/// yaw_rate - commanded turn rate, rad/s, positive counter-clockwise
/// phase    - gait cycle position, wrapping 0..1
/// group    - 0 or 1, the leg's tripod
inline Vec3 foot_offset(const Vec3& neutral, float vx, float vy, float yaw_rate,
                        float phase, uint8_t group,
                        float cycle = CYCLE_TIME, float lift = STEP_HEIGHT) {
  // A planted foot must travel backward exactly as fast as the body travels
  // forward, or the robot drags itself along. The yaw terms are omega x r, the
  // extra motion a turn demands of a foot offset from the body centre.
  const float fvx = -(vx - yaw_rate * neutral.y);
  const float fvy = -(vy + yaw_rate * neutral.x);

  // Stance is half the cycle and the stroke is centred on neutral, so the foot
  // swings between -amp and +amp.
  const float ax = fvx * cycle * 0.25f;
  const float ay = fvy * cycle * 0.25f;

  // The two tripods run half a cycle apart. That single offset is the gait.
  float local = fmodf(phase + (group == 1 ? 0.5f : 0.0f), 1.0f);
  if (local < 0.0f) local += 1.0f;

  if (local < 0.5f) {
    // Stance: on the ground, pushing the body along at a constant rate.
    const float u = local / 0.5f;
    return Vec3{-ax + 2.0f * ax * u, -ay + 2.0f * ay * u, 0.0f};
  }

  // Swing: lift, return to the front of the stroke, set back down. Smoothstep
  // on the horizontal keeps lift-off and touchdown gentle; the sine arc puts
  // peak height at mid-swing and exactly zero at both ends, so the foot never
  // clips through the ground.
  const float u = (local - 0.5f) / 0.5f;
  const float s = u * u * (3.0f - 2.0f * u);
  return Vec3{ax - 2.0f * ax * s, ay - 2.0f * ay * s, lift * sinf(kPi * u)};
}

/// Progress through stance, 0..1, or -1 while the leg is in swing.
/// Contact checks should ignore the first and last stretch of stance: the foot
/// is still arriving or already leaving, and a switch reading nothing there is
/// timing, not a stumble.
inline float stance_progress(float phase, uint8_t group) {
  float local = fmodf(phase + (group == 1 ? 0.5f : 0.0f), 1.0f);
  if (local < 0.0f) local += 1.0f;
  return local < 0.5f ? local / 0.5f : -1.0f;
}

inline bool is_stance(float phase, uint8_t group) {
  return stance_progress(phase, group) >= 0.0f;
}

/// Where leg `i`'s foot rests when the robot simply stands, in the body frame.
inline Vec3 neutral_foot(int i) {
  const Leg& L = LEGS[i];
  return Vec3{L.mount_x + cosf(L.yaw) * REACH,
              L.mount_y + sinf(L.yaw) * REACH,
              L.mount_z - STAND_HEIGHT};
}

/// Full solve for one leg: body-frame command in, joint angles out.
/// Body frame to leg frame is a translation to the hip and a rotation by the
/// leg's rest yaw - the same change of basis the sim does.
inline Vec3 solve_leg(int i, float vx, float vy, float yaw_rate, float phase,
                      float blend = 1.0f) {
  const Leg& L = LEGS[i];
  const Vec3 n = neutral_foot(i);
  const Vec3 off = foot_offset(n, vx, vy, yaw_rate, phase, L.group);

  const float bx = n.x + off.x * blend;
  const float by = n.y + off.y * blend;
  const float bz = n.z + off.z * blend;

  const float dx = bx - L.mount_x;
  const float dy = by - L.mount_y;
  const float dz = bz - L.mount_z;
  const float c = cosf(L.yaw), s = sinf(L.yaw);
  return ik_solve(Vec3{c * dx + s * dy, -s * dx + c * dy, dz});
}

}  // namespace robot

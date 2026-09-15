// Does the firmware agree with the simulator?
//
// leg_ik.h and gait.h are ports of hexapod_gait/leg_ik.py and gait.py. Every
// expected value in fixtures.h was computed by that Python, so these assertions
// are a direct comparison between the two implementations - not a restatement
// of what the C++ already does.
//
// Build and run:  python tools/test_firmware.py
#include <math.h>
#include <stdio.h>

#include "fixtures.h"
#include "gait.h"
#include "leg_ik.h"

using namespace robot;

namespace {

int failures = 0;
int checks = 0;

// float32 survives two trig round-trips to about this; 1e-5 rad is 0.0006 deg.
constexpr float TOL = 1e-5f;

void near(float expect, float got, const char* what, const char* detail = "") {
  ++checks;
  if (fabsf(expect - got) > TOL || isnan(got)) {
    ++failures;
    printf("  FAIL %-34s %s expected %.8f got %.8f (d %.2e)\n", what, detail,
           (double)expect, (double)got, (double)fabsf(expect - got));
  }
}

void check(bool cond, const char* what, const char* detail = "") {
  ++checks;
  if (!cond) {
    ++failures;
    printf("  FAIL %-34s %s\n", what, detail);
  }
}

void report(const char* name, int before) {
  printf("%-44s %s\n", name, failures == before ? "ok" : "FAILED");
}

// --- the port is faithful ----------------------------------------------------

void ik_matches_python() {
  const int b = failures;
  for (int i = 0; i < N_IK; ++i) {
    const IkCase& k = IK[i];
    const Vec3 a = ik_solve(Vec3{k.x, k.y, k.z});
    near(k.coxa, a.x, "ik coxa");
    near(k.femur, a.y, "ik femur");
    near(k.tibia, a.z, "ik tibia");
  }
  report("IK matches the Python reference", b);
}

void gait_matches_python() {
  const int b = failures;
  for (int i = 0; i < N_GAIT; ++i) {
    const GaitCase& g = GAIT[i];
    const Vec3 n = neutral_foot(g.leg);
    const Vec3 o = foot_offset(n, g.vx, g.vy, g.wz, g.phase, LEGS[g.leg].group);
    near(g.ox, o.x, "gait offset x", LEGS[g.leg].name);
    near(g.oy, o.y, "gait offset y", LEGS[g.leg].name);
    near(g.oz, o.z, "gait offset z", LEGS[g.leg].name);
  }
  report("gait matches the Python reference", b);
}

// --- properties that must hold however the maths is written ------------------

void ik_round_trips() {
  const int b = failures;
  for (int i = 0; i < N_IK; ++i) {
    const IkCase& k = IK[i];
    const Vec3 p = ik_foot_position(ik_solve(Vec3{k.x, k.y, k.z}));
    near(k.x, p.x, "round-trip x");
    near(k.y, p.y, "round-trip y");
    near(k.z, p.z, "round-trip z");
  }
  report("IK round-trips through forward kinematics", b);
}

void tripods_alternate() {
  const int b = failures;
  for (float ph = 0.0f; ph < 1.0f; ph += 0.02f) {
    int planted = 0;
    for (int i = 0; i < NUM_LEGS; ++i)
      if (is_stance(ph, LEGS[i].group)) ++planted;
    check(planted == 3, "exactly 3 feet planted");
  }
  report("tripods alternate - 3 feet always down", b);
}

void swing_never_digs() {
  const int b = failures;
  const Vec3 n = neutral_foot(0);
  for (float ph = 0.0f; ph < 1.0f; ph += 0.005f) {
    const Vec3 o = foot_offset(n, 0.10f, 0.0f, 0.0f, ph, LEGS[0].group);
    check(o.z >= -1e-6f, "swing lift never negative");
    if (is_stance(ph, LEGS[0].group)) near(0.0f, o.z, "stance foot on the ground");
  }
  report("swing arc never digs into the ground", b);
}

void walking_fits_the_servos() {
  const int b = failures;
  float worst = 0.0f;
  const float d2r = kPi / 180.0f;
  const float cmds[][3] = {{0.10f, 0, 0}, {-0.10f, 0, 0}, {0, 0.10f, 0},
                           {0, 0, 1.0f}, {0.10f, 0, 1.0f}, {0.07f, 0.07f, 0.7f}};
  for (auto& c : cmds) {
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
      for (float ph = 0.0f; ph < 1.0f; ph += 0.02f) {
        const Vec3 a = solve_leg(leg, c[0], c[1], c[2], ph);
        check(within_limits(a), "inside servo travel", LEGS[leg].name);
        const Vec3 s = to_servo(a);
        worst = fmaxf(worst, fmaxf(fabsf(s.x), fmaxf(fabsf(s.y), fabsf(s.z))));
      }
    }
  }
  report("every walking pose fits the servo travel", b);
  printf("%-44s %.1f of %.0f deg\n", "  worst excursion from centre",
         (double)(worst / d2r), (double)TRAVEL_DEG);
}

void stance_is_the_designed_pose() {
  const int b = failures;
  // Standing still, every foot should sit exactly at its neutral position.
  for (int leg = 0; leg < NUM_LEGS; ++leg) {
    const Vec3 a = solve_leg(leg, 0, 0, 0, 0.0f);
    const Vec3 n = neutral_foot(leg);
    const Leg& L = LEGS[leg];
    const Vec3 p = ik_foot_position(a);
    // Back to the body frame: rotate by the leg's yaw, add the hip.
    const float c = cosf(L.yaw), s = sinf(L.yaw);
    near(n.x, L.mount_x + c * p.x - s * p.y, "stance foot x", L.name);
    near(n.y, L.mount_y + s * p.x + c * p.y, "stance foot y", L.name);
    near(n.z, L.mount_z + p.z, "stance foot z", L.name);
  }
  report("standing puts every foot at its neutral point", b);
}

}  // namespace

int main() {
  printf("\nfirmware kinematics vs the Python reference\n");
  printf("geometry %.0f/%.0f/%.0f mm, reach %.0f mm, centres %.0f/%.0f/%.0f deg\n\n",
         (double)(COXA * 1000), (double)(FEMUR * 1000), (double)(TIBIA * 1000),
         (double)(REACH * 1000), (double)CENTER_DEG[0], (double)CENTER_DEG[1],
         (double)CENTER_DEG[2]);

  ik_matches_python();
  gait_matches_python();
  ik_round_trips();
  stance_is_the_designed_pose();
  tripods_alternate();
  swing_never_digs();
  walking_fits_the_servos();

  printf("\n%d checks, %d failures\n", checks, failures);
  return failures == 0 ? 0 : 1;
}

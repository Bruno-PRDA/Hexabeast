// Hexabeast firmware - stage 1: servos, calibration and the gait loop.
//
// WiFi, the camera and the FPV link come next; this stage is everything you can
// test on a bench with a board, two PCA9685s and as few as one servo.
//
// It boots relaxed. Nothing moves until you say so, because the first thing a
// miswired hexapod does on power-up is fold a leg through its own chassis.
//
// Serial console at 115200 - send `?` for the command list.

#include <Arduino.h>

#include "gait.h"
#include "leg_ik.h"
#include "robot_geometry.h"
#include "servo_bus.h"

using namespace robot;

// --- wiring ------------------------------------------------------------------
// I2C for both PCA9685s, the IMU and the foot-switch expander. The ESP32-S3's
// GPIO matrix puts I2C on almost any free pin; avoid 19/20, which are USB D-/D+.
constexpr int PIN_SDA = 1;
constexpr int PIN_SCL = 2;

constexpr uint32_t LOOP_HZ = 50;             // servo update rate
constexpr uint32_t LOOP_US = 1000000 / LOOP_HZ;
constexpr float STARTUP_TIME = 2.0f;         // ease from centre into stance
constexpr float BLEND_RATE = 4.0f;           // how fast legs settle when stopped
constexpr float MAX_SPEED = 0.10f;           // m/s
constexpr float MAX_TURN = 1.0f;             // rad/s

// Watchdog on the command channel. A joystick that goes quiet - dropped WiFi,
// closed browser - must stop the robot rather than leave it walking away.
constexpr uint32_t CMD_TIMEOUT_MS = 500;

enum class Mode { Idle, Calibrate, Run };

ServoBus bus;
Mode mode = Mode::Idle;

float cmd_vx = 0, cmd_vy = 0, cmd_wz = 0;
uint32_t cmd_stamp_ms = 0;
float phase = 0, blend = 0, startup = 0;
int sel_joint = 0;                            // calibration cursor
uint32_t last_loop_us = 0;

static float move_toward(float v, float target, float step) {
  if (fabsf(target - v) <= step) return target;
  return v + (target > v ? step : -step);
}

static void help() {
  Serial.println();
  Serial.println(F("Hexabeast - commands"));
  Serial.println(F("  ?          this list"));
  Serial.println(F("  e / x      enable output / relax (x is the panic key)"));
  Serial.println(F("  c          all servos to mechanical centre"));
  Serial.println(F("  k          calibration mode"));
  Serial.println(F("  r          run mode (gait)"));
  Serial.println(F("  i          status"));
  Serial.println(F("calibration:"));
  Serial.println(F("  j<n>       select joint 0-17      n / p   next / previous"));
  Serial.println(F("  + / -      trim +/-10 us          > / <   trim +/-2 us"));
  Serial.println(F("  f          flip this joint's direction"));
  Serial.println(F("  s          save calibration to NVS"));
  Serial.println(F("run mode:"));
  Serial.println(F("  w<vx>,<vy>,<wz>   e.g. w0.08,0,0   walk forward"));
  Serial.println(F("  h          halt (zero command)"));
  Serial.println();
}

static void status() {
  Serial.printf("mode=%s output=%s phase=%.2f blend=%.2f\n",
                mode == Mode::Idle ? "idle" : mode == Mode::Calibrate ? "calib" : "run",
                bus.enabled() ? "ON" : "off", phase, blend);
  Serial.printf("cmd vx=%.3f vy=%.3f wz=%.3f (%lu ms ago)\n",
                cmd_vx, cmd_vy, cmd_wz, (unsigned long)(millis() - cmd_stamp_ms));
  Serial.printf("selected joint %d (%s)  ch=%u/%u dir=%+d trim=%d us  last=%u us\n",
                sel_joint, JOINT_NAMES[sel_joint], SERVOS[sel_joint].board,
                SERVOS[sel_joint].channel, bus.dir(sel_joint), bus.trim(sel_joint),
                bus.last_us(sel_joint));
  int clamped = 0;
  for (int j = 0; j < NUM_JOINTS; ++j) if (bus.clamped(j)) ++clamped;
  if (clamped) Serial.printf("WARNING %d joint(s) clamped at travel limit\n", clamped);
}

static void select(int j) {
  sel_joint = (j + NUM_JOINTS) % NUM_JOINTS;
  Serial.printf("joint %d (%s) ch=%u/%u dir=%+d trim=%d\n", sel_joint,
                JOINT_NAMES[sel_joint], SERVOS[sel_joint].board,
                SERVOS[sel_joint].channel, bus.dir(sel_joint), bus.trim(sel_joint));
}

// Hold the selected joint at centre while its trim is adjusted, so the horn can
// be seen to line up. Everything else stays where it was.
static void refresh_calibration_pose() {
  const float d2r = kPi / 180.0f;
  bus.set_joint(sel_joint, CENTER_DEG[sel_joint % 3] * d2r);
}

static void handle(const String& line) {
  if (line.length() == 0) return;
  const char c = line[0];
  const String arg = line.substring(1);

  switch (c) {
    case '?': help(); break;
    case 'i': status(); break;
    case 'e':
      bus.enable(true);
      startup = 0;
      Serial.println(F("output ENABLED - ramping to pose"));
      break;
    case 'x':
      bus.relax();
      mode = Mode::Idle;
      Serial.println(F("relaxed"));
      break;
    case 'c':
      bus.center_all();
      Serial.println(F("all servos at mechanical centre"));
      break;
    case 'k':
      mode = Mode::Calibrate;
      bus.center_all();
      Serial.println(F("calibration mode - fit horns at this pose"));
      select(sel_joint);
      break;
    case 'r':
      mode = Mode::Run;
      startup = 0;
      Serial.println(F("run mode"));
      break;
    case 'j': select(arg.toInt()); refresh_calibration_pose(); break;
    case 'n': select(sel_joint + 1); refresh_calibration_pose(); break;
    case 'p': select(sel_joint - 1); refresh_calibration_pose(); break;
    case '+': bus.set_trim(sel_joint, bus.trim(sel_joint) + 10); refresh_calibration_pose();
              Serial.printf("trim %d us\n", bus.trim(sel_joint)); break;
    case '-': bus.set_trim(sel_joint, bus.trim(sel_joint) - 10); refresh_calibration_pose();
              Serial.printf("trim %d us\n", bus.trim(sel_joint)); break;
    case '>': bus.set_trim(sel_joint, bus.trim(sel_joint) + 2); refresh_calibration_pose();
              Serial.printf("trim %d us\n", bus.trim(sel_joint)); break;
    case '<': bus.set_trim(sel_joint, bus.trim(sel_joint) - 2); refresh_calibration_pose();
              Serial.printf("trim %d us\n", bus.trim(sel_joint)); break;
    case 'f':
      bus.set_dir(sel_joint, -bus.dir(sel_joint));
      refresh_calibration_pose();
      Serial.printf("direction %+d\n", bus.dir(sel_joint));
      break;
    case 's': bus.save_calibration(); break;
    case 'h': cmd_vx = cmd_vy = cmd_wz = 0; cmd_stamp_ms = millis(); break;
    case 'w': {
      const int c1 = arg.indexOf(',');
      const int c2 = arg.indexOf(',', c1 + 1);
      if (c1 < 0 || c2 < 0) { Serial.println(F("usage: w<vx>,<vy>,<wz>")); break; }
      cmd_vx = clampf(arg.substring(0, c1).toFloat(), -MAX_SPEED, MAX_SPEED);
      cmd_vy = clampf(arg.substring(c1 + 1, c2).toFloat(), -MAX_SPEED, MAX_SPEED);
      cmd_wz = clampf(arg.substring(c2 + 1).toFloat(), -MAX_TURN, MAX_TURN);
      cmd_stamp_ms = millis();
      Serial.printf("cmd %.3f %.3f %.3f\n", cmd_vx, cmd_vy, cmd_wz);
      break;
    }
    default: Serial.println(F("unknown - send ? for help"));
  }
}

static void read_serial() {
  static String line;
  while (Serial.available()) {
    const char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (line.length()) { handle(line); line = ""; }
    } else if (line.length() < 64) {
      line += ch;
    }
  }
}

static void step_gait(float dt) {
  // A command that has gone stale stops the robot. This is the same guarantee
  // the WiFi link will need, so it lives here rather than in the transport.
  if (millis() - cmd_stamp_ms > CMD_TIMEOUT_MS) {
    cmd_vx = cmd_vy = cmd_wz = 0;
  }

  startup = fminf(startup + dt / STARTUP_TIME, 1.0f);
  const bool moving = fabsf(cmd_vx) > 1e-3f || fabsf(cmd_vy) > 1e-3f ||
                      fabsf(cmd_wz) > 1e-3f;
  blend = move_toward(blend, moving ? 1.0f : 0.0f, BLEND_RATE * dt);
  if (blend > 1e-3f) {
    phase = fmodf(phase + dt / CYCLE_TIME, 1.0f);
  }

  for (int leg = 0; leg < NUM_LEGS; ++leg) {
    const Vec3 a = solve_leg(leg, cmd_vx, cmd_vy, cmd_wz, phase, blend);
    // Ease out of the boot pose: interpolate from servo centre to the solved
    // pose over STARTUP_TIME rather than snapping to it.
    const float d2r = kPi / 180.0f;
    const Vec3 eased{
        CENTER_DEG[0] * d2r + (a.x - CENTER_DEG[0] * d2r) * startup,
        CENTER_DEG[1] * d2r + (a.y - CENTER_DEG[1] * d2r) * startup,
        CENTER_DEG[2] * d2r + (a.z - CENTER_DEG[2] * d2r) * startup};
    bus.set_leg(leg, eased);
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println(F("Hexabeast firmware - servos and gait"));
  Serial.printf("geometry %.0f/%.0f/%.0f mm, reach %.0f mm, %d joints\n",
                COXA * 1000, FEMUR * 1000, TIBIA * 1000, REACH * 1000, NUM_JOINTS);

  if (!bus.begin(PIN_SDA, PIN_SCL)) {
    Serial.println(F("!! a PCA9685 is missing. Check the A0 jumper on board 1,"));
    Serial.println(F("!! and that the INA219 has been moved off 0x40."));
  }
  cmd_stamp_ms = millis();
  last_loop_us = micros();
  help();
  Serial.println(F("output is OFF. `e` to enable, `x` to relax at any time."));
}

void loop() {
  read_serial();

  const uint32_t now = micros();
  if (now - last_loop_us < LOOP_US) return;
  const float dt = (now - last_loop_us) * 1e-6f;
  last_loop_us = now;

  if (mode == Mode::Run) step_gait(dt);
}

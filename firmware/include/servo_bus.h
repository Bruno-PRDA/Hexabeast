// Two PCA9685 boards driving 18 servos over one I2C bus.
//
// The PCA9685 generates the 50 Hz pulses in hardware, so servo timing is immune
// to whatever the CPU is doing - which matters once the same chip is also
// encoding JPEG and serving WiFi.
//
// Board 0 sits at 0x40 and carries the left legs, board 1 at 0x41 the right.
// Channel and direction per joint come from robot_geometry.h, generated from
// robot_config.py.
//
// Safety: nothing is driven until enable() is called. A hexapod that snaps to
// its stance pose the instant power arrives can strip a gear or break a link
// before you reach the switch, so boot leaves every output dark and the legs
// limp.
#pragma once

#include <stdint.h>

#include "leg_ik.h"
#include "robot_geometry.h"

namespace robot {

class ServoBus {
 public:
  static constexpr uint8_t ADDR[2] = {0x40, 0x41};
  static constexpr float PWM_HZ = 50.0f;

  /// Bring up I2C and both boards with all outputs off. Returns false if either
  /// board does not acknowledge - almost always the A0 jumper on board 1, or
  /// the INA219 still sitting on 0x40.
  bool begin(int sda, int scl, uint32_t i2c_hz = 400000);

  /// Allow output. Until this is called every write is accepted and ignored,
  /// so the control loop can run and be watched before anything moves.
  void enable(bool on);
  bool enabled() const { return enabled_; }

  /// Command one joint, in radians of JOINT angle (not servo angle). Applies
  /// the servo centre, the mirror direction and the per-joint trim, clamps to
  /// usable travel, and writes the pulse.
  void set_joint(int joint, float angle_rad);

  /// Command a whole leg at once - the three joints in coxa/femur/tibia order.
  void set_leg(int leg, const Vec3& angles);

  /// Every servo to its mechanical centre. This is the pose to fit horns in.
  void center_all();

  /// Cut all pulses; servos go limp. Also what enable(false) does.
  void relax();

  /// Per-joint zero correction in microseconds, persisted to NVS.
  /// Calibration writes these; nothing else should.
  void set_trim(int joint, int16_t us);
  int16_t trim(int joint) const { return trim_us_[joint]; }
  void set_dir(int joint, int8_t dir);
  int8_t dir(int joint) const { return dir_[joint]; }
  bool save_calibration();
  bool load_calibration();

  /// Last pulse written, for diagnostics and the calibration UI.
  uint16_t last_us(int joint) const { return last_us_[joint]; }

  /// True if a joint was clamped on its last write - the command asked for
  /// more travel than the servo has, which means a geometry or gait problem
  /// rather than a servo one.
  bool clamped(int joint) const { return clamped_[joint]; }

 private:
  void write_reg(uint8_t addr, uint8_t reg, uint8_t val);
  void write_pulse(uint8_t board, uint8_t channel, uint16_t us);
  bool probe(uint8_t addr);

  bool enabled_ = false;
  bool ready_ = false;
  int16_t trim_us_[NUM_JOINTS] = {0};
  int8_t dir_[NUM_JOINTS] = {0};
  uint16_t last_us_[NUM_JOINTS] = {0};
  bool clamped_[NUM_JOINTS] = {false};
};

}  // namespace robot

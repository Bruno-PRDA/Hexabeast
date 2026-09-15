#include "servo_bus.h"

#include <Arduino.h>
#include <Preferences.h>
#include <Wire.h>

namespace robot {

namespace {
// PCA9685 registers
constexpr uint8_t MODE1 = 0x00;
constexpr uint8_t MODE2 = 0x01;
constexpr uint8_t LED0_ON_L = 0x06;
constexpr uint8_t ALL_LED_ON_L = 0xFA;
constexpr uint8_t PRESCALE = 0xFE;

constexpr uint8_t MODE1_RESTART = 0x80;
constexpr uint8_t MODE1_AI = 0x20;      // auto-increment, so a channel is one burst
constexpr uint8_t MODE1_SLEEP = 0x10;
constexpr uint8_t MODE2_OUTDRV = 0x04;  // totem pole, not open drain

// Nominal internal oscillator. Clone boards are often 26-27 MHz, which stretches
// every pulse by a few percent; if the centre pose sits visibly off on all 18
// servos at once, measure the real PWM period and correct here rather than
// trimming each servo around the same systematic error.
constexpr float OSC_HZ = 25000000.0f;

constexpr const char* NVS_NAMESPACE = "hexapod";
}  // namespace

bool ServoBus::probe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

void ServoBus::write_reg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool ServoBus::begin(int sda, int scl, uint32_t i2c_hz) {
  Wire.begin(sda, scl, i2c_hz);

  for (int j = 0; j < NUM_JOINTS; ++j) {
    dir_[j] = SERVOS[j].dir;
    trim_us_[j] = 0;
  }

  ready_ = true;
  for (int b = 0; b < 2; ++b) {
    if (!probe(ADDR[b])) {
      Serial.printf("[servo] PCA9685 at 0x%02X did not respond\n", ADDR[b]);
      ready_ = false;
      continue;
    }
    // Sleep is required before PRESCALE will latch.
    write_reg(ADDR[b], MODE1, MODE1_SLEEP);
    const uint8_t prescale =
        (uint8_t)(roundf(OSC_HZ / (4096.0f * PWM_HZ)) - 1.0f);
    write_reg(ADDR[b], PRESCALE, prescale);
    write_reg(ADDR[b], MODE1, 0x00);
    delayMicroseconds(500);  // oscillator needs time before RESTART
    write_reg(ADDR[b], MODE1, MODE1_RESTART | MODE1_AI);
    write_reg(ADDR[b], MODE2, MODE2_OUTDRV);
    Serial.printf("[servo] PCA9685 0x%02X up, prescale %u (%.1f Hz)\n",
                  ADDR[b], prescale, PWM_HZ);
  }

  relax();  // boot dark - nothing moves until enable(true)
  load_calibration();
  return ready_;
}

void ServoBus::write_pulse(uint8_t board, uint8_t channel, uint16_t us) {
  // 4096 counts span one period; at 50 Hz that is 20000 us, so 0.2048 counts/us.
  const uint16_t counts =
      us == 0 ? 0 : (uint16_t)(us * 4096.0f * PWM_HZ / 1000000.0f);
  const uint8_t reg = LED0_ON_L + 4 * channel;
  Wire.beginTransmission(ADDR[board]);
  Wire.write(reg);
  Wire.write(0x00);              // ON_L  - rising edge at count 0
  Wire.write(0x00);              // ON_H
  Wire.write(counts & 0xFF);     // OFF_L
  Wire.write((counts >> 8) & 0x0F);  // OFF_H
  Wire.endTransmission();
}

void ServoBus::set_joint(int joint, float angle_rad) {
  if (joint < 0 || joint >= NUM_JOINTS) return;

  const int which = joint % 3;  // 0 coxa, 1 femur, 2 tibia
  const float d2r = kPi / 180.0f;

  // Joint angle -> servo angle, measured from the servo's mechanical centre.
  float servo_deg = (angle_rad - CENTER_DEG[which] * d2r) / d2r;

  clamped_[joint] = fabsf(servo_deg) > TRAVEL_DEG;
  servo_deg = clampf(servo_deg, -TRAVEL_DEG, TRAVEL_DEG);

  // Mirror, then convert to a pulse. (MAX-MIN)/RANGE us per degree.
  const float us_per_deg =
      (float)(SERVO_MAX_US - SERVO_MIN_US) / SERVO_RANGE_DEG;
  float us = (float)SERVO_CENTER_US + trim_us_[joint] +
             dir_[joint] * servo_deg * us_per_deg;
  us = clampf(us, (float)SERVO_MIN_US, (float)SERVO_MAX_US);

  last_us_[joint] = (uint16_t)us;
  if (!enabled_ || !ready_) return;
  write_pulse(SERVOS[joint].board, SERVOS[joint].channel, last_us_[joint]);
}

void ServoBus::set_leg(int leg, const Vec3& a) {
  set_joint(3 * leg + 0, a.x);
  set_joint(3 * leg + 1, a.y);
  set_joint(3 * leg + 2, a.z);
}

void ServoBus::center_all() {
  const float d2r = kPi / 180.0f;
  for (int j = 0; j < NUM_JOINTS; ++j) {
    set_joint(j, CENTER_DEG[j % 3] * d2r);
  }
}

void ServoBus::relax() {
  enabled_ = false;
  if (!ready_) return;
  // ALL_LED_OFF_H bit 4 turns every output fully off in one write per board.
  for (int b = 0; b < 2; ++b) {
    Wire.beginTransmission(ADDR[b]);
    Wire.write(ALL_LED_ON_L);
    Wire.write(0x00);
    Wire.write(0x00);
    Wire.write(0x00);
    Wire.write(0x10);
    Wire.endTransmission();
  }
}

void ServoBus::enable(bool on) {
  if (!on) {
    relax();
    return;
  }
  enabled_ = true;
}

void ServoBus::set_trim(int joint, int16_t us) {
  if (joint >= 0 && joint < NUM_JOINTS) trim_us_[joint] = us;
}

void ServoBus::set_dir(int joint, int8_t d) {
  if (joint >= 0 && joint < NUM_JOINTS) dir_[joint] = d >= 0 ? 1 : -1;
}

bool ServoBus::save_calibration() {
  Preferences p;
  if (!p.begin(NVS_NAMESPACE, false)) return false;
  p.putBytes("trim", trim_us_, sizeof(trim_us_));
  p.putBytes("dir", dir_, sizeof(dir_));
  p.end();
  Serial.println("[servo] calibration saved to NVS");
  return true;
}

bool ServoBus::load_calibration() {
  Preferences p;
  if (!p.begin(NVS_NAMESPACE, true)) return false;
  const bool have = p.getBytesLength("trim") == sizeof(trim_us_);
  if (have) {
    p.getBytes("trim", trim_us_, sizeof(trim_us_));
    p.getBytes("dir", dir_, sizeof(dir_));
    Serial.println("[servo] calibration loaded from NVS");
  } else {
    Serial.println("[servo] no saved calibration - using generated defaults");
  }
  p.end();
  return have;
}

}  // namespace robot

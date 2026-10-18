// drive_core: protocol framing and the drive safety state machine, free of Arduino dependencies so it can be
// unit-tested on a PC (pio test -e native). The framing matches the rover's planned CAN-master serial link:
// version | type | seq | payload | crc16-le, COBS-encoded, terminated by 0x00.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace drive {

constexpr uint8_t kProtocolVersion = 1;
constexpr uint8_t kTypeDriveStatus = 0x04;   // ESP32 -> PC
constexpr uint8_t kTypeDriveCommand = 0x82;  // PC -> ESP32
constexpr uint8_t kFlagEstop = 0x01;
constexpr size_t kMaxPacket = 64;  // decoded bytes: header + payload + CRC
constexpr size_t kMaxWire = kMaxPacket + 2;  // COBS overhead + delimiter
constexpr int kWheels = 4;  // order everywhere: FL, FR, RL, RR
constexpr size_t kCommandPayload = 9;
constexpr size_t kStatusPayload = 26;

// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF; crc16("123456789") == 0x29B1.
uint16_t crc16(const uint8_t* data, size_t len);
// Returns the encoded length; `out` needs len + len / 254 + 1 bytes. The result never contains 0x00.
size_t cobs_encode(const uint8_t* in, size_t len, uint8_t* out);
// Returns the decoded length, or 0 if `in` is not valid COBS.
size_t cobs_decode(const uint8_t* in, size_t len, uint8_t* out);
// Builds the complete wire bytes (including the 0x00 delimiter) into `out` (kMaxWire bytes). Returns 0 if too long.
size_t encode_packet(uint8_t type, uint8_t seq, const uint8_t* payload, size_t payload_len, uint8_t* out);

struct Packet {
  uint8_t type;
  uint8_t seq;
  uint8_t payload[kMaxPacket];
  size_t length;
};

// Byte-at-a-time decoder. Drops anything that fails COBS, length, CRC or version checks and resynchronises on 0x00.
class PacketDecoder {
 public:
  // Returns true when `packet` now holds a complete, valid packet.
  bool feed(uint8_t byte, Packet& packet);
  uint16_t good() const { return good_; }
  uint16_t bad() const { return bad_; }

 private:
  uint8_t buffer_[kMaxWire];
  size_t length_ = 0;
  bool overflow_ = false;
  bool first_ = true;  // the first block after power-up may be the tail of a packet joined mid-stream
  uint16_t good_ = 0;
  uint16_t bad_ = 0;
};

struct DriveCommand {
  bool estop;
  int16_t target_decirpm[kWheels];  // wheel targets in 0.1 rpm, positive = forward
};
bool parse_drive_command(const Packet& packet, DriveCommand& command);

enum class State : uint8_t {
  Boot = 0,         // no command received since power-up; motors off
  NoCommand = 1,    // no valid command for longer than the timeout; motors off
  Estop = 2,        // the PC sent the e-stop flag; motors off
  WaitNeutral = 3,  // commands arriving but not all zero yet after a stop; motors off
  Driving = 4,      // applying wheel targets
};

struct ControllerConfig {
  float max_wheel_rpm;          // open-loop calibration: PWM 255 drives the wheel at this rpm
  uint32_t command_timeout_ms;  // stop if no valid command for this long
  float slew_pwm_per_s;         // max PWM change per second while driving (0 = unlimited); stops are immediate
};

// Safety state machine and open-loop rpm -> PWM mapping. Never drives after power-up, a timeout or an e-stop until
// a neutral (all-zero, no e-stop) command has been received, so the rover cannot lurch when commands resume.
class DriveController {
 public:
  explicit DriveController(const ControllerConfig& config) : config_(config) {}

  void on_command(const DriveCommand& command, uint32_t now_ms);
  // Call every control period: applies the command timeout and the slew limit.
  void update(uint32_t now_ms);

  State state() const { return state_; }
  int16_t pwm(int wheel) const { return pwm_[wheel]; }  // -255..255, positive = forward
  uint32_t command_age_ms(uint32_t now_ms) const;
  bool motors_enabled() const { return state_ == State::Driving; }

 private:
  void stop(State reason);
  int16_t target_pwm(int wheel) const;

  ControllerConfig config_;
  State state_ = State::Boot;
  int16_t target_decirpm_[kWheels] = {0, 0, 0, 0};
  int16_t pwm_[kWheels] = {0, 0, 0, 0};
  float pwm_exact_[kWheels] = {0, 0, 0, 0};
  bool have_command_ = false;
  uint32_t last_command_ms_ = 0;
  uint32_t last_update_ms_ = 0;
  bool have_update_ = false;
};

struct StatusInfo {
  State state;
  uint16_t command_age_ms;
  int16_t pwm[kWheels];
  uint16_t rx_good;
  uint16_t rx_bad;
  uint32_t uptime_ms;
  uint8_t reset_reason;
  uint16_t max_wheel_rpm_x10;
  uint8_t version[3];
};
// Writes the DRIVE_STATUS payload (kStatusPayload bytes) into `out`.
size_t encode_status(const StatusInfo& status, uint8_t* out);

}  // namespace drive

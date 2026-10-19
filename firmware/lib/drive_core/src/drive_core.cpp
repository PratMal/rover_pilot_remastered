#include "drive_core.h"

#include <math.h>
#include <string.h>

namespace drive {

namespace {

void put_u16(uint8_t* out, uint16_t value) {
  out[0] = static_cast<uint8_t>(value & 0xFF);
  out[1] = static_cast<uint8_t>(value >> 8);
}

void put_u32(uint8_t* out, uint32_t value) {
  put_u16(out, static_cast<uint16_t>(value & 0xFFFF));
  put_u16(out + 2, static_cast<uint16_t>(value >> 16));
}

int16_t get_i16(const uint8_t* in) {
  return static_cast<int16_t>(static_cast<uint16_t>(in[0]) | (static_cast<uint16_t>(in[1]) << 8));
}

// Milliseconds from `then` to `now`, wraparound-safe; 0 if `then` is (slightly) after `now`.
uint32_t elapsed_ms(uint32_t now, uint32_t then) {
  const int32_t delta = static_cast<int32_t>(now - then);
  return delta > 0 ? static_cast<uint32_t>(delta) : 0;
}

}  // namespace

uint16_t crc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int bit = 0; bit < 8; bit++) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

size_t cobs_encode(const uint8_t* in, size_t len, uint8_t* out) {
  size_t read = 0, write = 1, code_index = 0;
  uint8_t code = 1;
  while (read < len) {
    if (in[read] == 0) {
      out[code_index] = code;
      code = 1;
      code_index = write++;
      read++;
    } else {
      out[write++] = in[read++];
      if (++code == 0xFF) {
        out[code_index] = code;
        code = 1;
        code_index = write++;
      }
    }
  }
  out[code_index] = code;
  return write;
}

size_t cobs_decode(const uint8_t* in, size_t len, uint8_t* out) {
  size_t read = 0, write = 0;
  while (read < len) {
    uint8_t code = in[read++];
    if (code == 0 || read + code - 1 > len) return 0;
    for (uint8_t i = 1; i < code; i++) {
      if (in[read] == 0) return 0;
      out[write++] = in[read++];
    }
    if (code != 0xFF && read < len) out[write++] = 0;
  }
  return write;
}

size_t encode_packet(uint8_t type, uint8_t seq, const uint8_t* payload, size_t payload_len, uint8_t* out) {
  if (payload_len + 5 > kMaxPacket) return 0;
  uint8_t body[kMaxPacket];
  body[0] = kProtocolVersion;
  body[1] = type;
  body[2] = seq;
  memcpy(body + 3, payload, payload_len);
  const size_t body_len = 3 + payload_len;
  put_u16(body + body_len, crc16(body, body_len));
  const size_t encoded = cobs_encode(body, body_len + 2, out);
  out[encoded] = 0x00;
  return encoded + 1;
}

bool PacketDecoder::feed(uint8_t byte, Packet& packet) {
  if (byte != 0) {
    if (length_ < sizeof(buffer_)) {
      buffer_[length_++] = byte;
    } else {
      overflow_ = true;
    }
    return false;
  }
  const size_t length = length_;
  const bool overflow = overflow_;
  const bool first = first_;
  length_ = 0;
  overflow_ = false;
  first_ = false;
  if (length == 0) return false;

  uint8_t decoded[kMaxWire];
  const size_t decoded_len = overflow ? 0 : cobs_decode(buffer_, length, decoded);
  bool valid = decoded_len >= 5 && decoded_len <= kMaxPacket;
  if (valid) {
    const uint16_t crc = static_cast<uint16_t>(decoded[decoded_len - 2]) | (static_cast<uint16_t>(decoded[decoded_len - 1]) << 8);
    valid = crc16(decoded, decoded_len - 2) == crc && decoded[0] == kProtocolVersion;
  }
  if (!valid) {
    if (!first && bad_ < 0xFFFF) bad_++;
    return false;
  }
  if (good_ < 0xFFFF) good_++;
  packet.type = decoded[1];
  packet.seq = decoded[2];
  packet.length = decoded_len - 5;
  memcpy(packet.payload, decoded + 3, packet.length);
  return true;
}

bool parse_drive_command(const Packet& packet, DriveCommand& command) {
  if (packet.type != kTypeDriveCommand || packet.length != kCommandPayload) return false;
  if (packet.payload[0] & ~kFlagEstop) return false;  // reserved flag bits must be zero
  command.estop = (packet.payload[0] & kFlagEstop) != 0;
  for (int wheel = 0; wheel < kWheels; wheel++) command.target_decirpm[wheel] = get_i16(packet.payload + 1 + 2 * wheel);
  return true;
}

void DriveController::stop(State reason) {
  state_ = reason;
  for (int wheel = 0; wheel < kWheels; wheel++) {
    target_decirpm_[wheel] = 0;
    pwm_[wheel] = 0;
    pwm_exact_[wheel] = 0;
  }
}

void DriveController::on_command(const DriveCommand& command, uint32_t now_ms) {
  have_command_ = true;
  last_command_ms_ = now_ms;
  if (command.estop) {
    stop(State::Estop);
    return;
  }
  bool neutral = true;
  for (int wheel = 0; wheel < kWheels; wheel++) neutral = neutral && command.target_decirpm[wheel] == 0;
  if (state_ != State::Driving) {
    // After power-up, a timeout or an e-stop, motion resumes only after a neutral command.
    if (!neutral) {
      state_ = State::WaitNeutral;
      return;
    }
    state_ = State::Driving;
  }
  for (int wheel = 0; wheel < kWheels; wheel++) target_decirpm_[wheel] = command.target_decirpm[wheel];
}

int16_t DriveController::target_pwm(int wheel) const {
  if (config_.max_wheel_rpm <= 0) return 0;
  float pwm = (target_decirpm_[wheel] / 10.0f) / config_.max_wheel_rpm * 255.0f;
  if (pwm > 255.0f) pwm = 255.0f;
  if (pwm < -255.0f) pwm = -255.0f;
  return static_cast<int16_t>(lroundf(pwm));
}

void DriveController::update(uint32_t now_ms) {
  const uint32_t dt_ms = have_update_ ? elapsed_ms(now_ms, last_update_ms_) : 0;
  last_update_ms_ = now_ms;
  have_update_ = true;

  if ((state_ == State::Driving || state_ == State::WaitNeutral) && elapsed_ms(now_ms, last_command_ms_) > config_.command_timeout_ms) {
    stop(State::NoCommand);
  }
  if (state_ != State::Driving) return;

  const float max_step = config_.slew_pwm_per_s > 0 ? config_.slew_pwm_per_s * dt_ms / 1000.0f : 1e9f;
  for (int wheel = 0; wheel < kWheels; wheel++) {
    const float target = target_pwm(wheel);
    float next = pwm_exact_[wheel];
    if (target > next) next = (target - next > max_step) ? next + max_step : target;
    else if (target < next) next = (next - target > max_step) ? next - max_step : target;
    pwm_exact_[wheel] = next;
    pwm_[wheel] = static_cast<int16_t>(lroundf(next));
  }
}

uint32_t DriveController::command_age_ms(uint32_t now_ms) const {
  return have_command_ ? elapsed_ms(now_ms, last_command_ms_) : 0xFFFFFFFFu;
}

size_t encode_status(const StatusInfo& status, uint8_t* out) {
  out[0] = static_cast<uint8_t>(status.state);
  out[1] = 0;  // reserved flags
  put_u16(out + 2, status.command_age_ms);
  for (int wheel = 0; wheel < kWheels; wheel++) put_u16(out + 4 + 2 * wheel, static_cast<uint16_t>(status.pwm[wheel]));
  put_u16(out + 12, status.rx_good);
  put_u16(out + 14, status.rx_bad);
  put_u32(out + 16, status.uptime_ms);
  out[20] = status.reset_reason;
  put_u16(out + 21, status.max_wheel_rpm_x10);
  out[23] = status.version[0];
  out[24] = status.version[1];
  out[25] = status.version[2];
  return kStatusPayload;
}

}  // namespace drive

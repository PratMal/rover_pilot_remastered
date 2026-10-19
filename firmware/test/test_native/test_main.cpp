// Native unit tests for drive_core: pio test -e native
#include <string.h>
#include <unity.h>

#include "drive_core.h"
#include "vectors.h"

using drive::DriveCommand;
using drive::DriveController;
using drive::Packet;
using drive::PacketDecoder;
using drive::State;

static const drive::ControllerConfig kConfig = {94.7f, 300, 0.0f};

static DriveCommand command(bool estop, int16_t fl, int16_t fr, int16_t rl, int16_t rr) {
  DriveCommand result = {estop, {fl, fr, rl, rr}};
  return result;
}

static bool decode_all(PacketDecoder& decoder, const uint8_t* bytes, size_t len, Packet& packet) {
  bool got = false;
  for (size_t i = 0; i < len; i++) got = decoder.feed(bytes[i], packet) || got;
  return got;
}

void setUp() {}
void tearDown() {}

void test_crc_check_value() {
  TEST_ASSERT_EQUAL_HEX16(kCrcCheck, drive::crc16(reinterpret_cast<const uint8_t*>("123456789"), 9));
  TEST_ASSERT_EQUAL_HEX16(0x29B1, kCrcCheck);
}

void test_cobs_round_trip_with_zero_runs() {
  uint8_t data[300], encoded[310], decoded[310];
  for (size_t len = 0; len < sizeof(data); len++) {
    for (size_t i = 0; i < len; i++) data[i] = (i % 7 == 0) ? 0 : static_cast<uint8_t>(i * 13);
    const size_t encoded_len = drive::cobs_encode(data, len, encoded);
    for (size_t i = 0; i < encoded_len; i++) TEST_ASSERT_NOT_EQUAL(0, encoded[i]);
    if (len == 0) continue;
    TEST_ASSERT_EQUAL(len, drive::cobs_decode(encoded, encoded_len, decoded));
    TEST_ASSERT_EQUAL_MEMORY(data, decoded, len);
  }
}

void test_encodes_commands_exactly_like_the_python_reference() {
  for (const CommandVector& vector : kCommandVectors) {
    DriveCommand cmd = {vector.estop, {0, 0, 0, 0}};
    uint8_t payload[drive::kCommandPayload];
    payload[0] = vector.estop ? drive::kFlagEstop : 0;
    for (int wheel = 0; wheel < drive::kWheels; wheel++) {
      const int16_t raw = static_cast<int16_t>(vector.targets[wheel] * 10.0f + (vector.targets[wheel] >= 0 ? 0.5f : -0.5f));
      cmd.target_decirpm[wheel] = raw;
      payload[1 + 2 * wheel] = static_cast<uint8_t>(raw & 0xFF);
      payload[2 + 2 * wheel] = static_cast<uint8_t>((static_cast<uint16_t>(raw) >> 8) & 0xFF);
    }
    uint8_t wire[drive::kMaxWire];
    const size_t wire_len = drive::encode_packet(drive::kTypeDriveCommand, vector.seq, payload, sizeof(payload), wire);
    TEST_ASSERT_EQUAL_MESSAGE(vector.wire_len, wire_len, vector.name);
    TEST_ASSERT_EQUAL_MEMORY_MESSAGE(vector.wire, wire, wire_len, vector.name);
  }
}

void test_decodes_python_commands() {
  PacketDecoder decoder;
  Packet packet;
  const uint8_t zero = 0;
  decoder.feed(zero, packet);  // leading delimiter: in sync
  for (const CommandVector& vector : kCommandVectors) {
    TEST_ASSERT_TRUE_MESSAGE(decode_all(decoder, vector.wire, vector.wire_len, packet), vector.name);
    DriveCommand cmd;
    TEST_ASSERT_TRUE(drive::parse_drive_command(packet, cmd));
    TEST_ASSERT_EQUAL(vector.estop, cmd.estop);
    TEST_ASSERT_EQUAL(vector.seq, packet.seq);
    for (int wheel = 0; wheel < drive::kWheels; wheel++) TEST_ASSERT_FLOAT_WITHIN(0.051f, vector.targets[wheel], cmd.target_decirpm[wheel] / 10.0f);
  }
  TEST_ASSERT_EQUAL(0, decoder.bad());
}

void test_status_encoding_matches_python() {
  drive::StatusInfo status = {State::Driving, 20, {128, -255, 0, 255}, 1234, 2, 600000, 1, 947, {2, 0, 0}};
  uint8_t payload[drive::kStatusPayload];
  TEST_ASSERT_EQUAL(drive::kStatusPayload, drive::encode_status(status, payload));
  uint8_t wire[drive::kMaxWire];
  const size_t wire_len = drive::encode_packet(drive::kTypeDriveStatus, kStatusSeq, payload, sizeof(payload), wire);
  TEST_ASSERT_EQUAL(sizeof(kStatusWire), wire_len);
  TEST_ASSERT_EQUAL_MEMORY(kStatusWire, wire, wire_len);
}

void test_garbage_and_corruption_never_produce_a_command() {
  PacketDecoder decoder;
  Packet packet;
  const uint8_t garbage[] = {0x13, 0x37, 0x00, 0x01, 0x02, 0x00, 0xFF, 0xFF, 0xFF, 0x00};
  TEST_ASSERT_FALSE(decode_all(decoder, garbage, sizeof(garbage), packet));
  uint8_t corrupted[sizeof(kWire_forward_full)];
  memcpy(corrupted, kWire_forward_full, sizeof(corrupted));
  corrupted[6] ^= 0x10;  // flip a bit inside the targets
  TEST_ASSERT_FALSE(decode_all(decoder, corrupted, sizeof(corrupted), packet));
  // Half a packet followed by a full one: only the full one decodes.
  TEST_ASSERT_FALSE(decode_all(decoder, kWire_forward_full, 5, packet));
  const uint8_t zero = 0;
  decoder.feed(zero, packet);
  TEST_ASSERT_TRUE(decode_all(decoder, kWire_neutral, sizeof(kWire_neutral), packet));
  TEST_ASSERT_TRUE(decoder.bad() >= 3);
  // An oversized block is dropped without overflowing the buffer.
  uint8_t long_block[200];
  memset(long_block, 0x41, sizeof(long_block));
  TEST_ASSERT_FALSE(decode_all(decoder, long_block, sizeof(long_block), packet));
  TEST_ASSERT_FALSE(decoder.feed(zero, packet));
}

void test_reserved_flags_are_rejected() {
  Packet packet = {drive::kTypeDriveCommand, 0, {0x02, 0, 0, 0, 0, 0, 0, 0, 0}, drive::kCommandPayload};
  DriveCommand cmd;
  TEST_ASSERT_FALSE(drive::parse_drive_command(packet, cmd));
}

void test_boot_requires_a_neutral_command_before_driving() {
  DriveController controller(kConfig);
  controller.update(0);
  TEST_ASSERT_EQUAL(State::Boot, controller.state());
  controller.on_command(command(false, 500, 500, 500, 500), 10);
  controller.update(10);
  TEST_ASSERT_EQUAL(State::WaitNeutral, controller.state());
  TEST_ASSERT_EQUAL(0, controller.pwm(0));
  controller.on_command(command(false, 0, 0, 0, 0), 20);
  controller.on_command(command(false, 947, 947, -947, 473), 30);
  controller.update(30);
  TEST_ASSERT_EQUAL(State::Driving, controller.state());
  TEST_ASSERT_EQUAL(255, controller.pwm(0));
  TEST_ASSERT_EQUAL(-255, controller.pwm(2));
  TEST_ASSERT_INT_WITHIN(1, 127, controller.pwm(3));
}

void test_command_timeout_stops_and_requires_neutral_again() {
  DriveController controller(kConfig);
  controller.on_command(command(false, 0, 0, 0, 0), 0);
  controller.on_command(command(false, 600, 600, 600, 600), 10);
  controller.update(10);
  TEST_ASSERT_TRUE(controller.pwm(1) > 0);
  controller.update(310);
  TEST_ASSERT_EQUAL(State::Driving, controller.state());  // exactly at the limit: still fine
  controller.update(311);
  TEST_ASSERT_EQUAL(State::NoCommand, controller.state());
  for (int wheel = 0; wheel < drive::kWheels; wheel++) TEST_ASSERT_EQUAL(0, controller.pwm(wheel));
  controller.on_command(command(false, 600, 600, 600, 600), 400);  // link back, stick still pushed
  controller.update(400);
  TEST_ASSERT_EQUAL(State::WaitNeutral, controller.state());
  TEST_ASSERT_EQUAL(0, controller.pwm(0));
}

void test_estop_latches_until_released_with_neutral() {
  DriveController controller(kConfig);
  controller.on_command(command(false, 0, 0, 0, 0), 0);
  controller.on_command(command(false, 900, 900, 900, 900), 10);
  controller.update(10);
  controller.on_command(command(true, 900, 900, 900, 900), 20);
  controller.update(20);
  TEST_ASSERT_EQUAL(State::Estop, controller.state());
  TEST_ASSERT_EQUAL(0, controller.pwm(0));
  controller.on_command(command(false, 900, 900, 900, 900), 30);  // released while the stick is pushed
  TEST_ASSERT_EQUAL(State::WaitNeutral, controller.state());
  controller.on_command(command(false, 0, 0, 0, 0), 40);
  TEST_ASSERT_EQUAL(State::Driving, controller.state());
}

void test_estop_state_does_not_time_out_into_motion() {
  DriveController controller(kConfig);
  controller.on_command(command(true, 0, 0, 0, 0), 0);
  controller.update(5000);
  TEST_ASSERT_EQUAL(State::Estop, controller.state());
}

void test_slew_limit_ramps_up_but_stops_immediately() {
  const drive::ControllerConfig config = {94.7f, 300, 1020.0f};  // full scale in 250 ms
  DriveController controller(config);
  controller.on_command(command(false, 0, 0, 0, 0), 0);
  controller.update(0);
  controller.on_command(command(false, 947, 947, 947, 947), 0);
  controller.update(50);
  TEST_ASSERT_INT_WITHIN(1, 51, controller.pwm(0));
  controller.on_command(command(false, 947, 947, 947, 947), 100);
  controller.update(100);
  TEST_ASSERT_INT_WITHIN(1, 102, controller.pwm(0));
  controller.on_command(command(true, 0, 0, 0, 0), 110);
  TEST_ASSERT_EQUAL(0, controller.pwm(0));
}

void test_millis_wraparound_is_handled() {
  DriveController controller(kConfig);
  const uint32_t near_wrap = 0xFFFFFF00u;
  controller.on_command(command(false, 0, 0, 0, 0), near_wrap);
  controller.on_command(command(false, 300, 300, 300, 300), near_wrap + 10);
  controller.update(near_wrap + 10);
  controller.on_command(command(false, 300, 300, 300, 300), near_wrap + 200);  // wraps past zero
  controller.update(near_wrap + 300);
  TEST_ASSERT_EQUAL(State::Driving, controller.state());
}

void test_a_command_stamped_after_the_update_time_is_not_a_timeout() {
  // main.cpp once stamped commands with a later millis() than the control update in the same loop pass.
  DriveController controller(kConfig);
  controller.on_command(command(false, 0, 0, 0, 0), 1000);
  controller.on_command(command(false, 600, 600, 600, 600), 1001);
  controller.update(1000);
  TEST_ASSERT_EQUAL(State::Driving, controller.state());
  TEST_ASSERT_TRUE(controller.pwm(0) > 0);
  TEST_ASSERT_EQUAL_UINT32(0, controller.command_age_ms(1000));
  controller.update(1302);
  TEST_ASSERT_EQUAL(State::NoCommand, controller.state());  // a real timeout still stops
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_crc_check_value);
  RUN_TEST(test_cobs_round_trip_with_zero_runs);
  RUN_TEST(test_encodes_commands_exactly_like_the_python_reference);
  RUN_TEST(test_decodes_python_commands);
  RUN_TEST(test_status_encoding_matches_python);
  RUN_TEST(test_garbage_and_corruption_never_produce_a_command);
  RUN_TEST(test_reserved_flags_are_rejected);
  RUN_TEST(test_boot_requires_a_neutral_command_before_driving);
  RUN_TEST(test_command_timeout_stops_and_requires_neutral_again);
  RUN_TEST(test_estop_latches_until_released_with_neutral);
  RUN_TEST(test_estop_state_does_not_time_out_into_motion);
  RUN_TEST(test_slew_limit_ramps_up_but_stops_immediately);
  RUN_TEST(test_millis_wraparound_is_handled);
  RUN_TEST(test_a_command_stamped_after_the_update_time_is_not_a_timeout);
  return UNITY_END();
}

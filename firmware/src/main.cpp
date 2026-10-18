// Rover drive firmware: one ESP32 driving four wheel motors from DRIVE_COMMAND packets sent by the onboard PC.
//
// Safety behaviour (implemented and unit-tested in lib/drive_core):
//  - motors are off after power-up until a neutral (all-zero) command arrives;
//  - no valid command for COMMAND_TIMEOUT_MS -> motors off, and a neutral command is needed again;
//  - e-stop flag -> motors off and latched until commands without the flag arrive, starting with a neutral one;
//  - garbled, partial or corrupted input is discarded (COBS + CRC framing), never interpreted as a speed.
// Nothing but protocol packets is written to the serial line.
#include <Arduino.h>
#include <esp_system.h>

#include "config.h"
#include "drive_core.h"

namespace {

const drive::ControllerConfig kControllerConfig = {MAX_WHEEL_RPM, COMMAND_TIMEOUT_MS, SLEW_PWM_PER_S};

drive::DriveController controller(kControllerConfig);
drive::PacketDecoder decoder;
uint8_t tx_seq = 0;
uint32_t last_control_ms = 0;
uint32_t last_status_ms = 0;
uint8_t reset_reason = 0;

void setup_motor(const MotorPins& motor) {
  pinMode(motor.dir_pin, OUTPUT);
  pinMode(motor.pwm_pin, OUTPUT);
  digitalWrite(motor.dir_pin, motor.forward_level);
  analogWrite(motor.pwm_pin, 0);
}

void drive_motor(const MotorPins& motor, int16_t pwm) {
  const bool forward = pwm >= 0;
  digitalWrite(motor.dir_pin, forward ? motor.forward_level : !motor.forward_level);
  analogWrite(motor.pwm_pin, forward ? pwm : -pwm);
}

void apply_outputs() {
  for (int wheel = 0; wheel < drive::kWheels; wheel++) drive_motor(kDriveMotors[wheel], controller.pwm(wheel));
#if DRIVE_MIDDLE_WHEELS
  drive_motor(kMiddleMotors[0], (controller.pwm(0) + controller.pwm(2)) / 2);
  drive_motor(kMiddleMotors[1], (controller.pwm(1) + controller.pwm(3)) / 2);
#endif
}

void update_led(uint32_t now_ms) {
  bool on;
  switch (controller.state()) {
    case drive::State::Driving: on = true; break;
    case drive::State::Estop: on = (now_ms / 100) % 2 == 0; break;  // 5 Hz
    default: on = (now_ms / 500) % 2 == 0; break;                   // 1 Hz: waiting for commands / neutral
  }
  digitalWrite(STATUS_LED_PIN, on ? HIGH : LOW);
}

void send_status(uint32_t now_ms) {
  drive::StatusInfo status;
  status.state = controller.state();
  const uint32_t age = controller.command_age_ms(now_ms);
  status.command_age_ms = age > 0xFFFF ? 0xFFFF : static_cast<uint16_t>(age);
  for (int wheel = 0; wheel < drive::kWheels; wheel++) status.pwm[wheel] = controller.pwm(wheel);
  status.rx_good = decoder.good();
  status.rx_bad = decoder.bad();
  status.uptime_ms = now_ms;
  status.reset_reason = reset_reason;
  status.max_wheel_rpm_x10 = static_cast<uint16_t>(MAX_WHEEL_RPM * 10.0f + 0.5f);
  status.version[0] = FIRMWARE_VERSION_MAJOR;
  status.version[1] = FIRMWARE_VERSION_MINOR;
  status.version[2] = FIRMWARE_VERSION_PATCH;

  uint8_t payload[drive::kStatusPayload];
  drive::encode_status(status, payload);
  uint8_t wire[drive::kMaxWire];
  const size_t length = drive::encode_packet(drive::kTypeDriveStatus, tx_seq++, payload, sizeof(payload), wire);
  if (Serial.availableForWrite() >= static_cast<int>(length)) Serial.write(wire, length);  // never block the control loop
}

}  // namespace

void setup() {
  for (const MotorPins& motor : kDriveMotors) setup_motor(motor);
  for (const MotorPins& motor : kMiddleMotors) setup_motor(motor);
  pinMode(STATUS_LED_PIN, OUTPUT);
  reset_reason = static_cast<uint8_t>(esp_reset_reason());
  Serial.begin(SERIAL_BAUD);
  enableLoopWDT();  // a hung loop resets the ESP32, which boots with motors off
}

void loop() {
  drive::Packet packet;
  drive::DriveCommand command;
  const uint32_t now_ms = millis();

  while (Serial.available() > 0) {
    if (decoder.feed(static_cast<uint8_t>(Serial.read()), packet) && drive::parse_drive_command(packet, command)) {
      controller.on_command(command, millis());
      if (!controller.motors_enabled()) apply_outputs();  // stops take effect immediately
    }
  }

  if (now_ms - last_control_ms >= CONTROL_PERIOD_MS) {
    last_control_ms = now_ms;
    controller.update(now_ms);
    apply_outputs();
    update_led(now_ms);
  }
  if (now_ms - last_status_ms >= STATUS_PERIOD_MS) {
    last_status_ms = now_ms;
    send_status(now_ms);
  }
  delay(1);  // yield to the idle task (keeps the task watchdog quiet)
}

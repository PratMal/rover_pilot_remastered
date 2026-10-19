// Rover drive firmware configuration. Pins and directions match the original rover_pilot firmware.
#pragma once

#include <stdint.h>

#define FIRMWARE_VERSION_MAJOR 2
#define FIRMWARE_VERSION_MINOR 0
#define FIRMWARE_VERSION_PATCH 1

// Serial link to the onboard PC (USB). Must match the drive node's `baud` parameter.
#define SERIAL_BAUD 115200

// Open-loop calibration: PWM 255 drives a wheel at this many rpm. 94.7 rpm on a 0.28 m wheel is 5 km/h.
// Must match the drive node's `max_wheel_rpm` parameter (the node warns if they differ). It is a motor property, so it
// stays the same when the wheels are swapped; only the drive node's wheel diameter changes.
#define MAX_WHEEL_RPM 94.7f

// Safety and smoothness.
#define COMMAND_TIMEOUT_MS 300  // stop if no valid DRIVE_COMMAND for this long
#define SLEW_PWM_PER_S 1020.0f  // max PWM change per second while driving (255 in 250 ms); 0 = unlimited. Stops are immediate.

// Timing.
#define CONTROL_PERIOD_MS 10  // state machine + PWM update
#define STATUS_PERIOD_MS 100  // DRIVE_STATUS to the PC

// Status LED (on-board LED of the DOIT DevKit V1): solid = driving, fast blink = e-stop, slow blink = waiting.
#define STATUS_LED_PIN 2

// Motor drivers: one direction pin and one PWM pin each. FORWARD_LEVEL is the direction pin level that drives the
// wheel forward (from the original firmware: front/back wheels HIGH = forward, middle wheels LOW = forward).
struct MotorPins {
  uint8_t dir_pin;
  uint8_t pwm_pin;
  uint8_t forward_level;
};

// Wheel order everywhere: FL, FR, RL, RR.
// Note: GPIO 12 (left-back direction) is an ESP32 strapping pin. If the motor driver pulls it high at power-up the
// ESP32 may fail to boot; move it to a free non-strapping GPIO if that happens.
static const MotorPins kDriveMotors[4] = {
    {25, 26, 1},  // FL  (leftFront  {dir, pwm})
    {22, 23, 1},  // FR  (rightFront)
    {12, 13, 1},  // RL  (leftBack)
    {18, 19, 1},  // RR  (rightBack)
};

// Middle wheels of the 6-wheel chassis. Not driven on this rover; their pins are held at PWM 0.
// Set DRIVE_MIDDLE_WHEELS to 1 to drive them with the average of their side's front and rear wheels.
#define DRIVE_MIDDLE_WHEELS 0
static const MotorPins kMiddleMotors[2] = {
    {14, 27, 0},  // left middle
    {32, 33, 0},  // right middle
};

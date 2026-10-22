# rover_pilot_remastered

Safe drive control for the current rover: **one ESP32 driving four wheel motors**, a PS5 controller at the base station, ROS 2 Humble over the MikroTik link. It replaces the old `rover_pilot` drive path (`drive_node` + `ctrl.py` + the single-ESP32 firmware).

```text
BASE STATION LAPTOP                                    ROVER LAPTOP (192.168.1.68)                     DRIVE ESP32
PS5 ─▶ joy/joy_node ─/joy─▶ base_teleop ─┬─ /cmd_vel (20 Hz) ──────▶ drive_node ─USB serial─▶ firmware ─▶ 4 motor drivers
                                                      └─ /rover/estop (10 Hz) ──▶            DRIVE_COMMAND 50 Hz
                                                                                             ◀── DRIVE_STATUS 10 Hz
```

`/cmd_vel` is the standard ROS velocity topic (m/s, rad/s), so Nav2 can drive the rover through the same path later.

## What was wrong before, and what this fixes

| Old behaviour | Now |
|---|---|
| The ESP32 kept the last command forever: if `drive_node` died or the radio link dropped, the rover kept driving at the last PWM | The firmware stops the motors when no valid command has arrived for **300 ms**. `drive_node` sends 50 times a second whether or not the joystick moves |
| The firmware read 2 bytes after seeing 1; a missing byte read as −1 → **255 (full speed)**, and one lost byte swapped command and speed forever | Commands are framed (COBS + CRC-16); anything incomplete or corrupted is dropped and can never become a speed |
| `find_port()` opened every USB serial port to look for the text "drive", which can reset ESP32s | The ESP32 has a fixed name (`/dev/rover-drive`, via udev); nothing scans ports |
| 5 discrete commands, and turning overrode driving (no arcs), no deadzone | Proportional arcade drive: speed and turn combine; deadzone; slow mode |
| No software stop | Latched joystick e-stop with a heartbeat; see below. The physical kill switch is still the hard stop |
| The firmware printed text on the same serial line; its control loop never yielded (watchdog warnings) | Only protocol packets on the line; non-blocking loop with a watchdog |

## Controls (PS5)

| Control | Action |
|---|---|
| Left stick up / down | Forward / backward (proportional) |
| Right stick left / right | Turn left / right (proportional; combines with the left stick for arcs; alone it spins in place) |
| **L1** (hold) | Slow mode: 35 % of normal speed |
| **Circle** | **E-stop**: latches immediately, the rover stops |
| **Options** | Release the e-stop: only works with **both sticks centred** |

The e-stop starts **latched**. After launching, centre the sticks and press Options to start driving.

**Connect the PS5 by USB cable at the base station.** The "controller lost" stop below relies on `joy` noticing the disconnect, which a USB unplug triggers at once. Over Bluetooth, a controller whose battery dies or that goes out of range can stay "connected" for several seconds while `joy` keeps repeating the last stick position, so the rover keeps moving until Linux gives up on it. If you must use Bluetooth, keep the controller charged and close to the base station, and remember Circle and the kill switch.

## Safety behaviour

| Situation | What happens | Time to stop | To drive again |
|---|---|---|---|
| Circle pressed | E-stop latched at the base station, sent to the rover | next command (≤ 20 ms) | Options with sticks centred |
| PS5 unplugged (USB) / `joy` node dies | `base_teleop` latches the e-stop | ≤ 0.6 s (0.5 s joystick timeout + one tick) | reconnect, Options with sticks centred |
| `base_teleop` stopped with Ctrl-C | It sends a final e-stop before exiting | next command (≤ 20 ms) | restart, Options with sticks centred |
| Base station laptop dies or **radio link lost** | E-stop heartbeat goes stale on the rover; `drive_node` sends the e-stop flag | about 0.5 s | link back, Options with sticks centred |
| `drive_node` killed / crashes, rover laptop sleeps | No commands reach the ESP32; firmware timeout | ≤ 0.3 s | restart `drive_node`; the ESP32 waits for centred sticks |
| USB cable unplugged | Same as above | ≤ 0.3 s | replug; the ESP32 waits for centred sticks |
| ESP32 resets or powers up | Motors off at boot | immediate | the ESP32 waits for centred sticks |
| ESP32 firmware hangs | The loop watchdog resets it; motors off at boot | ≤ 1 s | the ESP32 waits for centred sticks |
| Commands resume while a stick is pushed | The ESP32 stays stopped (`WAIT_NEUTRAL`) | — | centre the sticks |
| Kill switch | Battery cut | immediate | switch on, then as above |

The rule behind "centred sticks": after any stop, the ESP32 will not move again until it has received an all-zero command. This prevents a lurch when the link comes back with a stick still pushed. The status LED shows the state: **solid** = driving, **fast blink** = e-stop, **slow blink** = waiting for commands or centred sticks.

Kill-switch note: on the Windows/Ubuntu laptop the ESP32 is powered from the laptop's USB, so it stays on when the kill switch cuts the battery. Its motors lose power, but it keeps following commands. **Centre the sticks (or press Circle) before turning the kill switch back on**, or the wheels will resume whatever the stick says. On the Pi 4 / Jetson everything powers off together and the boot interlock covers it.

## Layout

```text
firmware/                      PlatformIO project for the drive ESP32 (DOIT ESP32 DevKit V1, Arduino)
  include/config.h             pins, directions, timeout, speed calibration, slew rate
  lib/drive_core/              protocol + safety state machine (plain C++, unit-tested on the PC)
  src/main.cpp                 ESP32 glue: serial, PWM, LED, watchdog
  test/test_native/            unit tests: pio test -e native
ros2_ws/src/rover_pilot_remastered/   ROS 2 Humble package (Python)
  rover_pilot_remastered/      base_teleop, drive_node, fake_esp (simulator), protocol, kinematics, logic
  config/drive.yaml            shared parameters (both machines)
  config/wheels/*.yaml         one file per wheel type
  launch/                      base_station.launch.py, rover.launch.py
  test/                        pytest (protocol, kinematics, e-stop logic, end-to-end, ROS integration)
tools/gen_test_vectors.py      generates byte-exact test vectors shared by the C++ and Python tests
tools/wheel_test.py            ROS-free wheel identification test for the bench
tools/rehearsal.py             dress rehearsal of the bench test against the simulated ESP32 and PS5
udev/99-rover-drive.rules      stable /dev/rover-drive name for the ESP32
```

## Setup

### 1. Flash the firmware

With PlatformIO (VS Code extension or CLI), from `firmware/`:

```bash
pio test -e native                          # optional: unit tests on your computer
pio run -e esp32doit-devkit-v1 -t upload    # build and flash over USB
```

It builds on both Arduino-ESP32 core 3.x (pioarduino) and 2.x (`espressif32@6.x`). Settings are in `include/config.h`: pins and directions are copied from the old firmware, and the defaults are a 300 ms timeout, `MAX_WHEEL_RPM` 94.7, a 250 ms ramp to full speed and a 1 s loop watchdog (Arduino core 3.x; core 2.x keeps its 5 s default).

GPIO 12 (left-back direction) is an ESP32 **strapping pin**. If the board ever fails to boot with the motor drivers connected, move that signal to a free non-strapping GPIO and update `config.h`.

The serial line is binary now, so a normal serial monitor shows garbage. Use `tools/wheel_test.py` or the drive node's logs instead.

### 2. Rover laptop (Ubuntu 22.04 + ROS 2 Humble)

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp ros-humble-diagnostic-msgs python3-serial
cp -r rover_pilot_remastered ~/   # or git clone
cd ~/rover_pilot_remastered/ros2_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install
```

Serial port name (do this once, with only the drive ESP32 plugged in):

```bash
udevadm info -a -n /dev/ttyUSB0 | grep '{serial}'      # copy the value into udev/99-rover-drive.rules
sudo cp ~/rover_pilot_remastered/udev/99-rover-drive.rules /etc/udev/rules.d/
sudo udevadm control --reload && sudo udevadm trigger && ls -l /dev/rover-drive
sudo usermod -aG dialout $USER                          # then log out and in
```

If the board uses a CH340 (no unique serial), use the commented `KERNELS==` line in the rule instead: it matches the physical USB port (`udevadm info -a -n /dev/ttyUSB0 | grep KERNELS`). Until the rule is installed you can pass `port:=/dev/ttyUSB0` to the launch file.

### 3. Base station laptop (Ubuntu 22.04 + ROS 2 Humble)

```bash
sudo apt install ros-humble-joy ros-humble-rmw-cyclonedds-cpp
# copy or clone the folder, then build the same package:
cd ~/rover_pilot_remastered/ros2_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install
```

Connect the PS5 controller (USB, or Bluetooth pairing in Ubuntu settings). Check the mapping once:

```bash
ros2 run joy joy_node &          # then: ros2 topic echo /joy
```

- Push the left stick **up**: the `axes` index that goes to **+1.0** is `axis_linear`.
- Push the right stick **left**: the `axes` index that goes to **+1.0** is `axis_angular`.
- Press the buttons you want for e-stop, release and slow mode: their `buttons` indices are `estop_button`, `release_button`, `slow_button`.

Put them in `config/drive.yaml` (base_teleop section; `invert_*` if a direction is reversed), commit and push, then pull and rebuild on the base station.

### 4. Every terminal on both machines

Add to `~/.bashrc` on the base station **and** the rover (same values everywhere):

```bash
source /opt/ros/humble/setup.bash
source ~/rover_pilot_remastered/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=<your team's domain id>
```

Check the link: run `ros2 topic list` on one machine while a node runs on the other.

**Rebuild after every change.** `ros2 launch` runs the installed copy of the package. With some setuptools versions `--symlink-install` still copies the files, so an edited `config/drive.yaml`, a wheel profile or updated code takes effect only after `cd ~/rover_pilot_remastered/ros2_ws && colcon build --symlink-install` (about a second) and a relaunch. `python3 tools/rehearsal.py` checks this first.

## Daily workflow

1. Kill switch on; the rover laptop boots.
2. Base station: `ros2 launch rover_pilot_remastered base_station.launch.py`
3. `ssh` to the rover (192.168.1.68): `ros2 launch rover_pilot_remastered rover.launch.py`
4. Centre both sticks and press **Options**. Drive.
5. Stop with **Circle**; in an emergency, the kill switch.

Launch arguments:
- `wheels:=wheel_28cm` (default) or `wheels:=wheel_other`, on **both** machines;
- `port:=/dev/ttyUSB0` on the rover if the udev rule isn't installed;
- `device_id:=1` at the base station if several controllers are connected.

The rover log shows every state change (`stopped: e-stop`, `driving enabled`, `ESP32 state: DRIVING`). `ros2 topic echo /diagnostics` shows the ESP32 status, PWM per wheel and link counters.

## First test on the bench (wheels off the ground!)

1. **Wheel identity and direction**, without ROS (stop `drive_node` first):
   ```bash
   python3 ~/rover_pilot_remastered/tools/wheel_test.py /dev/rover-drive
   ```
   It sends neutral until the ESP32 is ready (opening the port may reset it), then spins FL, FR, RL and RR forward one at a time, then all forward, then a right turn in place. Each line shows the firmware version: it must say **fw 2.0.2**, otherwise the new firmware is not flashed. Check that each named wheel turns **forward**. If one is wrong, fix its entry (pins or `forward_level`) in `firmware/include/config.h` and reflash.

   **Expect left and right to be swapped.** The pin names in `config.h` come from the old firmware, whose "left turn" drove the `leftFront`/`leftBack` pins forward. If the old rover really turned left that way, those pins drive the **right** wheels. If "FL forward only" spins the front-right wheel, swap the left and right entries in `kDriveMotors` (FL↔FR, RL↔RR) and reflash, rather than using `swap_sides`, so FL/FR/RL/RR mean the same everywhere.
2. **Joystick:**
   - Left stick up = forward.
   - Right stick right = rover turns **right**. If it turns left while forward is correct, set `swap_sides: true` in `config/drive.yaml` (left/right motors wired the other way round). Don't invert the joystick axis: `/cmd_vel` must stay correct for Nav2.
   - L1 = slower.
3. **Stops**; each must stop the wheels within the time in the safety table:
   - Circle (the rover stops within about 0.1 s);
   - pressing Options with a stick pushed (must be refused);
   - unplugging the PS5;
   - `Ctrl-C` on the base station launch;
   - `Ctrl-C` on the rover launch;
   - unplugging the ESP32's USB;
   - pressing the ESP32's EN (reset) button while a stick is pushed (must not move until centred).
4. Watch `/diagnostics`: `esp_rx_bad` should stay at 0.

## Configuration

`config/drive.yaml` (both machines) and `config/wheels/*.yaml`. Use decimal points for numbers: ROS 2 parameters are typed. After editing, rebuild (`colcon build --symlink-install`) on that machine and relaunch.

| Parameter | Default | Meaning |
|---|---|---|
| `track_width_m` | 0.84 | Distance between left and right wheel centres |
| `max_wheel_rpm` | 94.7 | Wheel rpm at full PWM. **Must equal `MAX_WHEEL_RPM` in the firmware**; the drive node warns if not. 94.7 rpm on a 0.28 m wheel is 5 km/h |
| `turn_track_factor` | 1.0 | Raise above 1 if the rover turns less than commanded (skid-steer slip). Only matters for Nav2 accuracy |
| `wheel_diameter_m` / `wheel_width_m` | 0.28 / 0.14 | Per wheel type, in `config/wheels/` |
| `max_linear_speed` / `max_angular_speed` | auto | Full-stick speed; auto = rover top speed / spin in place at full PWM |
| `deadzone`, `slow_scale` | 0.08, 0.35 | Stick deadzone; slow-mode factor |
| `estop_button`, `release_button`, `slow_button` | 1, 6, 9 | Circle, Options, L1 |
| `cmd_timeout_s` / `estop_timeout_s` | 0.3 / 0.5 | Rover side: stale `/cmd_vel` → zero; stale e-stop heartbeat → e-stop |
| `require_estop_heartbeat` | true | Set false only for autonomous runs that must continue without the base station link |
| `swap_sides` | false | Swap left and right (wiring fix) |

**Second wheel type:** measure its diameter, put it in `config/wheels/wheel_other.yaml`, and launch both sides with `wheels:=wheel_other`. The firmware doesn't change: max wheel rpm is a motor property. The top speed follows from the new diameter.

## Testing without hardware

```bash
# Simulated ESP32 on a pseudo-terminal, then the real launch files against it:
ros2 run rover_pilot_remastered fake_esp --link /tmp/rover-drive
ros2 launch rover_pilot_remastered rover.launch.py port:=/tmp/rover-drive
ros2 launch rover_pilot_remastered base_station.launch.py

# Dress rehearsal of the whole bench test: real launch files, simulated ESP32 and PS5, every stop timed (about 1 min)
python3 tools/rehearsal.py                                          # ROS sourced and the workspace built; exit 1 on any FAIL

# Test suites
cd ros2_ws/src/rover_pilot_remastered && python3 -m pytest test   # protocol, kinematics, e-stop logic, end-to-end, ROS integration
cd firmware && pio test -e native                                  # firmware state machine and protocol
python3 tools/gen_test_vectors.py --check                          # C++ and Python vectors agree
```

On a machine with ROS sourced, add `-p no:launch_testing -p no:launch_ros` to pytest if ROS's own pytest plugins complain.

## Serial protocol

The same framing as the rover's planned CAN-master link: `version | type | seq | payload | CRC-16/CCITT-FALSE`, COBS-encoded, `0x00`-terminated, at 115200 8N1. Multi-byte fields are little-endian.

| Type | Direction | Rate | Payload |
|---|---|---|---|
| `0x82` DRIVE_COMMAND | PC → ESP32 | 50 Hz | `flags` u8 (bit 0 = e-stop) + 4 × int16 wheel targets FL, FR, RL, RR in 0.1 rpm (positive = forward) |
| `0x04` DRIVE_STATUS | ESP32 → PC | 10 Hz | state, command age ms, 4 × int16 applied PWM, rx good/bad, uptime ms, reset reason, `MAX_WHEEL_RPM`×10, firmware version |

States: `BOOT`, `NO_COMMAND`, `ESTOP`, `WAIT_NEUTRAL`, `DRIVING`. The firmware is open-loop: it maps target rpm to PWM using `MAX_WHEEL_RPM`, because there are no encoders yet. When the CAN master/slave firmware arrives, the same `/cmd_vel` and e-stop topics will feed the onboard bridge (`telemetry_dashboard/bridge`, `drive:` section of its config) instead of `drive_node`, and the slaves will close the loop on their encoders. The bridge sends the same `DRIVE_COMMAND` (`0x82`) bytes to the CAN master, which applies the same rules (300 ms timeout, neutral interlock, e-stop latch); the firmware contract is `telemetry_dashboard/docs/FIRMWARE_INTERFACE_SPEC.md` (v2.0 on the `ethernet-udp` branch, v1.1 on `usb-serial`). Stop `drive_node` before the bridge drives: only one of them should follow `/cmd_vel`.

## Known limits and warnings

- **`arm_node` still scans every USB serial port** (`ctrl.SerialPortChecker`). Opening the drive ESP32's port that way can reset it or disturb the link while driving: the rover stops, by design, but it is disruptive. Give the arm ESP32 its own udev name and open that directly before running `arm_node` alongside this.
- No encoders, so no `/odom`: Nav2 will need visual odometry (ZED / rtabmap) until the CAN slaves report wheel speeds.
- Open-loop speed: actual speed varies with battery voltage and terrain.
- The software e-stop depends on the radio link and the laptops; it is not a replacement for the physical kill switch.

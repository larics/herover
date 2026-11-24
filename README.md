# herover

Drive stack for the herover mecanum robot: a ROS 2 bridge plus firmware for
the robot's two boards.

```
ROS 2 host ── /herover_N/cmd_vel (Twist) ──► cmd_vel_bridge          herover/cmd_vel_bridge.py
   ── WiFi, UDP unicast :5005 ── "CMD <id> <vx> <vy> <wz>\n" ──► ESP32     firmware/herover_esp32
   ── UART 115200 8N1 ────────── "CMD <vx> <vy> <wz>\n" ──────► OpenRB-150 firmware/herover_openrb
   ── mecanum IK (SI) → rpm ── DYNAMIXEL bus, Protocol 2.0, 1 Mbps ──► 4× XL430-W250
```

- **OpenRB-150** is the motor controller. It owns the robot's kinematics and
  runs the same firmware on every robot (it has no robot id).
- **ESP32** is only the WiFi link: it forwards commands for its `ROBOT_ID`
  to the OpenRB. It can be replaced by an RPi later (see below) without
  touching the OpenRB.
- Every command is used. Nothing is suppressed for being identical to the
  previous one, so a datagram lost on WiFi is replaced by the next.
- The robots never reply: nothing is sent back over WiFi or the UART.
  Diagnostics go to each board's own USB serial.

## Wire protocol

All values are ASCII decimal, separated by spaces, one command per line /
datagram. Units follow REP-103: `vx`, `vy` in m/s and `wz` in rad/s, body
frame (x forward, y left, z counter-clockwise), exactly as in the Twist
(`linear.x`, `linear.y`, `angular.z`).

| Link | Format | Notes |
|---|---|---|
| host → ESP32 (UDP) | `CMD <id> <vx> <vy> <wz>\n` | ESP32 drops it unless `<id>` == its `ROBOT_ID` (guards against swapped IPs) |
| ESP32 → OpenRB (UART) | `CMD <vx> <vy> <wz>\n` | exactly three numbers; a line with the id still in it is rejected |
| USB → OpenRB (bench) | `CMD <vx> <vy> <wz>\n` or `STOP\n` | type into a serial monitor at 115200 |

Malformed lines (missing/extra fields, non-numbers, NaN/inf) are dropped.

**Watchdog.** If the OpenRB gets no valid line (UART or USB) for
`CMD_TIMEOUT_MS` (2 s, `firmware/herover_openrb/config.h`), it stops the
wheels and prints `no command for 2000 ms: stopped` on its USB serial. A
sender therefore has to repeat its command to keep the robot moving. This
covers a crashed host, a stopped bridge, a lost WiFi link, and a crashed or
unplugged ESP32.

The bridge adds the other half: when a robot's `cmd_vel` has been quiet for
2 s (`cmd_timeout`), it sends zeros, so the robot also stops when the publisher
dies while the bridge keeps running. Publish faster than 1 Hz to keep a robot
moving; unlike in hero_sim, one `cmd_vel` message moves it for only 2 s.

After an outage (WiFi back, robot power-cycled) a robot drives again at once if
its publisher is still sending. Stop the publisher before handling it.

## Wiring

| ESP32 (NodeMCU-32S) | OpenRB-150 |
|---|---|
| GPIO17 (TX2) | pin 13 RX (`Serial3`) |
| GND | GND |

- Only one data wire is needed; ESP32 RX stays unconnected, and so does
  OpenRB pin 14 (TX). The separate 4-pin UART header (`Serial2`) works too:
  set `CMD_SERIAL` to `Serial2` in `firmware/herover_openrb/config.h` and
  match RX/GND by its silkscreen.
- Both boards use 3.3 V logic, so no level shifter is needed.
- Power the ESP32 from its own USB or a 5 V regulator. The OpenRB's 5 V pin
  is rated for 150 mA, below the ESP32's WiFi transmit peaks.
- **Power up the OpenRB first, then the ESP32.** A running ESP32 holds its
  TX line high and feeds current into the still-unpowered OpenRB through
  pin 13. The OpenRB then does not start cleanly and only runs after a
  press of its reset button.
- Other ESP32 chips work with the same sketch; only the UART pins differ.
  `config.h` picks the defaults per chip (ESP32: RX 16 / TX 17, S2/S3: 18 / 17,
  C3/C6: 4 / 5) and can override them.

## Flashing

### OpenRB-150 (`firmware/herover_openrb`)

1. The Boards Manager lists only Arduino's own boards until the ROBOTIS
   package URL is added. In Arduino IDE 2: **File → Preferences…** (Ctrl+,),
   and in **Additional boards manager URLs** at the bottom add
   `https://raw.githubusercontent.com/ROBOTIS-GIT/OpenRB-150/master/package_openrb_index.json`
   (one URL per line if the field already has one). Click **OK** and wait for
   the index download to finish, then install **OpenRB-150 by ROBOTIS** in the
   Boards Manager and **Dynamixel2Arduino** in the Library Manager.
2. Set the XL430s up once with DYNAMIXEL Wizard (they ship as ID 1): IDs
   **3 front-left, 4 front-right, 1 rear-left, 2 rear-right**, baud 1 Mbps
   (they ship at 57600), Protocol 2.0. A different baud works too if
   `DXL_BAUD` in both OpenRB sketches matches it.
3. Fill in the `TODO measure` geometry in `config.h`: `WHEEL_RADIUS_M`,
   `HALF_WHEELBASE_M` (centre to axle), `HALF_TRACK_M` (centre to wheel), and
   `MAX_WHEEL_RPM` for your supply voltage (keep it below the motors'
   Velocity Limit, 60.7 rpm by default).
4. Open `herover_openrb.ino`, select board **OpenRB-150** on its port
   (`/dev/ttyACM0` or similar on Linux), upload. On Linux, if the port is
   missing or the upload fails with "permission denied", run
   `sudo usermod -aG dialout $USER` and log out and back in.

At boot the OpenRB switches on the motor power (red `DXL` LED) and retries
any motor that does not answer for up to `DXL_BOOT_WAIT_MS` (3 s). Then the
USB serial prints `DXL id N not found` for any motor still missing and
`herover_openrb ready`. A missing motor is skipped until the next reset, and
commands are only read once this is done. If the board needs a reset press
after power-on, check the power-up order under Wiring.

### ESP32 (`firmware/herover_esp32`)

1. Additional Boards Manager URL
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`, then
   install **esp32** by Espressif.
2. Copy `secrets.example.h` to `secrets.h` (gitignored) and set the WiFi
   SSID and password.
3. In `config.h` set `ROBOT_ID` (unique per robot, matches `id` in
   `config/robots.yaml`). Optionally set `USE_STATIC_IP`. Otherwise use DHCP
   and reserve the address on the router so `robots.yaml` stays valid.
4. Board **NodeMCU-32S** (or your chip), upload.

At connect the USB serial prints `WiFi up: herover-<ID>  ip <addr>  udp port 5005`.
Every 5 s while traffic flows it prints received / forwarded / bad /
wrong-id counts. The board also answers mDNS as `herover-<ID>.local`. The
onboard LED flashes for each command forwarded to the OpenRB (pin and pulse
length in `config.h`; `USE_STATUS_LED 0` turns it off).

## ROS 2

```bash
colcon build --packages-select herover --symlink-install && source install/setup.bash
ros2 run herover cmd_vel_bridge          # uses share/herover/config/robots.yaml
ros2 run herover cmd_vel_bridge --ros-args -p config_file:=/path/to/robots.yaml
```

With `--symlink-install` the installed `robots.yaml` is a symlink to
`config/robots.yaml`, so edits only need a bridge restart, not a rebuild
(rebuild only when adding new config files). If the package was previously
built without it, run `rm -rf build/herover install/herover` once first.

Set each robot's `host` (the ESP32 IP) in `config/robots.yaml`. The bridge
subscribes to `/<namespace>/cmd_vel`, which matches what the
hero_mrs_control_stack `Herover` agent publishes (`/herover_N/cmd_vel`). Each
Twist is sent immediately as one datagram. Between Twists the last one is
re-sent at `resend_rate` (default 10 Hz), which replaces lost datagrams. After
`cmd_timeout` (default 2 s) without a new Twist, zeros are sent for about 1 s
and then nothing. With `-p resend_rate:=0.0` neither happens and only the
robot's watchdog stops it. An unresolvable host is retried every 2 s, so robots
can come up after the bridge. The bridge logs a per-robot count of Twists sent
(re-sends not counted) every 5 s while they arrive.

### Robot addressing

ROS code only publishes to `/herover_N/cmd_vel`; it never talks to an ESP32
directly. Each `robots.yaml` entry ties the three together:

```
namespace herover_1 ──► id 1 ──► host 192.168.1.101 (UDP unicast)
```

Routing is by IP: each Twist goes only to its robot's `host`. The id is a
check. The ESP32 drops a datagram whose id is not its `ROBOT_ID` and counts
it as `wrong_id` on its USB serial, so swapped IPs leave the robots still
instead of driving the wrong one. The OpenRB has no id. Per robot, these must
match:

| `robots.yaml` | must equal |
|---|---|
| `namespace` | the topic namespace your ROS code publishes to |
| `id` | `ROBOT_ID` in the ESP32's `config.h` |
| `host` | that ESP32's IP (printed at connect), or `herover-<id>.local` |

To add a robot: flash its ESP32 with a new `ROBOT_ID`, reserve its IP on the
router, add the entry to `robots.yaml`, and restart the bridge.

Manual driving, keyboard:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r __ns:=/herover_1 -p speed:=0.10 -p turn:=0.5
```

Manual driving, gamepad (Logitech F710, back switch on **X**):

```bash
ros2 launch herover joy_teleop.launch.py                        # herover_1
ros2 launch herover joy_teleop.launch.py namespace:=herover_2 joy_dev:=0
```

| Control | Does |
|---|---|
| LB (hold) | enable: nothing is sent unless it is held |
| RB (hold, with LB) | turbo: 0.17 m/s, 0.8 rad/s instead of 0.10 m/s, 0.5 rad/s |
| left stick | forward/back, strafe left/right |
| right stick, left/right | turn |

While LB is held, `joy_teleop` publishes at 20 Hz. On release, or if `joy`
goes quiet for 0.5 s, it sends zeros for 0.5 s and then nothing, so it does
not override other `cmd_vel` publishers. The zeros are repeated so a stop does
not depend on a single message. Speeds, axes and
buttons are node parameters (`speed`, `turn`, `speed_turbo`, `turn_turbo`,
`axis_vx`, `axis_vy`, `axis_wz`, `enable_button`, `turbo_button`). Check
another gamepad's indices with `ros2 topic echo /herover_1/joy`. In Docker
the container needs `/dev/input` (e.g. `-v /dev:/dev`).

### Mocap pose and odom

`pose_filter` is the hero_mrs_control_stack node of the same name, copied
here and extended with odometry. It needs `motion_capture_tracking` running
(OptiTrack → `/poses`); it does not start it. For each rigid body on
`/poses`, except names starting with `cf` (Crazyflies), it publishes:

| Topic | Type | Content |
|---|---|---|
| `/<name>/pose` | `geometry_msgs/PoseStamped` | the mocap pose, as received |
| `/<name>/odom` | `nav_msgs/Odometry` | same pose, plus a body-frame twist |

```bash
ros2 run herover pose_filter
ros2 run herover pose_filter --ros-args -p publish_pose:=false   # odom only
```

The OptiTrack rigid body must be named like the robot's namespace
(`herover_1`). Both topics keep the mocap header (frame `world`). The odom's
`child_frame_id` is the rigid-body name, and its twist is in that body frame
(`linear.x` forward, `linear.y` strafe left, `angular.z` CCW). The robots
send nothing back, so the velocity comes from differencing consecutive mocap
poses, low-pass filtered (`velocity_tau`, default 0.05 s). If a body is not
seen for over `max_gap` (0.5 s), the estimate starts again. Covariances are
zero. Parameters: `exclude_prefixes` (default `[cf]`), `publish_pose`,
`velocity_tau`, `max_gap`.

Run only one node that publishes `/<name>/pose`. When the stack's
`multi_agent.launch.py` runs with `support:=true`, its own `pose_filter`
already does, so start this one with `publish_pose:=false` (odom only).

### Launch everything

```bash
ros2 launch herover herover.launch.py
ros2 launch herover herover.launch.py teleop:=false publish_pose:=false
```

Starts `cmd_vel_bridge`, `pose_filter` and, unless `teleop:=false`, the
gamepad teleop. Arguments: `config_file` (robots YAML, default the installed
`robots.yaml`), `publish_pose` (default `true`), `teleop` (default `true`),
`namespace` (robot the gamepad drives, default `herover_1`), `joy_dev`.
The bridge sends every `cmd_vel` to the robots, so put them on blocks or
give them room before launching.

## Bring-up checklist

In steps 1 and 2 each command runs for 2 s and then the watchdog stops the
wheels; repeat it to keep them turning. Stop the bridge first, so it does not
send commands of its own in between.

0. **Motors alone.** Upload `firmware/herover_openrb_motor_test` with the
   robot on blocks. After reset it runs each wheel forward then backward,
   one at a time (FL, FR, RL, RR), and then stops. Each wheel should roll the
   robot forward in its forward phase. The USB serial at 115200 shows the
   wheel name and measured rpm. The onboard LED blinks fast while the test
   runs and slowly once it is done. Re-flash `herover_openrb` afterwards.
1. **OpenRB alone.** USB serial monitor at 115200, send `CMD 0.05 0 0`: all
   four wheels roll forward. Try `CMD 0 0.05 0` (strafe left) and
   `CMD 0 0 0.5` (turn CCW), then `STOP`. If a wheel turns the wrong way, flip
   its `INVERT_*` in `config.h`.
2. **ESP32 → OpenRB.** From the host:
   `echo "CMD 1 0.05 0 0" | nc -u -w0 <esp32-ip> 5005`, then
   `echo "CMD 1 0 0 0" | nc -u -w0 <esp32-ip> 5005`.
3. **ROS 2.** Run the bridge and teleop as above, or
   `ros2 launch herover herover.launch.py`.
4. **Watchdog.** With the robot on blocks, drive it slowly and stop the bridge
   (Ctrl-C) while the wheels turn. They must stop within about 2 s. Repeat by
   cutting the ESP32's power, and by stopping only the `cmd_vel` publisher.

## Units and speeds

Commands are real SI units now: `speed: 0.25` in a stack config means
0.25 m/s. The top speed is `MAX_WHEEL_RPM · 2π · WHEEL_RADIUS_M / 60`
(about 0.17 m/s with the placeholder 30 mm wheel at 55 rpm). Faster commands are
scaled down on all four wheels together, keeping the direction of travel.

## Replacing the ESP32 with a Raspberry Pi

The UART line protocol is the contract. An RPi wired to the same OpenRB
header (TX → RX, GND) only has to write `CMD <vx> <vy> <wz>\n` at 115200,
e.g. a ROS 2 node on the RPi subscribing to `cmd_vel` and writing to
`/dev/serial0`. It must repeat the last command (10 Hz, like the bridge) or the
watchdog stops the robot. The OpenRB firmware stays as it is.

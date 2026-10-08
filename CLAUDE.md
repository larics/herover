# herover: notes for Claude Code

Drive stack for the herover mecanum robot. `README.md` is the full reference
(architecture, wire protocol, wiring, flashing, bring-up); read it before
changing anything that crosses a board boundary.

## Layout

| Path | What |
|---|---|
| `herover/cmd_vel_bridge.py` | ROS 2 node: `/<ns>/cmd_vel` Twist → UDP `CMD <id> vx vy wz` to each robot's ESP32 |
| `config/robots.yaml` | Robot list for the bridge (id, namespace, ESP32 host) |
| `test/` | pytest for the bridge (needs a sourced ROS 2: it imports `rclpy`) |
| `firmware/herover_esp32/` | ESP32 WiFi link: UDP → UART to the OpenRB |
| `firmware/herover_openrb/` | OpenRB-150 motor controller: UART/USB lines → mecanum IK → 4× XL430 |
| `firmware/herover_openrb_motor_test/` | Bench test: each wheel forward/back in turn, no ESP32 |

## Commands

- Build: `colcon build --packages-select herover --symlink-install`
- Test: `colcon test --packages-select herover`, or `python3 -m pytest test/`
  with ROS 2 sourced.
- Firmware is built and flashed by the user in Arduino IDE 2 (boards
  OpenRB-150 and ESP32 Dev Module). If you could not compile a firmware
  change, say so plainly; never imply it was built or tested.

## Invariants

- The wire protocol table in `README.md` is the contract between host, ESP32
  and OpenRB. A change to it touches all three plus the README and tests in
  one go.
- Units are SI, REP-103 body frame (x forward, y left, z CCW).
- The OpenRB firmware is identical on every robot. The robot id lives only in
  `firmware/herover_esp32/config.h` (`ROBOT_ID`) and `config/robots.yaml`.
- Each Arduino sketch is self-contained in a folder named like its `.ino`; it
  cannot include files from a sibling sketch. The motor test copies the motor
  IDs, `INVERT_*` flags and `DXL_BAUD` from `herover_openrb/config.h`: keep
  them in sync.
- `MAX_WHEEL_RPM` must stay below the XL430 Velocity Limit (60.7 rpm factory).

## Rules

These are safety nets. `.claude/settings.json` enforces some of them as
permission prompts; this list is the full set and applies even where no
prompt fires.

### Git and GitHub

- **Ask before every `git commit` and every `git push`**, each time. An
  earlier "commit and push" does not cover later commits. When asking, show
  the files, a short summary of the diff, the commit message, and the target
  remote and branch.
- Stage files by name. No `git add -A` / `git add .`.
- Never force-push, and never rewrite history that has been pushed (amend,
  rebase, reset, filter-branch). Never delete branches or tags.
- Never skip hooks (`--no-verify`) or commit signing.
- Never change git config or remotes.
- Ask before any GitHub write: PRs, issues, releases, comments, merges.
- Never commit build output (`build/`, `install/`, `log/`), secrets, or large
  binaries.

### Secrets

- Do not read, print, or edit `firmware/herover_esp32/secrets.h`, `.env`
  files, keys, or any other credential file. To check one exists, use `ls` or
  `git check-ignore`, never `cat`.
- Never put credentials in commits, code, logs, or chat.

### Robots and hardware

Firmware and commands here move real motors.

- Ask before flashing a board (`arduino-cli upload`, `esptool`, `bossac`).
- Ask before sending anything that can move a robot: UDP to a robot
  (`nc -u`, scripts), writes to serial ports (`/dev/ttyACM*`,
  `/dev/ttyUSB*`), `ros2 topic pub` on a `cmd_vel`, or running the bridge or
  teleop. The user confirms the robot is on blocks or has room first.
- Do not add code that writes DYNAMIXEL EEPROM settings (ID, baud rate,
  Velocity Limit, ...) beyond the Operating Mode the firmware already sets,
  without asking. These persist in the motors, which are shared with another
  course.
- Keep test speeds low. Ask before raising `MAX_WHEEL_RPM`, `TEST_RPM`, or
  other speed limits.

### Scope

- Stay inside this repository. Do not read, clone, or fetch other repos
  (including other organization repos) unless asked.
- No `sudo`, system package installs, or changes to the user's environment
  (udev rules, groups, IDE settings). Give the command for the user to run.
- Ask before deleting files.

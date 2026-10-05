
Download these two as zip and add to 

https://github.com/akira215/HalfDuplexSerial-for-Arduino
https://github.com/akira215/DynamixelArduino

Run the bridge (one XBee, multiple robots — set `config_file` to your YAML):

```bash
ros2 run herover twist_serial_bridge \
  --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p baud:=115200 \
  -p config_file:=$(ros2 pkg prefix herover)/share/herover/config/robots.yaml
```

Drive a specific robot by publishing to its namespaced topic:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r __ns:=/robot1 -p speed:=0.15 -p turn:=0.5 -p holonomic:=true
```

## Wire format

Each line sent over the shared XBee serial:

```
CMD <id> <linear_x> <linear_y> <angular_z>\n
```

`<id>` matches `ROBOT_ID` in the Arduino sketch. `0` is broadcast (every robot acts on it). Edit `config/robots.yaml` to change the id → namespace mapping; set `ROBOT_ID` in `libs/normal_mode.ino` per robot before flashing.

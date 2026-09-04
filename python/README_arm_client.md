# arm_service — XML-RPC front door to the DOFBOT stack

A remote caller (the robot's higher-level brain, a laptop, a script) says
`pick_can(x, y, z)`; this service turns that into the `ros2` commands
`dofbot_ctrl` already provides. It replaces the pre-ROS `arm-service` in
`jetson-nano-services`, which drove the servos directly over serial.

Not a ROS package — it has no `package.xml`, so colcon passes over it, and it
imports no rclpy. It is a supervisor: one long-lived `ros2 launch` it keeps
alive, one short-lived `ros2 run` per motion command. Everything that is hard
(planning, IK, the planning scene, the gripper model) stays in `dofbot_ctrl`,
where the tests are.

## Commands

| method | what it runs |
|---|---|
| `enable_arm()` | `ros2 launch dofbot_ctrl pick_place.launch.py rviz:=false` |
| `disable_arm()` | stops that launch (whole process group) |
| `pick_can(x, y, z)` | `ros2 run dofbot_ctrl pick_place -- --pick x y z` |
| `place_can()` | `ros2 run dofbot_ctrl pick_place -- --place` |
| `move_to_state(name)` | `ros2 run dofbot_ctrl move_to_state -- NAME` |
| `reset_arm(state, force)` | `ros2 run dofbot_ctrl pick_place -- --reset STATE` |
| `wave_arm(waves, finish, seconds)` | `ros2 run dofbot_ctrl wave_arm -- --waves N` |

Plus `list_states()`, `stop()`, `tail_log()`, `get_status()`, `ping()`.

Every command returns the same dict:

```python
{'ok': bool, 'command': str, 'returncode': int,
 'output': str,      # the ros2 command's console output
 'error': str, 'seconds': float}
```

`output` is the tail of what the node printed, which is where a MoveIt failure
explains itself — hand it to a human, don't parse it.

## Running it

```bash
./start_arm_server.sh            # foreground, sources ROS + the workspace
sudo ./install.sh                # or as a systemd unit, enabled at boot
sudo systemctl start dofbot-arm
```

Port 8001 by default (`DOFBOT_ARM_PORT`) — the port the pre-ROS arm service
used, and which it no longer needs. Not 8002: that belongs to
`supervisor-service`.

## Using it

```bash
python3 arm_client.py status
python3 arm_client.py enable
python3 arm_client.py state ready
python3 arm_client.py pick 0.22 0.0 0.033
python3 arm_client.py place
python3 arm_client.py wave 3                   # a greeting, then stow at init
python3 arm_client.py reset                   # after a pick that failed partway
python3 arm_client.py disable --park init
python3 arm_client.py -i                     # interactive
```

From code:

```python
from arm_client import ArmClient
arm = ArmClient('http://192.168.55.1:8001/')
arm.enable_arm()
if arm.pick_can(0.22, 0.0, 0.033)['ok']:
    arm.place_can()
```

## Things worth knowing

- **x, y, z is the object's CENTRE in `base_link`, in metres** — for something
  on the floor that is half its height, not zero. Same convention as
  `pick_place` itself.
- **The arm must be enabled first.** Motion commands do not auto-enable; being
  explicit is what stops a stray call from starting a stack nobody expected.
- **One motion at a time.** A second request comes back `busy` rather than
  queueing: a queued arm command would execute against a world that has moved
  on since it was sent.
- **`stop()` aborts the command, not the servos.** It SIGINTs the running node;
  the arm settles wherever the waypoint already in flight leaves it.
- **`moveit_bridge` needs `/dev/ttyTHS1` to itself.** Stop `gui_teleop` and
  `joint_state_mirror` before `enable_arm()`.
- **A failed `enable_arm()` tears its own launch down.** A leftover half-stack
  would put a second `move_group` on the domain at the next attempt, and two
  action servers driving one arm is a far worse failure than a clean retry.
- **After an aborted pick the planning scene still holds the object**, and
  moves out of it fail instantly with `INVALID_MOTION_PLAN`. `reset_arm()` is
  the way out: it clears the scene, opens the gripper and goes home, in that
  order, because planning cannot get out of a hole the scene is holding it in.
  `move_to_state()` will not do it — that deliberately does not touch the
  scene. `reset_arm(force=True)` is the last resort for when MoveIt will not
  plan from where the arm is; the blind move it makes is not collision checked.
- **`reset_arm()` drops what is held, where it is**, before the arm moves.
  Carrying it home first would only drop it from higher up.
- **`place_can()` drops at a named state, not at a coordinate.** `over_trash`
  is nominally straight in front of the robot and is a placeholder for a bin
  perception has yet to find; when it does, this grows the coordinates
  `pick_can()` already takes.
- **`wave_arm()` is a gesture, not a pose command.** It plans into a raised
  pose, sends the swings as one timed trajectory so the arm does not stop dead
  at each end of the swing, and stows at `init`. It goes through
  `moveit_bridge` like everything else, so the arm stays enabled and the whole
  path is collision-checked against the live scene — a wave refuses rather than
  sweeping through something the scene knows about.
- **One wave takes 3 s, and `seconds` sets that pace for any wave count.** The
  swings run well above `max_joint_speed`, deliberately: that 30 deg/s is a
  *picking* speed, set so the servos do not trail the plan by the bridge's
  200 ms `track_time_ms` and put the gripper somewhere the plan did not. A wave
  has nothing to arrive at, so the same lag only softens the ends of the swing.
- **`pick_can()` and `place_can()` are two processes.** The carried object lives
  in the planning scene between them, and that is where `--place` reads it
  from — so a server restart between the two halves is survivable, and
  `reset_arm()` is what tidies up when something else is not.

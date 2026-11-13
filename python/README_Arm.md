# arm-service — XML-RPC front door to the DOFBOT stack

A remote caller (the robot's higher-level brain, a laptop, a script) says
`pick_can(x, y, z)`; this service turns that into the `ros2` commands
`dofbot_ctrl` already provides.

**The arm itself lives in another tree**: `~/ros2_ws/src/dofbot_ros2`, a colcon
workspace, and every file named here without a path is in it —
`ARCHITECTURE.md`, `gripper.py`, `chassis.xacro`. This service drives that
workspace from outside it: `DOFBOT_WS` says where it is, defaulting to
`~/ros2_ws`, and `start_arm_server.sh` sources it.

This service imports no rclpy and is not a ROS package. It is a supervisor: one
long-lived `ros2 launch` it keeps alive, one short-lived `ros2 run` per motion
command. Everything that is hard (planning, IK, the planning scene, the gripper
model) stays in `dofbot_ctrl`, where the tests are.

**It drives the servos through ROS, not over serial.** The version of this
service that opened `/dev/ttyTHS1` itself is gone; its bus-servo driver is now
`dofbot_arm_lib/Arm_Lib` in the workspace, and `moveit_bridge` is the only
thing that writes to the port. Two things must never hold it at once, so
nothing here touches serial.

## Commands

| method | what it runs |
|---|---|
| `enable_arm(timeout, bridge, rviz, port, floor, check_jaws)` | `ros2 run dofbot_ctrl check_jaws`, then `ros2 launch dofbot_ctrl pick_place.launch.py rviz:=false`, with `DOFBOT_FLOOR` set to `floor` |
| `disable_arm()` | opens the jaws, then stops that launch (whole process group) |
| `pick_can(x, y, z, object, vision, approach, held)` | `ros2 run dofbot_ctrl pick_place -- --pick x y z`, plus `--approach-check` when `approach` is True, `--no-held-check` when `held` is False, `--no-vision` when `vision` is False |
| `can_pick(x, y, z, object)` | `ros2 run dofbot_ctrl pick_place -- --check x y z`: would that pick plan? Moves nothing |
| `place_can()` | `ros2 run dofbot_ctrl pick_place -- --place` |
| `drop_can()` | `ros2 run dofbot_ctrl pick_place -- --drop` |
| `grip_can(delay, object)` | `ros2 run dofbot_ctrl pick_place -- --grip --grip-delay DELAY` -- opens, waits for a hand-over, grips and holds as if picked |
| `move_to_state(name)` | `ros2 run dofbot_ctrl move_to_state -- NAME` |
| `current_state()` | `ros2 run dofbot_ctrl move_to_state -- --where`: which saved state the arm is in, as `state`, or None |
| `reset_arm(state, force)` | `ros2 run dofbot_ctrl pick_place -- --reset STATE` |
| `wave_arm(waves, finish, seconds)` | `ros2 run dofbot_ctrl wave_arm -- --waves N` |
| `is_holding(object)` | `ros2 run dofbot_ctrl vision_check -- --held` |
| `held_color(object)` | `ros2 run dofbot_ctrl vision_check -- --color` |
| `set_torque(on)` | off: `disable_arm()` first, then `ros2 run dofbot_ctrl set_torque off`. on: `ros2 run dofbot_ctrl set_torque on` (nothing to do while enabled) |

Plus `list_states()`, `stop()`, `tail_log()`, `get_status()`, `ping()`.

Every command returns the same dict:

```python
{'ok': bool, 'command': str, 'returncode': int,
 'output': str,      # the ros2 command's console output
 'error': str,       # what went wrong, ending in `reason`
 'reason': str,      # the one line saying why, lifted out of `output`
 'seconds': float}
```

`output` is the tail of what the node printed, which is where a MoveIt failure
explains itself — hand it to a human, don't parse it. `reason` is that
explanation reduced to a sentence for a caller that is a program: the last
`[ERROR]` the node logged, or, when it died before reaching a logger, the last
thing it printed. It is empty on success and can be empty on a failure that
printed nothing to go on.

```python
{'ok': False, 'returncode': 1,
 'error': "pick_can exited 1: unknown object 'soda can'; catalogue has "
          "['soda_can', 'test_block']",
 'reason': "unknown object 'soda can'; catalogue has ['soda_can', 'test_block']"}
```

`current_state()` adds `state`: the saved state's name when every joint is
within half a degree of it, None when the arm is in none of them. `ok` False
(not enabled, or `busy` mid-move) also leaves `state` None, so read `ok` first.

`is_holding()` adds three keys to that dict: `held` (**True / False / None**),
`verdict` (the same three as text) and `reason`.

`held_color()` adds those and `color`: `'red'`, `'green'`, `'blue'` or None.
When None, `held` says why: False empty jaws, True held but the colour was not
clear, None the camera could not answer.

## Running it

```bash
./start_arm_server.sh            # foreground, sources ROS + the workspace
sudo ./install.sh                # or as a systemd unit, enabled at boot
sudo systemctl start dofbot-arm
```

Port 8001 by default (`DOFBOT_ARM_PORT`) — the port the pre-ROS arm service
used, and which it no longer needs. Not 8002: that belongs to
`supervisor-service`.

**The unit is `dofbot-arm.service`, and the old one was `arm.service`.** A
machine that ran the serial version still has that unit installed; it points at
a startup script that no longer exists, so remove it rather than leave it to be
started by mistake:

```bash
sudo systemctl disable --now arm.service
sudo rm /etc/systemd/system/arm.service && sudo systemctl daemon-reload
```

`start_arm_server.sh` runs the system `python3` with ROS sourced, not a venv.
There is nothing to pip-install: the server is stdlib only and everything it
drives is a `ros2` command.

## Logs

Three places, none of them the colcon build logs:

| | |
|---|---|
| `journalctl -u dofbot-arm -f` | the server: every command it ran and how each ended |
| `/tmp/dofbot_arm_launch.log` | the launch — move_group and the bridge. `tail_log()` serves it |
| `~/.ros/log/` | one file per node run, `<node>_<pid>_<stamp>.log`, written by ROS itself |

A one-shot command like `pick_place` leaves its own file in `~/.ros/log/`, so
the error behind a failed pick is on disk there as well as in `reason`.

## Using it

```bash
python3 arm_client.py status
python3 arm_client.py enable --floor hard       # the surface the robot is on
python3 arm_client.py state init
python3 arm_client.py where                    # init, stow, ... or none
python3 arm_client.py check 0.30 0.0 0.035      # would it plan? moves nothing
python3 arm_client.py pick 0.30 0.0 0.035
python3 arm_client.py pick 0.30 0.0 0.035 --approach --no-held   # pick the checks
python3 arm_client.py place
python3 arm_client.py drop                     # let go where the arm is
python3 arm_client.py held                     # is the can still in the jaws?
python3 arm_client.py wave 3                   # a greeting, then stow
python3 arm_client.py reset                   # giving up after a failed pick
python3 arm_client.py disable                  # parks at stow, then stops
python3 arm_client.py -i                     # interactive
```

From code:

```python
from arm_client import ArmClient
arm = ArmClient('http://192.168.55.1:8001/')
arm.enable_arm(floor='hard')
if not arm.can_pick(0.30, 0.0, 0.035)['ok']:
    ...                                  # nudge the base, then ask again
elif arm.pick_can(0.30, 0.0, 0.035)['ok']:
    arm.place_can()
```

## Where to put the robot

`pick_can()` takes the object where it is; it does not drive. Whoever does drive
gets to choose where the can ends up relative to the arm, and that choice decides
whether the pick is comfortable or knife-edge. **Aim for 0.30 m.**

```python
GRASP_STANDOFF = 0.30    # drive until the object sits here, base_link x
ARM_MIN_REACH  = 0.24    # closer than this, reposition rather than reach
ARM_MAX_REACH  = 0.36    # further than this, drive in
ARM_MAX_YAW    = 100.0   # degrees off centre, either side
```

**Test the radius, `hypot(x, y)`, not x.** The arm is rotationally symmetric
about its base yaw, so a can at (0.24, 0.20) is 0.31 m out and is a good target;
an x-only test reads it as too close and drives away from a comfortable pick.

Measured offline against the same pitch/standoff/height search `pick_place`
runs, soda can, extended fingers, carpet:

| object x | what the pick gets |
|---|---|
| < 0.20 | nothing solves at any pitch |
| 0.20–0.23 | solves, but off the proven grip height, standoff down to its 20 mm floor, under 0.05 rad of joint margin |
| 0.24–0.27 | works; standoff climbing 45 → 80 mm |
| **0.28–0.32** | **the sweet spot** — full 80 mm standoff, the 80 mm grip height proven on hardware, `grasp_pitch` landing on its preferred 2.2, > 0.12 rad of margin |
| 0.33–0.36 | still full standoff and full margin |
| 0.37–0.39 | degrades into shallow-pitch side grabs at a 20–45 mm standoff |
| ≥ 0.40 | nothing solves at any pitch |

**The near edge is the one that bites, and it bites suddenly.** A ±25 mm base
error either side of 0.30 stays inside the sweet spot; the same error from 0.22
lands at 0.195, where nothing solves. That asymmetry is why `GRASP_STANDOFF`
sits mid-band rather than at the closest workable distance — closer is not
safer here. See ARCHITECTURE.md, "The approach sweep": the standoff pose is the
grasp pulled *back along the tool axis*, which at a steep pitch is up and
**inward**, so it runs out before the reach does.

**Yaw is nearly free.** Nothing degrades out to 90°; margin falls to 0.14 rad at
100° and 0.06 at 105°, and 110° is past `arm1_Joint`'s stop. The chassis is not
what limits it — the arm's closest approach to the chassis cylinder is the
shoulder, at a fixed 60 mm, and swinging sideways only opens that up. 100° is
the gate; prefer to turn the base anyway, since a square approach costs one
cheap rotation.

**Hard floor: the same band.** Two picks at 0.338 and 0.343 m, about 8° of
yaw, once failed with `INVALID_MOTION_PLAN` and looked like the far edge
moving in. They were not: the stack was modelling carpet (see the floor note
under *Things worth knowing*), and every hard-floor pick failed the same way at
any distance. With the floor set right, 0.335 m at 8° picks, and so do
0.20–0.22 straight ahead. The band above has not been re-swept on hard floor;
it was never what failed.

**Check before committing.** `can_pick(x, y, z)` runs the pick's own approach
search and collision checks against the live scene and moves nothing. When it
says no, `reason` says which way: *move the base closer* past the far edge,
*move the base FURTHER AWAY* at the near one. It does not check the vision steps
or the moves after the grasp (a wrong floor still gets past it), refuses while
something is held, and comes back `busy` during a pick.

**These numbers are for the EXTENDED fingers.** They move the whole working ring
outward. On the stock jaws with the 30 mm test block the same sweep gives a band
of 0.13–0.31 and a sweet spot of 0.20–0.29 — different constants entirely, so
anything that hard-codes the values above has to read `DOFBOT_GRIPPER` the way
`gripper.py` and `dofbot.urdf` both do.

## Things worth knowing

- **x, y, z is the object's CENTRE in `base_link`, in metres** — for something
  standing on the floor that is the floor height plus half the object's HEIGHT,
  not half its width and not zero. Same convention as `pick_place` itself. An
  upright 122 mm can on office carpet is `z = -0.022 + 0.061 = 0.039`; on a hard
  floor `-0.026 + 0.061 = 0.035`. Both ground figures are measured, and which
  one applies is what `DOFBOT_FLOOR` selects (see `chassis.xacro`).
- **The floor is chosen at `enable_arm(floor=...)`, `'carpet'` or `'hard'`.**
  It is built into the URDF when the stack starts, so it holds until
  `disable_arm()`; asking for a different one while enabled is an error, not
  ignored. Left out, it is the server's own `DOFBOT_FLOOR` (the unit file sets
  `hard`), else hard. `get_status()` reports it as `floor`. **Get it wrong and
  every pick fails right after the grasp**: carpet's floor is 4 mm above a hard
  one, so a hard-floor can stands inside it, and once attached the move up to
  `ready` starts in collision -- `INVALID_MOTION_PLAN` at `move_named('ready')`,
  with `floor_link`/`soda_can` in the launch log.
- **The arm must be enabled first.** Motion commands do not auto-enable; being
  explicit is what stops a stray call from starting a stack nobody expected.
- **One motion at a time.** A second request comes back `busy` rather than
  queueing: a queued arm command would execute against a world that has moved
  on since it was sent.
- **`stop()` aborts the command, not the servos.** It SIGINTs the running node;
  the arm settles wherever the waypoint already in flight leaves it.
- **`moveit_bridge` needs `/dev/ttyTHS1` to itself.** Stop `gui_teleop` and
  `joint_state_mirror` before `enable_arm()`.
- **`enable_arm()` checks the jaws first.** With the bridge on, it opens,
  closes and opens the empty jaws and reads the gripper servo back. A jaw that
  does not follow gets a torque cycle and one retry, and if it still does not
  follow, the enable is refused with `jaw check failed: ...`. The fix is then
  `torque off`, work the jaws by hand, enable again. `check_jaws=False`
  (`enable --no-jaw-check`) skips it. The check opens the jaws first, so
  anything left in them is dropped where the arm stands -- as the launch's own
  first sync would do anyway.
- **A failed `enable_arm()` tears its own launch down.** A leftover half-stack
  would put a second `move_group` on the domain at the next attempt, and two
  action servers driving one arm is a far worse failure than a clean retry.
- **After an aborted pick the planning scene still holds the object**, and
  moves out of it fail instantly with `INVALID_MOTION_PLAN`. **Retrying needs
  nothing extra:** `pick_can()` finds what the earlier run left and resets
  first. `reset_arm()` is for giving up rather than retrying: it clears the
  scene, opens the gripper and goes home, in that order, because planning
  cannot get out of a hole the scene is holding it in.
  `move_to_state()` will not do it — that deliberately does not touch the
  scene. `reset_arm(force=True)` is the last resort for when MoveIt will not
  plan from where the arm is; the blind move it makes is not collision checked.
- **`reset_arm()` drops what is held, where it is**, before the arm moves.
  Carrying it home first would only drop it from higher up.
- **Shutdown lets go first.** `disable_arm()` opens the jaws before it stops
  the launch, and so does a `systemctl stop` or any other SIGTERM. Stopping the
  stack cuts control, not torque, so a gripper left closed stays clamped on
  whatever it was holding — or on its own stall current once that works free —
  and nothing can drive the servo to fix it afterwards. It drops what is held
  where the arm is standing, deliberately, for the same reason `reset_arm()`
  does.
- **`reset_arm()` parks at `stow`**, the pose the robot drives in, clear of
  the lidar. `init` is only for the OAK-D's last look at a can before a pick.
  Pass a state to override it.
- **`place_can()` drops at a named state, not at a coordinate.** `trash`
  is nominally straight in front of the robot and is a placeholder for a bin
  perception has yet to find; when it does, this grows the coordinates
  `pick_can()` already takes.
- **`wave_arm()` is a gesture, not a pose command.** It plans into a raised
  pose, sends the swings as one timed trajectory so the arm does not stop dead
  at each end of the swing, and stows at `stow`. It goes through
  `moveit_bridge` like everything else, so the arm stays enabled and the whole
  path is collision-checked against the live scene — a wave refuses rather than
  sweeping through something the scene knows about.
- **One wave takes 3 s, and `seconds` sets that pace for any wave count.** The
  swings run well above `max_joint_speed`, deliberately: that 30 deg/s is a
  *picking* speed, set so the servos do not trail the plan by the bridge's
  200 ms `track_time_ms` and put the gripper somewhere the plan did not. A wave
  has nothing to arrive at, so the same lag only softens the ends of the swing.
- **Torque off disables the arm.** `set_torque(False)` runs `disable_arm()`
  first -- the jaws open and drop whatever is held -- and then the arm goes
  limp under its own weight. A limp arm under a running stack is one MoveIt
  still believes is where it left it, so the two are never separated.
  `set_torque(True)` on a disabled arm holds it where it stands and leaves it
  disabled; `enable_arm()` is what makes it commandable, and its bridge turns
  torque on by itself.
- **`is_holding()` LOOKS, it does not remember.** It grabs a frame from the
  wrist camera and puts it to nanoOWL. The point of the question is exactly the
  case where a live look and the planning scene disagree — a can that was
  picked and has since slipped out.
- **`held` has three values and `None` is not `False`.** `True` seen, `False`
  the camera had a good look and there was nothing there, `None` the question
  could not be asked: nanoOWL not running, or a dark frame. `ok` says only that
  the question got asked, so `ok` with `held` False is a working camera
  reporting an empty gripper. Treating `None` as `False` turns "the detector is
  busy elsewhere" into "you dropped it".
- **It classifies the bottom-centre of the frame, and asks about size.**
  nanoOWL chooses between *a close-up of a can filling the picture*, *a small
  can on the floor* and *a floor* for the bottom half, middle 70% of the frame
  (`vision.HELD_CROP`). Only the close-up means held. At `stow` the floor is in
  view, but when the jaws are empty the only cans that can be in it are further
  away, so they come out small; a held can fills that region. A whole-frame
  question could not tell the two apart and called every empty frame with a
  can in view held.
- **It means what it says at `stow` and nowhere else** -- the crop is where a
  held can lands from that pose. Score changes against labelled frames with
  `ros2 run dofbot_ctrl rescore_held` (frames in `~/held_frames/`, named
  `stow_held_*.png` / `stow_empty_*.png`; `grip_can()` and
  `calibrate_view --shot` make them).
- **It does not need the arm enabled**, since a camera and a detector are not
  `move_group` — but it does take the motion lock, so it comes back `busy`
  during a pick rather than photographing a moving arm.
- **A pick can ask the same question itself, at two points**, and aborts on a
  definite no: the *approach* check, with the jaws open around the can before
  they close, and the *held* check at stow. Each is its own switch:
  `approach` is **off** by default, `held` is on, and `vision=False` turns both
  off. It never aborts on `unknown`, so a pick on a machine with nanoOWL
  stopped runs exactly as it always did. `reset_arm()` is the way out of one
  that aborted after the grasp.
- **`pick_can()` and `place_can()` are two processes.** The carried object lives
  in the planning scene between them, and that is where `--place` reads it
  from — so a server restart between the two halves is survivable, and
  `reset_arm()` is what tidies up when something else is not.

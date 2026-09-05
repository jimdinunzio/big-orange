#!/usr/bin/env python3
"""
Coordinate frames for Big Orange.

Defines the robot base frame and the rigid transforms that relate the
Oak-D camera and the arm to it, so a spatial detection can be converted
into a target the arm can reach.

Frames
------
robot   Origin at the underside of the robot bottom plate, on the chassis
        axis (the axis of the 16.5 cm radius cylinder).  x forward, y
        left, z up (REP-103), yaw positive counter-clockwise seen from
        above.  This matches the convention already used for sdp poses
        in main.py.

        Note this is NOT base_footprint -- it rides with the chassis
        rather than sitting on the ground.  That is the useful choice
        here, because every mount on the robot is then a fixed number:
        the arm offset and the camera geometry below do not change with
        the floor.  Only the floor itself moves, which is FLOOR_Z.

oakd    The DepthAI spatial coordinate frame of the moving camera head:
        X right, Y up, Z forward along the optical axis, metres.
        (my_depthai.Detection stores exactly this in .x/.y/.z after the
        mm-to-m divide.)  The head pans and tilts, so this frame moves
        relative to `robot`.

arm     base_link of the bus-servo arm, axis-aligned with `robot`
        (rpy 0 0 0).  Its z = 0 is the UNDERSIDE OF THE ARM MOUNTING
        PLATE, which sits 0.003 m below the robot bottom plate.  That is
        the one convention here that is not guessable from the hardware,
        so state it on the robot side too.

Relation to base_footprint
--------------------------
If the URDF parents the arm to a ground-level base_footprint, its z is
the ride height and does change with the floor:

    base_footprint -> base_link    z = 0.022 (carpet) / 0.026 (hard)
    base_footprint -> robot        z = 0.025 (carpet) / 0.029 (hard)
    robot          -> base_link    z = -0.003            (either floor)

Angles are degrees at the boundary of this module and radians inside it.
"""

import math
import os
import sys

# --- Mount geometry, all in the robot frame, metres --------------------------
#
# Chassis-fixed, so none of these depend on the floor surface.

# Arm base.  The arm mounting plate sits 3 mm below the robot bottom
# plate that defines z = 0.
ARM_ORIGIN = (0.265, 0.0, -0.003)

# Yaw axis of the camera head is the vertical line through this x, y; the
# pitch pivot sits on that axis at PITCH_PIVOT_Z, so panning rotates the
# head without translating the pivot.
PAN_AXIS_X = -0.1125
PAN_AXIS_Y = 0.0
PITCH_PIVOT_Z = 0.59

# Lens position relative to the pitch pivot, in head-fixed axes (forward,
# left, up) with the head at home.  LENS_FORWARD is measured directly
# along the head axis and agrees with the two x measurements
# (-0.0575 lens, -0.1125 pivot).
#
# LENS_UP is positive: the tilt mechanism puts the pivot below the lens.
# Measured directly rather than differenced, so the lens sits at
# z = 0.61 with the pivot at 0.59.
LENS_FORWARD = 0.055
LENS_LEFT = 0.0
LENS_UP = 0.02

# Servo home angles, mirrored from move_oak_d so this module can be
# imported without pulling in pyFirmata and the board.
YAW_HOME_DEG = 90.0
PITCH_HOME_DEG = 115.0

# Grasp targets, metres: (height, radius).  A 12 oz soda can measures
# 0.122 m tall by 0.066 m across.
OBJECT_SIZES = {
    "soda can": (0.122, 0.033),
}

# --- What the arm can reach --------------------------------------------------
#
# Swept offline against the same pitch/standoff/height search pick_place runs
# -- see README_arm_client.md, "Where to put the robot", for the band, the
# sweet spot and how each edge fails.  MoveIt is still the authority on what
# plans; these exist so the robot repositions instead of asking for a pick
# that cannot solve, and so a caller can say where to reposition TO.
#
# Test the RADIUS, hypot(x, y) -- which is what arm_reach returns and what
# arm_can_reach compares.  Never x alone: the arm is symmetric about its base
# yaw, so a can at (0.24, 0.20) is 0.31 m out and a fine target, while an
# x-only test calls it too close and drives away from a good pick.
#
# GRASP_STANDOFF sits mid-band rather than at the closest workable distance
# because the near edge bites suddenly and asymmetrically: +-25 mm either side
# of 0.30 stays in the sweet spot, while the same error from 0.22 lands at
# 0.195, where nothing solves at any pitch.  Closer is not safer here.
#
# Which fingers are fitted moves the whole working ring, so read it the way
# gripper.py and dofbot.urdf do -- from DOFBOT_GRIPPER.

_GRIPPER_REACH = {
    #             standoff,  min,   max, max yaw, sweet spot
    # Extended fingers: the swept table in the README.  min/max trim the
    # marginal rows off each end of the band, keeping what "works"; the sweet
    # spot is the row with full standoff, the proven grip height and the most
    # joint margin, and is what a measured bias should be judged against.
    "extended":   (0.30,    0.24,  0.36,  100.0, (0.28, 0.32)),
    # Stock jaws with the 30 mm test block.  The README gives this variant's
    # band (0.13-0.31) and sweet spot (0.20-0.29) but not the row-by-row
    # sweep, so these stay inside the sweet spot rather than extrapolating
    # into rows nobody measured.
    "stock":      (0.245,   0.20,  0.29,  100.0, (0.20, 0.29)),
}

GRIPPER = (os.environ.get("DOFBOT_GRIPPER") or "extended").strip().lower()
if GRIPPER not in _GRIPPER_REACH:
    sys.stderr.write(
        "robot_frames: DOFBOT_GRIPPER=%r is not one of %s; using 'extended'. "
        "The reach band is wrong for any other fingers.\n"
        % (GRIPPER, "/".join(_GRIPPER_REACH)))
    GRIPPER = "extended"

(GRASP_STANDOFF, ARM_MIN_REACH, ARM_MAX_REACH, ARM_MAX_YAW,
 (ARM_SWEET_MIN, ARM_SWEET_MAX)) = _GRIPPER_REACH[GRIPPER]

# Arm-frame height above which this is a table, not the floor.  Not part of
# the reach sweep -- it only keeps floor_object_to_arm's floor assumption
# honest -- so it does not vary with the fingers.
ARM_MAX_GRASP_Z = 0.20

# --- The floor ---------------------------------------------------------------

# Height of the robot bottom plate above the floor, by surface.  Since
# the robot frame rides on that plate, the floor sits this far BELOW the
# origin, and it is the only quantity here the surface changes.
PLATE_HEIGHTS = {
    "carpet": 0.025,    # current testing
    "hard": 0.029,      # contest
}
DEFAULT_SURFACE = "carpet"

FLOOR_Z = None


def set_floor_surface(surface=DEFAULT_SURFACE):
    """
    Select the surface the robot is standing on, which sets how far the
    floor lies below the robot origin.  Call once at startup; carpet is
    assumed until you do.
    """
    global FLOOR_Z
    FLOOR_Z = -PLATE_HEIGHTS[surface]
    return surface


set_floor_surface()


# --- Transforms --------------------------------------------------------------

def _rot_y(x, y, z, pitch_rad):
    """Rotate about the +y (left) axis. Positive pitch tips the nose down."""
    c, s = math.cos(pitch_rad), math.sin(pitch_rad)
    return (c * x + s * z, y, -s * x + c * z)


def _rot_z(x, y, z, yaw_rad):
    """Rotate about the +z (up) axis. Positive yaw turns left."""
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    return (c * x - s * y, s * x + c * y, z)


def oakd_to_robot(x, y, z, yaw_deg=0.0, pitch_deg=0.0, absolute=False):
    """
    Convert a point from the Oak-D spatial frame to the robot base frame.

    x, y, z     Detection coordinates in metres, DepthAI convention
                (X right, Y up, Z forward) -- i.e. Detection.x/.y/.z.
    yaw_deg     Head pan.  By default relative to home, exactly what
                MoveOakD.getYaw() returns; positive turns the head left.
    pitch_deg   Head tilt, likewise MoveOakD.getPitch(); positive tilts
                the camera down, matching the sign used by offsetPitch.
    absolute    Pass True to give raw servo angles instead of the
                relative-to-home angles the MoveOakD getters return.

    Returns (x, y, z) in the robot frame.
    """
    if absolute:
        yaw_deg -= YAW_HOME_DEG
        pitch_deg -= PITCH_HOME_DEG

    # Optical axes -> head-fixed robot-style axes (forward, left, up).
    fx, fy, fz = z, -x, y

    # Offset of the point from the pitch pivot, still in head-fixed axes.
    fx += LENS_FORWARD
    fy += LENS_LEFT
    fz += LENS_UP

    # Head-fixed -> robot: tilt about the pivot, then pan about the mast.
    fx, fy, fz = _rot_y(fx, fy, fz, math.radians(pitch_deg))
    fx, fy, fz = _rot_z(fx, fy, fz, math.radians(yaw_deg))

    return (fx + PAN_AXIS_X, fy + PAN_AXIS_Y, fz + PITCH_PIVOT_Z)


def robot_to_arm(p):
    """Convert a robot-frame point to the arm base frame."""
    return (p[0] - ARM_ORIGIN[0], p[1] - ARM_ORIGIN[1], p[2] - ARM_ORIGIN[2])


def arm_to_robot(p):
    """Convert an arm-frame point back to the robot base frame."""
    return (p[0] + ARM_ORIGIN[0], p[1] + ARM_ORIGIN[1], p[2] + ARM_ORIGIN[2])


def oakd_to_arm(x, y, z, yaw_deg=0.0, pitch_deg=0.0, absolute=False):
    """Convert an Oak-D detection straight to the arm base frame."""
    return robot_to_arm(oakd_to_robot(x, y, z, yaw_deg, pitch_deg, absolute))


def lens_position(yaw_deg=0.0, pitch_deg=0.0, absolute=False):
    """Robot-frame position of the lens at the given head angles."""
    return oakd_to_robot(0.0, 0.0, 0.0, yaw_deg, pitch_deg, absolute)


def floor_object_to_arm(x, y, z, yaw_deg=0.0, pitch_deg=0.0,
                        obj=None, half_height=None, radius=None,
                        absolute=False):
    """
    Arm-frame centre of an object known to be standing on the floor.

    Takes the same detection coordinates and head angles as
    oakd_to_robot, then applies what the geometry tells us and the depth
    map does not:

      * height comes from the object, not the depth point -- the centre
        is half_height above FLOOR_Z, which beats a bbox whose vertical
        centre wanders with framing and occlusion;
      * the depth point lies on the near surface facing the camera, so
        the centre is one radius further along the horizontal bearing
        from lens to object.

    Give either `obj` (a key in OBJECT_SIZES) or explicit `half_height` /
    `radius`; explicit values win where both are given.  Omitting all of
    them returns the raw surface point at floor height.

    The returned z is NOT half_height: the floor is below the robot
    origin and the arm plate below that, so it works out to about
    half_height - 0.022 on carpet.  Use set_floor_surface() to switch.

    Returns (x, y, z) in the arm frame.
    """
    if obj is not None:
        height, obj_radius = OBJECT_SIZES[obj]
        if half_height is None:
            half_height = height / 2.0
        if radius is None:
            radius = obj_radius
    half_height = half_height or 0.0
    radius = radius or 0.0

    p = oakd_to_robot(x, y, z, yaw_deg, pitch_deg, absolute)

    if radius:
        lens = lens_position(yaw_deg, pitch_deg, absolute)
        bearing = math.atan2(p[1] - lens[1], p[0] - lens[0])
        p = (p[0] + radius * math.cos(bearing),
             p[1] + radius * math.sin(bearing),
             p[2])

    return robot_to_arm((p[0], p[1], FLOOR_Z + half_height))


def arm_reach(p):
    """
    Describe an arm-frame point the way a bus-servo arm wants it:
    (range on the arm horizontal plane, base yaw in degrees, height).
    """
    return (math.hypot(p[0], p[1]), math.degrees(math.atan2(p[1], p[0])), p[2])


def arm_can_reach(p):
    """
    Can the arm get to this arm-frame point?

    Returns (ok, reason).  `reason` is empty when ok, otherwise it names the
    limit that was hit and by how much, in words a caller can pass on.
    """
    rng, yaw, z = arm_reach(p)
    if rng > ARM_MAX_REACH:
        return False, ("%.2f m away, %.2f m beyond the arm's %.2f m reach"
                       % (rng, rng - ARM_MAX_REACH, ARM_MAX_REACH))
    if rng < ARM_MIN_REACH:
        return False, ("%.2f m away, %.2f m inside the arm's %.2f m minimum "
                       "reach -- too close, back off rather than lean in"
                       % (rng, ARM_MIN_REACH - rng, ARM_MIN_REACH))
    if abs(yaw) > ARM_MAX_YAW:
        return False, ("%.0f degrees off the arm's centre line, past its %.0f "
                       "degree limit" % (yaw, ARM_MAX_YAW))
    if z > ARM_MAX_GRASP_Z:
        return False, ("%.2f m up, above the %.2f m the arm grasps at"
                       % (z, ARM_MAX_GRASP_Z))
    return True, ""


def _wrap180(deg):
    """Fold an angle into [-180, 180]."""
    return (deg + 180.0) % 360.0 - 180.0


def approach_pose(p, pose, standoff=GRASP_STANDOFF):
    """
    Where the robot should stand to pick up what it can currently see.

    p       The object in the ROBOT frame, as seen from where the robot is
            now -- arm_to_robot(floor_object_to_arm(...)).
    pose    The robot's current sdp pose (x, y metres; yaw degrees).

    Returns a map-frame (x, y, yaw_deg) goal: the robot square on to the
    object with the object `standoff` in front of the arm base.  Driving
    there puts the grasp inside arm_can_reach, so the caller can hand these
    three numbers straight to a go-to-coordinates move and look again.
    """
    wx, wy, _ = robot_to_world(p, pose)
    heading = pose.yaw + math.degrees(math.atan2(p[1], p[0]))
    d = ARM_ORIGIN[0] + standoff
    h = math.radians(heading)
    return (wx - d * math.cos(h), wy - d * math.sin(h), _wrap180(heading))


def robot_to_world(p, pose):
    """
    Convert a robot-frame point to the map/world frame using an sdp pose
    (pose.x, pose.y in metres, pose.yaw in degrees).  Planar only -- the
    base has no roll or pitch, so world z equals robot z.
    """
    wx, wy, _ = _rot_z(p[0], p[1], p[2], math.radians(pose.yaw))
    return (wx + pose.x, wy + pose.y, p[2])


if __name__ == '__main__':
    def show(label, p):
        print("%-36s % .3f % .3f % .3f" % (label, p[0], p[1], p[2]))

    print("Lens at home (0.055 forward of the pivot):")
    show("  robot frame", lens_position())
    show("  arm origin", ARM_ORIGIN)
    print("  floor on carpet: z = %.3f" % FLOOR_Z)

    print()
    print("Camera at home, object 1.0 m straight down the optical axis:")
    r = oakd_to_robot(0.0, 0.0, 1.0)
    show("  robot frame", r)
    show("  arm frame", robot_to_arm(r))

    print()
    print("Same object with the head panned 30 deg left:")
    show("  robot frame", oakd_to_robot(0.0, 0.0, 1.0, yaw_deg=30.0))

    print()
    print("Raw servo angles (yaw 90, pitch 115) must match home:")
    show("  robot frame", oakd_to_robot(0.0, 0.0, 1.0, 90.0, 115.0, absolute=True))

    print()
    print("Round trip through the arm frame:")
    r = oakd_to_robot(0.12, -0.05, 0.8, yaw_deg=-15.0, pitch_deg=10.0)
    show("  robot frame", r)
    show("  back from arm frame", arm_to_robot(robot_to_arm(r)))

    # A soda can on the floor, head tilted down to see it.  The depth
    # point sits partway up the near face; height is overridden from the
    # can, so only x and y come from the depth map.
    print()
    print("Soda can on the floor, head tilted 35 deg down, 0.80 m range:")
    a = floor_object_to_arm(0.0, 0.0, 0.80, pitch_deg=35.0, obj="soda can")
    show("  arm frame (centre)", a)
    print("  arm reach (range, yaw deg, height): %.3f %.1f %.3f" % arm_reach(a))
    show("  without radius correction",
         floor_object_to_arm(0.0, 0.0, 0.80, pitch_deg=35.0, half_height=0.061))

    print()
    print("Same can, seen 0.15 m right of the optical axis:")
    a = floor_object_to_arm(0.15, 0.0, 0.80, pitch_deg=35.0, obj="soda can")
    show("  arm frame (centre)", a)
    print("  arm reach (range, yaw deg, height): %.3f %.1f %.3f" % arm_reach(a))

    print()
    print("Hard floor drops the floor 4 mm; mounts are unchanged:")
    set_floor_surface("hard")
    show("  lens at home", lens_position())
    show("  same can, arm frame",
         floor_object_to_arm(0.0, 0.0, 0.80, pitch_deg=35.0, obj="soda can"))
    set_floor_surface("carpet")

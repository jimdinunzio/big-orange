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

#!/usr/bin/env python3
"""
Pan/tilt angles for the Oak-D head, in servo degrees.

The single source for the head's angle envelope: move_oak_d drives the
servos with it, robot_frames subtracts the home angles to get head angles
relative to straight ahead, and the console tools sweep about home.

Deliberately import-free so anything can read it.  move_oak_d pulls in
pyFirmata, depthai and the sdp client, which is far too much to drag into
robot_frames or an offline sweep just to learn where straight ahead is.

The homes are trimmed against the mount, not the servo's own 90: measure
them on the robot and set them here, and everything downstream follows.
"""

YAW_HOME_DEG = 84
PITCH_HOME_DEG = 115

YAW_LIMITS_DEG = (0, 180)
PITCH_LIMITS_DEG = (0, 150)

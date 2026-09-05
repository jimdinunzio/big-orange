"""
NanoOwlManager — manages OAK-D frame streaming to the NanoOWL server
and computes spatial coordinates from OWL detections using the OAK-D
VPU's SpatialLocationCalculator (ROI-based depth query, no CPU cost).

Usage from main.py:
    mgr = NanoOwlManager(owl_client, mdai)
    mgr.start_streaming()
    mgr.set_prompt("[lamp, lava lamp]")
    ...
    found, det = mgr.check_for_object("lamp")
    if found:
        loc = getLocationOfObj(sdp, "lamp", det, cam_yaw=...)
    ...
    mgr.stop_streaming()
"""

import math
import time
from threading import Thread
from move_oak_d import MoveOakD
from latte_panda_arduino import LattePandaArduino

# OAK-D color camera horizontal field of view (degrees)
OAKD_HFOV_DEG = 69.0


class OwlSpatialDetection:
    """Detection with spatial info, compatible with MyDetection for use with getLocationOfObj."""

    def __init__(self, label, bbox_norm, x_m, y_m, z_m, theta_deg):
        """
        Args:
            label: detected object label string
            bbox_norm: [cx, cy] normalized 0-1 in the preview frame
            x_m: lateral offset in meters (from VPU)
            y_m: vertical offset in meters (from VPU)
            z_m: depth in meters (from VPU)
            theta_deg: horizontal angle in degrees (positive = right of center)
        """
        self.label = label
        self.bboxCtr = bbox_norm
        self.x = x_m
        self.y = y_m
        self.z = z_m
        self.theta = theta_deg
        self.confidence = 0.0  # set by caller from detection scores

    def __repr__(self):
        return (f"OwlSpatialDetection(label={self.label!r}, z={self.z:.2f}m, "
                f"theta={self.theta:.1f}deg, confidence={self.confidence:.3f})")


class NanoOwlManager:
    """Manages frame streaming to NanoOWL server and VPU-based depth lookups."""

    def __init__(self, owl_client, mdai):
        """
        Args:
            owl_client: NanoOwlClient instance (already connected)
            mdai: MyDepthAI instance (already running)
        """
        self._owl = owl_client
        self._mdai = mdai

        # Streaming state
        self._streaming = False
        self._paused = False
        self._stream_thread = None

    # ------------------------------------------------------------------ #
    #  Streaming control
    # ------------------------------------------------------------------ #

    def start_streaming(self, fps=16):
        """Start pushing OAK-D preview frames to the OWL server."""
        if self._streaming:
            return
        self._streaming = True
        self._stream_thread = Thread(
            target=self._stream_loop, args=(fps,), name="owl_stream", daemon=True
        )
        self._stream_thread.start()

    def stop_streaming(self):
        """Stop pushing frames."""
        self._streaming = False
        if self._stream_thread:
            self._stream_thread.join(timeout=3)
            self._stream_thread = None

    @property
    def is_streaming(self):
        return self._streaming

    def pause_streaming(self):
        """Temporarily pause frame pushing so detection queries don't contend."""
        self._paused = True

    def resume_streaming(self):
        """Resume frame pushing after a pause."""
        self._paused = False

    def push_fresh_frame(self):
        """Push one frame synchronously and wait for server inference."""
        preview, seq = self._mdai.getLatestFrame()
        if preview is not None:
            self._owl.push_frame(preview, seq)
            time.sleep(0.35)

    def _stream_loop(self, fps):
        interval = 1.0 / fps
        last_seq = -1
        while self._streaming:
            if not self._paused:
                preview, seq = self._mdai.getLatestFrame()
                if preview is not None and seq != last_seq:
                    last_seq = seq
                    self._owl.push_frame(preview, seq)
            time.sleep(interval)

    # ------------------------------------------------------------------ #
    #  Prompt helpers (delegate to client)
    # ------------------------------------------------------------------ #

    def set_prompt(self, prompt):
        return self._owl.set_prompt(prompt)

    def clear_prompt(self):
        return self._owl.clear_prompt()

    # ------------------------------------------------------------------ #
    #  Detection retrieval
    # ------------------------------------------------------------------ #

    def get_detections_nms(self, iou_threshold=0.5, score_threshold=0.08):
        """Get NMS-filtered detections from the OWL server."""
        return self._owl.get_detections_nms(iou_threshold, score_threshold)

    def check_for_object(self, obj_name):
        """
        Check if an object matching obj_name is currently detected.

        Compatible with the checkForObject(obj) callback signature used by
        searchForObject — returns (found: bool, spatial_detection_or_None).
        Returns only the single highest-scoring match.
        """
        dets_result = self.get_detections_nms()
        if dets_result is None:
            return False, None

        det_list = dets_result.get("detections", [])

        # Find the highest-scoring detection whose label matches
        best = None
        best_score = 0.0
        for d in det_list:
            label = d.get("label", "")
            if label.lower() != obj_name.lower():
                continue
            scores = d.get("scores", [])
            score = max(scores) if scores else 0.0
            if score > best_score:
                best = d
                best_score = score

        if best is None:
            return False, None

        box = best.get("box", [])
        if len(box) != 4:
            return False, None

        self._mdai.clear_roi_rects()
        spatial = self._box_to_spatial(box, confidence=best_score)
        if spatial is not None:
            spatial.label = obj_name
            spatial.confidence = best_score
        return True, spatial

    def check_for_all_objects(self, obj_name):
        """
        Return all detections matching obj_name (not just the best one).

        Returns (found: bool, list_of_OwlSpatialDetection).
        All matching bboxes are drawn on the overlay simultaneously.
        """
        dets_result = self.get_detections_nms()
        if dets_result is None:
            return False, []

        det_list = dets_result.get("detections", [])

        matches = []
        for d in det_list:
            label = d.get("label", "")
            if label.lower() != obj_name.lower():
                continue
            scores = d.get("scores", [])
            score = max(scores) if scores else 0.0
            box = d.get("box", [])
            if len(box) == 4:
                matches.append((score, box))

        if not matches:
            return False, []

        self._mdai.clear_roi_rects()
        spatials = []
        for score, box in matches:
            spatial = self._box_to_spatial(box, confidence=score)
            if spatial is not None:
                spatial.label = obj_name
                spatial.confidence = score
                spatials.append(spatial)

        return bool(spatials), spatials

    # ------------------------------------------------------------------ #
    #  Depth via VPU SpatialLocationCalculator
    # ------------------------------------------------------------------ #

    def _box_to_spatial(self, box, confidence=0.0):
        """
        Convert an OWL pixel-coordinate bbox into an OwlSpatialDetection
        using the OAK-D VPU's SpatialLocationCalculator for depth.

        Args:
            box: [x1, y1, x2, y2] in preview-frame pixel coordinates

        Returns:
            OwlSpatialDetection or None if preview size unavailable.
        """
        preview_size = self._mdai.getPreviewSize()
        if preview_size is None:
            return None
        pw, ph = preview_size

        # Normalize bbox to 0-1 range
        xmin = max(0.0, box[0] / pw)
        ymin = max(0.0, box[1] / ph)
        xmax = min(1.0, box[2] / pw)
        ymax = min(1.0, box[3] / ph)

        # Bbox center normalized
        cx = (xmin + xmax) / 2.0
        cy = (ymin + ymax) / 2.0

        # Horizontal angle from center of frame
        nx = (cx - 0.5) * 2.0  # -1 (left) to +1 (right)
        theta = nx * (OAKD_HFOV_DEG / 2.0)

        # Query VPU for spatial coordinates at this ROI
        spatial_coords = self._mdai.getSpatialForROI(xmin, ymin, xmax, ymax, draw=True, confidence=confidence)
        if spatial_coords is None:
            # No depth data — still draw the detection bbox so the overlay updates
            conf_str = f"{confidence:.2f}" if confidence > 0 else ""
            self._mdai.drawROIRect(xmin, ymin, xmax, ymax, text=conf_str)
            return OwlSpatialDetection("", [cx, cy], 0.0, 0.0, 0.0, theta)

        x_m, y_m, z_m = spatial_coords
        # Use theta from VPU's x/z if depth is valid, otherwise use FOV-based
        if z_m > 0:
            theta = math.degrees(-math.asin(x_m / z_m)) if z_m != 0 else 0.0

        return OwlSpatialDetection("", [cx, cy], x_m, y_m, z_m, theta)


if __name__ == "__main__":
    import sys
    from nano_owl_client import NanoOwlClient
    from my_depthai import MyDepthAI
    import robot_frames

    client = NanoOwlClient()
    if not client.connect():
        print("Could not connect to NanoOWL server.")
        sys.exit(1)

    print(f"Server status: {client.get_status()}")
    print(f"is_enabled={client.is_enabled()}  is_running={client.is_running()}")

    _lpArduino = LattePandaArduino()
    _lpArduino.initialize()
    m = MoveOakD()
    m.initialize(_lpArduino.board)
    m.pitchServo.setAngle(145)

    client.enable()
    time.sleep(0.5)
    print(f"After enable: is_enabled={client.is_enabled()}  is_running={client.is_running()}")

    # Create and start MyDepthAI in a background thread
    mdai = MyDepthAI()
    mdai_thread = Thread(target=mdai.startUp, args=("TOP", True, False), daemon=True)
    mdai_thread.start()
    time.sleep(2)  # Wait for camera to initialize

    mgr = NanoOwlManager(client, mdai)
    mgr.start_streaming(fps=16)
    mdai.show_yolo_boxes = False

    import queue as _queue
    import sys as _sys
    _input_q = _queue.Queue()
    _stop = [False]

    HELP = """
Commands:
  <text>          set the OWL prompt and track it (e.g. soda can)
  truth F [L]     tell it where the can REALLY is: tape-measured from the ARM
                  BASE, metres, F forward and L to the robot's left (default 0).
                  Bare 'truth' forgets it.  This is what makes `check` possible
  check           THE test: is the real can where the numbers say?  Reads at
                  several head angles and reports the end-to-end bias -- what
                  pick_can would be handed minus where the can actually is
  where           one reading of the tracked object: oakd -> robot -> arm,
                  with the error against `truth` if one is set
  sweep           consistency only, no truth needed: the same can from several
                  head angles must read the same.  Catches only the lens
                  offsets -- see sweep.__doc__ for what a flat sweep does NOT
                  prove, and prefer `check` for a real answer
  pitch N         aim the head, servo degrees (115 level, 145 full down)
  yaw N           pan the head, servo degrees (90 straight ahead)
  floor carpet|hard   which surface the robot is standing on (sets FLOOR_Z)
  help, quit
"""

    # Input thread — uses readline so we control when "prompt> " appears,
    # reprinting it immediately after each Enter without waiting for the detect loop.
    def _input_loop():
        print(HELP)
        _sys.stdout.write("prompt> ")
        _sys.stdout.flush()
        while not _stop[0]:
            try:
                raw = _sys.stdin.readline()
            except EOFError:
                _input_q.put("quit")
                break
            raw = raw.strip()
            _input_q.put(raw)
            if raw.lower() == "quit":
                break
            _sys.stdout.write("prompt> ")
            _sys.stdout.flush()

    # ---------------------------------------------------------------- #
    #  Frame-transform check
    #
    #  The point of this is not the number -- it is whether the number
    #  HOLDS STILL.  A soda can on the floor does not move when the head
    #  moves, so if oakd -> robot -> arm is right, every head angle must
    #  put the can at the same arm-frame point.  Whatever it drifts by is
    #  the error in the mount geometry, and which axis drifts against
    #  which rotation says which constant is wrong (see `sweep`).
    # ---------------------------------------------------------------- #

    READ_SAMPLES = 7

    def read_object(obj, samples=READ_SAMPLES):
        """Median-nearest detection over a few frames, as pick_up does.

        Returns (detection, hits, tries).  The median SAMPLE rather than a
        per-axis mean: with two cans in view the nearest one can flip
        between frames, and a mean would land between them.
        """
        got = []
        for _ in range(samples):
            found, spatials = mgr.check_for_all_objects(obj)
            valid = [s for s in (spatials or []) if s.z > 0]
            if found and valid:
                got.append(min(valid, key=lambda s: s.z))
            time.sleep(1.0 / 16)
        if not got:
            return None, 0, samples
        got.sort(key=lambda s: s.z)
        return got[len(got) // 2], len(got), samples

    # Where the can REALLY is, arm frame, from a tape measure.  Only x and y:
    # z is never sensed -- floor_object_to_arm takes it from FLOOR_Z and the
    # object's height -- so a camera reading cannot be wrong about it, and
    # checking z means measuring the can and the ride height, not looking.
    TRUTH = [None]

    def error_vs_truth(centre):
        """End-to-end miss: what pick_can would be handed, minus the truth.

        Returns (ex, ey, magnitude, truth_radius, system_radius) or None.
        """
        if TRUTH[0] is None:
            return None
        tx, ty = TRUTH[0]
        ex, ey = centre[0] - tx, centre[1] - ty
        return (ex, ey, math.hypot(ex, ey),
                math.hypot(tx, ty), math.hypot(centre[0], centre[1]))

    def judge(mag, radial):
        """Say what a miss of this size does to the pick, in the README's own
        terms: the near edge allows about 25 mm either side of the standoff."""
        band = "%.2f-%.2f" % (robot_frames.ARM_SWEET_MIN, robot_frames.ARM_SWEET_MAX)
        aimed = robot_frames.GRASP_STANDOFF + radial
        lines = ["a can placed at the %.2f m standoff would be picked as if at "
                 "%.3f m" % (robot_frames.GRASP_STANDOFF, aimed)]
        if robot_frames.ARM_SWEET_MIN <= aimed <= robot_frames.ARM_SWEET_MAX:
            lines.append("still inside the %s m sweet spot" % band)
        elif robot_frames.ARM_MIN_REACH <= aimed <= robot_frames.ARM_MAX_REACH:
            lines.append("OUTSIDE the %s m sweet spot, though still in the "
                         "%.2f-%.2f m band" % (band, robot_frames.ARM_MIN_REACH,
                                               robot_frames.ARM_MAX_REACH))
        else:
            lines.append("OUT OF THE BAND ENTIRELY -- picks would not solve")
        lines.append("the gripper closes %.0f mm from the can's centre, on a "
                     "%.0f mm can" % (mag * 1000,
                                      robot_frames.OBJECT_SIZES["soda can"][1] * 2000))
        if mag < 0.010:
            lines.append("=> GOOD")
        elif mag < 0.025:
            lines.append("=> USABLE, but it eats %.0f of the ~25 mm of nav error "
                         "the near edge allows" % (mag * 1000))
        else:
            lines.append("=> TOO BIG, find this before trusting a pick")
        return lines

    def show_error(centre, indent="  "):
        e = error_vs_truth(centre)
        if e is None:
            print("%sno truth set -- `truth F [L]` with a tape measure to get "
                  "the real error" % indent)
            return
        ex, ey, mag, tr, sr = e
        print("%smeasured  arm x %+.3f  y %+.3f   radius %.3f" % (indent, *TRUTH[0], tr))
        print("%ssystem    arm x %+.3f  y %+.3f   radius %.3f" % (indent, centre[0], centre[1], sr))
        print("%sERROR         %+.3f    %+.3f   radius %+.3f -> %.0f mm off"
              % (indent, ex, ey, sr - tr, mag * 1000))
        for line in judge(mag, sr - tr):
            print("%s  %s" % (indent, line))

    def check(obj, pitches=(125, 130, 135, 140, 145), yaws=(80, 90, 100)):
        """Is the real can where the numbers say it is?

        The system-level test, and the one that catches everything: OWL's
        bbox, the depth ROI, every frame transform and the object correction
        all fold into one number here -- what pick_can gets handed, minus
        where the can actually is.

        Reads from several head angles because one reading cannot tell a
        systematic error from a noisy one.  BIAS (the mean) is what a pick
        would be off by every single time and is the number that matters;
        NOISE (the spread) says how much to trust the bias.
        """
        if TRUTH[0] is None:
            print("  set the truth first: put the can somewhere you can measure,")
            print("  then `truth <forward> [<left>]` in metres from the ARM BASE.")
            print("  The arm base is %.3f m forward of the chassis axis."
                  % robot_frames.ARM_ORIGIN[0])
            return
        if obj not in robot_frames.OBJECT_SIZES:
            print("  check needs a known object size; %r is not in OBJECT_SIZES" % obj)
            return

        start_yaw = m.yawServo.getAngle(relToHome=False)
        start_pitch = m.pitchServo.getAngle(relToHome=False)
        tx, ty = TRUTH[0]
        rows = []
        print()
        print("  truth (tape)   arm x %+.3f  y %+.3f   radius %.3f"
              % (tx, ty, math.hypot(tx, ty)))
        print()
        print("  %-16s %8s %8s %8s %8s %7s" % ("head", "sys x", "sys y", "err x", "err y", "miss"))
        try:
            for label, setter, angles, other in (
                    ("pitch", m.setPitch, pitches, None),
                    ("yaw", m.setYaw, yaws, None)):
                for a in angles:
                    setter(a)
                    time.sleep(1.0)
                    det, hits, tries = read_object(obj)
                    if det is None:
                        print("  %-16s %8s   (not seen)" % ("%s %.0f" % (label, a), "-"))
                        continue
                    centre, _ = arm_point(det, obj)
                    ex, ey, mag, _, _ = error_vs_truth(centre)
                    rows.append((centre, ex, ey, mag))
                    print("  %-16s %8.3f %8.3f %+8.3f %+8.3f %7.0f"
                          % ("%s %.0f" % (label, a), centre[0], centre[1], ex, ey, mag * 1000))
                setter(start_pitch if label == "pitch" else start_yaw)
                time.sleep(0.5)
        finally:
            m.setYaw(start_yaw)
            m.setPitch(start_pitch)

        if not rows:
            print("  the can was never seen -- nothing to judge")
            return
        n = len(rows)
        bx = sum(r[1] for r in rows) / n
        by = sum(r[2] for r in rows) / n
        bias = math.hypot(bx, by)
        nx = max(r[0][0] for r in rows) - min(r[0][0] for r in rows)
        ny = max(r[0][1] for r in rows) - min(r[0][1] for r in rows)
        # The radial part of the bias is what slides the grasp along the band.
        sr = sum(math.hypot(r[0][0], r[0][1]) for r in rows) / n
        radial = sr - math.hypot(tx, ty)
        print()
        print("  BIAS  (mean)   %+.3f %+.3f  -> %.0f mm, radial %+.0f mm"
              % (bx, by, bias * 1000, radial * 1000))
        print("  NOISE (spread)  %.3f  %.3f  -> %.0f mm, over %d readings"
              % (nx, ny, max(nx, ny) * 1000, n))
        print()
        if max(nx, ny) < 0.002 and bias < 0.002:
            print("  Both under a millimetre -- the chain is as good as the tape.")
        elif max(nx, ny) > bias:
            print("  Noise exceeds the bias: take it as 'no bias worth chasing yet'")
            print("  rather than a measured offset. Steady the can and re-run.")
        for line in judge(bias, radial):
            print("  %s" % line)
        print()
        print("  A bias is a constant the geometry is wrong by, and `sweep` cannot")
        print("  see it: PAN_AXIS_X/Y shift arm x/y directly, a servo mis-zero")
        print("  rotates the whole answer, and FLOOR_Z and the can's own size set")
        print("  arm z outright. Correct those in robot_frames.py and re-run.")

    def current_surface():
        """Which surface is selected now -- robot_frames keeps only FLOOR_Z,
        and `floor` can have changed it since import."""
        for name, height in robot_frames.PLATE_HEIGHTS.items():
            if abs(-height - robot_frames.FLOOR_Z) < 1e-9:
                return name
        return "custom"

    def head_angles():
        """(yaw, pitch) relative to home -- what robot_frames wants -- and
        the raw servo angles, which is what you set and can eyeball."""
        return (m.getYaw(), m.getPitch(),
                m.yawServo.getAngle(relToHome=False),
                m.pitchServo.getAngle(relToHome=False))

    def arm_point(det, obj):
        """Arm-frame can centre, and the untouched depth point beside it."""
        yaw, pitch, _, _ = head_angles()
        centre = robot_frames.floor_object_to_arm(
            det.x, det.y, det.z, yaw_deg=yaw, pitch_deg=pitch, obj=obj)
        raw = robot_frames.oakd_to_arm(det.x, det.y, det.z,
                                       yaw_deg=yaw, pitch_deg=pitch)
        return centre, raw

    def show_reading(obj):
        det, hits, tries = read_object(obj)
        if det is None:
            print("  no %s with valid depth in %d frames" % (obj, tries))
            return None
        yaw, pitch, syaw, spitch = head_angles()
        known = obj in robot_frames.OBJECT_SIZES
        centre, raw = (arm_point(det, obj) if known else
                       (None, robot_frames.oakd_to_arm(det.x, det.y, det.z,
                                                       yaw_deg=yaw, pitch_deg=pitch)))

        print()
        print("  %s   %d/%d frames, confidence %.2f" % (obj, hits, tries, det.confidence))
        print("  head    yaw %+.1f  pitch %+.1f   (servo %.0f / %.0f)"
              % (yaw, pitch, syaw, spitch))
        print("  oakd    x %+.3f  y %+.3f  z %+.3f   (right / up / forward)"
              % (det.x, det.y, det.z))
        rb = robot_frames.oakd_to_robot(det.x, det.y, det.z, yaw, pitch)
        print("  robot   x %+.3f  y %+.3f  z %+.3f   depth point, from the chassis axis"
              % rb)
        print("  arm     x %+.3f  y %+.3f  z %+.3f   depth point, from the arm base" % raw)
        if centre is None:
            print("  (no entry in OBJECT_SIZES for %r, so no centre correction)" % obj)
            return None

        print("  arm     x %+.3f  y %+.3f  z %+.3f   CENTRE -- what pick_up sends"
              % centre)
        print("          correction from the depth point: %+.3f %+.3f %+.3f"
              % (centre[0] - raw[0], centre[1] - raw[1], centre[2] - raw[2]))
        rng, byaw, height = robot_frames.arm_reach(centre)
        ok, reason = robot_frames.arm_can_reach(centre)
        print("  reach   range %.3f m, base yaw %+.1f deg, height %.3f m"
              % (rng, byaw, height))
        print("  verdict %s" % ("IN REACH" if ok else "out of reach -- " + reason))
        print()
        print()
        show_error(centre)
        rc = robot_frames.arm_to_robot(centre)
        print()
        print("  the same point measured from elsewhere, if that is easier to reach:")
        print("    %.3f m forward of the chassis axis, %.3f m to the %s"
              % (rc[0], abs(rc[1]), "left" if rc[1] >= 0 else "right"))
        print("    the arm base is %.3f m forward of that axis and %.3f m below"
              % (robot_frames.ARM_ORIGIN[0], -robot_frames.ARM_ORIGIN[2]))
        print("    arm z %+.3f is NOT sensed -- it is the %s floor %.3f m down plus"
              % (centre[2], current_surface(), -robot_frames.FLOOR_Z))
        print("    half the can's %.3f m height, so measure those to check it"
              % robot_frames.OBJECT_SIZES[obj][0])
        return centre

    def sweep(obj, pitches=(125, 130, 135, 140, 145), yaws=(75, 85, 95, 105)):
        """Read one stationary object from several head angles.

        A NARROW test, so be clear about what it can and cannot catch.
        Only the two lens offsets are applied BEFORE the head rotations, so
        only they change with head angle:

          arm x spreads over the PITCH sweep -> LENS_UP  (about 3x the
                                                spread), then LENS_FORWARD
          arm y spreads over the YAW sweep   -> LENS_FORWARD (about 2x the
                                                spread), then LENS_UP

        Everything else is applied after the rotations, or not at all, and
        so shifts EVERY row by the same amount.  A flat sweep says nothing
        about any of it, and only a tape measure will:

          PAN_AXIS_X / PAN_AXIS_Y     constant offset in arm x / y
          PITCH_PIVOT_Z               no effect at all on a floor object --
                                      floor_object_to_arm discards the
                                      measured height
          a constant servo mis-zero   rotates every row alike
          FLOOR_Z, OBJECT_SIZES       constant offset in arm z

        arm z is constant BY CONSTRUCTION here -- it comes from the floor
        and the object's own height, never from depth -- so its zero spread
        is not evidence of anything.  Use `where` and a tape measure for
        the absolute check; this only tells you the lens offsets are sane.
        """
        if obj not in robot_frames.OBJECT_SIZES:
            print("  sweep needs a known object size; %r is not in OBJECT_SIZES" % obj)
            return
        start_yaw = m.yawServo.getAngle(relToHome=False)
        start_pitch = m.pitchServo.getAngle(relToHome=False)

        def run(label, setter, angles, restore):
            rows = []
            print()
            print("  %-14s %8s %8s %8s %8s" % (label, "oakd z", "arm x", "arm y", "arm z"))
            for a in angles:
                setter(a)
                time.sleep(1.0)  # let the servo settle and the pipeline catch up
                det, hits, tries = read_object(obj)
                if det is None:
                    print("  %-14s %8s   (not seen)" % ("%.0f" % a, "-"))
                    continue
                centre, _ = arm_point(det, obj)
                rows.append(centre)
                print("  %-14s %8.3f %8.3f %8.3f %8.3f"
                      % ("%.0f" % a, det.z, centre[0], centre[1], centre[2]))
            setter(restore)
            time.sleep(0.5)
            if len(rows) < 2:
                print("  not enough readings to judge the spread")
                return
            spread = [max(r[i] for r in rows) - min(r[i] for r in rows) for i in range(3)]
            print("  %-14s %8s %8.3f %8.3f %8.3f" % ("spread", "", *spread))
            # z cannot spread -- it never came from the depth map -- so judge
            # on x and y alone or this always passes.
            worst = max(spread[0], spread[1])
            print("  %s: worst of arm x/y moves %.0f mm across the sweep"
                  % ("GOOD" if worst < 0.010 else "SUSPECT", worst * 1000))

        try:
            run("pitch servo", m.setPitch, pitches, start_pitch)
            run("yaw servo", m.setYaw, yaws, start_yaw)
        finally:
            m.setYaw(start_yaw)
            m.setPitch(start_pitch)
        print()
        print("  A stationary can should read the same from every head angle.")
        print("  Drift in arm x over pitch accuses LENS_UP; drift in arm y over")
        print("  yaw accuses LENS_FORWARD -- roughly 2-3x the spread you see.")
        print("  A FLAT sweep does NOT clear PAN_AXIS_X/Y, PITCH_PIVOT_Z, a servo")
        print("  mis-zero or FLOOR_Z: those shift every row alike and only `where`")
        print("  against a tape measure can see them.  See sweep.__doc__.")

    input_thread = Thread(target=_input_loop, name="owl_input", daemon=True)
    input_thread.start()

    # Main loop — handles detection; picks up new prompts from the queue
    current_obj = ""
    poll_interval = 1.0 / 16

    try:
        while True:
            # Check for new prompt or command (non-blocking)
            try:
                raw = _input_q.get_nowait()
                if not raw or raw.lower() == "quit":
                    break
                cmd = raw.split()
                verb = cmd[0].lower()

                if verb == "help":
                    print(HELP)
                elif verb == "truth":
                    if len(cmd) == 1:
                        TRUTH[0] = None
                        print("  truth forgotten")
                    else:
                        try:
                            fwd = float(cmd[1])
                            left = float(cmd[2]) if len(cmd) > 2 else 0.0
                        except ValueError:
                            print("  truth needs metres: truth <forward> [<left>]")
                        else:
                            TRUTH[0] = (fwd, left)
                            print("  truth set: the can centre is %.3f m forward and "
                                  "%.3f m to the %s of the ARM BASE"
                                  % (fwd, abs(left), "left" if left >= 0 else "right"))
                elif verb in ("where", "sweep", "check"):
                    if not current_obj:
                        print("  set a prompt first, e.g. 'soda can'")
                    elif verb == "where":
                        show_reading(current_obj)
                    elif verb == "check":
                        check(current_obj)
                    else:
                        sweep(current_obj)
                elif verb in ("pitch", "yaw") and len(cmd) == 2:
                    try:
                        angle = float(cmd[1])
                    except ValueError:
                        print("  %s needs a servo angle in degrees" % verb)
                    else:
                        (m.setPitch if verb == "pitch" else m.setYaw)(angle)
                        y, p, sy, sp = head_angles()
                        print("  head now yaw %+.1f pitch %+.1f (servo %.0f / %.0f)"
                              % (y, p, sy, sp))
                elif verb == "floor" and len(cmd) == 2:
                    try:
                        surface = robot_frames.set_floor_surface(cmd[1].lower())
                    except KeyError:
                        print("  surface must be one of: %s"
                              % ", ".join(robot_frames.PLATE_HEIGHTS))
                    else:
                        print("  floor is %s, %.3f m below the robot origin"
                              % (surface, -robot_frames.FLOOR_Z))
                else:
                    prompt = raw if raw.startswith("[") else f"[{raw}]"
                    current_obj = raw.strip("[]").split(",")[0].strip()
                    mgr.set_prompt(prompt)
            except _queue.Empty:
                pass

            # Detect against current prompt — update overlay, no terminal output
            if current_obj:
                found, spatials = mgr.check_for_all_objects(current_obj)
                if found and spatials:
                    mdai.drawText(f"{current_obj} x{len(spatials)}", 1, 24)

            time.sleep(poll_interval)
    finally:
        _stop[0] = True
        mdai.show_yolo_boxes = True
        mgr.stop_streaming()
        mgr.clear_prompt()
        client.disable()
        mdai.run_flag = False
        mdai.outer_run_flag = False
        mdai_thread.join(timeout=5)
        client.disconnect()
        print("Exited.")

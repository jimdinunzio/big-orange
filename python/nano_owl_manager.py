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

    def start_streaming(self, fps=10):
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

    def get_detections_nms(self, iou_threshold=0.5, score_threshold=0.1):
        """Get NMS-filtered detections from the OWL server."""
        return self._owl.get_detections_nms(iou_threshold, score_threshold)

    def check_for_object(self, obj_name):
        """
        Check if an object matching obj_name is currently detected.

        Compatible with the checkForObject(obj) callback signature used by
        searchForObject — returns (found: bool, spatial_detection_or_None).

        The returned OwlSpatialDetection has .z, .theta, .bboxCtr so it can be
        passed to getLocationOfObj / getLocationNearObj.
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

        spatial = self._box_to_spatial(box)
        if spatial is not None:
            spatial.label = obj_name
            spatial.confidence = best_score
        return True, spatial

    # ------------------------------------------------------------------ #
    #  Depth via VPU SpatialLocationCalculator
    # ------------------------------------------------------------------ #

    def _box_to_spatial(self, box):
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
        spatial_coords = self._mdai.getSpatialForROI(xmin, ymin, xmax, ymax, draw=True)
        if spatial_coords is None:
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

    client = NanoOwlClient()
    if not client.connect():
        print("Could not connect to NanoOWL server.")
        sys.exit(1)

    # Create and start MyDepthAI in a background thread
    mdai = MyDepthAI()  # Customize params if needed, e.g., MyDepthAI(model="tinyYolo")
    mdai_thread = Thread(target=mdai.startUp, args=("TOP", True, False), daemon=True)  # loc="TOP" or "BOTTOM", showRgbWindow=False, showDepthWindow=False
    mdai_thread.start()
    time.sleep(2)  # Brief wait for mdai to initialize (adjust as needed)

    mgr = NanoOwlManager(client, mdai)

    prompt = "[lava lamp]"
    if len(sys.argv) > 1:
        prompt = sys.argv[1]

    print(f"Setting prompt: {prompt}")
    mgr.set_prompt(prompt)

    # Start streaming frames from the OAK-D to the NanoOWL server
    mgr.start_streaming(fps=5)

    print("Polling detections for 3 seconds")
    try:
        for i in range(6):
            dets = mgr.get_detections_nms()
            if dets is not None:
                det_list = dets.get("detections", [])
                frame_seq = dets.get("frame_seq")
                print(f"  [{i}] frame_seq={frame_seq}, {len(det_list)} detection(s)")
                for d in det_list:
                    label = d.get("label", "?")
                    box = d.get("box", [])
                    scores = d.get("scores", [])
                    box_str = ", ".join(f"{v:.1f}" for v in box) if box else "N/A"
                    score_str = ", ".join(f"{s:.3f}" for s in scores) if scores else "N/A"
                    print(f"    {label}: box=[{box_str}] scores=[{score_str}]")
            else:
                print(f"  [{i}] No detections yet")
            time.sleep(0.5)

        # Test check_for_object (with spatial using the running mdai instance)
        obj = prompt.strip("[]").split(",")[0].strip()
        print(f"\ncheck_for_object('{obj}'):")
        for i in range(1000):
            found, spatial = mgr.check_for_object(obj)
            print(f"  found={found}, spatial={spatial}")
            if found and spatial:
                mdai.drawText(f"{spatial.label}",1,24)
                print(f"  label={spatial.label}, z={spatial.z:.2f}m, theta={spatial.theta:.1f}deg")
            time.sleep(0.25)
    except KeyboardInterrupt:
        print("\nInterrupted")

    finally:
        mgr.stop_streaming()

    mgr.clear_prompt()
    # Stop mdai
    mdai.run_flag = False
    mdai.outer_run_flag = False
    mdai_thread.join(timeout=5)

    client.disconnect()
    print("Done.")

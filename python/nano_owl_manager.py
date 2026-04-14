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

    client = NanoOwlClient()
    if not client.connect():
        print("Could not connect to NanoOWL server.")
        sys.exit(1)

    print(f"Server status: {client.get_status()}")
    print(f"is_enabled={client.is_enabled()}  is_running={client.is_running()}")

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

    # Input thread — uses readline so we control when "prompt> " appears,
    # reprinting it immediately after each Enter without waiting for the detect loop.
    def _input_loop():
        print("\nEnter a prompt (e.g. 'lamp' or '[lamp, table lamp]') to search, or 'quit' to exit.")
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

    input_thread = Thread(target=_input_loop, name="owl_input", daemon=True)
    input_thread.start()

    # Main loop — handles detection; picks up new prompts from the queue
    current_obj = ""
    poll_interval = 1.0 / 16

    try:
        while True:
            # Check for new prompt (non-blocking)
            try:
                raw = _input_q.get_nowait()
                if not raw or raw.lower() == "quit":
                    break
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

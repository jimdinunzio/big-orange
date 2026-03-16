#!/usr/bin/env python3
"""
NanoOwl XML-RPC Client Example

Connects to the NanoOwl XML-RPC server for open-vocabulary object detection.

The server starts in disabled (low power) mode. Call enable() to start processing.
Auto-disables after 30s of no get_detections() calls.

Usage:
    python3 nano_owl_client.py         # Run demo
    python3 nano_owl_client.py -i      # Interactive mode
"""

import xmlrpc.client
import time
import sys
import cv2
from typing import Optional, Dict, List

# Default server address
DEFAULT_SERVER_URL = "http://192.168.55.1:8000/"


class NanoOwlClient:
    """
    Client class for interacting with the NanoOwl XML-RPC server.

    Provides methods for open-vocabulary object detection including:
    - Connection management (ping, connect, disconnect)
    - Enable/disable control (enable, disable, is_enabled)
    - Status queries (get_status, is_running)
    - Prompt management (get_prompt, set_prompt)
    - Detection retrieval (get_detections)
    - System control (reboot)
    """

    def __init__(self, server_url: str = DEFAULT_SERVER_URL):
        self.server_url = server_url
        self._proxy: Optional[xmlrpc.client.ServerProxy] = None
        self._connected = False

    def connect(self, timeout: float = 5.0) -> bool:
        try:
            self._proxy = xmlrpc.client.ServerProxy(self.server_url, allow_none=True)
            result = self._proxy.ping()
            self._connected = True
            print(f"Connected to NanoOWL server at {self.server_url}")
            return True
        except ConnectionRefusedError:
            print(f"Error: Could not connect to server at {self.server_url}")
            self._connected = False
            return False
        except Exception as e:
            print(f"Error connecting to server: {e}")
            self._connected = False
            return False

    def is_connected(self) -> bool:
        return self._connected

    def ping(self) -> Optional[str]:
        if not self._connected or self._proxy is None:
            return None
        try:
            return str(self._proxy.ping())
        except Exception as e:
            print(f"Ping error: {e}")
            self._connected = False
            return None

    def get_status(self) -> Optional[dict]:
        if not self._connected or self._proxy is None:
            return None
        try:
            result = self._proxy.get_status()
            return dict(result) if result else None
        except Exception as e:
            print(f"Get status error: {e}")
            return None

    def is_running(self) -> Optional[bool]:
        if not self._connected or self._proxy is None:
            return None
        try:
            return bool(self._proxy.is_running())
        except Exception as e:
            print(f"Is running error: {e}")
            return None

    def enable(self) -> bool:
        if not self._connected or self._proxy is None:
            return False
        try:
            return bool(self._proxy.enable())
        except Exception as e:
            print(f"Enable error: {e}")
            return False

    def disable(self) -> bool:
        if not self._connected or self._proxy is None:
            return False
        try:
            return bool(self._proxy.disable())
        except Exception as e:
            print(f"Disable error: {e}")
            return False

    def is_enabled(self) -> Optional[bool]:
        if not self._connected or self._proxy is None:
            return None
        try:
            return bool(self._proxy.is_enabled())
        except Exception as e:
            print(f"Is enabled error: {e}")
            return None

    def get_prompt(self) -> Optional[str]:
        if not self._connected or self._proxy is None:
            return None
        try:
            return str(self._proxy.get_prompt())
        except Exception as e:
            print(f"Get prompt error: {e}")
            return None

    def set_prompt(self, prompt: str) -> bool:
        if not self._connected or self._proxy is None:
            return False
        try:
            return bool(self._proxy.set_prompt(prompt))
        except Exception as e:
            print(f"Set prompt error: {e}")
            return False

    def push_frame(self, frame, seq: int, jpeg_quality: int = 85) -> bool:
        """
        Push a BGR numpy frame to the server for detection.

        Args:
            frame: numpy BGR array (e.g. from depthai or cv2).
            seq: Monotonically increasing sequence number. Used to correlate
                 detections back to the local depth frame.
            jpeg_quality: JPEG encode quality, 0-100.

        Returns:
            True on success, False on error.
        """
        if not self._connected or self._proxy is None:
            return False
        try:
            ret, buf = cv2.imencode('.jpg', frame,
                                    [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
            if not ret:
                print("Error: JPEG encoding failed")
                return False
            binary_data = xmlrpc.client.Binary(buf.tobytes())
            return bool(self._proxy.push_frame(binary_data, seq))
        except Exception as e:
            print(f"Push frame error: {e}")
            return False

    def clear_prompt(self) -> bool:
        """Clear the prompt to avoid stale detections between searches."""
        return self.set_prompt("[]")

    def get_detections(self) -> Optional[Dict]:
        """
        Get the latest detections from the server.

        Returns:
            Dictionary with keys 'prompt', 'inference_time', 'detections'
            where detections is a list of dicts with 'label', 'parent_label',
            'box', 'scores'. Returns None if failed.
        """
        if not self._connected or self._proxy is None:
            return None
        try:
            result = self._proxy.get_detections()
            return dict(result) if result else None
        except Exception as e:
            print(f"Get detections error: {e}")
            return None

    def reboot(self) -> bool:
        if not self._connected or self._proxy is None:
            return False
        try:
            self._proxy.reboot()
            return True
        except Exception as e:
            print(f"Reboot error: {e}")
            return False

    @staticmethod
    def _iou(box_a: List[float], box_b: List[float]) -> float:
        """Compute Intersection over Union for two [x1,y1,x2,y2] boxes."""
        x1 = max(box_a[0], box_b[0])
        y1 = max(box_a[1], box_b[1])
        x2 = min(box_a[2], box_b[2])
        y2 = min(box_a[3], box_b[3])
        inter = max(0, x2 - x1) * max(0, y2 - y1)
        area_a = (box_a[2] - box_a[0]) * (box_a[3] - box_a[1])
        area_b = (box_b[2] - box_b[0]) * (box_b[3] - box_b[1])
        union = area_a + area_b - inter
        return inter / union if union > 0 else 0.0

    @staticmethod
    def nms(detections: List[Dict], iou_threshold: float = 0.5,
            score_threshold: float = 0.1) -> List[Dict]:
        """Apply Non-Maximum Suppression to a list of detections.

        Filters out the root 'image' detection, detections below score_threshold,
        then greedily keeps highest-scoring boxes and suppresses overlapping ones.
        """
        # Filter out root "image" detection, empty labels, and low-score detections
        filtered = []
        for d in detections:
            label = d.get('label', '')
            if not label or label == 'image':
                continue
            scores = d.get('scores', [])
            max_score = max(scores) if scores else 0.0
            if max_score < score_threshold:
                continue
            filtered.append((max_score, d))

        # Sort by score descending
        filtered.sort(key=lambda x: x[0], reverse=True)

        keep = []
        suppressed = set()
        for i, (score_i, det_i) in enumerate(filtered):
            if i in suppressed:
                continue
            keep.append(det_i)
            box_i = det_i.get('box', [])
            if len(box_i) != 4:
                continue
            for j in range(i + 1, len(filtered)):
                if j in suppressed:
                    continue
                box_j = filtered[j][1].get('box', [])
                if len(box_j) == 4 and NanoOwlClient._iou(box_i, box_j) > iou_threshold:
                    suppressed.add(j)

        return keep

    def get_detections_nms(self, iou_threshold: float = 0.5,
                           score_threshold: float = 0.1) -> Optional[Dict]:
        """Get detections with NMS post-processing applied.

        Returns same format as get_detections() but with filtered detections list.
        """
        result = self.get_detections()
        if result is None:
            return None
        det_list = result.get('detections', [])
        result['detections'] = self.nms(det_list, iou_threshold, score_threshold)
        return result

    def print_detections(self, detections):
        """Pretty-print a detections result dict."""
        if detections is None:
            print("  No detections available yet.")
            return
        print(f"  Prompt: {detections.get('prompt')}")
        print(f"  Inference time: {detections.get('inference_time', 0):.3f}s")
        det_list = detections.get('detections', [])
        print(f"  {len(det_list)} detection(s)")
        for d in det_list:
            label = d.get('label', '?')
            parent = d.get('parent_label', '')
            box = d.get('box', [])
            scores = d.get('scores', [])
            box_str = ', '.join(f'{v:.1f}' for v in box) if box else 'N/A'
            score_str = ', '.join(f'{s:.3f}' for s in scores) if scores else 'N/A'
            parent_str = f" (parent: {parent})" if parent else ""
            print(f"    {label}{parent_str}: box=[{box_str}] scores=[{score_str}]")

    def disconnect(self):
        self._proxy = None
        self._connected = False

SERVER_URL = DEFAULT_SERVER_URL

def main():
    """Main function demonstrating XML-RPC client usage."""

    print(f"Connecting to NanoOWL server at {SERVER_URL}")

    client = NanoOwlClient()
    if not client.connect():
        print("Make sure nano_owl_server.py is running on the target machine.")
        sys.exit(1)

    # Get status
    print("\n--- Server Status ---")
    status = client.get_status()
    print(f"Status: {status}")

    # Enable processing
    print("\n--- Enabling Detection ---")
    success = client.enable()
    print(f"Enable result: {success}")
    print(f"Is enabled: {client.is_enabled()}")

    # Set a detection prompt
    print("\n--- Setting Prompt ---")
    prompt = "[a sofa]"
    success = client.set_prompt(prompt)
    print(f"Set prompt result: {success}")
    print(f"Current prompt: {client.get_prompt()}")

    # Poll for detections
    print("\n--- Polling detections for 10 seconds ---")
    for i in range(10):
        time.sleep(1)

        is_running = client.is_running()
        is_enabled = client.is_enabled()
        print(f"  Second {i+1}: running={is_running}, enabled={is_enabled}")

        detections = client.get_detections_nms()
        client.print_detections(detections)

    # Try a different prompt
    print("\n--- Changing Prompt ---")
    new_prompt = "[framed artwork]"
    client.set_prompt(new_prompt)
    print(f"Prompt set to: {client.get_prompt()}")

    # Wait a moment then check
    time.sleep(2)
    print("\n--- Detections with new prompt ---")
    detections = client.get_detections_nms()
    client.print_detections(detections)

    # Disable
    print("\n--- Disabling Detection (Low Power Mode) ---")
    success = client.disable()
    print(f"Disable result: {success}")
    print(f"Is enabled: {client.is_enabled()}")

    # Final status
    status = client.get_status()
    print(f"\nFinal status: {status}")

    print("\nDone!")

def interactive_mode():
    """Interactive mode for manual testing."""

    print(f"NanoOWL Interactive Client")
    print(f"Server: {SERVER_URL}")
    print("Commands: ping, status, enable, disable, enabled, running, prompt, detect, detect-raw, reboot, quit")
    print()

    client = NanoOwlClient()
    if not client.connect():
        print("Could not connect to server.")
        return

    while True:
        try:
            cmd = input(">>> ").strip().lower()

            if cmd == "quit" or cmd == "exit":
                break
            elif cmd == "ping":
                print(client.ping())
            elif cmd == "status":
                print(client.get_status())
            elif cmd == "enable":
                result = client.enable()
                print(f"Enable result: {result}")
                print(f"Is enabled: {client.is_enabled()}")
            elif cmd == "disable":
                result = client.disable()
                print(f"Disable result: {result}")
                print(f"Is enabled: {client.is_enabled()}")
            elif cmd == "enabled":
                print(f"Is enabled: {client.is_enabled()}")
            elif cmd == "running":
                print(f"Is running: {client.is_running()}")
            elif cmd == "prompt":
                action = input("get/set? ").strip().lower()
                if action == "get":
                    print(client.get_prompt())
                elif action == "set":
                    prompt_str = input("Enter tree prompt (e.g. [a person, a dog]): ").strip()
                    result = client.set_prompt(prompt_str)
                    print(f"Set prompt result: {result}")
            elif cmd == "detect":
                detections = client.get_detections_nms()
                client.print_detections(detections)
            elif cmd == "detect-raw":
                detections = client.get_detections()
                client.print_detections(detections)
            elif cmd == "reboot":
                confirm = input("Are you sure you want to reboot the Jetson? (yes/no): ").strip().lower()
                if confirm == "yes":
                    client.reboot()
                    print("Reboot command sent. Connection will be lost as system reboots...")
                    break
                else:
                    print("Reboot cancelled")
            elif cmd == "help":
                print("Commands: ping, status, enable, disable, enabled, running, prompt, detect, detect-raw, reboot, quit")
            else:
                print(f"Unknown command: {cmd}")

        except KeyboardInterrupt:
            print("\nExiting...")
            break
        except Exception as e:
            print(f"Error: {e}")

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "-i":
        interactive_mode()
    else:
        main()

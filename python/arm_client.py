#!/usr/bin/env python3
"""
Arm XML-RPC Client

Connects to the Arm XML-RPC server to control the serial bus-servo arm.

Usage:
    python3 arm_client.py         # Run demo (wave + read angles)
    python3 arm_client.py -i      # Interactive mode
"""

import sys
import xmlrpc.client
from typing import List, Optional

# Default server address (Jetson over the USB-device network link).
DEFAULT_SERVER_URL = "http://192.168.55.1:8001/"


class ArmClient:
    """
    Client class for interacting with the Arm XML-RPC server.

    Provides:
    - Connection management (connect, ping, is_connected, disconnect)
    - Motion (wave)
    - State queries (read_angles, get_status)
    - System control (reboot)
    """

    def __init__(self, server_url: str = DEFAULT_SERVER_URL):
        self.server_url = server_url
        self._proxy: Optional[xmlrpc.client.ServerProxy] = None
        self._connected = False

    def connect(self) -> bool:
        try:
            self._proxy = xmlrpc.client.ServerProxy(self.server_url, allow_none=True)
            self._proxy.ping()
            self._connected = True
            print(f"Connected to Arm server at {self.server_url}")
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

    def wave(self) -> bool:
        if not self._connected or self._proxy is None:
            return False
        try:
            return bool(self._proxy.wave())
        except Exception as e:
            print(f"Wave error: {e}")
            return False

    def read_angles(self) -> Optional[List[Optional[int]]]:
        if not self._connected or self._proxy is None:
            return None
        try:
            result = self._proxy.read_angles()
            return list(result) if result is not None else None
        except Exception as e:
            print(f"Read angles error: {e}")
            return None

    def move_to(self, name: str, time_ms: int = 1500) -> bool:
        """Move to a named full-arm pose (e.g. 'rest', 'raised', 'stowed')."""
        if not self._connected or self._proxy is None:
            return False
        try:
            return bool(self._proxy.move_to(name, time_ms))
        except Exception as e:
            print(f"Move to pose error: {e}")
            return False

    def list_poses(self) -> Optional[dict]:
        """Return the server's named poses as {name: [s1..s6]}."""
        if not self._connected or self._proxy is None:
            return None
        try:
            result = self._proxy.list_poses()
            return dict(result) if result is not None else None
        except Exception as e:
            print(f"List poses error: {e}")
            return None

    def move_servo(self, id: int, angle: float, time_ms: int = 1000) -> bool:
        if not self._connected or self._proxy is None:
            return False
        try:
            return bool(self._proxy.move_servo(id, angle, time_ms))
        except Exception as e:
            print(f"Move servo error: {e}")
            return False

    def move_servo_any(self, id: int, angle: float, time_ms: int = 1000) -> bool:
        if not self._connected or self._proxy is None:
            return False
        try:
            return bool(self._proxy.move_servo_any(id, angle, time_ms))
        except Exception as e:
            print(f"Move servo (any) error: {e}")
            return False

    def move_all(self, s1: float, s2: float, s3: float, s4: float, s5: float, s6: float,
                 time_ms: int = 1000) -> bool:
        if not self._connected or self._proxy is None:
            return False
        try:
            return bool(self._proxy.move_all(s1, s2, s3, s4, s5, s6, time_ms))
        except Exception as e:
            print(f"Move all error: {e}")
            return False

    def move_joints(self, joints: List[float], time_ms: int = 1000) -> bool:
        if not self._connected or self._proxy is None:
            return False
        try:
            return bool(self._proxy.move_joints(list(joints), time_ms))
        except Exception as e:
            print(f"Move joints error: {e}")
            return False

    def read_servo(self, id: int) -> Optional[int]:
        if not self._connected or self._proxy is None:
            return None
        try:
            return self._proxy.read_servo(id)
        except Exception as e:
            print(f"Read servo error: {e}")
            return None

    def read_servo_any(self, id: int) -> Optional[int]:
        if not self._connected or self._proxy is None:
            return None
        try:
            return self._proxy.read_servo_any(id)
        except Exception as e:
            print(f"Read servo (any) error: {e}")
            return None

    def ping_servo(self, id: int) -> Optional[int]:
        if not self._connected or self._proxy is None:
            return None
        try:
            return self._proxy.ping_servo(id)
        except Exception as e:
            print(f"Ping servo error: {e}")
            return None

    def set_torque(self, onoff: int) -> bool:
        if not self._connected or self._proxy is None:
            return False
        try:
            return bool(self._proxy.set_torque(onoff))
        except Exception as e:
            print(f"Set torque error: {e}")
            return False

    def servo_control(self, id: int, num: int, time_ms: int = 1000) -> bool:
        if not self._connected or self._proxy is None:
            return False
        try:
            return bool(self._proxy.servo_control(id, num, time_ms))
        except Exception as e:
            print(f"Servo control error: {e}")
            return False

    def servo_control_array(self, array: List[int], time_ms: int = 1000) -> bool:
        if not self._connected or self._proxy is None:
            return False
        try:
            return bool(self._proxy.servo_control_array(list(array), time_ms))
        except Exception as e:
            print(f"Servo control array error: {e}")
            return False

    def get_serial_port(self) -> Optional[str]:
        if not self._connected or self._proxy is None:
            return None
        try:
            return str(self._proxy.get_serial_port())
        except Exception as e:
            print(f"Get serial port error: {e}")
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

    def disconnect(self):
        self._proxy = None
        self._connected = False


SERVER_URL = DEFAULT_SERVER_URL


def main():
    """Demonstrate basic client usage."""
    print(f"Connecting to Arm server at {SERVER_URL}")

    client = ArmClient()
    if not client.connect():
        print("Make sure arm_server.py is running on the target machine.")
        sys.exit(1)

    print("\n--- Server Status ---")
    print(f"Status: {client.get_status()}")

    print("\n--- Current Angles ---")
    print(f"Angles: {client.read_angles()}")

    print("\n--- Waving ---")
    print(f"Wave result: {client.wave()}")

    print("\n--- Angles After Wave ---")
    print(f"Angles: {client.read_angles()}")

    print("\nDone!")


def interactive_mode():
    """Interactive mode for manual testing."""
    print(f"Arm Interactive Client")
    print(f"Server: {SERVER_URL}")
    print("Commands: ping, status, wave, angles, pose <name>, poses, torque <0|1>,")
    print("          move <id> <angle> [time_ms], moveall <s1..s6> [time_ms],")
    print("          readservo <id>, pingservo <id>, reboot, quit")
    print()

    client = ArmClient()
    if not client.connect():
        print("Could not connect to server.")
        return

    while True:
        try:
            cmd = input(">>> ").strip().lower()

            if cmd in ("quit", "exit"):
                break
            elif cmd == "ping":
                print(client.ping())
            elif cmd == "status":
                print(client.get_status())
            elif cmd == "wave":
                print(f"Wave result: {client.wave()}")
            elif cmd == "angles":
                print(f"Angles: {client.read_angles()}")
            elif cmd.startswith("torque"):
                parts = cmd.split()
                if len(parts) != 2 or parts[1] not in ("0", "1"):
                    print("Usage: torque <0|1>")
                else:
                    print(f"Set torque result: {client.set_torque(int(parts[1]))}")
            elif cmd.startswith("moveall"):
                parts = cmd.split()
                if len(parts) not in (7, 8):
                    print("Usage: moveall <s1> <s2> <s3> <s4> <s5> <s6> [time_ms]")
                else:
                    angles = [float(p) for p in parts[1:7]]
                    time_ms = int(parts[7]) if len(parts) == 8 else 1000
                    print(f"Move all result: {client.move_all(*angles, time_ms=time_ms)}")
            elif cmd.startswith("move"):
                parts = cmd.split()
                if len(parts) not in (3, 4):
                    print("Usage: move <id> <angle> [time_ms]")
                else:
                    id, angle = int(parts[1]), float(parts[2])
                    time_ms = int(parts[3]) if len(parts) == 4 else 1000
                    print(f"Move result: {client.move_servo(id, angle, time_ms=time_ms)}")
            elif cmd.startswith("readservo"):
                parts = cmd.split()
                if len(parts) != 2:
                    print("Usage: readservo <id>")
                else:
                    print(f"Servo {parts[1]} angle: {client.read_servo(int(parts[1]))}")
            elif cmd.startswith("pingservo"):
                parts = cmd.split()
                if len(parts) != 2:
                    print("Usage: pingservo <id>")
                else:
                    print(f"Ping servo {parts[1]}: {client.ping_servo(int(parts[1]))}")
            elif cmd == "poses":
                poses = client.list_poses()
                if poses:
                    for name, joints in sorted(poses.items()):
                        print(f"  {name}: {joints}")
                else:
                    print("No poses available")
            elif cmd.startswith("pose"):
                parts = cmd.split()
                if len(parts) not in (2, 3):
                    print("Usage: pose <name> [time_ms]   (names: rest, raised, stowed)")
                else:
                    name = parts[1]
                    time_ms = int(parts[2]) if len(parts) == 3 else 1500
                    print(f"Move to '{name}': {client.move_to(name, time_ms=time_ms)}")
            elif cmd == "reboot":
                confirm = input("Are you sure you want to reboot the Jetson? (yes/no): ").strip().lower()
                if confirm == "yes":
                    client.reboot()
                    print("Reboot command sent. Connection will be lost as system reboots...")
                    break
                else:
                    print("Reboot cancelled")
            elif cmd == "help":
                print("Commands: ping, status, wave, angles, pose <name>, poses, torque <0|1>,")
                print("          move <id> <angle> [time_ms], moveall <s1..s6> [time_ms],")
                print("          readservo <id>, pingservo <id>, reboot, quit")
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

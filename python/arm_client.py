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
    print("Commands: ping, status, wave, angles, reboot, quit")
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
            elif cmd == "reboot":
                confirm = input("Are you sure you want to reboot the Jetson? (yes/no): ").strip().lower()
                if confirm == "yes":
                    client.reboot()
                    print("Reboot command sent. Connection will be lost as system reboots...")
                    break
                else:
                    print("Reboot cancelled")
            elif cmd == "help":
                print("Commands: ping, status, wave, angles, reboot, quit")
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

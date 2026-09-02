#!/usr/bin/env python3
"""
Service Supervisor XML-RPC Client

Talks to service_supervisor.py to switch the Jetson between its two GPU services,
NanoOWL and Live-VLM (only one can run at a time; both serve on port 8000).

Switching is ASYNCHRONOUS. switch_to() returns immediately ({accepted: True});
the supervisor then stops the other service, starts the requested one, and waits
for its READY MARKER in the log before declaring it "up". The model is loaded
ONCE -- if it fails to load, the supervisor REBOOTS the Jetson automatically
(it does not retry, because repeated back-to-back loads brown out the board).
After the reboot, boot-restore brings the last-requested service back up. Learn
the outcome one of two ways:

  1. Poll get_status() until phase is "up" (or the box reboots on failure; the
     connection drops and comes back with the service loading again).
  2. Register a callback: run a CallbackServer on this host and pass its URL to
     switch_to(); the supervisor pushes attempt/up/rebooting events to it.

Because model load is memory-heavy on the Jetson, you can fire switch_to() and
then DISCONNECT/quit this client to free memory, reconnecting later to poll
get_status().

Usage:
    python3 jetson_supervisor_client.py            # Run demo
    python3 jetson_supervisor_client.py -i         # Interactive mode
    python3 jetson_supervisor_client.py -i --watch # Interactive; block+print progress
"""

import socket
import sys
import threading
import time
import xmlrpc.client
from typing import Callable, Optional
from xmlrpc.server import SimpleXMLRPCServer, SimpleXMLRPCRequestHandler

# Jetson over the USB-device network link. Supervisor control port is 8002.
DEFAULT_SERVER_URL = "http://192.168.55.1:8002/"

# Terminal phases: the switch has settled and no more progress will happen.
# "rebooting" is terminal too -- the box is going down and will come back up
# loading the last-requested service.
SETTLED_PHASES = ("up", "failed", "idle", "cancelled", "rebooting")


class SupervisorClient:
    """Client for the service supervisor."""

    def __init__(self, server_url: str = DEFAULT_SERVER_URL):
        self.server_url = server_url
        self._proxy: Optional[xmlrpc.client.ServerProxy] = None
        self._connected = False

    def connect(self) -> bool:
        try:
            self._proxy = xmlrpc.client.ServerProxy(self.server_url, allow_none=True)
            self._proxy.ping()
            self._connected = True
            print(f"Connected to supervisor at {self.server_url}")
            return True
        except ConnectionRefusedError:
            print(f"Error: Could not connect to supervisor at {self.server_url}")
            self._connected = False
            return False
        except Exception as e:
            print(f"Error connecting to supervisor: {e}")
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

    def current(self) -> Optional[str]:
        if not self._connected or self._proxy is None:
            return None
        try:
            return str(self._proxy.current())
        except Exception as e:
            print(f"Current error: {e}")
            return None

    def switch_to(self, name: str, callback_url: Optional[str] = None) -> Optional[dict]:
        """
        Request a switch to 'owl', 'vlm', or 'none'. Returns immediately with
        {accepted: True, desired}. Use wait_until_settled() or a callback to
        learn the outcome. On a failed load the Jetson reboots automatically.
        """
        if not self._connected or self._proxy is None:
            return None
        try:
            return dict(self._proxy.switch_to(name, callback_url))
        except Exception as e:
            print(f"Switch error: {e}")
            return None

    def stop_all(self) -> Optional[dict]:
        return self.switch_to("none")

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

    def wait_until_settled(self, timeout: float = 900.0,
                           poll: float = 3.0,
                           on_update: Optional[Callable[[dict], None]] = None
                           ) -> Optional[dict]:
        """
        Poll get_status() until the switch settles (phase up/failed/idle/
        cancelled) or `timeout` elapses. Returns the final status dict.

        NOTE: this keeps the client running (and connected) for the duration.
        On the memory-constrained Jetson prefer firing switch_to() and quitting,
        then reconnecting to check get_status() once.
        """
        deadline = time.time() + timeout
        last_key = None
        while time.time() < deadline:
            status = self.get_status()
            if status is None:
                return None
            key = (status.get("phase"), status.get("attempt"),
                   status.get("error"))
            if key != last_key:
                last_key = key
                if on_update:
                    on_update(status)
            if status.get("phase") in SETTLED_PHASES:
                return status
            time.sleep(poll)
        return self.get_status()


# --- Optional push-callback listener ----------------------------------------

class _CallbackHandler(SimpleXMLRPCRequestHandler):
    rpc_paths = ("/RPC2", "/")


class CallbackServer:
    """
    Tiny XML-RPC server the supervisor can push switch progress to.

    Run this on the REMOTE HOST (which has memory to spare), pass its URL to
    switch_to(), and your handler receives attempt/up/retrying/failed events.

    Example:
        cb = CallbackServer("192.168.55.100", 8003, handler=print)
        cb.start()
        sup.switch_to("vlm", callback_url=cb.url_for("192.168.55.1"))
        ...
        cb.stop()
    """

    def __init__(self, bind_host: str = "0.0.0.0", port: int = 8003,
                 handler: Optional[Callable[[dict], None]] = None):
        self.bind_host = bind_host
        self.port = port
        self.handler = handler or (lambda ev: print(f"[switch] {ev}"))
        self._server: Optional[SimpleXMLRPCServer] = None
        self._thread: Optional[threading.Thread] = None

    def switch_progress(self, event):
        try:
            self.handler(dict(event))
        except Exception as e:
            print(f"Callback handler error: {e}")
        return True

    def url_for(self, my_ip: Optional[str] = None) -> str:
        """The callback_url to hand to switch_to(). Pass the IP the Jetson uses
        to reach this host (e.g. 192.168.55.100 on the USB-device link)."""
        host = my_ip or socket.gethostbyname(socket.gethostname())
        return f"http://{host}:{self.port}/"

    def start(self):
        self._server = SimpleXMLRPCServer(
            (self.bind_host, self.port), requestHandler=_CallbackHandler,
            allow_none=True, logRequests=False,
        )
        self._server.register_function(self.switch_progress, "switch_progress")
        self._thread = threading.Thread(target=self._server.serve_forever,
                                        daemon=True)
        self._thread.start()
        print(f"Callback server listening on {self.bind_host}:{self.port}")

    def stop(self):
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()


SERVER_URL = DEFAULT_SERVER_URL


def _print_update(status: dict):
    print(f"  phase={status.get('phase')} active={status.get('active')}"
          f" error={status.get('error')}")


def main():
    """Demonstrate basic client usage."""
    print(f"Connecting to supervisor at {SERVER_URL}")
    client = SupervisorClient()
    if not client.connect():
        print("Make sure service_supervisor.py is running on the Jetson.")
        sys.exit(1)

    print("\n--- Status ---")
    print(f"Status: {client.get_status()}")
    print(f"\nCurrent service: {client.current()}")
    print("\nDone!")


def interactive_mode(watch: bool = False):
    """Interactive mode for manual testing.

    watch=True blocks after a switch and prints progress until it settles
    (keeps the client running). Default: fire-and-return, poll with 'status'.
    """
    print("Supervisor Interactive Client")
    print(f"Server: {SERVER_URL}   watch={watch}")
    print("Commands: owl, vlm, none, current, status, watch, ping, reboot, quit")
    print()

    client = SupervisorClient()
    if not client.connect():
        print("Could not connect to supervisor.")
        return

    def do_switch(name):
        print(f"Requesting switch to {name}...")
        print(client.switch_to(name))
        if watch:
            print("Watching progress (Ctrl-C to stop watching)...")
            start = time.time()
            final = client.wait_until_settled(on_update=_print_update)
            print(f"Settled: {final}")
            print(f"Elapsed: {time.time() - start:.1f}s")
        else:
            print("Switch running in background. Use 'status' to check, or "
                  "quit to free memory and reconnect later.")

    while True:
        try:
            cmd = input(">>> ").strip().lower()
            if cmd in ("quit", "exit"):
                break
            elif cmd == "ping":
                print(client.ping())
            elif cmd == "status":
                print(client.get_status())
            elif cmd == "current":
                print(client.current())
            elif cmd == "watch":
                final = client.wait_until_settled(on_update=_print_update)
                print(f"Settled: {final}")
            elif cmd in ("owl", "vlm", "none"):
                do_switch(cmd)
            elif cmd == "reboot":
                confirm = input("Reboot the Jetson? (yes/no): ").strip().lower()
                if confirm == "yes":
                    client.reboot()
                    print("Reboot sent. Connection will drop...")
                    break
                else:
                    print("Reboot cancelled")
            elif cmd == "help":
                print("Commands: owl, vlm, none, current, status, watch, "
                      "ping, reboot, quit")
            else:
                print(f"Unknown command: {cmd}")
        except KeyboardInterrupt:
            print("\n(stopped watching)")
        except Exception as e:
            print(f"Error: {e}")


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "-i":
        interactive_mode(watch="--watch" in sys.argv)
    else:
        main()

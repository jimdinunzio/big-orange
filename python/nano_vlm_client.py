#!/usr/bin/env python3
"""
NanoVlm XML-RPC Client

A COPY. This file belongs to the jetson-nano-services repo, at live-vlm-service/,
beside the server it talks to. Edit it there, not here -- but diff before
copying a new version down, since this copy can carry local fixes.

This module provides a client class for connecting to and interacting with
the NanoVlm XML-RPC server running on 192.168.55.1:8000.

The server automatically creates and starts the VLM instance on startup,
but starts in disabled (low power) mode. Call enable() to start processing.
Auto-disables after 20s of no get_output() calls.

Usage:
    from nano_vlm_client import NanoVlmClient
    
    client = NanoVlmClient()
    if client.connect():
        client.enable()  # Enable frame processing
        status = client.get_status()
        print(status)
"""

import xmlrpc.client
import time
import sys
import os
from typing import List, Tuple, Optional

# Platform-specific imports for non-blocking key detection
if sys.platform == 'win32':
    import msvcrt
else:
    import select
    import tty
    import termios

# Default server address
DEFAULT_SERVER_URL = "http://192.168.55.1:8000/"
#DEFAULT_SERVER_URL = 'http://localhost:8000/'

class NanoVlmClient:
    """
    Client class for interacting with the NanoVlm XML-RPC server.
    
    Provides methods for all proxy commands including:
    - Connection management (ping, connect, disconnect)
    - Enable/disable control (enable, disable, is_enabled)
    - Status queries (get_status, is_running)
    - Output management (get_output, get_output_stack_size, clear_output_stack)
    - Prompt management (get_prompts, set_prompts)
    - System control (reboot)
    """
    
    def __init__(self, server_url: str = DEFAULT_SERVER_URL):
        """
        Initialize the NanoVlmClient.
        
        Args:
            server_url: URL of the NanoVlm XML-RPC server
        """
        self.server_url = server_url
        self._proxy: Optional[xmlrpc.client.ServerProxy] = None
        self._connected = False
    
    def connect(self, timeout: float = 5.0) -> bool:
        """
        Connect to the NanoVlm server.
        
        Args:
            timeout: Connection timeout in seconds
            
        Returns:
            True if connection successful, False otherwise
        """
        try:
            self._proxy = xmlrpc.client.ServerProxy(self.server_url, allow_none=True)
            # Test connection with ping
            result = self._proxy.ping()
            self._connected = True
            print(f"Connected to NanoVLM server at {self.server_url}")
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
        """Check if client is connected to server."""
        return self._connected
    
    def ping(self) -> Optional[str]:
        """
        Ping the server to check connectivity.
        
        Returns:
            Server response string or None if failed
        """
        if not self._connected or self._proxy is None:
            return None
        try:
            return str(self._proxy.ping())
        except Exception as e:
            print(f"Ping error: {e}")
            self._connected = False
            return None
    
    def get_status(self) -> Optional[dict]:
        """
        Get the current server status.
        
        Returns:
            Status dictionary or None if failed
        """
        if not self._connected or self._proxy is None:
            return None
        try:
            result = self._proxy.get_status()
            return dict(result) if result else None
        except Exception as e:
            print(f"Get status error: {e}")
            return None
    
    def is_running(self) -> Optional[bool]:
        """
        Check if the VLM is currently running.
        
        Returns:
            True if running, False if not, None if failed
        """
        if not self._connected or self._proxy is None:
            return None
        try:
            return bool(self._proxy.is_running())
        except Exception as e:
            print(f"Is running error: {e}")
            return None
    
    def enable(self) -> bool:
        """
        Enable frame processing.
        
        The server starts in disabled (low power) mode. Call this to start
        processing frames. Auto-disables after 20s of no get_output() calls.
        
        Returns:
            True if successful, False otherwise
        """
        if not self._connected or self._proxy is None:
            return False
        try:
            return bool(self._proxy.enable())
        except Exception as e:
            print(f"Enable error: {e}")
            return False
    
    def disable(self) -> bool:
        """
        Disable frame processing (enter low power mode).
        
        Returns:
            True if successful, False otherwise
        """
        if not self._connected or self._proxy is None:
            return False
        try:
            return bool(self._proxy.disable())
        except Exception as e:
            print(f"Disable error: {e}")
            return False
    
    def is_enabled(self) -> Optional[bool]:
        """
        Check if frame processing is enabled.
        
        Returns:
            True if enabled, False if disabled, None if failed
        """
        if not self._connected or self._proxy is None:
            return None
        try:
            return bool(self._proxy.is_enabled())
        except Exception as e:
            print(f"Is enabled error: {e}")
            return None
    
    def get_output(self, index: int = 0, timeout: float = 5, prompt_filter: Optional[str] = None) -> Tuple[Optional[str], Optional[str], Optional[bool]]:
        """
        Get output from the output stack at specified index.
        
        Args:
            index: Index in the output stack (0 = most recent)
            timeout: Timeout in seconds to wait for matching output (0 = no wait)
            prompt_filter: Optional prompt string to wait for. If specified with timeout > 0,
                          will wait until an output with this prompt is available.
            
        Returns:
            Tuple of (output_text, prompt, same_as_next) or (None, None, None) if failed
        """
        if not self._connected or self._proxy is None:
            return None, None, None
        try:
            if prompt_filter is not None:
                result = self._proxy.get_output(index, timeout, prompt_filter)
            else:
                result = self._proxy.get_output(index)
            if result:
                output = result.get('output')
                prompt = result.get('prompt')
                same_as_next = result.get('sameAsNext', False)
                return output, prompt, same_as_next
            return None, None, None
        except Exception as e:
            print(f"Get output error: {e}")
            return None, None, None
    
    def get_output_stack_size(self) -> Optional[int]:
        """
        Get the current size of the output stack.
        
        Returns:
            Stack size or None if failed
        """
        if not self._connected or self._proxy is None:
            return None
        try:
            return int(self._proxy.get_output_stack_size())
        except Exception as e:
            print(f"Get output stack size error: {e}")
            return None
    
    def clear_output_stack(self) -> bool:
        """
        Clear the output stack.
        
        Returns:
            True if successful, False otherwise
        """
        if not self._connected or self._proxy is None:
            return False
        try:
            self._proxy.clear_output_stack()
            return True
        except Exception as e:
            print(f"Clear output stack error: {e}")
            return False
    
    def get_prompts(self) -> Optional[List[str]]:
        """
        Get the current prompts.
        
        Returns:
            List of prompt strings or None if failed
        """
        if not self._connected or self._proxy is None:
            return None
        try:
            result = self._proxy.get_prompts()
            return list(result) if result else None
        except Exception as e:
            print(f"Get prompts error: {e}")
            return None
    
    def set_prompts(self, prompts: List[str]) -> bool:
        """
        Set new prompts for the VLM.
        
        Args:
            prompts: List of prompt strings
            
        Returns:
            True if successful, False otherwise
        """
        if not self._connected or self._proxy is None:
            return False
        try:
            return bool(self._proxy.set_prompts(prompts))
        except Exception as e:
            print(f"Set prompts error: {e}")
            return False
    
    def reboot(self) -> bool:
        """
        Send reboot command to the server.
        
        Returns:
            True if command sent, False otherwise
        """
        if not self._connected or self._proxy is None:
            return False
        try:
            self._proxy.reboot()
            return True
        except Exception as e:
            print(f"Reboot error: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from the server."""
        self._proxy = None
        self._connected = False


def main():
    """Main function demonstrating XML-RPC client usage."""
    
    print(f"Connecting to NanoVLM server at {DEFAULT_SERVER_URL}")
    
    # Create client and connect
    client = NanoVlmClient()
    
    if not client.connect():
        print("Make sure the nano_vlm_server.py is running on the target machine.")
        sys.exit(1)
    
    # Test ping
    result = client.ping()
    print(f"Server ping: {result}")
    
    # Get status (instance is auto-created but starts disabled)
    print("\n--- Server Status ---")
    status = client.get_status()
    print(f"Status: {status}")
    
    # Check if enabled (should be False initially)
    is_enabled = client.is_enabled()
    print(f"Is enabled: {is_enabled}")
    
    # Enable frame processing
    print("\n--- Enabling Frame Processing ---")
    success = client.enable()
    print(f"Enable result: {success}")
    
    # Verify enabled
    is_enabled = client.is_enabled()
    print(f"Is enabled now: {is_enabled}")

    # Set initial prompts
    print("\n--- Setting Initial Prompts ---")
    success = client.set_prompts([
        "Describe the scene in detail."])
    print(f"Set prompts result: {success}")

    # Get current prompts
    print(f"\n--- Current Prompts ---")
    prompts = client.get_prompts()
    print(f"Prompts: {prompts}")
    
    # Monitor outputs for a bit
    print("\n--- Monitoring outputs for 10 seconds ---")
    for i in range(10):
        time.sleep(1)
        
        # Check status
        is_running = client.is_running()
        is_enabled = client.is_enabled()
        stack_size = client.get_output_stack_size()
        print(f"  Second {i+1}: running={is_running}, enabled={is_enabled}, outputs={stack_size}")
        
        # Get latest output if available
        if stack_size and stack_size > 0:
            output, prompt, same_as_next = client.get_output(0)
            if output:
                # Truncate long outputs for display
                display_output = output[:100] + "..." if len(output) > 100 else output
                print(f"    Prompt: {prompt}")
                print(f"    Output: {display_output}")
                if same_as_next:
                    print(f"    (sameAsNext)")
    
    # Update prompts
    print("\n--- Updating Prompts ---")
    new_prompts = [
        "What objects do you see in the image?"
    ]
    success = client.set_prompts(new_prompts)
    print(f"Prompts updated: {success}")
    print(f"Current prompts: {client.get_prompts()}")
    
    # Let it process with new prompts
    print("\n--- Processing with new prompts for 5 seconds ---")
    for i in range(5):
        time.sleep(1)
        stack_size = client.get_output_stack_size()
        print(f"  Second {i+1}: outputs={stack_size}")
    
    # Get multiple outputs from the stack
    print("\n--- Output Stack Contents ---")
    stack_size = client.get_output_stack_size()
    print(f"Total outputs in stack: {stack_size}")
    
    # Show last 5 outputs
    if stack_size:
        num_to_show = min(5, stack_size)
        for i in range(num_to_show):
            output, prompt, same_as_next = client.get_output(i)
            if output:
                display_output = output[:80] + "..." if len(output) > 80 else output
                print(f"  [{i}] prompt: {prompt}")
                print(f"       output: {display_output}")
                if same_as_next:
                    print(f"       (sameAsNext)")
    
    # Disable frame processing (enter low power mode)
    print("\n--- Disabling Frame Processing (Low Power Mode) ---")
    success = client.disable()
    print(f"Disable result: {success}")
    print(f"Is enabled: {client.is_enabled()}")
    
    # Final status
    status = client.get_status()
    print(f"\nFinal status: {status}")
    
    print("\nDone!")


def check_for_esc():
    """Check if ESC key was pressed (non-blocking)."""
    if sys.platform == 'win32':
        if msvcrt.kbhit():
            key = msvcrt.getch()
            if key == b'\x1b':  # ESC key
                return True
    else:
        # Unix/Linux/Mac
        if select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            if key == '\x1b':  # ESC key
                return True
    return False


def interactive_mode():
    """Interactive mode for manual testing."""
    
    print(f"NanoVLM Interactive Client")
    print(f"Server: {DEFAULT_SERVER_URL}")
    print("Commands: ping, status, enable, disable, enabled, running, prompts, output, stack, clear, top, reboot, quit")
    print()
    
    client = NanoVlmClient()
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
            elif cmd == "reboot":
                confirm = input("Are you sure you want to reboot the Jetson? (yes/no): ").strip().lower()
                if confirm == "yes":
                    client.reboot()
                    print("Reboot command sent. Connection will be lost as system reboots...")
                    break
                else:
                    print("Reboot cancelled")
            elif cmd == "prompts":
                action = input("get/set? ").strip().lower()
                if action == "get":
                    print(client.get_prompts())
                elif action == "set":
                    prompt_str = input("Enter prompts (comma-separated): ").strip()
                    prompts = [p.strip() for p in prompt_str.split(",")]
                    client.set_prompts(prompts)
                    print("Prompts updated")
            elif cmd == "output":
                index = input("Index [0]: ").strip()
                index = int(index) if index else 0
                prompt_filter = input("Wait for prompt (blank for any): ").strip()
                if prompt_filter:
                    timeout = input("Timeout [5]: ").strip()
                    timeout = float(timeout) if timeout else 5.0
                    output, prompt, same_as_next = client.get_output(index, timeout, prompt_filter)
                else:
                    output, prompt, same_as_next = client.get_output(index)
                print(f"Prompt: {prompt}")
                print(f"Output: {output}")
                if same_as_next:
                    print(f"sameAsNext: True")
            elif cmd == "stack":
                print(f"Stack size: {client.get_output_stack_size()}")
            elif cmd == "clear":
                client.clear_output_stack()
                print("Stack cleared")
            elif cmd == "top":
                print("Polling top output every 1s. Press ESC to stop...")
                # For Unix, set terminal to raw mode
                if sys.platform != 'win32':
                    old_settings = termios.tcgetattr(sys.stdin)
                    tty.setcbreak(sys.stdin.fileno())
                try:
                    while True:
                        output, prompt, same_as_next = client.get_output(0)
                        # Clear line and print output
                        timestamp = time.strftime("%H:%M:%S")
                        if output:
                            display_output = output[:100] + "..." if len(output) > 100 else output
                            print(f"[{timestamp}] {display_output}")
                            if same_as_next:
                                print(f"  (sameAsNext)")
                        else:
                            print(f"[{timestamp}] No output available")
                        
                        # Check for ESC key during the 1 second wait
                        for _ in range(10):  # Check every 0.1s
                            if check_for_esc():
                                print("\nStopped.")
                                raise StopIteration
                            time.sleep(0.1)
                except StopIteration:
                    pass
                finally:
                    # Restore terminal settings on Unix
                    if sys.platform != 'win32':
                        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            elif cmd == "help":
                print("Commands: ping, status, enable, disable, enabled, running, prompts, output, stack, clear, top, reboot, quit")
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

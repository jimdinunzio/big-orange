#!/usr/bin/env python3
"""
NanoVlm XML-RPC Server Simulation

A simulation server for testing the NanoVlmClient without actual VLM hardware.
Generates simulated VLM output strings periodically.

Usage:
    python nano_vlm_server_sim.py
"""

import xmlrpc.server
import threading
import time
import random
from typing import List

# Server configuration
DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 8000
MAX_OUTPUT_STACK_SIZE = 25

# Simulated VLM outputs - sequential journey from office to kitchen and back
# Each tuple is (output_text, repeat_count) where repeat_count = 1-3 based on prominence
# Larger/more prominent objects get higher repeat counts
SIMULATED_OUTPUTS_SEQUENCE = [
    # Office (start) - prominent items
    ("office with desk and computer monitor", 3),
    ("desk with keyboard and mouse on mousepad", 2),
    ("office chair in front of computer desk", 2),
    ("doorway leading out of office", 1),
    # Hallway from office - transitional, less prominent
    ("hallway with wooden floor", 1),
    ("hallway with framed pictures on wall", 2),
    ("open doorway ahead leading to larger room", 1),
    # Living room - large prominent items
    ("entering living room with large couch", 2),
    ("living room with couch and coffee table", 3),
    ("television mounted on wall in living room", 3),
    ("potted plant in corner of living room", 1),
    ("carpet rug in center of living room floor", 2),
    ("passing through living room toward dining area", 1),
    # Dining area - prominent furniture
    ("entering dining area with wooden table", 2),
    ("dining table with four chairs around it", 3),
    ("chandelier hanging above dining table", 2),
    ("window with curtains in dining area", 2),
    ("cabinet with dishes visible through glass doors", 2),
    ("archway leading to kitchen ahead", 1),
    # Kitchen - many prominent appliances
    ("entering kitchen with tile floor", 1),
    ("kitchen counter with fruit bowl", 2),
    ("stainless steel refrigerator against wall", 3),
    ("kitchen sink with window above it", 2),
    ("stove and oven next to counter", 3),
    ("microwave on kitchen counter", 1),
    ("kitchen island in center of room", 3),
    ("arrived at kitchen destination", 2),
    # Pause at kitchen
    ("standing in kitchen near counter", 2),
    ("kitchen cabinets and appliances visible", 2),
    # Return journey - Kitchen to Dining
    ("turning around in kitchen", 1),
    ("leaving kitchen through archway", 1),
    # Dining area (return)
    ("entering dining area from kitchen", 1),
    ("passing dining table on right side", 2),
    ("chandelier overhead in dining room", 2),
    ("exiting dining area toward living room", 1),
    # Living room (return) - still prominent
    ("entering living room from dining area", 1),
    ("couch and coffee table on left side", 3),
    ("television on wall straight ahead", 3),
    ("carpet rug underfoot in living room", 2),
    ("heading toward hallway from living room", 1),
    # Hallway (return)
    ("entering hallway from living room", 1),
    ("framed pictures on hallway walls", 2),
    ("wooden floor in hallway", 1),
    ("office doorway visible ahead", 1),
    # Office (end)
    ("entering office through doorway", 1),
    ("desk and computer monitor ahead", 3),
    ("arrived back at office", 2),
    ("office with desk chair and computer", 3),
]


class NanoVlmServerSim:
    """Simulated NanoVLM XML-RPC Server."""
    
    def __init__(self):
        self._running = True
        self._output_stack: List[tuple] = []  # (output_text, same_count)
        self._prompts: List[str] = ["Describe what you see in the image."]
        self._lock = threading.Lock()
        self._output_thread = None
        self._last_output = ""
        self._sequence_index = 0
        self._sequence_direction = 1  # 1 = forward, -1 = reverse (loops)
        self._repeat_remaining = 0  # How many more times to repeat current output
        self._current_output = ""
        
    def start_output_generation(self):
        """Start the background thread that generates outputs."""
        self._output_thread = threading.Thread(target=self._generate_outputs, daemon=True)
        self._output_thread.start()
        
    def _generate_outputs(self):
        """Background thread that pushes simulated outputs sequentially.
        
        Simulates a journey from office to kitchen and back.
        Each output repeats 1-3 times based on object prominence.
        ~1.2s between outputs means total journey is ~2-3 minutes.
        """
        while self._running:
            time.sleep(random.uniform(1.0, 1.4))  # ~1.2s average
            
            # Check if we need to repeat current output
            if self._repeat_remaining > 0:
                output = self._current_output
                self._repeat_remaining -= 1
            else:
                # Get next output in sequence
                output_text, repeat_count = SIMULATED_OUTPUTS_SEQUENCE[self._sequence_index]
                output = output_text
                self._current_output = output
                self._repeat_remaining = repeat_count - 1  # -1 because we're outputting once now
                
                # Advance sequence index
                self._sequence_index += self._sequence_direction
                
                # Handle end of sequence - loop back
                if self._sequence_index >= len(SIMULATED_OUTPUTS_SEQUENCE):
                    self._sequence_index = len(SIMULATED_OUTPUTS_SEQUENCE) - 2
                    self._sequence_direction = -1
                elif self._sequence_index < 0:
                    self._sequence_index = 1
                    self._sequence_direction = 1
            
            with self._lock:
                # Calculate same_count (how many times same output in a row)
                if output == self._last_output:
                    if self._output_stack:
                        same_count = self._output_stack[0][1] + 1
                    else:
                        same_count = 1
                else:
                    same_count = 0
                    
                self._last_output = output
                
                # Push to front of stack
                self._output_stack.insert(0, (output, same_count))
                
                # Limit stack size
                if len(self._output_stack) > MAX_OUTPUT_STACK_SIZE:
                    self._output_stack.pop()
                    
            direction_str = "→" if self._sequence_direction == 1 else "←"
            repeat_str = f"(+{self._repeat_remaining})" if self._repeat_remaining > 0 else ""
            print(f"[SIM {direction_str}] {output} {repeat_str}")
    
    # XML-RPC exposed methods
    
    def ping(self) -> str:
        """Ping the server."""
        return "pong"
    
    def get_status(self) -> dict:
        """Get server status."""
        with self._lock:
            return {
                "running": self._running,
                "output_stack_size": len(self._output_stack),
                "prompts": self._prompts,
                "mode": "simulation"
            }
    
    def is_running(self) -> bool:
        """Check if VLM is running."""
        return self._running
    
    def get_output(self, index: int = 0) -> tuple:
        """Get output from stack at specified index."""
        with self._lock:
            if index < 0 or index >= len(self._output_stack):
                return ("", 0)
            return self._output_stack[index]
    
    def get_output_stack_size(self) -> int:
        """Get the current size of the output stack."""
        with self._lock:
            return len(self._output_stack)
    
    def clear_output_stack(self) -> bool:
        """Clear the output stack."""
        with self._lock:
            self._output_stack.clear()
            self._last_output = ""
        print("[SIM] Output stack cleared")
        return True
    
    def get_prompts(self) -> List[str]:
        """Get current prompts."""
        with self._lock:
            return self._prompts.copy()
    
    def set_prompts(self, prompts: List[str]) -> bool:
        """Set new prompts."""
        with self._lock:
            self._prompts = list(prompts)
        print(f"[SIM] Prompts updated: {prompts}")
        return True
    
    def reboot(self) -> bool:
        """Simulate reboot (clears stack, resets state)."""
        print("[SIM] Reboot requested - clearing state")
        with self._lock:
            self._output_stack.clear()
            self._last_output = ""
        return True
    
    def shutdown(self):
        """Shutdown the server."""
        self._running = False


def main():
    """Main function to run the simulation server."""
    print(f"Starting NanoVLM Simulation Server on {DEFAULT_HOST}:{DEFAULT_PORT}")
    
    # Create server instance
    sim = NanoVlmServerSim()
    
    # Create XML-RPC server
    server = xmlrpc.server.SimpleXMLRPCServer(
        (DEFAULT_HOST, DEFAULT_PORT),
        allow_none=True,
        logRequests=False
    )
    
    # Register functions
    server.register_function(sim.ping, "ping")
    server.register_function(sim.get_status, "get_status")
    server.register_function(sim.is_running, "is_running")
    server.register_function(sim.get_output, "get_output")
    server.register_function(sim.get_output_stack_size, "get_output_stack_size")
    server.register_function(sim.clear_output_stack, "clear_output_stack")
    server.register_function(sim.get_prompts, "get_prompts")
    server.register_function(sim.set_prompts, "set_prompts")
    server.register_function(sim.reboot, "reboot")
    
    # Start output generation thread
    sim.start_output_generation()
    
    print("Server started. Press Ctrl+C to stop.")
    print(f"Connect with: NanoVlmClient('http://localhost:{DEFAULT_PORT}/')")
    
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down server...")
        sim.shutdown()


if __name__ == "__main__":
    main()

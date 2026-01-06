#!/usr/bin/env python3
"""
NanoVlm XML-RPC Client Example

This script demonstrates how to connect to and interact with
the NanoVlm XML-RPC server running on 192.168.55.1:8000.

The server automatically creates and starts the VLM instance on startup,
but starts in disabled (low power) mode. Call enable() to start processing.
Auto-disables after 20s of no get_output() calls.

Usage:
    python3 nano_vlm_client.py         # Run demo
    python3 nano_vlm_client.py -i      # Interactive mode
"""

import xmlrpc.client
import time
import sys

# Server address - adjust as needed
#SERVER_URL = "http://127.0.0.1:8000/"

SERVER_URL = "http://192.168.55.1:8000/"

def main():
    """Main function demonstrating XML-RPC client usage."""
    
    print(f"Connecting to NanoVLM server at {SERVER_URL}")
    
    # Create the XML-RPC proxy
    proxy = xmlrpc.client.ServerProxy(SERVER_URL, allow_none=True)
    
    try:
        # Test connection with ping
        result = proxy.ping()
        print(f"Server ping: {result}")
    except ConnectionRefusedError:
        print(f"Error: Could not connect to server at {SERVER_URL}")
        print("Make sure the nano_vlm_server.py is running on the target machine.")
        sys.exit(1)
    except Exception as e:
        print(f"Error connecting to server: {e}")
        sys.exit(1)
    
    # Get status (instance is auto-created but starts disabled)
    print("\n--- Server Status ---")
    status = proxy.get_status()
    print(f"Status: {status}")
    
    # Check if enabled (should be False initially)
    is_enabled = proxy.is_enabled()
    print(f"Is enabled: {is_enabled}")
    
    # Enable frame processing
    print("\n--- Enabling Frame Processing ---")
    success = proxy.enable()
    print(f"Enable result: {success}")
    
    # Verify enabled
    is_enabled = proxy.is_enabled()
    print(f"Is enabled now: {is_enabled}")
    
    # Get current prompts
    print(f"\n--- Current Prompts ---")
    prompts = proxy.get_prompts()
    print(f"Prompts: {prompts}")
    
    # Monitor outputs for a bit
    print("\n--- Monitoring outputs for 10 seconds ---")
    for i in range(10):
        time.sleep(1)
        
        # Check status
        is_running = proxy.is_running()
        is_enabled = proxy.is_enabled()
        stack_size = proxy.get_output_stack_size()
        print(f"  Second {i+1}: running={is_running}, enabled={is_enabled}, outputs={stack_size}")
        
        # Get latest output if available
        if stack_size > 0:
            result = proxy.get_output(0)
            output = result.get('output')
            prompt = result.get('prompt')
            if output:
                # Truncate long outputs for display
                display_output = output[:100] + "..." if len(output) > 100 else output
                print(f"    Prompt: {prompt}")
                print(f"    Output: {display_output}")
    
    # Update prompts and wait for matching output
    print("\n--- Updating Prompts ---")
    new_prompt = "What objects do you see in the image?"
    new_prompts = [new_prompt]
    success = proxy.set_prompts(new_prompts)
    print(f"Prompts updated: {success}")
    print(f"Current prompts: {proxy.get_prompts()}")
    
    # Wait for output matching the new prompt
    print(f"\n--- Waiting for output with prompt: '{new_prompt}' ---")
    result = proxy.get_output(0, 30.0, new_prompt)
    print(f"Got result:")
    print(f"  Prompt: {result.get('prompt')}")
    print(f"  Output: {result.get('output')}")
    
    # Try another prompt
    print("\n--- Testing another prompt ---")
    test_prompt = "Describe the lighting conditions."
    proxy.set_prompts([test_prompt])
    print(f"Waiting for output with prompt: '{test_prompt}'")
    result = proxy.get_output(0, 30.0, test_prompt)
    print(f"Got result:")
    print(f"  Prompt: {result.get('prompt')}")
    print(f"  Output: {result.get('output')}")
    
    # Get multiple outputs from the stack
    print("\n--- Output Stack Contents ---")
    stack_size = proxy.get_output_stack_size()
    print(f"Total outputs in stack: {stack_size}")
    
    # Show last 5 outputs
    num_to_show = min(5, stack_size)
    for i in range(num_to_show):
        result = proxy.get_output(i)
        output = result.get('output')
        prompt = result.get('prompt')
        if output:
            display_output = output[:80] + "..." if len(output) > 80 else output
            print(f"  [{i}] prompt: {prompt}")
            print(f"       output: {display_output}")
            if result.get('sameAsNext'):
                print(f"       (sameAsNext)")
    
    # Disable frame processing (enter low power mode)
    print("\n--- Disabling Frame Processing (Low Power Mode) ---")
    success = proxy.disable()
    print(f"Disable result: {success}")
    print(f"Is enabled: {proxy.is_enabled()}")
    
    # Final status
    status = proxy.get_status()
    print(f"\nFinal status: {status}")
    
    print("\nDone!")

def interactive_mode():
    """Interactive mode for manual testing."""
    
    print(f"NanoVLM Interactive Client")
    print(f"Server: {SERVER_URL}")
    print("Commands: ping, status, enable, disable, enabled, running, prompts, output, stack, clear, reboot, quit")
    print()
    
    proxy = xmlrpc.client.ServerProxy(SERVER_URL, allow_none=True)
    
    while True:
        try:
            cmd = input(">>> ").strip().lower()
            
            if cmd == "quit" or cmd == "exit":
                break
            elif cmd == "ping":
                print(proxy.ping())
            elif cmd == "status":
                print(proxy.get_status())
            elif cmd == "enable":
                result = proxy.enable()
                print(f"Enable result: {result}")
                print(f"Is enabled: {proxy.is_enabled()}")
            elif cmd == "disable":
                result = proxy.disable()
                print(f"Disable result: {result}")
                print(f"Is enabled: {proxy.is_enabled()}")
            elif cmd == "enabled":
                print(f"Is enabled: {proxy.is_enabled()}")
            elif cmd == "running":
                print(f"Is running: {proxy.is_running()}")
            elif cmd == "prompts":
                action = input("get/set? ").strip().lower()
                if action == "get":
                    print(proxy.get_prompts())
                elif action == "set":
                    prompt_str = input("Enter prompts (comma-separated): ").strip()
                    prompts = [p.strip() for p in prompt_str.split(",")]
                    proxy.set_prompts(prompts)
                    print("Prompts updated")
            elif cmd == "output":
                index = input("Index [0]: ").strip()
                index = int(index) if index else 0
                prompt_filter = input("Wait for prompt (blank for any): ").strip()
                if prompt_filter:
                    timeout = input("Timeout [5]: ").strip()
                    timeout = float(timeout) if timeout else 5.0
                    result = proxy.get_output(index, timeout, prompt_filter)
                else:
                    result = proxy.get_output(index)
                print(f"Prompt: {result.get('prompt')}")
                print(f"Output: {result.get('output')}")
                if result.get('sameAsNext'):
                    print(f"sameAsNext: True")
            elif cmd == "stack":
                print(f"Stack size: {proxy.get_output_stack_size()}")
            elif cmd == "clear":
                proxy.clear_output_stack()
                print("Stack cleared")
            elif cmd == "reboot":
                confirm = input("Are you sure you want to reboot the Jetson? (yes/no): ").strip().lower()
                if confirm == "yes":
                    result = proxy.reboot()
                    print(result)
                    print("Connection will be lost as system reboots...")
                    break
                else:
                    print("Reboot cancelled")
            elif cmd == "help":
                print("Commands: ping, status, enable, disable, enabled, running, prompts, output, stack, clear, reboot, quit")
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

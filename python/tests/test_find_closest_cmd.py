import os
import sys
sys.path.append(os.path.abspath(os.path.join('..')))

from cmd_embed_mgr import CmdEmbedMgr

if __name__ == "__main__":
    cmd_embed_mgr = CmdEmbedMgr()
    cmd_embed_mgr.load_cmds_embeddings("../commands_embeddings.pkl")

if sys.stdin.isatty():
    # Interactive mode
    while True:
        test_string = input("Enter a command to find the closest match (or 'exit' to quit): ")
        if test_string.lower() == 'exit':
            sys.exit(0)
        closest_command, distance = cmd_embed_mgr.find_closest_command(test_string)
        print(f"Closest command to '{test_string}': '{closest_command}' with distance {distance}")
else:
    # Piped input mode
    for test_string in sys.stdin:
        test_string = test_string.strip()
        if not test_string:
            continue
        closest_command, distance = cmd_embed_mgr.find_closest_command(test_string)
        print(f"Closest command to '{test_string}': '{closest_command}' with distance {distance}")
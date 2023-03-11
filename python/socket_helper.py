import struct
import socket

# Helper function to recv n bytes or return None if EOF is hit
def recvall(socket: socket, n) -> bytes:
    fragments = []
    rcount = 0
    while rcount < n: 
        chunk = socket.recv(n - rcount)
        if not chunk:
            return None
        fragments.append(chunk)
        rcount += len(chunk)
    arr = b''.join(fragments)
    return arr

def get_response_real(socket: socket) -> str:
    raw_msglen = recvall(socket, 4)
    if not raw_msglen:
        return None
    msglen = struct.unpack('>I', raw_msglen)[0]
    # Read the message data
    return recvall(socket, msglen).decode()

def send_msg_real(socket: socket, msg: str):
    # Prefix each message with a 4-byte length (network byte order)
    bytes = msg.encode()
    bytes = struct.pack('>I', len(msg)) + bytes
    socket.sendall(bytes)  # send message
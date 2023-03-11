import socket
from socket_helper import *

class ChatbotSocketClient():
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.socket = None
        self._is_connected = False

    def connect(self):
        if self._is_connected:
            self.close()
        try:
            self.socket = socket.socket()  # instantiate
            self.socket.connect((self.host, self.port))  # connect to the server
            print("chatbot: opening connection")
            self._is_connected = True
            return True
        except Exception as e:
            print(f"chatbot: could not connect to chatbot service. Error: {e}")
            return False

    def get_response(self) -> str:
        try:
            return get_response_real(self.socket)
        except Exception as e:
            self.socket.close()  # close the connection
            self._is_connected = False
            print(f"chatbot: error getting response: {e}")
            return ""
                
    def send_msg(self, msg: str):
        try:
            send_msg_real(self.socket, msg)
            return True
        except Exception as e:
            print(f"chatbot: error sending message: {e}")
            self.socket.close()  # close the connection
            self._is_connected = False
            return False

    def close(self):
        try:
            print("chatbot: closing connection")
            self.socket.close()  # close the connection
            self._is_connected = False
            
        except Exception as e:
            None
    
    def is_connected(self):
        return self._is_connected

if __name__ == "__main__":
    def speak(text):
        print(f"SPEAKING: {text}")

    _chatbot = ChatbotSocketClient("192.168.1.41", 5124)
    # send text to chatbot and get response if connected
    if _chatbot.connect():
        intro = _chatbot.get_response()
        if not _chatbot.is_connected():
            exit()
        speak(intro)
    else:
        speak("sorry, i'm unable to open a chat right now.")
        exit()

    while True:
        print("input >", end='')
        phrase = input()
        if _chatbot.is_connected():
            if len(phrase) > 0:
                speak_response = True

                if phrase == "reset chat":
                    phrase = ".reset"
                elif phrase == "restart chat":
                    phrase = ".restart"
                elif phrase == "show log":
                    phrase = ".log"
                    speak_response = False
                
                if phrase[0] != '.':
                    print(f"Human: {phrase}")
                    
                _chatbot.send_msg(phrase)
                
                response = _chatbot.get_response()
                if not _chatbot.is_connected():
                    break

                if speak_response:
                    print("Orange: ", end='')
                    speak(response)

                print(response)

                if "goodbye" in phrase.lower() or phrase == ".restart":
                    _chatbot.close()
                    print("chat has ended")
                    break
        else:
            break
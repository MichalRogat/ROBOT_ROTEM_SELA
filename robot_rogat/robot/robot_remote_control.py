import socket
import json
import queue
from enum import Enum


PORT=1234


class CommandOpcode(Enum):
    motor = 1
    keep_alive = 2
    camera = 3
    telemetric = 4
    
class RobotRemoteControl():
    def __init__(self, control_q):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.control_q = control_q
        # bind the socket to a specific address and port
        self.server.bind(("0.0.0.0", PORT))
        print("Control Channel Server is Listening on all network interfaces")

    def start(self):
        self.server.listen(0)
        # accept incoming connections
        self.client_socket, self.client_address = self.server.accept()
        print(f"Accepted connection from {self.client_address[0]}:{self.client_address[1]}")
        while True:
            try:
                request = self.client_socket.recv(1024).decode('utf-8')
                for event_str in request.split('}'):
                    try:
                        event_str += '}'
                        event=json.loads(event_str)
                        self.control_q.put(event)
                    except ValueError as err:
                        continue
            except Exception as e :
                print(str(e))
                self.client_socket.close() #close connection to client
                break
           

    def Transmit(self, data):
        payload = bytes(json.dumps(data, cls=EnumEncoder),'utf-8')
        self.client_socket.sendall(payload)

class EnumEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, Enum):
            return obj.value
        return json.JSONEncoder.default(self, obj)
            
if __name__ == "__main__":
    dummy = queue.Queue
    rc= RobotRemoteControl(dummy)
    rc.start()
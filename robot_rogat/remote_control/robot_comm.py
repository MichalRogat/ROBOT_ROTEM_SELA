import socket
import queue
import threading
import json
from enum import Enum

HOST = "192.168.137.191"  # The server's hostname or IP address
PORT = 1234  # The port used by the server

class RobotComm():
    def __init__(self, rx_q):
        self.rx_q = rx_q
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((HOST, PORT))

    def RxHandler(self):
        while True:
            try:
                request = self.socket.recv(1024).decode('utf-8')
                for event_str in request.split('}'):
                    try:
                        event_str += '}'
                        event=json.loads(event_str)
                        #self.control_q.put(event)
                        print(event)
                    except ValueError as err:
                            continue
            except Exception as e :
                print(str(e))
                self.client_socket.close() #close connection to client
                break
            

    
    def Transmit(self, data):
        payload = bytes(json.dumps(data, cls=EnumEncoder),'utf-8')
        self.socket.sendall(payload)


class EnumEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, Enum):
            return obj.value
        return json.JSONEncoder.default(self, obj)
    

if __name__ == "__main__":
    dummy = queue.Queue()
    robot_comm = RobotComm(dummy)
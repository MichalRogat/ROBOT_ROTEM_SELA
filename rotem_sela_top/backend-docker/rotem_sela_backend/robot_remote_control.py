import socket
import json
import queue
from enum import Enum

PORT=1234

def set_keepalive_linux(sock, after_idle_sec=1, interval_sec=3, max_fails=5):
    """Set TCP keepalive on an open socket.

    It activates after 1 second (after_idle_sec) of idleness,
    then sends a keepalive ping once every 3 seconds (interval_sec),
    and closes the connection after 5 failed ping (max_fails), or 15 seconds
    """
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, after_idle_sec)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, interval_sec)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, max_fails)

class CommandOpcode(Enum):
    motor = 1
    keep_alive = 99
    camera = 3
    telemetric = 4
    pump = 5
    acc_calib = 6
    stop_all = 7

    
class RobotRemoteControl():
    def __init__(self, control_q):
        print(f"Start socket server")
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
        self.client_socket.settimeout(1)
        set_keepalive_linux(self.client_socket)
        print(f"Accepted connection from {self.client_address[0]}:{self.client_address[1]}")
        while True:
            try:
                requestData = self.client_socket.recv(1024)
                request = requestData.decode('utf-8')
                if len(requestData) == 0:
                    self.client_socket.close()
                    raise Exception()
                for event_str in request.split('}'):
                    try:
                        event_str += '}'
                        event=json.loads(event_str)
                        self.control_q.put(event)
                    except ValueError as err:
                        continue
            except socket.timeout:
                pass
            except Exception as e :
                print(str(e))
                try:
                    self.client_socket.close() #close connection to client
                except Exception as e:
                    pass
                self.control_q.put({'event':CommandOpcode.stop_all.value, 'value':0})
                self.server.listen(0)
                self.client_socket, self.client_address = self.server.accept()
                set_keepalive_linux(self.client_socket)
                continue
           

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
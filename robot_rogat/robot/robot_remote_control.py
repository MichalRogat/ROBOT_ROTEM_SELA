import socket
import json
import queue


PORT=1234

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
                for event in request.split('}'):
                    if event != '':
                        json.loads(event+'}')
                        print(event)

            except Exception as e :
                print(str(e))
                self.client_socket.close() #close connection to client
                break
           

    def Transmit(self, response):
        self.client_socket.send(pickle.dump(response))
            
if __name__ == "__main__":
    dummy = queue.Queue
    rc= RobotRemoteControl(dummy)
    rc.start()
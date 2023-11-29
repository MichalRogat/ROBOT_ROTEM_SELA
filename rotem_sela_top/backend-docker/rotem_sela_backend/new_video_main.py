import json
from linux_system_monitor import LinuxSystemStatus
import tornado.gen
import tornado.ioloop
import tornado.web
import tornado.websocket
from tornado.web import RequestHandler, Application
import asyncio
import v4l2py
from robot_main2 import RobotMain, stopVideo
import threading
import json
import multiprocessing
import time
import traceback

CAM_PORTS = [
    ([['2','1'],['7','8']], 5000),
    ([['3','9'], ['6','10']], 5001),
    ([['6','5'],['3','4']], 5002),
    ([['7','8'],['2','1']], 5003),
]

cameras = {}
devices = {}
isMain = True
subQueues = []
txQueues = []
barrier = multiprocessing.Barrier(4)

def sendCamsCB():
    return txQueues

def map_cams():
    cameras = LinuxSystemStatus.list_usb_cameras()
    map = {
            '1':{'dev' : cameras[0][1], 'width' : 640 ,'height' : 400, 'name':'cam1-side'},
            '2':{'dev' : cameras[0][1], 'width' : 640 ,'height' : 480, 'name':'cam1-front'},
            '3':{'dev' : cameras[1][1], 'width' : 640 ,'height' : 480, 'name':'cam2-front'},
            '4':{'dev' : cameras[1][1], 'width' : 640 ,'height' : 400, 'name':'cam2-side'},
            '5':{'dev' : cameras[2][1], 'width' : 640 ,'height' : 400, 'name':'cam3-side'},
            '6':{'dev' : cameras[2][1], 'width' : 640 ,'height' : 480, 'name':'cam3-front'},
            '7':{'dev' : cameras[3][1], 'width' : 640 ,'height' : 480, 'name':'cam4-front'},
            '8':{'dev' : cameras[3][1], 'width' : 640 ,'height' : 400, 'name':'cam4-side'},
            '9':{'dev' : cameras[4][1], 'width' : 640 ,'height' : 480, 'name':'cam5-front'},
            '10':{'dev' : cameras[4][1], 'width' : 640 ,'height' : 400, 'name':'cam5-side'},
           }
    
    print(map)
    return map
    
class CorsHandler(RequestHandler):
    def set_default_headers(self):
        self.set_header("Access-Control-Allow-Origin", "*")
        self.set_header("Access-Control-Allow-Headers", "*")
        self.set_header('Access-Control-Allow-Methods', 'POST, GET, OPTIONS')

    def options(self, *args, **kwargs):
        self.set_status(204)
        self.finish()


class IndexHandler(CorsHandler):
    def get(self):
        self.write(json.dumps('backend only'))


def websocket_server(port):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        app = tornado.web.Application(ChannelHandler.urls())
        # Setup HTTP Server
        http_server = tornado.httpserver.HTTPServer(app)
        http_server.listen(port, '0.0.0.0')
        tornado.ioloop.IOLoop.instance().start()
       
def flipCams():
    for q in subQueues:
        q.put({'opcode':'flip'})

def toggleCams():
    for q in subQueues:
        q.put({'opcode':'toggle'})

def videoFeedHandler(port, cam_id, queue, barrier, qt):
        global isMain
        isMain = False
        webSocket_thread = threading.Thread(target=websocket_server, args=(port,))
        webSocket_thread.start()

        res = map_cams()
        global cameras
        global devices
        camIdx = 0
        sideIdx = 0

        while True:
            video_dev = res[cam_id[sideIdx][camIdx]]['dev']
            if video_dev not in cameras:
                cameras[cam_id[sideIdx][camIdx]] = video_dev

            print(f"{cam_id} start feed")
            print(f"Open video device {video_dev}")
            barrier.wait();
            
            startTS = time.time()
            
            with v4l2py.Device(video_dev) as device:
                devices[cam_id[sideIdx][camIdx]] = device
                device.set_format(buffer_type=1, width=res[cam_id[sideIdx][camIdx]]['width'], height=res[cam_id[sideIdx][camIdx]]['height'], pixel_format='MJPG')
                device.set_fps(buffer_type=1, fps=10)
                for frame in device:
                    try:
                        if stopVideo:
                            break
                       
                        ChannelHandler.send_message(frame.data)
                        try:
                            item = queue.get(block=False)
                            if item['opcode'] == 'toggle':
                                camIdx = camIdx+1
                                if camIdx > 1:
                                    camIdx = 0
                            else:
                                sideIdx = sideIdx+1
                                if sideIdx > 1:
                                    sideIdx = 0
                            qt.put({"port":port,
                                    "cam_name":res[cam_id[sideIdx][camIdx]]['name']})
                            break
                        except Exception:
                            # traceback.print_exc()
                            pass
                    except Exception:
                        # traceback.print_exc()
                        break
                  

def make_app():
    return Application([
        (r"/", IndexHandler)
    ])

class ChannelHandler(tornado.websocket.WebSocketHandler):
    
    clients = set()

    def open(self):
        ChannelHandler.clients.add(self)

    def on_close(self):
        try:
            ChannelHandler.clients.remove(self)
        except Exception as e:
            print(str(e))

    @classmethod
    def send_message(cls, message: str):
        # print(f"Sending message {message} to {len(cls.clients)} client(s).")
        
        try:
            for client in cls.clients:
                try:
                    if isMain:
                        client.write_message(message, binary=False)
                    else:
                        client.write_message(message, binary=True)
                except Exception as e:
                    cls.clients.remove(client)
                    break
        except Exception as e:
            print(str(e))
          
    """
    Handler that handles a websocket channel
    """
    @classmethod
    def urls(cls):
        return [
            (r'/ws', cls,{}),  # Route/Handler/kwargs
        ]
    
    def initialize(self):
        self.channel = None
    
    def check_origin(self, origin):
        """
        Override the origin check if needed
        """
        return True
    


if __name__ == "__main__":
    processes = []
    
    for item in CAM_PORTS:
        queue = multiprocessing.Queue()
        qt = multiprocessing.Queue()
        process = multiprocessing.Process(target=videoFeedHandler, args=(item[1], item[0], queue, barrier, qt))
        processes.append(process)
        subQueues.append(queue)
        txQueues.append(qt)
        process.start()

    app = tornado.web.Application(ChannelHandler.urls())
    http_server = tornado.httpserver.HTTPServer(app)
    # Setup HTTP Server
    http_server.listen(8888, '0.0.0.0')
    print(f"Websocket started")
    # Start IO/Event loop
   
    obj = RobotMain()
    obj.setTelemetryChannel(ChannelHandler)
    obj.setFlipCallback(flipCams)
    obj.setToggleCallback(toggleCams)
    obj.setCamsCallback(sendCamsCB)


    tornado.ioloop.IOLoop.instance().start()
    for process in processes:
        process.join()
 


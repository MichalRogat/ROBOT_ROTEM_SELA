import json
from linux_system_monitor import LinuxSystemStatus
import cv2
import tornado.gen
import tornado.ioloop
import tornado.web
import tornado.websocket
from tornado.web import RequestHandler, Application
import asyncio
import sys
import v4l2py
from robot_main import RobotMain, stopVideo
import threading
import os
import numpy as np
import datetime
import os
import subprocess
import ffmpeg
import base64
import json
from robot_remote_control import CommandOpcode



if sys.platform == 'win32':
    asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
import multiprocessing

CAM_PORTS = {
    '2': 5000,
    '3': 5001,
    '6': 5002,
    '7': 5003
}

# Dictionary of cameras; the key is an identifier, the value is the OpenCV VideoCapture object
cameras = {
}
devices = {}
isMain = True
def map_cams():
    cameras = LinuxSystemStatus.list_usb_cameras()
    map = {
            '1':{'dev' : cameras[0][1], 'width' : 640 ,'height' : 400},
            '2':{'dev' : cameras[0][1], 'width' : 640 ,'height' : 480},
            '3':{'dev' : cameras[1][1], 'width' : 640 ,'height' : 480},
            '4':{'dev' : cameras[1][1], 'width' : 640 ,'height' : 400},
            '5':{'dev' : cameras[2][1], 'width' : 640 ,'height' : 400},
            '6':{'dev' : cameras[2][1], 'width' : 640 ,'height' : 480},
            '7':{'dev' : cameras[3][1], 'width' : 640 ,'height' : 480},
            '8':{'dev' : cameras[3][1], 'width' : 640 ,'height' : 400},
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
        print(f"Websocket started")


def videoFeedHandler(port, cam_id):
        global isMain
        isMain = False
        webSocket_thread = threading.Thread(target=websocket_server, args=(port,))
        webSocket_thread.start()

        res = map_cams()
        global cameras
        global devices
        frame_index = 1
        video_dev = res[cam_id]['dev']
        if video_dev not in cameras:
            cameras[cam_id] = video_dev

        # set_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        print(f"{cam_id} start feed")
        print(f"Open video device {video_dev}")
        with v4l2py.Device(video_dev) as device:
                devices[cam_id] = device
                device.set_format(buffer_type=1, width=res[cam_id]['width'], height=res[cam_id]['height'], pixel_format='MJPG')
                device.set_fps(buffer_type=1, fps=10)
                for frame in device:
                    try:
                        if stopVideo or devices[cam_id] is None:
                            break
                        save_path = f"/home/rogat/video_frames"
                        cam_path = os.path.join(save_path, str(cam_id))
                        filename = os.path.join(cam_path, f"{frame_index}.jpg")
                        frame_index += 1
                        try:
                            with open(filename, 'wb') as file:
                                file.write(frame.data)
                        except Exception as ex:
                            print(ex)

                        frame_msg = {'opcode':CommandOpcode.frame_data.value, 'cam_id':cam_id, 'frame': base64.b64encode(frame.data).decode('utf-8')}
                        ChannelHandler.send_message(frame.data)
                    
                        # self.write(b'--frame\r\n')
                        # self.write(b'Content-Type: image/jpeg\r\n\r\n')
                        # self.write(frame.data)  # Assuming frame data is already in MJPEG format
                        # self.write(b'\r\n')
                        # await self.flush()
                    except Exception as e:
                        print(f"video feeding error: {e}")
                        break
                    # await tornado.gen.sleep(0.01)  # Sleep for 10ms
                # timestamp = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
                # video_filename = os.path.join(save_path, f"{cam_id}_output_{timestamp}.mp4")
                # os.system(f"ffmpeg -framerate 10 -i {os.path.join(cam_path, '%d.jpg')} -c:v libx264 -pix_fmt yuv420p {video_filename}")



def make_app():
    return Application([
        (r"/", IndexHandler)
    ])


def run_server(port, cam):
    print(f"start {cam}, port {port}")
    app = make_app()
    app.listen(port, address="0.0.0.0")
    tornado.ioloop.IOLoop.current().start()


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
        
            for client in cls.clients:
                try:
                    if isMain:
                        client.write_message(message, binary=False)
                    else:
                        client.write_message(message, binary=True)
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
    
    for cam, port in CAM_PORTS.items():
        process = multiprocessing.Process(target=videoFeedHandler, args=(port, cam))
        processes.append(process)
        process.start()

    app = tornado.web.Application(ChannelHandler.urls())
    
    # Setup HTTP Server
    http_server = tornado.httpserver.HTTPServer(app)
    http_server.listen(8888, '0.0.0.0')
    print(f"Websocket started")
    # Start IO/Event loop
   
    
    obj = RobotMain()
    obj.setTelemetryChannel(ChannelHandler)


    # cam1_thread = threading.Thread(target=videoFeedHandler, args=['2'] )
    # cam1_thread.start()
    # cam2_thread = threading.Thread(target=videoFeedHandler, args=['3'] )
    # cam2_thread.start()
    # cam3_thread = threading.Thread(target=videoFeedHandler, args=['6'] )
    # cam3_thread.start()
    # cam4_thread = threading.Thread(target=videoFeedHandler, args=['7'] )
    # cam4_thread.start()
    tornado.ioloop.IOLoop.instance().start()
    for process in processes:
        process.join()
 


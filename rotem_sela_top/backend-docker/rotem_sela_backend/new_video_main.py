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

if sys.platform == 'win32':
    asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
import multiprocessing

CAM_PORTS = {
    'cam1': 5000,
    'cam2': 5001,
    'cam3': 5002,
    'cam4': 5003
}

# Dictionary of cameras; the key is an identifier, the value is the OpenCV VideoCapture object
cameras = {
}

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


class VideoFeedHandler(CorsHandler):
    async def get(self, cam_id):
        res = map_cams()
        global cameras

        if stopVideo:
            self.set_status(404)
            self.write("Camera stopped")
       
        video_dev = res[cam_id]['dev']
        if video_dev not in cameras:
            cameras[cam_id] = video_dev

        self.set_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        print(f"{cam_id} start feed")

        if sys.platform == 'win32':
            while True:
                if cam_id not in cameras:
                    break
                try:
                    ret, frame = cameras[cam_id].read()
                    if not ret:
                        break
                    _, jpeg = cv2.imencode('.jpg', frame)
                    img_data = jpeg.tobytes()
                    self.write(b'--frame\r\n')
                    self.write(b'Content-Type: image/jpeg\r\n\r\n')
                    self.write(img_data)
                    self.write(b'\r\n')
                    await self.flush()
                except Exception as e:
                    print(f"video feeding error: {e}")
                await tornado.gen.sleep(0.01)  # Sleep for 10ms
        else:
            print(f"Open video device {video_dev}")
            with v4l2py.Device(video_dev) as device:
                device.set_format(buffer_type=1, width=res[cam_id]['width'], height=res[cam_id]['height'], pixel_format='MJPG')
                device.set_fps(buffer_type=1, fps=10)
                for frame in device:
                    try:
                        # if video_dev not in cameras:
                        #     print("!!!!exit")
                        #     break 
                        if stopVideo:
                            break
                        self.write(b'--frame\r\n')
                        self.write(b'Content-Type: image/jpeg\r\n\r\n')
                        self.write(frame.data)  # Assuming frame data is already in MJPEG format
                        self.write(b'\r\n')
                        await self.flush()
                    except Exception as e:
                        print(f"video feeding error: {e}")
                        break
                    await tornado.gen.sleep(0.01)  # Sleep for 10ms


class TurnOnHandler(CorsHandler):
    def post(self, cam_id):
        print(f"{cam_id} turn on")
        global cameras
        if  sys.platform == 'win32':
            if cam_id not in cameras:
                index = int(cam_id[-1]) - 1
                self.write(json.dumps({'success': True, 'message': f'{cam_id} turned on'}))
            else:
                self.write(json.dumps({'success': False, 'message': 'Camera already on'}))
        else:
            video_dev = f"/dev/{cam_id}"
            if video_dev not in cameras:
                cameras[video_dev] = video_dev
                self.write(json.dumps({'success': True, 'message': f'{cam_id} turned on'}))
            else:
                self.write(json.dumps({'success': False, 'message': 'Camera already on'}))



class TurnOffHandler(CorsHandler):
    def post(self, cam_id):
        global cameras
        print(f"{cam_id} turn off")
        if cam_id in cameras:
            if sys.platform == 'win32':
                cameras[cam_id].release()
            del cameras[cam_id]
            self.write(json.dumps({'success': True, 'message': f'{cam_id} turned off'}))
        else:
            self.write(json.dumps({'success': False, 'message': 'Camera not found'}))


def make_app():
    return Application([
        (r"/", IndexHandler),
        (r"/video_feed/(.*)", VideoFeedHandler),
        (r"/turn_on/(.*)", TurnOnHandler),
        (r"/turn_off/(.*)", TurnOffHandler),
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
        try:
            for client in cls.clients:
           
                client.write_message(message)
        except Exception as e:
            print(str(e))
            # if client.get_status() == 101:
            #     try:
            #         cls.clients.remove(client)
            #     except Exception as e:
            #         print(str(e))
            #     break

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
        process = multiprocessing.Process(target=run_server, args=(port, cam))
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

    tornado.ioloop.IOLoop.instance().start()

    for process in processes:
        process.join()

 


import json

import cv2
import tornado.gen
import tornado.ioloop
import tornado.web
import tornado.websocket

from tornado.web import RequestHandler, Application
import asyncio
import sys
# from robot_main import RobotMain

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

def map_cam_id(cam_id):
    print(f"map camId {cam_id}")
    if cam_id == 'cam2':
        return 'cam3'
    
    elif cam_id == 'cam4':
        return 'cam5'
    else:
        return cam_id
    

        # self.comm.Transmit(info)
class WebSocketHandler(tornado.websocket.WebSocketHandler):
    def initialize(self):
        print('Websocket opened')

    def open(self):
        print('New connection')
        self.write_message('Test from server')

    def on_close(self):
        print('Connection closed')

    def test(self):
        self.write_message("scheduled!")

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
        global cameras
        cam_id = map_cam_id(cam_id)
        if cam_id not in cameras:
            self.set_status(404)
            self.write("Camera not found")
            return

        self.set_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        print(f"{cam_id} start feed")
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
            await tornado.gen.sleep(0.1)  # Sleep for 10ms

class TurnOnHandler(CorsHandler):
    def post(self, cam_id):
        cam_id = map_cam_id(cam_id)
        print(f"{cam_id} turn on")
        global cameras
        if cam_id not in cameras:
            index = int(cam_id[-1]) - 1
            cameras[cam_id] = cv2.VideoCapture(index)
            cameras[cam_id].set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cameras[cam_id].set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cameras[cam_id].set(cv2.CAP_PROP_FPS, 10)
            #if not sys.platform == 'win32':
                #cameras[cam_id].set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

            self.write(json.dumps({'success': True, 'message': f'{cam_id} turned on'}))
        else:
            self.write(json.dumps({'success': False, 'message': 'Camera already on'}))


class TurnOffHandler(CorsHandler):
    def post(self, cam_id):
        cam_id = map_cam_id(cam_id)
        global cameras
        print(f"{cam_id} turn off")
        if cam_id in cameras:
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

def make_ws_app():
    return tornado.web.Application([
    (r'/ws', WebSocketHandler),
    ])


def run_server(port, cam):
    print(f"start {cam}, port {port}")
    app = make_app()
    app.listen(port, address="0.0.0.0")
    tornado.ioloop.IOLoop.current().start()

 
if __name__ == "__main__":
    processes = []

    for cam, port in CAM_PORTS.items():
        process = multiprocessing.Process(target=run_server, args=(port, cam))
        processes.append(process)
        process.start()
    
    # robotObj = RobotMain()
    
    # application = make_ws_app()

    # http_server = tornado.httpserver.HTTPServer(application)
    # http_server.listen(8888)
  
    # tornado.ioloop.IOLoop.current().start()

    for process in processes:
        process.join()

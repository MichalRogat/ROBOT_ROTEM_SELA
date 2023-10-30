import json
import multiprocessing

import tornado.gen
import tornado.ioloop
import tornado.web
import v4l2py
from tornado.web import RequestHandler, Application

CAM_PORTS = {
    'cam1': 5000,
    'cam2': 5001,
    'cam3': 5002,
    'cam4': 5003,
    'system': 5004
}

V4L2_BUF_TYPE_VIDEO_CAPTURE = 1

# Dictionary of cameras; the key is an identifier, the value is the OpenCV VideoCapture object
cameras = {
}

def map_cam(camId):
    # TODO add call to linux monitor
    map = {
            1:{'dev' : '/dev/video0', 'width' : 640 ,'height' : 400},
            2:{'dev' : '/dev/video0', 'width' : 640 ,'height' : 480},
            3:{'dev' : '/dev/video2', 'width' : 640 ,'height' : 480},
            4:{'dev' : '/dev/video2', 'width' : 640 ,'height' : 400},
            5:{'dev' : '/dev/video4', 'width' : 640 ,'height' : 400},
            6:{'dev' : '/dev/video4', 'width' : 640 ,'height' : 480},
            7:{'dev' : '/dev/video6', 'width' : 640 ,'height' : 480},
            8:{'dev' : '/dev/video6', 'width' : 640 ,'height' : 400},
           }

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
        if cam_id not in cameras:
            self.set_status(404)
            self.write("Camera not found")
            return

        self.set_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        print(f"{cam_id}({cameras[cam_id]}) start feed")

        with v4l2py.Device(cameras[cam_id]) as device:
            device.set_format(buffer_type=V4L2_BUF_TYPE_VIDEO_CAPTURE, width=640, height=480, pixel_format='MJPG')
            device.video_capture.set_fps(buffer_type=V4L2_BUF_TYPE_VIDEO_CAPTURE, fps=10)
            for frame in device:
                try:
                    if cam_id not in cameras:
                        break
                    self.write(b'--frame\r\n')
                    self.write(b'Content-Type: image/jpeg\r\n\r\n')
                    self.write(frame.data)  # Assuming frame data is already in MJPEG format
                    self.write(b'\r\n')
                    await self.flush()
                except Exception as e:
                    print(f"video feeding error: {e}")
                await tornado.gen.sleep(0.01)  # Sleep for 10ms


class TurnOnHandler(CorsHandler):
    def post(self, cam_id):
        print(f"{cam_id} turn on")
        global cameras
        if cam_id not in cameras:
            video_dev = f"/dev/{cam_id}"
            cameras[cam_id] = video_dev
            self.write(json.dumps({'success': True, 'message': f'{cam_id} turned on'}))
        else:
            self.write(json.dumps({'success': False, 'message': 'Camera already on'}))


class TurnOffHandler(CorsHandler):
    def post(self, cam_id):
        global cameras
        print(f"{cam_id} turn off")
        if cam_id in cameras:
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


if __name__ == "__main__":
    processes = []

    for cam, port in CAM_PORTS.items():
        process = multiprocessing.Process(target=run_server, args=(port, cam))
        processes.append(process)
        process.start()

      app = tornado.web.Application(ChannelHandler.urls())
    
    # Setup HTTP Server
    http_server = tornado.httpserver.HTTPServer(application)
    http_server.listen(LISTEN_PORT, LISTEN_ADDRESS)
    
    # Start IO/Event loop
    tornado.ioloop.IOLoop.instance().start()

    for process in processes:
        process.join()

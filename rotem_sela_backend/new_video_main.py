import json

import cv2
import tornado.gen
import tornado.ioloop
import tornado.web
from tornado.web import RequestHandler, Application
import asyncio
import sys
import v4l2py

if sys.platform == 'win32':
    asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
import multiprocessing

CAM_PORTS = {
    'cam1': 5000,
    'cam2': 5001,
    'cam3': 5002,
    'cam4': 5003,
    'system': 5004
}

# Dictionary of cameras; the key is an identifier, the value is the OpenCV VideoCapture object
cameras = {
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
        video_dev = f"/dev/{cam_id}"
        if video_dev not in cameras:
            self.set_status(404)
            self.write("Camera not found")
            return

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
            with v4l2py.Device(video_dev) as device:
                device.set_format(buffer_type=1, width=640, height=400, pixel_format='MJPG')
                device.set_fps(buffer_type=1, fps=10)
                for frame in device:
                    try:
                        if video_dev not in cameras:
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


if __name__ == "__main__":
    processes = []

    for cam, port in CAM_PORTS.items():
        process = multiprocessing.Process(target=run_server, args=(port, cam))
        processes.append(process)
        process.start()

    for process in processes:
        process.join()


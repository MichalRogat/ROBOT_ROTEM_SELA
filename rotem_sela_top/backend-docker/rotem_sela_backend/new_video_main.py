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
from datetime import datetime
import subprocess
import sys


CAM_PORTS_FLIP = {}
# {
#             5000 :['5.1','5.1','5.1','5.1','5.1','5.1'],
#             5001: ['4.1','4.1','4.1','4.1','3.1','4.1'],
#             5002: ['1.1','1.1','1.2','3.1','4.2','3.1'],
#             5003: ['6.1','2.1','2.2','6.2','7.2','6.1']
           
#             }

CAM_PORTS_NOT_FLIP ={}
# {
           
#             5000: ['1.1','5.1','5.1','5.1','5.1','5.1'],
#             5001: ['2.1','4.1','4.1','4.1','4.1','4.1'],
#             5002:['3.1','1.1','1.2','3.1','3.2','3.1'],
#             5003: ['4.1','2.1','2.2','6.2','7.2','6.1']
#             }


CAM_PORTS = CAM_PORTS_NOT_FLIP

cameras = {}
devices = {}
isMain = True
subQueues = []
txQueues = []
barrier = multiprocessing.Barrier(4)
startTS = time.time()
lock = threading.Lock()
map={}

def sendCamsCB():
    return txQueues



def map_cams():
    camNotFaund=0
    cameras_list_bash = []
    cameras = LinuxSystemStatus.list_usb_cameras()
    for i in range(len(cameras)):
        temp2=cameras[i][0]
        temp=temp2.split('-')
        cameras_list_bash.append(temp[1])
    
    with open('../../../../entitiesFlipping.json', 'r') as file:
            json_data = json.load(file)
            cameras_list = json_data.get("cameras", [])     
    for i,x in enumerate(cameras_list):
        if x not in cameras_list_bash:
            cameras_list[i]=""


    id2name = {x: i + 1 for i, x in enumerate(cameras_list)}
    #id2name = {"1.1":1,"1.2.1":7,"1.2":3,"1.2.3":6,"1.2.4":5,"1.4":4, "1.3":2}

    # mapExample = {
    #         str(id2name[cameras[0][0].split("-")[1]])+".1":{'dev' : cameras[0][1], 'width' : 640 ,'height' : 480, 'name':'cam1-side'},
    #         str(id2name[cameras[0][0].split("-")[1]])+".2":{'dev' : cameras[0][1], 'width' : 640 ,'height' : 400, 'name':'cam1-front'},
    #         str(id2name[cameras[1][0].split("-")[1]])+".1":{'dev' : cameras[1][1], 'width' : 640 ,'height' : 480, 'name':'cam2-front'},
    #         str(id2name[cameras[1][0].split("-")[1]])+".2":{'dev' : cameras[1][1], 'width' : 640 ,'height' : 400, 'name':'cam2-side'},
    #         str(id2name[cameras[2][0].split("-")[1]])+".1":{'dev' : cameras[2][1], 'width' : 640 ,'height' : 480, 'name':'cam3-side'},
    #         str(id2name[cameras[2][0].split("-")[1]])+".2":{'dev' : cameras[2][1], 'width' : 640 ,'height' : 400, 'name':'cam3-front'},
    #         str(id2name[cameras[3][0].split("-")[1]])+".1":{'dev' : cameras[3][1], 'width' : 640 ,'height' : 480, 'name':'cam4-front'},
    #         str(id2name[cameras[3][0].split("-")[1]])+".2":{'dev' : cameras[3][1], 'width' : 640 ,'height' : 400, 'name':'cam4-side'},
    #         str(id2name[cameras[4][0].split("-")[1]])+".1":{'dev' : cameras[4][1], 'width' : 640 ,'height' : 480, 'name':'cam5-front'},
    #         str(id2name[cameras[4][0].split("-")[1]])+".2":{'dev' : cameras[4][1], 'width' : 640 ,'height' : 400, 'name':'cam5-side'},
    #         str(id2name[cameras[5][0].split("-")[1]])+".1":{'dev' : cameras[5][1], 'width' : 640 ,'height' : 480, 'name':'cam5-front'},
    #         str(id2name[cameras[5][0].split("-")[1]])+".2":{'dev' : cameras[5][1], 'width' : 640 ,'height' : 400, 'name':'cam5-side'},
    #         str(id2name[cameras[6][0].split("-")[1]])+".1":{'dev' : cameras[6][1], 'width' : 640 ,'height' : 480, 'name':'cam5-front'},
    #         str(id2name[cameras[6][0].split("-")[1]])+".2":{'dev' : cameras[6][1], 'width' : 640 ,'height' : 400, 'name':'cam5-side'},
    #         }
    
    for i,x in enumerate(cameras_list):
        if x != '':
            map[str(id2name[cameras[i-camNotFaund][0].split("-")[1]])+".1"]={'dev' : cameras[i-camNotFaund][1], 'width' : 640 ,'height' : 480, 'name':'cam1-side'}
            map[str(id2name[cameras[i-camNotFaund][0].split("-")[1]])+".2"]={'dev' : cameras[i-camNotFaund][1], 'width' : 640 ,'height' : 400, 'name':'cam1-side'}
        else:
            camNotFaund+=1       

    print(map)
    print("")
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
        q.put({'event':'flip'})

def toggleCams(event):
    for q in subQueues:
        q.put({'event':f'{event}'})

def sendCommand(command:int):
    for q in subQueues:
        q.put({
            'command':command
        })

def videoFeedHandler(port, cam_id, queue, barrier, qt):
        
            global isMain
            isMain = False
            webSocket_thread = threading.Thread(target=websocket_server, args=(port,))
            webSocket_thread.start()
            res = map_cams()
            print ("len-res",len(res))
            global cameras
            global devices
            camIdx = 0
            isFlip = False
            isError = False
            CamResflag=True

            while True:
                try:    
                    video_dev = res[cam_id[camIdx]]['dev']
                    if video_dev not in cameras:
                        cameras[cam_id[camIdx]] = video_dev

                    print(f"{cam_id} start feed")
                    print(f"Open video device {video_dev}")

                    if not isError:
                       # barrier.wait();
                        pass
                    else:
                        isError = False
                    
                
                    with v4l2py.Device(video_dev) as device:
                        devices[cam_id[camIdx]] = device
                        device.set_format(buffer_type=1, width=res[cam_id[camIdx]]['width'], height=res[cam_id[camIdx]]['height'], pixel_format='MJPG')
                        device.set_fps(buffer_type=1, fps=15)
                        for frame in device:
                            try:
                                if stopVideo:
                                    break
                            
                                ChannelHandler.send_message(frame.data)
                                qt.put({"port":port,
                                            "cam_name":res[cam_id[camIdx]]['name']})
                                try:
                                    item = queue.get(block=False)
                                    if item['event'] == str(ord('0')) or item['event'] == str(ord('1')) or item['event'] == str(ord('2')) or item['event'] == str(ord('3')) or item['event'] == str(ord('4')) or item['event'] == str(ord('`')):
                                        with open('../../../../entitiesFlipping.json', 'r') as file:
                                            json_data = json.load(file)
                                            CAM_PORTS_NOT_FLIP = json_data.get("CAM_PORTS_NOT_FLIP", {}) 
                                            CAM_PORTS_FLIP = json_data.get("CAM_PORTS_FLIP", {})

                                    if isFlip:
                                            cam_id = CAM_PORTS_FLIP[port]
                                    else:
                                            cam_id = CAM_PORTS_NOT_FLIP[port]        

                                    if item['event'] == 'flip':
                                        isFlip = not isFlip
                                        if isFlip:
                                            cam_id = CAM_PORTS_FLIP[port]
                                        else:
                                            cam_id = CAM_PORTS_NOT_FLIP[port]
                                    if item['event'] == str(ord('0')):
                                        print("0 press")  
                                        camIdx = 0
                                    if item['event'] ==  str(ord('1')):
                                        print("1 press")
                                        camIdx = 1
                                    if item['event'] == str(ord('2')):
                                        print("2 press") 
                                        camIdx = 2
                                    if item['event'] == str(ord('3')):
                                        print("3 press")
                                        camIdx = 3
                                    if item['event'] == str(ord('4')):
                                        print("4 press") 
                                        camIdx = 4
                                    if item['event'] == str(ord('`')):
                                        print("` press")
                                        camIdx = 5  
                                        
                                    qt.put({"port":port,
                                            "cam_name":res[cam_id[camIdx]]['name']})
                                    break
                                except Exception:
                                    # traceback.print_exc()
                                    #videoFeedHandler(port, cam_id, queue, barrier, qt)
                                    pass
                            except Exception:
                                # traceback.print_exc()
                                break
                            # if port == 5000:
                            #     print("Cam "+str(port)+" "+str(time.time()))
                            time.sleep(0.067)
                except Exception as e:
                    print(e)
                   
                    try:
                        res = map_cams()
                    except:
                        pass
                    isError = True
                    #result = subprocess.run(['v4l2-ctl', '--list-devices'], capture_output=True, text=True, check=False)
                    #camerasList = LinuxSystemStatus.list_usb_cameras()
                    # if len(camerasList) <7:
                         
                    #       res=subprocess.run(['./camHubRestart.sh'], check=True, capture_output=True, text=True)
                    #       print(res.stdout)
                    pass    

def make_app():
    return Application([
        (r"/", IndexHandler)
    ])

class ChannelHandler(tornado.websocket.WebSocketHandler):
    
    clients = set()

    def open(self):
        ChannelHandler.clients.add(self)
        print("client add " ,time.time())


    def on_close(self):
        try:
            ChannelHandler.clients.remove(self)
        except Exception as e:
            print(str(e))

    
    @classmethod
    def send_message(cls, message):
        # print(f"Sending message {message} to {len(cls.clients)} client(s).")
        try:
            for client in cls.clients:
                try:
                    if isMain:
                        client.write_message(message, binary=False)
                    else:
                        ts = int((time.time() - startTS)*1000)
                        client.write_message(ts.to_bytes(4, byteorder='big') + message, binary=True)
                        
                except Exception as e:
                   
                    print("error " ,str(e))
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
    

    with open('../../../../entitiesFlipping.json', 'r') as file:
        json_data = json.load(file)
        CAM_PORTS_NOT_FLIP = json_data.get("CAM_PORTS_NOT_FLIP", {}) 
        CAM_PORTS_FLIP = json_data.get("CAM_PORTS_FLIP", {})
    # with open('../../../../entitiesFlipping.json', 'r') as file:
    #     json_data = json.load(file)
    #     CAM_PORTS_FLIP = json_data.get("CAM_PORTS_FLIP", {})     

    CAM_PORTS = CAM_PORTS_NOT_FLIP  
    
    for item in CAM_PORTS:
        queue = multiprocessing.Queue()
        qt = multiprocessing.Queue()
        process = multiprocessing.Process(target=videoFeedHandler, args=(item, CAM_PORTS[item], queue,barrier, qt))
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
    obj.setCommandKB(toggleCams) #michal - cameras, toggle from keyboard
    # obj.setToggleCallback(toggleCams) -> no toggle from button circle
    obj.setCamsCallback(sendCamsCB)


    tornado.ioloop.IOLoop.instance().start()
    for process in processes:
        process.join()
 


import cv2
import time
import numpy as np
import cv2
from VideoStreaming import *
import websocket 
import json
import time
import base64

class DdsVideoEngine:
    def __init__(self):
        super(DdsVideoEngine, self).__init__()

        camera_source = "rtsp://admin:Abc.12345@192.168.0.65/ch0/stream0"
        # camera_source = 0
        self.VIDEO = VideoStreaming(callback=self.drone_message, video_source=camera_source)

        self.is_scan_drone = False
        self.ws = None
        t2 = Thread(target=self.websocket_thread)
        t2.daemon = True
        t2.start() 

    def drone_message(self, detectedImage, cropImage, largestBoundingBox, droneNumber, droneImage):

        # (flag, encodedImage) = cv2.imencode(".jpg", detectedImage)
        currentDate = datetime.datetime.now()
        
        IMAGE_SHAPE = detectedImage.shape
        (flag, jpegImage) = cv2.imencode(".jpg", detectedImage)
        encoded_image = base64.encodestring(jpegImage)
        # print(type(encoded_image))
        # payload = {
        #     'from': 'rasp',
        #     'image': encoded_image.decode('utf-8'),
        #     'shape': IMAGE_SHAPE,
        # }
        incidentType = 1 # motion
        if droneNumber > 0:
            incidentType = 2

        print(currentDate.strftime("%Y%m%d%H%M%S.%f"),'incident_message',incidentType)

        sendObj = {
            "from": 0,
                    "time": currentDate.strftime("%Y%m%d%H%M%S.%f"),
                    "type": 1, # alarm
                    "incidentType": incidentType,
                    "info": "",
                    'image': encoded_image.decode('utf-8'),
                    'shape': IMAGE_SHAPE,
                }
        self.ws.send(
            json.dumps(sendObj)
        )

    def websocket_thread(self):
        while True:
            try:
                if self.ws == None:
                    self.ws = websocket.create_connection("ws://127.0.0.1:5001/VideoWebSocket")

                data = json.loads(self.ws.recv())
                print(data)
                if data['Type'] == 0 and data['Info'] == "start_scan":
                    self.is_scan_drone = True
                elif data['Type'] == 0 and data['Info'] == "stop_scan":
                    self.is_scan_drone = False
                # data = self.ws.recv()
                self.VIDEO.is_scan_drone = self.is_scan_drone
                print(self.is_scan_drone)

            except:
                self.ws = None
                pass
            
            time.sleep(0.01)

    def show(self):
        while True:
            if self.VIDEO.outputFrame is None:
                continue
            # encode the frame in JPEG format
            (flag, encodedImage) = cv2.imencode(".jpg", self.VIDEO.outputFrame)
            # ensure the frame was successfully encoded
            if not flag:
                continue
            # yield the output frame in the byte format
            yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' +
                  bytearray(encodedImage) + b'\r\n')

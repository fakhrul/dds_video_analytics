import time
import cv2
import numpy as np


from threading import Thread, Lock
# from time import sleep
import detect_square

from websocket import create_connection, WebSocketConnectionClosedException
import json

import datetime
import os
from dotenv import load_dotenv

lock = Lock()
outputFrame = None


class VideoStreaming(object):
    def __init__(self):
        super(VideoStreaming, self).__init__()

        load_dotenv()
        # CAMERA_IP = '192.168.0.65'
        CAMERA_SOURCE = os.getenv('CAMERA_SOURCE')
        # self.VIDEO = cv2.VideoCapture(
        #     'rtsp://admin:Abc.12345@'+CAMERA_IP+'/ch0/stream0')
        print(CAMERA_SOURCE)
        self.VIDEO = cv2.VideoCapture(0)


        # self.VIDEO = cv2.VideoCapture(0)

        # self.MODEL = ObjectDetection()

        self._preview = True
        self._flipH = False
        self._detect = False
        self._exposure = self.VIDEO.get(cv2.CAP_PROP_EXPOSURE)
        self._contrast = self.VIDEO.get(cv2.CAP_PROP_CONTRAST)
        # self.FRAME = None
        # thread = Thread(target = self.threaded_function, args=(1,))
        # thread.start()
        # thread.join()

        self.frame_count = 0
        self.previous_frame = None
        self.previous_alarm_date = datetime.datetime.now()

        # self.processVideo()
        t = Thread(target=self.detect_motion, args=(1,))
        t.daemon = True
        t.start()

        self.ws = create_connection("ws://127.0.0.1:5001/VideoWebSocket")
        # self.ws.send(
        #     json.dumps(
        #         {
        #         "type": "subscribe",
        #         "product_ids": ["BTC-USD"],
        #         "channels": ["matches"],
        #         }
        #     )
        # )

        self.is_scan_drone = False

        t2 = Thread(target=self.websocket_thread, args=(1,))
        t2.daemon = True
        t2.start()

    def websocket_thread(self, args):
        while True:
            try:
                data = json.loads(self.ws.recv())
                print(data)
                if data['Type'] == "command" and data['Info'] == "start_scan":
                    self.is_scan_drone = True
                elif data['Type'] == "command" and data['Info'] == "stop_scan":
                    self.is_scan_drone = False
                # data = self.ws.recv()
                print(self.is_scan_drone)

            except:
                pass

    def detect_motion(self, args):
        global outputFrame, lock

        while True:
            # wait until the lock is acquired
            # with lock:
            ret, snap = self.VIDEO.read()
            if ret == False:
                continue

            img_brg = snap.copy()
            if self.is_scan_drone == False:
                outputFrame = snap.copy()
                continue
            
            img_rgb = cv2.cvtColor(src=img_brg, code=cv2.COLOR_BGR2RGB)
            img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2GRAY)
            prepared_frame = cv2.GaussianBlur(
                src=img_gray, ksize=(5, 5), sigmaX=0)

            # to exclude certain area
            # prepared_frame = cv2.rectangle(prepared_frame,(0,0),(600,200),(255,255,255),-1)

            if (self.previous_frame is None):
                self.previous_frame = prepared_frame
                continue

            # calculate difference and update previous frame
            diff_frame = cv2.absdiff(
                src1=self.previous_frame, src2=prepared_frame)
            self.previous_frame = prepared_frame

            # 4. Dilute the image a bit to make differences more seeable; more suitable for contour detection
            kernel = np.ones((5, 5))
            diff_frame = cv2.dilate(diff_frame, kernel, 1)

            # 5. Only take different areas that are different enough (>20 / 255)
            thresh_frame = cv2.threshold(
                src=diff_frame, thresh=20, maxval=255, type=cv2.THRESH_BINARY)[1]

            contours, _ = cv2.findContours(
                image=thresh_frame, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(image=img_rgb, contours=contours, contourIdx=-1,
                             color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)

            contours, _ = cv2.findContours(
                image=thresh_frame, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)

            largestArea = 0
            largestImage = None

            for contour in contours:
                area = cv2.contourArea(contour)
                if area < 1500:
                    # too small: skip!
                    continue
                
                (x, y, w, h) = cv2.boundingRect(contour)
                img_update = cv2.rectangle(img=img_brg, pt1=(x, y), pt2=(
                    x + w, y + h), color=(0, 255, 0), thickness=2)

                if area > largestArea:
                    largestArea = area
                    largestImage = img_update

            if largestArea > 0:
                currentDate = datetime.datetime.now()
                delta = currentDate - self.previous_alarm_date

                if delta.total_seconds() > 5:
                    if os.getenv('IS_DISABLE_ALERT') != 'TRUE':
                        self.ws.send(
                            json.dumps(
                                {
                                    "type": "alarm",
                                    "info": "motion",
                                    "alarm": "motion",
                                }
                            )
                        )
                        # save image 
                        # filename = currentDate.strftime("%m/%d/%Y, %H:%M:%S")
                        filename =  currentDate.strftime("%Y%m%d%H%M%S.%f") + ".jpg"
                        cv2.imwrite(filename, largestImage)
                        self.previous_alarm_date = currentDate

            snap = img_brg.copy()
            # snap = detect_square.capture(snap)

            # check if the output frame is available, otherwise skip
            # the iteration of the loop
            label = 'Scanning Intrusion'
            H, W = snap.shape[0], snap.shape[1]
            font = cv2.FONT_HERSHEY_COMPLEX
            color = (255, 255, 255)
            cv2.putText(snap, label, (W//2, H//2), font, 1, color, 2)
    #
            outputFrame = snap.copy()

            time.sleep(0.01)

    @property
    def preview(self):
        return self._preview

    @preview.setter
    def preview(self, value):
        self._preview = bool(value)

    @property
    def flipH(self):
        return self._flipH

    @flipH.setter
    def flipH(self, value):
        self._flipH = bool(value)

    @property
    def detect(self):
        return self._detect

    @detect.setter
    def detect(self, value):
        self._detect = bool(value)

    @property
    def exposure(self):
        return self._exposure

    @exposure.setter
    def exposure(self, value):
        self._exposure = value
        self.VIDEO.set(cv2.CAP_PROP_EXPOSURE, self._exposure)

    @property
    def contrast(self):
        return self._contrast

    @contrast.setter
    def contrast(self, value):
        self._contrast = value
        self.VIDEO.set(cv2.CAP_PROP_CONTRAST, self._contrast)

    def show(self):
        # grab global references to the output frame and lock variables
        global outputFrame, lock
        # loop over frames from the output stream
        while True:
            # wait until the lock is acquired
            with lock:
                # check if the output frame is available, otherwise skip
                # the iteration of the loop
                if outputFrame is None:
                    continue
                # encode the frame in JPEG format
                (flag, encodedImage) = cv2.imencode(".jpg", outputFrame)
                # ensure the frame was successfully encoded
                if not flag:
                    continue
            # yield the output frame in the byte format
            yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' +
                  bytearray(encodedImage) + b'\r\n')

    # def show(self):
    #     # while self.FRAME != None:
    #     #     frame = cv2.imencode('.jpg', self.FRAME)[1].tobytes()
    #     #     yield (b'--frame\r\n'b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
    #     #     time.sleep(0.01)

    #     while(self.VIDEO.isOpened()):
    #         ret, snap = self.VIDEO.read()
    #         if self.flipH:
    #             snap = cv2.flip(snap, 1)

    #         if ret == True:
    #             if self._preview:
    #                 # snap = cv2.resize(snap, (0, 0), fx=0.5, fy=0.5)
    #                 if self.detect:
    #                     snap = self.MODEL.detectObj(snap)

    #                 label = 'FLYBOTS'
    #                 H, W = snap.shape[0],snap.shape[1]
    #                 font = cv2.FONT_HERSHEY_COMPLEX
    #                 color = (255,255,255)
    #                 cv2.putText(snap, label, (W//2, H//2), font, 2, color, 2)
    #             else:
    #                 snap = np.zeros((
    #                     int(self.VIDEO.get(cv2.CAP_PROP_FRAME_HEIGHT)),
    #                     int(self.VIDEO.get(cv2.CAP_PROP_FRAME_WIDTH))
    #                 ), np.uint8)
    #                 label = 'camera disabled'
    #                 H, W = snap.shape
    #                 font = cv2.FONT_HERSHEY_COMPLEX
    #                 color = (255,255,255)
    #                 cv2.putText(snap, label, (W//2, H//2), font, 2, color, 2)

    #             frame = cv2.imencode('.jpg', snap)[1].tobytes()
    #             yield (b'--frame\r\n'b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
    #             time.sleep(0.01)

    #         else:
    #             break
    #     print('off')

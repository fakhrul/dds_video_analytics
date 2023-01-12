import time
import cv2
import numpy as np
from threading import Thread, Lock
import websocket 
import json
import datetime
import os
from dotenv import load_dotenv
from MotionDetection import *
from YoloDetection import *
lock = Lock()
# outputFrame = None


class VideoStreaming(object):
    def __init__(self, callback, video_source = 0):
        super(VideoStreaming, self).__init__()
        self.callback = callback
        self.VIDEO = cv2.VideoCapture(video_source)
        self.outputFrame = None
        self.outputObjectFrame = None
        self.droneImage = None

        self.previous_alarm_date = datetime.datetime.now()

        self.motion = MotionDetection()
        self.yolo = YoloDetection(capture_index=0, model_name='drone.pt')

        self.frame_count = 0
        self.previous_frame = None

        # self.processVideo()
        t = Thread(target=self.detect_motion)
        t.daemon = True
        t.start()

        self.is_scan_drone = False
        self.ws = None


    def detect_motion(self):
        print('detect motion')
        while True:
            if self.VIDEO.isOpened() == False:
                continue

            ret, frame = self.VIDEO.read()
            if ret == False:
                continue
            

            if self.is_scan_drone == False:
                self.outputFrame = frame
                self.outputObjectFrame = None
                self.droneImage = None
                continue

            droneImage = None

        # # exclude area
        # start_point = (0, 0)
        # end_point = (500, 125)
        # color = (0, 0, 0)
        # thickness = -1
        # currentFrame = cv2.rectangle(currentFrame, start_point, end_point, color, thickness)

        # start_point = (0, 500)
        # end_point = (500, 900)
        # color = (0, 0, 0)
        # thickness = -1
        # currentFrame = cv2.rectangle(currentFrame, start_point, end_point, color, thickness)
        # 
            excluded_area = [
                [0, 0,500, 125],
                [0, 500, 500, 900]
            ]            
            is_detect_motion, displayImage, detectedImage, cropImage, largestBoundingBox= self.motion.process_image(frame.copy(),excluded_area)
            if is_detect_motion == True:
                currentDate = datetime.datetime.now()
                delta = currentDate - self.previous_alarm_date
                if delta.total_seconds() > 5:
                    # droneImage, droneNumber = self.yolo.process_image(cropImage.copy())
                    # self.callback(detectedImage, cropImage, largestBoundingBox, droneNumber, droneImage)
                    print('callback')
                    self.callback(detectedImage, cropImage, largestBoundingBox, 0, droneImage)
                    self.previous_alarm_date = currentDate

            self.outputFrame = displayImage
            self.outputObjectFrame = cropImage
            self.droneImage = droneImage



def message(largestImage, largestCropImage, largestBoundingBox, droneNumber, droneImage):
    print("CallBack", droneNumber)

# if __name__ == "__main__":
#     callback = message
#     # video_source ="rtsp://admin:Abc.12345@192.168.0.65/ch0/stream0"
#     video_source =0

#     videoStream = VideoStreaming(callback, video_source)
#     while True:
#         try:
#             cv2.imshow('HD Webcam', videoStream.outputFrame)
#             # cv2.imshow('Motion', videoStream.outputObjectFrame)
#             # cv2.imshow('Drone', videoStream.droneImage)
#             if cv2.waitKey(1) & 0xFF == ord('q'):
#                 break
#         except:
#             pass






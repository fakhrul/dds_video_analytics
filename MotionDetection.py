import time
import cv2
import numpy as np


from threading import Thread, Lock
# from time import sleep
import detect_square

import websocket 
import json

import datetime
import os
from dotenv import load_dotenv

import rel
lock = Lock()
outputFrame = None


class MotionDetection(object):
    def __init__(self):
        super(MotionDetection, self).__init__()

        self.previous_detected_date = datetime.datetime.now()
        self.prevFrame = None

    def process_image(self,  currentFrame):
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

        img_rgb = cv2.cvtColor(src=currentFrame, code=cv2.COLOR_BGR2RGB)
        img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2GRAY)
        prepared_frame = cv2.GaussianBlur(
            src=img_gray, ksize=(5, 5), sigmaX=0)

        if self.prevFrame is None:
            self.prevFrame = prepared_frame
            return False, currentFrame

        # calculate difference and update previous frame
        diff_frame = cv2.absdiff(src1=self.prevFrame, src2=prepared_frame)
        self.prevFrame = prepared_frame

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
            img_update = cv2.rectangle(img=currentFrame, pt1=(x, y), pt2=(
                x + w, y + h), color=(0, 255, 0), thickness=2)

            if area > largestArea:
                largestArea = area
                largestImage = img_update

        is_detect_motion = False

        if largestArea > 0:
            currentDate = datetime.datetime.now()
            delta = currentDate - self.previous_detected_date

            # if delta.total_seconds() > 5:
            is_detect_motion = True
            filename =  currentDate.strftime("%Y%m%d%H%M%S.%f") + ".jpg"
            cv2.imwrite(filename, largestImage)
            self.previous_detected_date = currentDate

        snap = currentFrame.copy()
        label = 'Scanning Intrusion'
        H, W = snap.shape[0], snap.shape[1]
        font = cv2.FONT_HERSHEY_COMPLEX
        color = (255, 255, 255)
        cv2.putText(snap, label, (W//2, H//2), font, 1, color, 2)

        return is_detect_motion, snap

    def detect_motion(self, args):
        # global outputFrame, lock
        cam = cv2.VideoCapture(0)
        while True:
            if cam.isOpened() == False:
                continue

            ret, self.snap = cam.read()
            if ret == False:
                continue
            
            if self.snap is None:
                continue
            
            cv2.imshow('HD Webcam', self.snap)
            continue

        while True:
            time.sleep(0.01)
            # wait until the lock is acquired
            # with lock:
            if self.VIDEO.isOpened() == False:
                continue

            ret, snap = self.VIDEO.read()
            if ret == False:
                continue
            
            if snap is None:
                continue
            
            cv2.imshow('HD Webcam', snap)
            continue

            img_brg = snap.copy()
            if self.is_scan_drone == False:
                outputFrame = snap.copy()
                continue
            
            # exclude area
            start_point = (0, 0)
            end_point = (500, 125)
            color = (0, 0, 0)
            thickness = -1
            img_brg = cv2.rectangle(img_brg, start_point, end_point, color, thickness)

            start_point = (0, 500)
            end_point = (500, 900)
            color = (0, 0, 0)
            thickness = -1
            img_brg = cv2.rectangle(img_brg, start_point, end_point, color, thickness)

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


if __name__ == "__main__":
    motion = MotionDetection()
    
    camera = cv2.VideoCapture(0)
    while True:
        success, frame = camera.read()
        if not success:
            break
        result, outputFrame = motion.process_image(frame.copy())
        print(result)
        cv2.imshow('HD Webcam', outputFrame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


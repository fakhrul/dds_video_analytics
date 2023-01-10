import cv2
import time
import numpy as np
import mediapipe as mp
import time
# reference
# https://pynerds.blogspot.com/2021/08/basic-pose-detection-and-tracking-using.html

vid = cv2.VideoCapture(0)

# Creating object for pose detection
myPose = mp.solutions.pose
pose = myPose.Pose()

mpDraw = mp.solutions.drawing_utils

currentTime = 0
previousTime = 0

while(vid.isOpened()):
    ret, frame = vid.read()

    img = frame.copy()
    # Converting BGR image to RGB image
    RGBimg = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = cv2.resize(img, None, fx=0.15, fy=0.15)

    # Getting landmark points and storing in poseLandmarks variable
    result = pose.process(RGBimg)
    poseLandmarks = result.pose_landmarks

    # This 'if' block will run when it's having the poseLandmark points
    if poseLandmarks:
        # Accessing each of the 33 landmarks
        """
        for index, poseLandmark in enumerate(poseLandmarks.landmark):
            w, h, c = img.shape
            cx, cy = int(poseLandmark.x * h), int(poseLandmark.y * w)
            if index == 11:
                cv2.circle(img, (cx, cy), 10, (255, 0, 0), cv2.FILLED)
        """
        # Drawing and connecting the points
        mpDraw.draw_landmarks(img, poseLandmarks, myPose.POSE_CONNECTIONS)

    # Calculating frames per second
    currentTime = time.time()
    fps = 1 / (currentTime - previousTime)
    previousTime = currentTime

    # This will show the fps value on screen
    cv2.putText(img, str(int(fps)), (30, 70), cv2.FONT_HERSHEY_COMPLEX, 2, (186, 4, 22), 5)

    cv2.imshow('frame', frame)


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break        
vid.release()
cv2.destroyAllWindows()

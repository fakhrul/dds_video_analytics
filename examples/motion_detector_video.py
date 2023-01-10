import cv2
import time
import numpy as np

# reference
# https://www.geeksforgeeks.org/webcam-motion-detector-python/


vid = cv2.VideoCapture('251120222/500M.mp4')
# vid = cv2.VideoCapture(0)
# prev_frame_time = 0
# new_frame_time = 0

frame_count = 0
previous_frame = None

while(vid.isOpened()):
    ret, frame = vid.read()

    img_brg = frame.copy()

    img_rgb = cv2.cvtColor(src=img_brg, code=cv2.COLOR_BGR2RGB)
    img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2GRAY)
    prepared_frame = cv2.GaussianBlur(src=img_gray, ksize=(5,5), sigmaX=0)

    #to exclude certain area
    # prepared_frame = cv2.rectangle(prepared_frame,(0,0),(600,200),(255,255,255),-1)


    if (previous_frame is None):
        previous_frame = prepared_frame
        continue
    
    # calculate difference and update previous frame
    diff_frame = cv2.absdiff(src1=previous_frame, src2=prepared_frame)
    previous_frame = prepared_frame

    # 4. Dilute the image a bit to make differences more seeable; more suitable for contour detection
    kernel = np.ones((5, 5))
    diff_frame = cv2.dilate(diff_frame, kernel, 1)

    # 5. Only take different areas that are different enough (>20 / 255)
    thresh_frame = cv2.threshold(src=diff_frame, thresh=20, maxval=255, type=cv2.THRESH_BINARY)[1]

    contours, _ = cv2.findContours(image=thresh_frame, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(image=img_rgb, contours=contours, contourIdx=-1, color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)

    contours, _ = cv2.findContours(image=thresh_frame, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        if cv2.contourArea(contour) < 50:
            # too small: skip!
            continue
        (x, y, w, h) = cv2.boundingRect(contour)
        cv2.rectangle(img=img_brg, pt1=(x, y), pt2=(x + w, y + h), color=(0, 255, 0), thickness=2)


    # cv2.imshow('img_rgb', img_rgb)
    cv2.imshow('img_brg', img_brg)
    # cv2.imshow('img_gray', img_gray)
    # cv2.imshow('prepared_frame', prepared_frame)
    cv2.imshow('thresh_frame', thresh_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break        
vid.release()
cv2.destroyAllWindows()

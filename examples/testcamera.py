#!/usr/bin/env python

import cv2

camera = cv2.VideoCapture(0)

while True:
	success, frame = camera.read()
	if not success:
		break
	cv2.imshow('HD Webcam', frame)
	
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break


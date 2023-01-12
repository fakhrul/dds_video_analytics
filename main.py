import cv2
import time
import numpy as np
import cv2
from flask import Flask,render_template,  Response

from DdsVideoEngine import *

app = Flask(__name__)

dds = DdsVideoEngine()

@app.route('/')
def home():
    TITLE = 'Object detection'
    return render_template('index.html', TITLE=TITLE)
    

@app.route('/video_feed')
def video_feed():
    return Response(dds.show(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(debug=True,host='0.0.0.0')

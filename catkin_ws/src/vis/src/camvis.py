#!/usr/bin/env python
import cv2
import numpy as np
import requests

port = '8081'  # Replace with the actual port of your Yawcam server
stream_url = 'http://' + "192.168.1.68" + ':' + port + '/video.mjpg'

stream = requests.get(stream_url, stream=True)
bytes = b''

while True:
    bytes += stream.raw.read(1024)
    a = bytes.find(b'\xff\xd8')
    b = bytes.find(b'\xff\xd9')
    if a != -1 and b != -1:
        jpg = bytes[a:b + 2]
        bytes = bytes[b + 2:]
        frame = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
        #cv2.imshow('Yawcam Stream', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()

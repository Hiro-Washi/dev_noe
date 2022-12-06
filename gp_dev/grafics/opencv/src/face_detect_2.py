#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, time, cv2
cascade_path="/home/hiroto/src_opencv/opencv-4.2.0/data/haarcascades/haarcascade_frontalface_default.xml"
window = "Push ESC key to stop this program"
assert os.path.isfile(cascade_path), 'haarcascade_frontalface_default.xml: not found'

def detect_face(img):
    cascade = cv2.CascadeClassifier(cascade_path)
    resized_img = cv2.resize(img, (360,480))
    facerect = cascade.detectMultiScale(resized_img, scaleFactor=1.1, minNeighbors=1, minSize=(100, 100))
    # processing as much as the number of faces
    if len(facerect) > 0:
        for rect in facerect:
            cv2.rectangle(img, tuple(rect[0:2]),tuple(rect[0:2]+rect[2:4]),(255,255,255),3)
    cv2.imshow(window, img) 
    print(facerect)

if(__name__ == '__main__'):
    # in case of default cama: 0
    capture = cv2.VideoCapture(0)
    # capture processing
    while(True):
        key = cv2.waitKey(5) # ([ms])
        if(key == 27):
            print("exit.")
            break
        # capture the image
        ret, img = capture.read()
        # if doesn't start reading
        if(ret == False):
            print("Capture Failed. ")
            break
        detect_face(img)
        time.sleep(0.050)
    # close cam
    capture.release()
    cv2.destroyAllWindows()

#!/usr/bin/env python3
# -*- conding: utf-* -*-
import cv2

image = cv2.VideoCapture(0)

wshile True:
    ret, frame = image.read()

    cv2.imshow('camera', frame)
    
    key = cv2.raitKey(10)
    if key ===27:
        break

image.release()
cv2.destroyAllWindows()

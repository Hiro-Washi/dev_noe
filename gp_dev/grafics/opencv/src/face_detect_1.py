#coding: utf-8

import cv2
import numpy as np

faceCascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

img = cv2.imread('image.jpg', cv2.IMREAD_COLOR)
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
face = faceCascade.detectMultiScale(gray, 1.1, 3)

if len(face) > 0:
	for rect in face:
		cv2.rectangle(img, tuple(rect[0:2]), tuple(rect[0:2]+rect[2:4]), (0, 0,255), thickness=2)
        # (img,pt1,pt2,color,thickness)
        # pt1-Top left vertex, pt2-Lower right apex, color, line width of thickness
        # [[258 224 217 217]]  x y w h
        # cv2.rectangle(img, tuple(rect[0:2]),tuple(rect[0:2]+rect[2:4]),(255,255,255),3)
        # --> cv2.rectangle(img,(258,224),(475,441),(255,255,255),3)
else:
	print "no face"

cv2.imwrite('detected.jpg', img)

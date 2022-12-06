#!/usr/bin/env python3
# -*- coding: utf-8 -*
import rospy
import cv2
import collections
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
#from ros_openpose.msg import Frame
from cv_bridge import CvBridge, CvBridgeError
from happymimi_msgs.srv import SetStr, SetStrResponse

class DetectObjectColor(object):


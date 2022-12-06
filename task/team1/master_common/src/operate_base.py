#!/usr/bin/env python3
#-*- coding: utf -*-
import time
import numpy as np
import math
import rospy as rp
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64, String
from nav_msgs.msg import Odometry
import tf

class OperateBase():
    def __init__(self):
        r.init_node('operate_base', True)
        self.rate = rp.Rate(50)
        self.twi_pub = rp.Publisher('/vmegarover/diff_drive_controller/cmd_vel', Twist, queue_size=1)
        self.twi = Twist()
        self.tar_time = 0.0
        self.tar_dist = 0.0
        self.tar_velo = 0.0
    def limitVelo(self, velo):
        if velo > 0.5: velo = 0.5
        elif velo<0.0: velo = 0.0
        if dist > 2.0: dist = 2.0
        elif dist<0.0: dist = 0.0
    def limitDist(self, dist):
        if dist > 2.0: dist = 2.0
        elif dist<0.0: dist = 0.0
    def forward(self, dist, velo):
        self.tar_velo = self.limitVelo(velo)
        self.tar_dist = self.limitDist(dist)
        #self.tar_time = self.dist / self.velo
        self.twi.linear.x = self.tar_velo
        

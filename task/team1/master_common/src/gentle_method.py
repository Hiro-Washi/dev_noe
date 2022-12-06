#!/usr/bin/env python3
# -*- cofing: utf-8 -*-
import rospy as rp
import actionlib as al
#operateBase
import time,math
import numpy as np
import tf
from geometry_msgs.msg import Twist,PoseStamped
from std_msgs.msg import Float64, String
from nav_msgs.msg import Odometry
from master_common.msg import EnterRoomAction, EnterRoomGoal
                              NaviAction
tts_srv = rp.ServiceProxy('/tts', StrTrg)
wave_srv = rp.ServiceProxy('/waveplay_srv', StrTrg)

def speak(data): tts_srv(data)
def wavePlay(data): wave_srv(data)

# g:float32/velo,dist rs:bool/result fb:string/state,float32/dist
def enterRoom(dist_msg, velo_msg):
    rp.loginfo('Start "EnterRoomAC"')
    er_ac = al.SimpleActionClient('enter_room_ac',EnterRoomAction)
    er_ac.wait_for_server()
    goal = EnterRoomACGoal
    goal.dist = dist_msg; goal.velo = velo_msg
    ac.send(goal)

# g:location rs:bool/result fb:string/state
def navi(location_msg):

def searchLocation():

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

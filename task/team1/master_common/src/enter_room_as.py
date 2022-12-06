#!/usr/bin/env python3
#-*- coding:utf-8 -*-
import rospy as rp
import actionlib as al
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from master_common.msg import EnterRoomAction,EnterRoomResult
from gentle_method import OperateBase, speak

class EnterRoomAS():
    rp.init_node('enter_room_ac', True)
    rp.loginfo('EnterRoomActionServer is started...')
    self.sas = actionlib.SimpleActionServer('/enter_room_ac', EnterRoomAction, executeCB(), False); self.sas.start()
    self.2dlidar_sub = rp.Subscriber('/scan', LaserScan, self.2dLidarCB)
    self.2dlidar_dist = 999.9
    self.safe_dist = 0.8
    #self.move = OperateBase()
    
    def 2dLidarCB(self,msg):
        self.2dlidar_dist = msg.ranges[359]

    def wait(self):
        #while self.2dlidar_dist <
        speak('Please open the door')
        while self.2dlidar_dist < self.safe_dist:
            rp.sleep(0.1)
        speak('Thank you')
    def executeCB(self, goal):
        try:
            rp.loginfo('Execute: EnterRoomAC')
            if self.2dlidar_dist < safe_dist:
                wait()
            move_dist = goal.dist + 

        except rp.ROSInterruptException as ROSIE:
            rp.loginfo('Interrupted!: EnterRoomAC:', ROSIE)
            pass
        except KeyboardInterrupt as KI:
            rp.loginfo('Finish!: EnterRoomAC:', KI)
            pass
        except:
            rp.loginfo('Error: EnterRoomAC')
            pass
if __name__='__main__':
    er_as = EnterRoomAS()
    rp.spin()

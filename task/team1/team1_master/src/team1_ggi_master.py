#!/usr/bin/env python3
#-*- conding: utf-8 -*-
import rospy as rp, smach as sm
from enter_room.srv import EnterRoom
from happymimi_navigation.srv import NaviLocation
from voice_common_pkg.srv import GgiLearning#####
from common_function import *
#from ggi_voice import 
#from ggi_graphics import
class SetPosition(sm.State):
	def __init__(self):
		sm.State.__init__(self, outcomes = ['sp_done']''',
                               input_key = ['sp_cmd_in']''')
		self.navi_srv = rp.ServiceProxy('/navi_location_server',NaviLocation)
	# Move to Operator
    def execute(sm.State):
		rp.loginfo('Executing state: SET POSITION')
        
        #sp_state = 1
        #if sp_state = 1:
        #    sp_state += 1
        #    self.navi_srv('start'); return 'sp_done'#####
        #elif sp_state = 4:
        #    self.navi_srv('exit'); return 'sp_done'
        #else:
        #    self.navi_srv('operater'); return 'sp_done'
class Enter(sm.State):
    def __init__(self):
        sm.State.__init__(self, outcomes = ['enter_done'])
		self.er = rp.Proxy('/enter_room_server', EnterRoom)
	def execute(self, userdata):
		result = self.er(1.0, 0.3).result
		if result: return 'enter_done' ; else: pass #####
class Train(smach.State):
	def __init__(self):
		sm.State.__init__(self, outcomes = ['train_done'])
		self.train = rp.ServiceProxy('/ggi_training_phase',Empty)
	def execute(self, userdata):
		rp.loginfo('Executing state')
		self.train()
		return 'train_done'
class Learn(sm.State):
    def __init__(self):
        sm.State.__init__(self, outcomes = ['learn_done'])
        self.learn_srv = rp.ServiceProxy('/test_phase',GgiLearning)
    def execute(self, userdata):
		rospy.loginfo('Executing state: LEARN')
        self.learn_srv()
		return 'learn_done'
class Listen(sm.State):
    def __init__(self):
        sm.State.__init__(self, outcomes = [],
                               input_key = ['listen_count_in'],
                              output_key = ['listen_count_out',
                                            ''])
    ##### 3 times
    def execute(self, userdata):
        rp.loginfo('Executing state: LISTEN')
        lc = userdata.lc_count_in
        
        if lc

class Action(sm.State):
    def __init__(self):
        sm.State.__init__(self, outcomes = ['act_done','act_falture'],
                              input_keys = ['',''],
                             output_keys = ['',''])
        ###
    def execute(self, userdata):
        
#class Test(sm.State):
#    def __init__(self):
#        sm.State.__init__(self, outcomes = [])
#	def execute(self, userdata):
#		rp.loginfo('Executing state: TEST')

def main():
    sm0 = sm.StateMachine(outcomes = ['succeeded','aborted','preempted']
    #sm0.userdata.ggi_state = 0
    sm0.userdata.listen_count = 0
    with sm0:
        sm.StateMachine.add('SET_POSITION', SetPosition(),
                            transitions = {'sp_done':'LISTEN'},
							remapping   = {'':''})
        sm.StateMachine.add('ENTER', Enter(),
                            transitions = {'enter_done':'LEARN'})
        sm.StateMachine.add('TRAIN', Train(),
                            transitions = {'train_done':''})
        sm.StateMachine.add('LEARN', Learn(),
                            transitions = {'learn_done':'SET_POSITION'})
        sm.StateMachine.add('LISTEN', Listen(),
                            transitions = {'listen_count_in':'listen'})
        sm.StateMachine.add('LEARN', Learn(),
                            transitions = {'learn_done'})
    outcomes = sm0.execute()
if __name__ == '__main__':
    rp.init_node('team1_ggi_master')
    main()

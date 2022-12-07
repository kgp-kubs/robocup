import rospy,sys
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from role import HeadOn_recieve, DribbleAndKickUpayan
from kubs import kubs
from krssg_ssl_msgs.srv import bsServer
from utils.geometry import Vector2D
from utils.functions import *
from utils.geometry import *
from krssg_ssl_msgs.msg import point_2d
from utils.config import *
from math import *
from utils.functions import *
from utils.math_functions import *
import time

pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)
ball_moving=False

def function(id_1,state):
	kub = kubs.kubs(id_1,state,pub)
	kub.update_state(state)
	print(kub.kubs_id)
	g_fsm = HeadOn_recieve.HeadonRecieve(id_1)
	g_fsm.add_kub(kub)
	print('something before spin')
	g_fsm.spin()

def function2(id_,state):
	kub = kubs.kubs(id_,state,pub)
	kub.update_state(state)
	print(kub.kubs_id)
	g_fsm = DribbleAndKickUpayan.DribbleAndKick()
	# g_fsm = GoToPoint.GoToPoint()
	g_fsm.add_kub(kub)
	# g_fsm.add_point(point=kub.state.ballPos,orient=normalize_angle(pi+atan2(state.ballPos.y,state.ballPos.x-3000)))
	#g_fsm.add_theta(theta=normalize_angle(pi+atan2(state.ballPos.y,state.ballPos.x+3000)))
	if state.ballPos.x==0:
		if state.ballPos.y >0:
			g_fsm.add_theta(theta= pi/2)
		if state.ballPos.y <0:
			g_fsm.add_theta(theta = -pi/2)
		else:
			g_fsm.add_theta(theta = 0)
	else:
		g_fsm.add_theta(theta = normalize_angle(atan2(state.ballPos.y, state.ballPos.x)))
	g_fsm.as_graphviz()
	g_fsm.write_diagram_png()
	print('something before spin')
	g_fsm.spin()
	return True
	

def main(ball_moving):
	rospy.init_node('node',anonymous=False)
	while (True):
		state=None
		rospy.wait_for_service('bsServer',)
		getState = rospy.ServiceProxy('bsServer',bsServer)
		try:
			state=getState(state)
		except rospy.ServiceException, e:
			print ("Service call failed:",e)
		if state:
			
			if(ball_moving==False):
				print("process 1")
				ball_moving=function2(1,state.stateB)
				print("ball moving:",ball_moving)

			# kub = kubs.kubs(0,state,pub)
			# print(type(BeliefState.ballVel))
			if(ball_moving):
				print("process 2")
				function(0,state.stateB)
				ball_moving=False
				print("ball moving p2:",ball_moving)
				
if __name__ == '__main__':
    main(ball_moving)

				
            
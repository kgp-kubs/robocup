import rospy,sys
from utils.geometry import Vector2D
from utils import functions
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from krssg_ssl_msgs.msg import gr_Robot_Command
from krssg_ssl_msgs.msg import BeliefState
from role import  GoOnArc, GoToPoint
from multiprocessing import Process
from kubs import kubs
from krssg_ssl_msgs.srv import bsServer
from math import atan2,pi
pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)

def run_function(id_,state):
	kub = kubs.kubs(id_,state,pub)
	kub.update_state(state)
	print(kub.kubs_id)
	g_fsm = GoOnArc.GoOnArc()
	g_fsm.add_kub(kub)
	center = Vector2D(0,0)
	g_fsm.add_center(center)
	kub_pos = kub.get_pos()
	g_fsm.add_target(Vector2D(-kub.state.awayPos[2].x,-kub.state.awayPos[2].y))
	# g_fsm.bigger_arc()
	# g_fsm.bigger_arc()
	print('something before spin')
	g_fsm.spin()
	# 



rospy.init_node('node',anonymous=False)
start_time = rospy.Time.now()
start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   


while True:
	state = None
	rospy.wait_for_service('bsServer',)
	getState = rospy.ServiceProxy('bsServer',bsServer)
	try:
		state = getState(state)
	except rospy.ServiceException, e:
		print("Error ",e)	
	if state:
		print('lasknfcjscnajnstate',state.stateB.homePos)
		run_function(4,state.stateB)
		print('chal ja')
		# break
rospy.spin()


import rospy,sys
from utils.geometry import Vector2D
from utils.functions import *
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from krssg_ssl_msgs.msg import gr_Robot_Command
from krssg_ssl_msgs.msg import BeliefState
from role import  PassReceive22
from multiprocessing import Process
from kubs import kubs
from krssg_ssl_msgs.srv import bsServer
from math import atan2,pi
from utils.functions import *
import multiprocessing
from multiprocessing.managers import BaseManager
import threading

def1 = int(sys.argv[1])
def2 = int(sys.argv[2])


flag1 = False
flag2 = False

class CustomManager(BaseManager):
    # nothing
    pass

def function1(id_1,id_2,conn,state,pub):
    kub = kubs.kubs(id_1,state,pub)
    kub.update_state(state)
    g_fsm = PassReceive22.PassReceive()
    g_fsm.add_kub(kub)
    g_fsm.add_conn(conn)
    g_fsm.add_receiver(id_2)
    print('something before spin')
    flag = g_fsm.spin() 
    return flag

def function2(id_1,id_2,conn,state,pub):
    kub = kubs.kubs(id_2,state,pub)
    kub.update_state(state)
    g_fsm = PassReceive22.PassReceive()
    g_fsm.add_kub(kub)
    g_fsm.add_conn(conn)
    g_fsm.add_receiver(id_1)
    print('something before spin')
    flag = g_fsm.spin() 
    return flag

def main1(process_id,conn):
    global def1,def2,flag1
    pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)
    rospy.init_node('node' + str(process_id),anonymous=False)

    while True:
        state = None
        rospy.wait_for_service('bsServer',)
        getState = rospy.ServiceProxy('bsServer',bsServer)
        try:
            state = getState(state)
        except rospy.ServiceException, e:
            print("service_exception")		
        if state:
            print("process 1")
            flag = function1(def1,def2,conn,state.stateB,pub)
            if flag:
                flag1 = True
                break

def main2(process_id,conn):
    global def1,def2,flag2
    pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)
    rospy.init_node('node' + str(process_id),anonymous=False)

    while True:
        state = None
        rospy.wait_for_service('bsServer',)
        getState = rospy.ServiceProxy('bsServer',bsServer)
        try:
            state = getState(state)
        except rospy.ServiceException, e:
            print("service_exception")		
        if state:
            print("process 2")
            flag = function2(def1,def2,conn,state.stateB,pub)
            if flag:
                flag2 = True
                break


conn1, conn2 = multiprocessing.Pipe(duplex=True)
p1 = multiprocessing.Process(target=main1, args=(0,conn1))
p2 = multiprocessing.Process(target=main2, args=(1,conn2))
p1.start()
p2.start()
p1.join()
p2.join()

if not flag1 and not flag2:
    rospy.spin()


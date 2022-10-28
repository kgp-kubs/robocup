from kubs import kubs, cmd_node
from velocity import run
from velocity import run_w
import rospy,sys
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from krssg_ssl_msgs.msg import gr_Robot_Command
from utils.geometry import Vector2D
from utils import config, functions
import krssg_ssl_msgs.srv as srv
import math
import numpy as np

kub = None
start_time = None
TARGET = None
CENTER = None
RADIUS = None
ROTATE = True
FLAG_move = False
FLAG_turn = False
POINT = None
rotate = False
TAKE_BIGGER_ARC = False
vx_end = 0
vy_end = 0


rospy.wait_for_service("bsServer",)
getState = rospy.ServiceProxy('bsServer',srv.bsServer)

prev_state = None
try:
    prev_state = getState(prev_state).stateB
except rospy.ServiceException, e:
    print("Error ", e)

def init(_kub,target,center,radius,rotate, point, take_bigger_arc):
    global kub,TARGET,RADIUS,CENTER,FLAG_move,FIRST_CALL,POINT,TAKE_BIGGER_ARC,ROTATE
    kub = _kub
    TARGET = Vector2D()
    TARGET.x = target.x
    TARGET.y = target.y
    CENTER = Vector2D()
    CENTER.x = center.x
    CENTER.y = center.y
    RADIUS = radius
    ROTATE = rotate
    FLAG_move = False
    FIRST_CALL = True
    POINT = point
    TAKE_BIGGER_ARC = take_bigger_arc

v_tan_prev = 0
v_norm_prev = 0

def velocity_planner(bot_pos):
    global v_tan_prev, v_norm_prev
    #PD controller
    # angular velocity control
    if ROTATE:
        kub_pos = kub.get_pos()
        bot_ball_dir = math.atan2(kub_pos.y-kub.state.ballPos.y, kub_pos.x-kub.state.ballPos.x)
        vw = run_w.Get_Omega(kub.kubs_id, bot_ball_dir, kub.state.homePos)
    else:
        vw = 0
    # tangential velocity controller
    consts_tan = np.array([4,0.05])
    if POINT is None:
        theta1 = math.atan2(bot_pos.y-CENTER.y,bot_pos.x-CENTER.x)
        theta2 = math.atan2(TARGET.y-CENTER.y,TARGET.x-CENTER.x)
        theta = theta2-theta1
        theta = functions.normalize_angle(theta)
        if TAKE_BIGGER_ARC == True:
            if theta<0:
                theta = 2*math.pi+theta
            else:
                theta = theta-2*math.pi
    else:
        theta1 = math.atan2(bot_pos.y-CENTER.y,bot_pos.x-CENTER.x)
        theta2 = math.atan2(TARGET.y-CENTER.y,TARGET.x-CENTER.x)
        theta3 = math.atan2(POINT.y-CENTER.y,POINT.x-CENTER.x)
        alpha1 = functions.normalize_angle(theta3-theta1)
        alpha2 = functions.normalize_angle(theta2-theta3)
        theta = alpha1+alpha2
    dist_tan = theta*RADIUS
    values_tan = np.array([[dist_tan],[-v_tan_prev]])
    vel_tangential = consts_tan.dot(values_tan)
    #print("vel tangential", vel_tangential)
    #normal velocity controller
    consts_rad = np.array([4,0.05])
    radius1 = math.sqrt((bot_pos.y-CENTER.y)**2+(bot_pos.x-CENTER.x)**2)
    radius2 = math.sqrt((TARGET.y-CENTER.y)**2+(TARGET.x-CENTER.x)**2)
    dist_radial = radius2-radius1
    values_rad = np.array([[dist_radial],[-v_norm_prev]])
    vel_radial = consts_rad.dot(values_rad)
    #print("vel radial", vel_radial)
    v_tan_prev = vel_tangential
    v_norm_prev = vel_radial
    vx = vel_radial*math.cos(theta1)-vel_tangential*math.sin(theta1)
    vy = vel_tangential*math.cos(theta1)+vel_radial*math.sin(theta1)
    if math.sqrt(vx**2+vy**2)>config.MAX_BOT_SPEED:
        alpha = math.atan2(vy, vx)
        vx = config.MAX_BOT_SPEED*math.cos(alpha)
        vy = config.MAX_BOT_SPEED*math.sin(alpha)
    #print("vx,vy", vx, vy)
    print("bot_pos", bot_pos)
    return vx,vy,vw

def execute(startTime,DIST_THRESH,ROTATION_FACTOR,avoid_ball=False):
    global getState,TARGET, start_time,FIRST_CALL,FLAG_turn,FLAG_move,kub,prev_state,vx_end,vy_end

    while not (FLAG_move and FLAG_turn):


        try:
            kub.state = getState(prev_state).stateB
        except rospy.ServiceException, e:
            print("Error ", e)



        if not(prev_state == kub.state):
            prev_state = kub.state

            t = rospy.Time.now()
            t = t.secs + 1.0*t.nsecs/pow(10,9)

            vx, vy, vw = velocity_planner(kub.get_pos())

            #if not vw:
            #    vw = 0

            if not vx and not vy:
                vx,vy = vx_end,vy_end
            else:
                vx_end,vy_end = vx,vy

            if functions.dist(kub.state.homePos[kub.kubs_id],TARGET)<DIST_THRESH :
                kub.move(0,0)
                FLAG_move = True
            else:
                kub.move(vx, vy)

            if abs(functions.normalize_angle(kub.state.homePos[kub.kubs_id].theta-rotate))<ROTATION_FACTOR:
                kub.turn(0)
                # print("Angle completed")
                FLAG_turn = True
            else:
                kub.turn(vw)

            kub.execute()
            yield kub,TARGET



    kub.execute()

    yield kub,TARGET

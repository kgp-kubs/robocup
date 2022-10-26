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
from utils.math_functions import *
import krssg_ssl_msgs.srv as srv
import math
import numpy as np

DIST_THRESH = 75
dboxp = Vector2D((abs(OUR_DBOX_X)-(DBOX_HEIGHT/2)),0)
DRIBBLE_DIST = BOT_RADIUS*10
BOT_BALL_THRESH = 0.9*BOT_RADIUS
V_CLOSE=MAX_BOT_SPEED/5
D_CLOSE=BOT_RADIUS*6

kub = None
start_time = None
RADIUS = None
CENTER = None
TARGET = None
FLAG_move = False
VELDIR = None
vx_end = 0
vy_end = 0


rospy.wait_for_service("bsServer",)
getState = rospy.ServiceProxy('bsServer',srv.bsServer)

prev_state = None
try:
    prev_state = getState(prev_state).stateB
except rospy.ServiceException, e:
    print("Error ", e)

def init(_kub,target,veldir, center=None,radius=None):
    global kub,RADIUS,CENTER,FIRST_CALL,FLLAG_move,TARGET, VELDIR
    kub = _kub
    TARGET = Vector2D()
    TARGET.x = target.x
    TARGET.y = target.y
    if center is not None:
        CENTER = Vector2D()
        CENTER.x = center.x
        CENTER.y = center.y
    if radius is not None:
        RADIUS = radius
    VELDIR = veldir
    FLAG_move = False

def execute_inside_circle():
    bot_pos = kub.get_pos()
    normal_dir = math.atan2(bot_pos.y - Center.y),(bot_pos.x - Center.x)
    #if kub.state.ballVel.x == 0 and kub.state.ballVel.y == 0:
    #    VELDIR = math.atan2(kub.state.ballPos.y-bot_pos.y,kub.state.ballPos.x-bot_pos.x)    
    point=getPointBehindTheBall(kub.state.ballPos,VELDIR)
    #theta = math.atan2(point.y-bot_pos.y, point.x-bot_pos.x)
    magnitude=V_CLOSE + dist(point,bot_pos)(MAX_BOT_SPEED/2-V_CLOSE)/(D_CLOSE)
    l = dist(Center,bot_pos)
    if Center == circle1.center:
        relvel_dir = normal_dir + (math.pi*l)/(2*circle.radius)
    if Center == circle2.center:
        relvel_dir = normal_dir - (math.pi*l)/(2*circle.radius)
    relvel = Vector2D(magnitude*math.cos(relvel_dir),magnitude*math.sin(relvel_dir))
    FinalVel=relvel+kub.state.ballVel
    return FinalVel.x, FinalVel.y

def execute_outside_circle():
    #if (kub.state.ballVel.x**2+kub.state.ballVel.y**2) <= 0.1:
    #    VELDIR = math.atan2(kub.state.ballPos.y-bot_pos.y,kub.state.ballPos.x-bot_pos.x)    
    point=getPointBehindTheBall(kub.state.ballPos,VELDIR)
    magnitude=V_CLOSE + dist(point,bot_pos)*(MAX_BOT_SPEED/2-V_CLOSE)/(D_CLOSE)
    bot_pos = kub.get_pos()
    l = dist(CENTER,bot_pos)
    if l<RADIUS:
        return None
    beta = math.asin(RADIUS/l)
    theta = math.atan(bot_pos.y - CENTER.y)/(bot_pos.x - CENTER.x)
    angle1 = theta + beta
    angle2 = theta - beta
    multiplier1 = 1
    multiplier2 = 1
    kub_pos = Vector2D(bot_pos.x,bot_pos.y)
    if dist(kub_pos + Vector2D(BOT_RADIUS*math.cos(angle1),BOT_RADIUS*math.sin(angle1)),CENTER) > dist(kub_pos - Vector2D(BOT_RADIUS*math.cos(angle1),BOT_RADIUS*math.sin(angle1)),CENTER):
        multiplier1 = -1
    if dist(kub_pos + Vector2D(BOT_RADIUS*math.cos(angle2),BOT_RADIUS*math.sin(angle2)),CENTER) > dist(kub_pos - Vector2D(BOT_RADIUS*math.cos(angle2),BOT_RADIUS*math.sin(angle2)),CENTER):
        multiplier2 = -1
    if (math.cos(VELDIR)*math.cos(angle1) + math.sin(VELDIR)*math.sin(angle1))*multiplier1 < (math.cos(VELDIR)*math.cos(angle2) + math.sin(VELDIR)*math.sin(angle2))*multiplier2:
        cosangle = math.cos(angle1)*multiplier1
        sinangle = math.sin(angle1)*multiplier1
    else:
        cosangle = math.cos(angle2)*multiplier2
        sinangle = math.sin(angle2)*multiplier2
    relvel=Vector2D(magnitude*cosangle,magnitude*sinangle)
    FinalVel=relvel+kub.state.ballVel
    print("FinalVel=",FinalVel.x,FinalVel.y)
    print("ballVelocity=",kub.state.ballVel.x,kub.state.ballVel.y)
    print("Relative Velocity=",relvel.x,relvel.y)
    vx=FinalVel.x
    vy=FinalVel.y
    return vx,vy

def execute_inside_alpha():
    kub_pos = bot_pos
    l = dist(CENTER,kub_pos)
    if l>RADIUS:
        return None
    #if kub.state.ballVel.x == 0 and kub.state.ballVel.y == 0:
    #    VELDIR = math.atan2(kub.state.ballPos.y-bot_pos.y,kub.state.ballPos.x-bot_pos.x)    
    point=getPointBehindTheBall(kub.state.ballPos,VELDIR)
    magnitude=V_CLOSE + dist(point,bot_pos)*(MAX_BOT_SPEED/2-V_CLOSE)/(D_CLOSE)
    theta = math.atan2(point.y-kub_pos.y, point.x-kub_pos.x)
    vx = magnitude*math.cos(theta)
    vy = magnitude*math.sin(theta)
    return vx,vy
    
def execute(strategy, startTime,DIST_THRESH,avoid_ball=False):
    global getState,TARGET, start_time,FIRST_CALL,FLAG_turn,FLAG_move,kub,prev_state,vx_end,vy_end


    while not (FLAG_move):


        try:
            kub.state = getState(prev_state).stateB
        except rospy.ServiceException, e:
            print("Error ", e)



        if not(prev_state == kub.state):
            prev_state = kub.state

            t = rospy.Time.now()
            t = t.secs + 1.0*t.nsecs/pow(10,9)

            func_name = 'execute_' + strategy+ '()'


            out = eval(func_name)
        if out is None:
            FLAG_move=True
                vx, vy = vx_end,vy_end
        else:
            vx, vy = out
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

            kub.execute()
            yield kub,TARGET



    kub.execute()

    yield kub,TARGET

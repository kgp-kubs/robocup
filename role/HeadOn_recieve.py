from enum import Enum
import behavior
import _GoToPoint_ , _turnAround_ , KickToPoint
import rospy
from utils.functions import *
from utils.geometry import *
from utils.config import *
from math import *
from velocity.run import *
import time
from krssg_ssl_msgs.srv import *

rospy.wait_for_service('bsServer',)
getState = rospy.ServiceProxy('bsServer',bsServer)
time_thresh=0

prev_state = None
try:
    prev_state = getState(prev_state).stateB
except rospy.ServiceException, e:
    print("Error ", e)

class HeadonRecieve(behavior.Behavior):

    class State(Enum):
        setup = 1
        HOR = 2

    def __init__(self,id_1,course_approch_thresh =  DISTANCE_THRESH/3):
        super(HeadonRecieve,self).__init__()
        self.name = "defender"+str(id_1)
        self.defender_id=id_1	#The bot running the program
        self.intercepting_point = Vector2D()
        self.behavior_failed = False
        self.course_approch_thresh=course_approch_thresh
        self.entered_setup = False

        for state in HeadonRecieve.State:
            self.add_state(state,behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,HeadonRecieve.State.setup,
            lambda:True,"immediately")

        self.add_transition(HeadonRecieve.State.setup,HeadonRecieve.State.HOR,lambda:self.entered_setup,"only one state")
        

    def add_kub(self,kub):
        self.kub = kub

    def add_theta(self,theta):
        self.theta = theta

    def find_intercept_point(self):
        global prev_state
        
        time.sleep(0.05)
        while(magnitute(self.kub.state.ballVel)<10):
            try:
                self.kub.state = getState(prev_state).stateB
            except rospy.ServiceException, e:
                print("Error ", e)
            # print(kub.state)
            if not(prev_state == self.kub.state):
                prev_state = self.kub.state

        ball_vel_line=Line(point1=Vector2D(self.kub.state.ballPos),angle=math.atan2(self.kub.state.ballVel.y,self.kub.state.ballVel.x))
        ball_to_bot_line = Line(point2=Vector2D(self.kub.state.homePos[self.defender_id].x,self.kub.state.homePos[self.defender_id].y),point1=Vector2D(self.kub.state.ballPos.x,self.kub.state.ballPos.y))
        angle_at_vextex = ball_to_bot_line.angle_with_line(ball_vel_line)
        ball_velocity=magnitute(self.kub.state.ballVel)
        bot_velocity=1*MAX_BOT_SPEED
        print(self.kub.state.ballVel)
        print("Ball velocity",ball_velocity)
        print("Bot velocity",bot_velocity)
        k=ball_velocity/bot_velocity
        print("K=",k," angle at vertex=",angle_at_vextex)
        alpha=math.asin(k*math.sin(angle_at_vextex))
        print("Alpha: ",alpha)
        angle_of_bot_motion=ball_to_bot_line.angle-alpha
        bot_vel_line=Line(point1=Vector2D(self.kub.state.homePos[self.defender_id].x,self.kub.state.homePos[self.defender_id].y),angle=angle_of_bot_motion)
        intercepting_point=ball_vel_line.intersection_with_line(bot_vel_line)
        print("Intercepting point: ",intercepting_point)
        return intercepting_point

    def on_enter_setup(self):
        self.behavior_failed=False
        pass

    def execute_setup(self):
        self.entered_setup = True
        pass

    def on_exit_setup(self):
        pass

    def on_enter_HOR(self):
        pass

    def execute_HOR(self):
        self.intercepting_point=self.find_intercept_point()
        _GoToPoint_.init(self.kub,self.intercepting_point,0) 
        start_time = rospy.Time.now()
        start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
        generatingfunction = _GoToPoint_.execute(start_time,self.course_approch_thresh)

        prev_point=Vector2D(self.kub.state.ballPos)
        next_point = prev_point
        movement=self.kub.get_pos()
        point=Vector2D()
        for gf in generatingfunction:
            next_point = Vector2D(self.kub.state.ballPos)
            self.kub,point = gf
            if dist(next_point,prev_point) > 3*BOT_RADIUS:
                print("breaking")
                break
            if not vicinity_points(point,self.intercepting_point,thresh=BOT_RADIUS*2.0):
                print("breaking")
                break
            prev_point = next_point

    def on_exit_HOR(self):
        pass

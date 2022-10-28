from ast import In
from doctest import FAIL_FAST
from enum import Enum
import behavior
import _GoToPoint_
import _GoOnArc_
import rospy
from utils.functions import *
from utils.geometry import *
from utils.config import *
from math import *

class Intercept(behavior.Behavior):

    class State(Enum):

        setup = 1
        in_2_alpha = 2
        outside_circles = 3
        inside_circles = 4
        move_on_circle = 5


    def __init__(self,max_vel = MAX_BOT_SPEED/1.2,min_vel = MIN_BOT_SPEED*1.2):

        self.name = "Intercept"

        self.power = 7.0

        self.alpha = math.pi/8

        self.d = BOT_BALL_THRESH*1.5

        self.radius = self.d/2*sin(self.alpha)

        self.theta = None
        
        self.circle1 = None

        self.circle2 = None

        self.to_move_circle = None

        self.max_vel = max_vel

        self.vel_close = max_vel/2

        self.min_vel = min_vel

        self.ball_thresh = BOT_BALL_THRESH*1.5

        self.DISTANCE_THRESH = DISTANCE_THRESH

        self.ROTATION_FACTOR = ROTATION_FACTOR

        self.behavior_failed = False

        self.add_state(Intercept.State.setup,behavior.Behavior.State.running)

        self.add_state(Intercept.State.in_2_alpha,behavior.Behavior.State.running)

        self.add_state(Intercept.State.outside_circles,behavior.Behavior.State.running)

        self.add_state(Intercept.State.inside_circles,behavior.Behavior.State.running)

        self.add_state(Intercept.State.move_on_circle, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,Intercept.State.setup,lambda: True,'immediately')

        self.add_transition(Intercept.State.setup,Intercept.State.in_2_alpha, lambda:self.bot_inside_2alpha() ,'In 2 alpha')

        self.add_transition(Intercept.State.setup,Intercept.State.outside_circles, lambda:self.bot_outside_circles() ,'Outside circles')


        self.add_transition(Intercept.State.setup,Intercept.State.inside_circles, lambda:not self.bot_inside_2alpha() and not self.bot_outside_circles() ,'Inside circles')



    def bot_inside_2alpha(self):
        if self.theta > math.pi - self.alpha:
            return True
        return False

    def bot_outside_circles(self):
        kub_pos = Vector2D(self.kub.state.homePos[self.kub.kubs_id].x, self.kub.state.homePos[self.kub.kubs_id].y)

        if kub_pos.dist(self.center1)>self.radius and kub_pos.dist(self.center2) > self.radius:
            return True

        return False
        

    def calcTheta(self):          
        kub_pos = Vector2D(self.kub.state.homePos[self.kub.kubs_id].x, self.kub.state.homePos[self.kub.kubs_id].y)
        ball_vel = Vector2D(self.kub.state.ballVel.x, self.kub.state.ballVel.y)
        vec_ball_bot = kub_pos - self.target
        dot_temp = vec_ball_bot.dot(ball_vel)
        final = dot_temp / (ball_vel.abs(ball_vel) * vec_ball_bot.abs(vec_ball_bot))
        self.theta = acos(final)

    def calcCenter(self):
        ball_vel = Vector2D(self.kub.state.ballVel.x, self.kub.state.ballVel.y)
        ball_vel_dir = ball_vel/ball_vel.abs()
        cos_beta = sin(self.alpha)
        sin_beta = cos(self.alpha)
        self.center1 = Vector2D()
        self.center1.x = (cos_beta*ball_vel_dir.x - sin_beta*ball_vel_dir.y)*self.radius
        self.center1.y = (sin_beta*ball_vel_dir.x + cos_beta*ball_vel_dir.y)*self.radius

        self.circle1 = Circle(self.center1,self.radius)

        cos_beta = sin(self.alpha)
        sin_beta = -cos(self.alpha)
        self.center2 = Vector2D()
        self.center2.x = (cos_beta*ball_vel_dir.x - sin_beta*ball_vel_dir.y)*self.radius
        self.center2.y = (sin_beta*ball_vel_dir.x + cos_beta*ball_vel_dir.y)*self.radius

        self.circle2 = Circle(self.center1,self.radius)


    def add_kub(self,kub):
        self.kub = kub

    def add_target(self):
        ball_pos = Vector2D(self.kub.state.ballPos.x,self.kub.state.ballPos.y)
        ball_vel = Vector2D(self.kub.state.ballVel.x, self.kub.state.ballVel.y)
        self.target = ball_pos - self.d*ball_vel/ball_vel.abs()


    def on_enter_setup(self):
        pass

    def execute_setup(self):
        self.add_target()
        self.calcTheta()
        self.calcCenter()

    def on_exit_setup(self):
        pass

    def on_enter_inside_2alpha(self):
        pass

    def execute_inside_2alpha(self):
        pass    

    def on_exit_inside_2alpha(self):
        pass


    def on_enter_outside_circles(self):
        ball_vel = Vector2D(self.kub.state.ballVel.x, self.kub.state.ballVel.y)
        angle = ball_vel.tan_inverse()
        ball_pos = Vector2D(self.kub.state.ballPos.x,self.kub.state.ballPos.y)
        ball_dir_line = Line(ball_pos,angle)
        kub_pos = Vector2D(self.kub.state.homePos[self.kub.kubs_id].x, self.kub.state.homePos[self.kub.kubs_id].y)
        
        if ball_dir_line.check_on_same_side(kub_pos,self.center1):
            self.to_move_circle = self.circle1
        else:
            self.to_move_circle = self.circle2

        bot_center_vec = kub_pos - self.to_move_circle.center

        if ball_vel.dot(bot_center_vec) > 0:
            self.take_bigger_arc = True
        else:
            self.take_bigger_arc = False

        
        _GoOnArc_.init(self.kub,self.target,self.to_move_circle.center,self.radius,False,None,self.take_bigger_arc)


    def execute_outside_circles(self):
        # print("Execute drive")
        # start_time = rospy.Time.now()
        # start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)
        # generatingfunction = _GoOnArc_.execute(start_time,self.DISTANCE_THRESH, self.ROTATION_FACTOR)
        # print("Datatype of gf:",type(generatingfunction))
        # for gf in generatingfunction:
        #     self.kub,target = gf

        #     if not vicinity_points(self.target,target):
        #         self.behavior_failed = True
        #         break
        # self.new_point = self.kub.get_pos()

        pass

    def on_exit_outside_circles(self):
        pass

    def on_enter_inside_circles(self):
        pass

    def execute_inside_circles(self):
        pass

    def on_exit_inside_circles(self):
        pass

        



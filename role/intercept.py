from distutils.log import ERROR
from enum import Enum
from locale import ERA
from velocity.run_w import *
import behavior
import _GoToPoint_
import _GoOnArc_intercept

import rospy
from utils.functions import *
from utils.geometry import *
from utils.config import *
from math import *
from krssg_ssl_msgs.srv import *
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from krssg_ssl_msgs.msg import gr_Robot_Command
from utils import config, functions
import rospy,sys


rospy.wait_for_service('bsServer',)
getState = rospy.ServiceProxy('bsServer',bsServer)

prev_state = None
try:
    prev_state = getState(prev_state).stateB
except rospy.ServiceException, e:
    print("Error ", e)

def mul(v,scale):
        v.x = v.x*scale
        v.y = v.y*scale
        return v

def div(v,scale):
        if scale != 0:
            v.x = v.x/scale
            v.y = v.y/scale
        else:
            v.x = 0
            v.y = 0
        return v

class Intercept(behavior.Behavior):

    class State(Enum):

        setup = 1
        in_2_alpha = 2
        outside_circles = 3
        inside_circles = 4
        move_on_circle = 5


    def __init__(self,max_vel = MAX_BOT_SPEED/1.2,min_vel = MIN_BOT_SPEED*1.2):

        super(Intercept,self).__init__()

        self.name = "Intercept"

        self.power = 7.0

        self.alpha = math.pi/8

        self.d = BOT_BALL_THRESH*2


        self.radius = self.d/2*sin(self.alpha)

        self.theta = None
        
        self.circle1 = None

        self.circle2 = None

        self.to_move_circle = None

        self.max_vel = max_vel

        self.vel_close = max_vel/2

        self.min_vel = min_vel

        self.ball_thresh = BOT_BALL_THRESH*2

        self.DISTANCE_THRESH = DISTANCE_THRESH

        self.ROTATION_FACTOR = ROTATION_FACTOR

        self.center_arc = None

        self.behavior_failed = False

        self.add_state(Intercept.State.setup,behavior.Behavior.State.running)

        self.add_state(Intercept.State.in_2_alpha,behavior.Behavior.State.running)

        self.add_state(Intercept.State.outside_circles,behavior.Behavior.State.running)

        self.add_state(Intercept.State.inside_circles,behavior.Behavior.State.running)

        self.add_state(Intercept.State.move_on_circle, behavior.Behavior.State.running)



        self.add_transition(behavior.Behavior.State.start,Intercept.State.setup,lambda: True,'immediately')

        self.add_transition(Intercept.State.setup,Intercept.State.in_2_alpha, lambda:self.bot_inside_2alpha() ,'In 2 alpha')

        self.add_transition(Intercept.State.setup,Intercept.State.outside_circles, lambda:self.bot_outside_circles() and not self.bot_inside_2alpha(),'Outside circles')

        self.add_transition(Intercept.State.setup,Intercept.State.move_on_circle, lambda:self.bot_on_circle() ,'On circle')

        self.add_transition(Intercept.State.outside_circles,Intercept.State.move_on_circle, lambda:self.bot_on_circle() ,'On circle')

        self.add_transition(Intercept.State.setup,Intercept.State.inside_circles, lambda: not self.bot_inside_2alpha() and not self.bot_outside_circles() ,'Inside circles')

        self.add_transition(Intercept.State.outside_circles,Intercept.State.setup,lambda:self.behavior_failed, 'Outside Circle failed')

        self.add_transition(Intercept.State.inside_circles,Intercept.State.setup,lambda:self.behavior_failed, 'Inside Circle failed')

        self.add_transition(Intercept.State.in_2_alpha,Intercept.State.setup,lambda:self.behavior_failed, 'In 2 alpha failed')

        self.add_transition(Intercept.State.move_on_circle,Intercept.State.setup,lambda:self.behavior_failed, 'Move on Circle failed')





    def bot_inside_2alpha(self):
        if self.theta < self.alpha:
            return True
        return False

    def bot_outside_circles(self):
        kub_pos = Vector2D(self.kub.state.homePos[self.kub.kubs_id].x, self.kub.state.homePos[self.kub.kubs_id].y)

        if kub_pos.dist(self.center1)>self.radius and kub_pos.dist(self.center2) > self.radius:
            return True

        return False


    def bot_on_circle(self):
        ERROR_RADIUS = 25
        kub_pos = Vector2D(self.kub.state.homePos[self.kub.kubs_id].x, self.kub.state.homePos[self.kub.kubs_id].y)

        if abs(kub_pos.dist(self.center1)-self.radius)<=ERROR_RADIUS or abs(kub_pos.dist(self.center2) - self.radius)<=ERROR_RADIUS:
            return True

        return False



    def calcTheta(self):          
        kub_pos = Vector2D(self.kub.state.homePos[self.kub.kubs_id].x, self.kub.state.homePos[self.kub.kubs_id].y)
        ball_vel = Vector2D(self.kub.state.ballVel.x, self.kub.state.ballVel.y)
        # ball_vel=Vector2D(1,-1)
        vec_ball_bot = kub_pos - self.target
        print("===========vec_trg_bot===============",vec_ball_bot.x,vec_ball_bot.y)
        dot_temp = vec_ball_bot.dot(ball_vel)
        final = dot_temp / (ball_vel.abs(ball_vel) * vec_ball_bot.abs(vec_ball_bot))
        self.theta = math.acos(final)
        print("=============theta=========",self.theta)

    def calcCenter(self):
        ball_vel = Vector2D(self.kub.state.ballVel.x, self.kub.state.ballVel.y)
        # ball_vel = Vector2D(1,-1)
        ball_vel_dir = -1*ball_vel*(1/ball_vel.abs(ball_vel))
        cos_beta = sin(self.alpha)
        sin_beta = cos(self.alpha)
        self.center1 = Vector2D()
        self.center1.x = self.target.x + (cos_beta*ball_vel_dir.x - sin_beta*ball_vel_dir.y)*self.radius
        self.center1.y = self.target.y + (sin_beta*ball_vel_dir.x + cos_beta*ball_vel_dir.y)*self.radius

        self.circle1 = Circle(self.center1,self.radius)

        cos_beta = sin(self.alpha)
        sin_beta = -cos(self.alpha)
        self.center2 = Vector2D()
        self.center2.x = self.target.x + (cos_beta*ball_vel_dir.x - sin_beta*ball_vel_dir.y)*self.radius
        self.center2.y = self.target.y + (sin_beta*ball_vel_dir.x + cos_beta*ball_vel_dir.y)*self.radius

        self.circle2 = Circle(self.center2,self.radius)


    def add_kub(self,kub):
        self.kub = kub

    def add_target(self):
        ball_pos = Vector2D(self.kub.state.ballPos.x,self.kub.state.ballPos.y)
        ball_vel = Vector2D(self.kub.state.ballVel.x,self.kub.state.ballVel.y)
        # ball_vel = Vector2D(1,-1)

        if ball_vel.abs(ball_vel) != 0:
            self.target = ball_pos + self.d*(ball_vel)*(1/ball_vel.abs(ball_vel))
        else:
            self.target = ball_pos 

    def decide_tangent(self):
        kub_pos = Vector2D(self.kub.state.homePos[self.kub.kubs_id].x, self.kub.state.homePos[self.kub.kubs_id].y)
        ball_vel = Vector2D(self.kub.state.ballVel.x, self.kub.state.ballVel.y)
        # ball_vel = Vector2D(1,-1)
        bot_target_vec = self.target - kub_pos
        dist_bot_center = kub_pos.dist(self.to_move_circle.center)
        theta = math.acos(self.radius/dist_bot_center)


        vec_center_bot = kub_pos - self.to_move_circle.center

        vec_center_bot = vec_center_bot*(1/vec_center_bot.abs(vec_center_bot))

        if ball_vel.cross_pdt(bot_target_vec) > 0:
            theta = -theta

        self.vec_to_point = Vector2D()
        self.vec_to_point.x = math.cos(theta)*vec_center_bot.x - math.sin(theta)*vec_center_bot.y
        self.vec_to_point.y = math.sin(theta)*vec_center_bot.x + math.cos(theta)*vec_center_bot.y


        self.point_on_circle = self.to_move_circle.center + self.radius*self.vec_to_point

        tangent = self.point_on_circle - kub_pos
        self.tangent = tangent*(1/ tangent.abs(tangent))

    def get_point_on_arc(self):
        kub_pos = Vector2D(self.kub.state.homePos[self.kub.kubs_id].x, self.kub.state.homePos[self.kub.kubs_id].y)
        bot_target_vec = self.target - kub_pos
        ball_vel = Vector2D(self.kub.state.ballVel.x, self.kub.state.ballVel.y)
        
        theta1 = math.atan2(kub_pos.y-self.to_move_circle.center.y,kub_pos.x-self.to_move_circle.center.x)
        theta2 = math.atan2(self.target.y-self.to_move_circle.center.y,self.target.x-self.to_move_circle.center.x)
        theta = theta1 - theta2
        theta = functions.normalize_angle(theta)
        
        if ball_vel.cross_pdt(bot_target_vec) > 0:
            if theta < 0:
                theta = 2*math.pi+theta
        else:
            if theta > 0:
                theta = theta -2*math.pi
    
        point = Vector2D()
        point.x = self.to_move_circle.center.x + self.radius*math.cos(theta/2)
        point.y = self.to_move_circle.center.y + self.radius*math.sin(theta/2)
        self.point_on_arc = point

    def add_point_on_arc(self):
        if self.center_arc is not None:
            print("center already given")
            return
        if self.target is None:
            print("target not given")
            return
        bot_pos = self.kub.get_pos()
        a1,b1,c1 = 2*(self.point_on_arc.x-bot_pos.x), 2*(self.point_on_arc.y-bot_pos.y), bot_pos.x**2+bot_pos.y**2-self.point_on_arc.x**2-self.point_on_arc.y**2
        a2,b2,c2 = 2*(self.target.x-bot_pos.x), 2*(self.target.y-bot_pos.y), bot_pos.x**2+bot_pos.y**2-self.target.x**2-self.target.y**2
        x = (b1*c2-b2*c1)/(a1*b2-a2*b1)
        y = (c1*a2-c2*a1)/(a1*b2-a2*b1)
        self.center_arc = Vector2D()
        self.center_arc.x = x
        self.center_arc.y = y
        self.radius_arc = dist(self.center_arc, self.point_on_arc)

    def update_all(self):
        self.add_target()
        self.calcCenter()
        if self.decided_circle == 'circle1':
            self.to_move_circle = self.circle1
        else:
            self.to_move_circle = self.circle2
        self.decide_tangent()

        # self.add_point_on_arc()

    def on_enter_setup(self):
        self.behavior_failed = False

    def execute_setup(self):
        self.add_target()
        self.calcTheta()
        self.calcCenter()

    def on_exit_setup(self):
        pass

    def on_enter_in_2_alpha(self):
        pass

    def execute_in_2_alpha(self):
        while True:
            
            k = 2
            kub_pos = Vector2D(self.kub.state.homePos[self.kub.kubs_id].x, self.kub.state.homePos[self.kub.kubs_id].y)
            direct_vel = (self.target-kub_pos) * MAX_BOT_SPEED * (1 / (self.target-kub_pos).abs(self.target-kub_pos))
            bot_target_vec = self.target - kub_pos
            bot_target_dir = bot_target_vec*(1/bot_target_vec.abs(bot_target_vec))
            ball_vel = Vector2D(self.kub.state.ballVel.x, self.kub.state.ballVel.y)
            
            vel_max_given = direct_vel - ball_vel
            vel = min(vel_max_given, k*kub_pos.dist(self.target))
            vel = vel*bot_target_dir + ball_vel

            # rotate = -1*ball_vel.angle()
            # vw = Get_Omega(self.kub.kubs_id,rotate,self.kub.state.homePos)

            # if abs(normalize_angle(self.kub.state.homePos[self.kub.kubs_id].theta-rotate))<ROTATION_FACTOR:
            #     self.kub.turn(0)
            #     # print("Angle completed")
            #     # FLAG_turn = True
            # else:
            #     self.kub.turn(vw)
            
            
            # print("Distance ______",dist(kub.state.homePos[kub.kubs_id], GOAL_POINT))
            # yield self.kub,GOAL_POINT
            self.kub.move(vel.x,vel.y)

            self.kub.execute()
            
            if  self.target.dist(kub_pos) < 5:
                break

            try:
                self.kub.state = getState(prev_state).stateB
            except rospy.ServiceException, e:
                print("Error ", e)
            # self.update_all()   
            self.add_target()
            self.calcCenter() 



    def on_exit_in_2_alpha(self):
        pass


    def on_enter_outside_circles(self):
        ball_vel = Vector2D(self.kub.state.ballVel.x, self.kub.state.ballVel.y)
        # ball_vel = Vector2D(1,-1)
        kub_pos = Vector2D(self.kub.state.homePos[self.kub.kubs_id].x, self.kub.state.homePos[self.kub.kubs_id].y)
        
        if kub_pos.dist(self.center1) < kub_pos.dist(self.center2):
            self.to_move_circle = self.circle1
            self.decided_circle = 'circle1'
        else:
            self.to_move_circle = self.circle2
            self.decided_circle = 'circle2'

        bot_center_vec = kub_pos - self.to_move_circle.center

        if ball_vel.dot(bot_center_vec) > 0:
            self.take_bigger_arc = True
        else:
            self.take_bigger_arc = False

        self.decide_tangent()




    def execute_outside_circles(self):
        
        while True:
            k = 0.3
            k1 = 1.5
            kub_pos = Vector2D(self.kub.state.homePos[self.kub.kubs_id].x, self.kub.state.homePos[self.kub.kubs_id].y)
            direct_vel = self.tangent*MAX_BOT_SPEED
            ball_vel = Vector2D(self.kub.state.ballVel.x, self.kub.state.ballVel.y)
            # ball_vel = Vector2D(1,-1)
            vel_max_given = direct_vel - ball_vel
            vel = min(vel_max_given.abs(vel_max_given), k*kub_pos.dist(self.point_on_circle))

            rotate = -1*ball_vel.angle()
            vw = Get_Omega(self.kub.kubs_id,rotate,self.kub.state.homePos)

            # if abs(normalize_angle(self.kub.state.homePos[self.kub.kubs_id].theta-rotate))<ROTATION_FACTOR:
            #     self.kub.turn(0)
            # else:
            #     self.kub.turn(vw)
            
            vel = vel*self.tangent + ball_vel*k1
            self.kub.move(vel.x,vel.y)

            self.kub.execute()
            
            if  self.point_on_circle.dist(kub_pos) < 100:
                break

            try:
                self.kub.state = getState(prev_state).stateB
            except rospy.ServiceException, e:
                print("Error ", e)
            self.update_all()

            

    def on_exit_outside_circles(self):
        pass



    def on_enter_move_on_circle(self):
        kub_pos = Vector2D(self.kub.state.homePos[self.kub.kubs_id].x, self.kub.state.homePos[self.kub.kubs_id].y)
        if abs(kub_pos.dist(self.center1) - self.radius) < 25 :
            self.to_move_circle = self.circle1
            self.decided_circle = 'circle1'

        else:
            self.to_move_circle = self.circle2
            self.decided_circle = 'circle2'                        

        self.get_point_on_arc()
        self.add_point_on_arc()
        _GoOnArc_intercept.init(self.kub,self.target,self.center_arc,self.radius_arc,False,self.point_on_arc,False)        

    def execute_move_on_circle(self):
        start_time = rospy.Time.now()
        start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)
        generatingfunction = _GoOnArc_intercept.execute(start_time,self.DISTANCE_THRESH, self.ROTATION_FACTOR)
        print("Datatype of gf:",type(generatingfunction))
        for gf in generatingfunction:
            self.kub,target = gf		# if FIRST_CALL == True:
		# 	function2(1,state.stateB)
		# 	FIRST_CALL = False
            self.update_all()
            _GoOnArc_intercept.init(self.kub,self.target,self.center_arc,self.radius_arc,False,self.point_on_arc,False)

            if not vicinity_points(self.target,target):
                self.behavior_failed = True
                break

    def on_exit_move_on_circle(self):
        pass

    def on_enter_inside_circles(self):
        ball_vel = Vector2D(self.kub.state.ballVel.x, self.kub.state.ballVel.y)
        kub_pos = Vector2D(self.kub.state.homePos[self.kub.kubs_id].x, self.kub.state.homePos[self.kub.kubs_id].y)
        
        if kub_pos.dist(self.center1) < kub_pos.dist(self.center2):
            self.to_move_circle = self.circle1
            self.decided_circle = 'circle1'
        else:
            self.to_move_circle = self.circle2
            self.decided_circle = 'circle2'

            self.get_point_on_arc()
            self.add_point_on_arc()
            _GoOnArc_intercept.init(self.kub,self.target,self.center_arc,self.radius_arc,False,self.point_on_arc,False)

    def execute_inside_circles(self):
        start_time = rospy.Time.now()
        start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)
        generatingfunction = _GoOnArc_intercept.execute(start_time,self.DISTANCE_THRESH, self.ROTATION_FACTOR)
        print("Datatype of gf:",type(generatingfunction))
        for gf in generatingfunction:
            self.kub,target = gf
            self.update_all()
            _GoOnArc_intercept.init(self.kub,self.target,self.center_arc,self.radius_arc,False,self.point_on_arc,False)

            if not vicinity_points(self.target,target):
                self.behavior_failed = True
                break

    def on_exit_inside_circles(self):
        pass

        


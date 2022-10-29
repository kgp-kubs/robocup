from enum import Enum
from typing import final
import behavior
import _GoToPoint_
import _GoOnArc_
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
from velocity.run_w import *
from kubs import kubs, cmd_node
from velocity.run_w import *
import rospy,sys
from krssg_ssl_msgs.msg import point_2d
from utils.geometry import Vector2D
from utils.config import *
from krssg_ssl_msgs.srv import *
from utils.functions import *
import math


rospy.wait_for_service('bsServer',)
getState = rospy.ServiceProxy('bsServer',bsServer)

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

        self.add_transition(Intercept.State.outside_circles,Intercept.State.move_on_circle, lambda:self.bot_on_circle() ,'On circle')

        self.add_transition(Intercept.State.setup,Intercept.State.inside_circles, lambda: not self.bot_inside_2alpha() and not self.bot_outside_circles() ,'Inside circles')



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

        if kub_pos.dist(self.center1)-self.radius<=ERROR_RADIUS or kub_pos.dist(self.center2) - self.radius<=ERROR_RADIUS:
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
        ball_vel_dir = -ball_vel/ball_vel.abs()
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
        self.target = ball_pos + self.d*ball_vel/ball_vel.abs()

    def decide_tangent(self):
        kub_pos = Vector2D(self.kub.state.homePos[self.kub.kubs_id].x, self.kub.state.homePos[self.kub.kubs_id].y)
        ball_vel = Vector2D(self.kub.state.ballVel.x, self.kub.state.ballVel.y)
        bot_target_vec = self.target - kub_pos
        dist_bot_center = kub_pos.dist(self.to_move_circle.center)
        theta = acos(self.radius/dist_bot_center)

        vec_center_bot = kub_pos - self.to_move_circle.center

        vec_center_bot = vec_center_bot/vec_center_bot.abs()

        if ball_vel.cross_pdt(bot_target_vec) > 0:
            theta = -theta

        self.vec_to_point = Vector2D()
        self.vec_to_point.x = cos(theta)*vec_center_bot.x - sin(theta)*vec_center_bot.y
        self.vec_to_point.y = sin(theta)*vec_center_bot.x + cos(theta)*vec_center_bot.y


        self.point_on_circle = self.to_move_circle.center + self.radius*self.vec_to_point

        tangent = self.point_on_circle - kub_pos
        self.tangent = tangent / tangent.abs()

    def update_all(self):
        self.add_target()
        self.calcCenter()
        if self.decided_circle == 'circle1':
            self.to_move_circle = self.circle1
        else:
            self.to_move_circle = self.circle2
        self.point_on_circle = self.to_move_circle.center + self.radius*self.vec_to_point


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
        while True:
            
            # t = rospy.Time.now()
            # t = t.secs + 1.0*t.nsecs/pow(10,9)
            # start_time = None
            # [vx, vy, vw, REPLANNED] = velocity.run.Get_Vel(start_time, t, kub.kubs_id, GOAL_POINT, kub.state.homePos, kub.state.awayPos, avoid_ball) 
            

            k = 0.75
            kub_pos = Vector2D(self.kub.state.homePos[self.kub.kubs_id].x, self.kub.state.homePos[self.kub.kubs_id].y)
            direct_vel = (kub_pos - self.target) * MAX_BOT_SPEED / (kub_pos - self.target).abs()
            ball_vel = Vector2D(self.kub.state.ballVel.x, self.kub.state.ballVel.y)
            vel_max_given = direct_vel - ball_vel
            vel = min(vel_max_given, k*self.to_move_circle.center.dist(self.target))
            rotate = self.theta
            vel = vel*direct_vel + ball_vel
            rotate = 
            vw = Get_Omega(self.kub.kubs_id,rotate,self.kub.state.homePos)

            if abs(normalize_angle(self.kub.state.homePos[self.kub.kubs_id].theta-rotate))<ROTATION_FACTOR:
                self.kub.turn(0)
                # print("Angle completed")
                # FLAG_turn = True
            else:
                self.kub.turn(vw)
            # print("Distance ______",dist(kub.state.homePos[kub.kubs_id], GOAL_POINT))
            if dist(self.kub.state.homePos[self.kub.kubs_id], GOAL_POINT)<DIST_THRESH :
                self.kub.move(0,0)
                # print("Distance completed"*200)
                # FLAG_move = True
            else:
                # print("Sending velocity",vx,vy)
                self.kub.move(vx, vy)

            self.kub.execute()
            # yield self.kub,GOAL_POINT
            self.kub.move(vel.x,vel.y)

            self.kub.execute()
            
            if  self.target.dist(kub_pos) < 10:
                break

            try:
                state = getState(state)
            except rospy.ServiceException, e:
                print("Error ",e)
            self.kub.update_state(state)
            self.update_all()

        pass    

    def on_exit_inside_2alpha(self):
        pass


    def on_enter_outside_circles(self):
        ball_vel = Vector2D(self.kub.state.ballVel.x, self.kub.state.ballVel.y)
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
            k = 0.75
            kub_pos = Vector2D(self.kub.state.homePos[self.kub.kubs_id].x, self.kub.state.homePos[self.kub.kubs_id].y)
            direct_vel = self.tangent*MAX_BOT_SPEED
            ball_vel = Vector2D(self.kub.state.ballVel.x, self.kub.state.ballVel.y)
            vel_max_given = direct_vel - ball_vel
            vel = min(vel_max_given, k*self.to_move_circle.center.dist(self.point_on_circle))
            
            vel = vel*self.tangent + ball_vel
            self.kub.move(vel.x,vel.y)

            self.kub.execute()
            
            if  self.point_on_circle.dist(kub_pos) < 10:
                break

            try:
                state = getState(state)
            except rospy.ServiceException, e:
                print("Error ",e)
            self.kub.update_state(state)
            self.update_all()

            

    def on_exit_outside_circles(self):
        pass

    def on_enter_inside_circles(self):
        pass

    def execute_inside_circles(self):
        pass

    def on_exit_inside_circles(self):
        pass

        


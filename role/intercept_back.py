from enum import Enum
from velocity.run_w import *
import behavior
import _GoToPoint_
import _GoOnArc_
import time

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
import numpy as np
import matplotlib.pyplot as plt


rospy.wait_for_service('bsServer',)
getState = rospy.ServiceProxy('bsServer',bsServer)

prev_state = None
try:
    prev_state = getState(prev_state).stateB
except rospy.ServiceException, e:
    print("Error ", e)


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

        self.radius = self.d/(2*sin(self.alpha))
        
        print(self.radius)

        self.theta = None
        
        self.circle1 = None

        self.circle2 = None

        self.to_move_circle = None

        self.max_vel = max_vel

        self.vel_close = max_vel/2

        self.min_vel = min_vel

        self.ball_thresh = BOT_BALL_THRESH*4

        self.DISTANCE_THRESH = BOT_BALL_THRESH

        self.ROTATION_FACTOR = ROTATION_FACTOR

        self.ball_vel_dir = None

        self.center_arc = None

        self.behavior_failed = False

        self.behavior_completed = False

        self.FIRST_CALL =True

        self.goal = Vector2D(6000,500)

        self.x_tar = []
        self.y_tar = []
        self.x_pt = []
        self.y_pt = []
        self.x_cent1 = []
        self.y_cent1 = []
        self.x_cent2 = []
        self.y_cent2 = []

        self.x_kub = []
        self.y_kub = []

        self.x_ball = []
        self.y_ball = []
        self.i = 0

        self.add_state(Intercept.State.setup,behavior.Behavior.State.running)

        self.add_state(Intercept.State.in_2_alpha,behavior.Behavior.State.running)

        self.add_state(Intercept.State.outside_circles,behavior.Behavior.State.running)

        self.add_state(Intercept.State.inside_circles,behavior.Behavior.State.running)

        self.add_state(Intercept.State.move_on_circle, behavior.Behavior.State.running)



        self.add_transition(behavior.Behavior.State.start,Intercept.State.setup,lambda: True,'immediately')

        self.add_transition(Intercept.State.setup,Intercept.State.in_2_alpha, lambda:self.bot_inside_2alpha() and self.bot_outside_circles() ,'In 2 alpha')

        self.add_transition(Intercept.State.setup,Intercept.State.outside_circles, lambda:self.bot_outside_circles() and not self.bot_inside_2alpha() ,'Outside circles')

        self.add_transition(Intercept.State.setup,Intercept.State.move_on_circle, lambda:self.bot_on_circle() ,'On circle')

        self.add_transition(Intercept.State.outside_circles,Intercept.State.move_on_circle, lambda:self.bot_on_circle() ,'On circle')

        self.add_transition(Intercept.State.setup,Intercept.State.inside_circles, lambda: not self.bot_inside_2alpha() and not self.bot_outside_circles() ,'Inside circles')

        self.add_transition(Intercept.State.outside_circles,Intercept.State.setup,lambda:self.behavior_failed, 'Outside Circle failed')

        self.add_transition(Intercept.State.inside_circles,Intercept.State.setup,lambda:self.behavior_failed, 'Inside Circle failed')

        self.add_transition(Intercept.State.in_2_alpha,Intercept.State.setup,lambda:self.behavior_failed, 'In 2 alpha failed')

        self.add_transition(Intercept.State.in_2_alpha,behavior.Behavior.State.completed,lambda:self.behavior_completed, 'In 2 alpha completed')

        self.add_transition(Intercept.State.move_on_circle,Intercept.State.setup,lambda:self.behavior_failed, 'Move on Circle failed')

        self.add_transition(Intercept.State.move_on_circle,behavior.Behavior.State.completed,lambda:self.at_target_point() or self.behavior_completed, 'Completed')

        self.add_transition(Intercept.State.inside_circles,behavior.Behavior.State.completed,lambda:self.behavior_completed or self.at_target_point(), 'Inside Circle completed')



    def bot_inside_2alpha(self):
        if self.theta > math.pi - self.alpha:
            return True
        return False

    def bot_outside_circles(self):
        kub_pos = Vector2D(self.kub.state.homePos[self.kub.kubs_id].x, self.kub.state.homePos[self.kub.kubs_id].y)

        if kub_pos.dist(self.center1)>self.radius+50 and kub_pos.dist(self.center2) > self.radius+50:
            return True

        return False


    def bot_on_circle(self):
        ERROR_RADIUS = 50
        kub_pos = Vector2D(self.kub.state.homePos[self.kub.kubs_id].x, self.kub.state.homePos[self.kub.kubs_id].y)

        if abs(kub_pos.dist(self.center1)-self.radius)<=ERROR_RADIUS or abs(kub_pos.dist(self.center2) - self.radius)<=ERROR_RADIUS:
            return True

        return False

    def at_target_point(self):
        kub_pos = Vector2D(self.kub.state.homePos[self.kub.kubs_id].x, self.kub.state.homePos[self.kub.kubs_id].y)
        if kub_pos.dist(self.target)<=100 or kub_pos.dist(self.kub.state.ballPos) <= 50:
            return True
        # if kub_pos.dist(self.target)<=200 and kub_pos.dist(self.kub.state.ballPos) <= 200:
        #     return True
        return False

    def add_ball_dir(self):
        recv_pos = Vector2D(self.kub.state.homePos[1])
        theta = functions.normalize_angle(self.kub.get_pos().theta)
        ball_pos = Vector2D(self.kub.state.ballPos)
        ball_vel = recv_pos - ball_pos
        ball_vel = Vector2D(sin(theta),cos(theta))
        print 'tHETA:' + str(theta)
        self.ball_vel_dir = ball_vel*(1/ball_vel.abs(ball_vel))

    def calcTheta(self):          
        kub_pos = Vector2D(self.kub.state.homePos[self.kub.kubs_id].x, self.kub.state.homePos[self.kub.kubs_id].y)
        # ball_vel = Vector2D(self.kub.state.ballVel.x, self.kub.state.ballVel.y)
        vec_ball_bot = kub_pos - self.target
        dot_temp = vec_ball_bot.dot(self.ball_vel_dir)
        #print("=============ball_vel_dir====================",self.ball_vel_dir.x,self.ball_vel_dir.y)
        #print("=============vec_ball_bot====================",vec_ball_bot.x,vec_ball_bot.y)
        final = dot_temp / vec_ball_bot.abs(vec_ball_bot)
        self.theta = functions.normalize_angle(math.acos(final))
        #print("=============theta====================",self.theta)

    def calcCenter(self):
        ball_vel_dir = self.ball_vel_dir
        cos_beta = sin(self.alpha)
        sin_beta = cos(self.alpha)
        self.center1 = Vector2D()
        self.center1.x = self.target.x + (cos_beta*ball_vel_dir.x - sin_beta*ball_vel_dir.y)*self.radius
        self.center1.y = self.target.y + (sin_beta*ball_vel_dir.x + cos_beta*ball_vel_dir.y)*self.radius

        self.x_cent1.append(self.center1.x)
        self.y_cent1.append(self.center1.y)

        self.circle1 = Circle(self.center1,self.radius)

        cos_beta = sin(self.alpha)
        sin_beta = -cos(self.alpha)
        self.center2 = Vector2D()
        self.center2.x = self.target.x + (cos_beta*ball_vel_dir.x - sin_beta*ball_vel_dir.y)*self.radius
        self.center2.y = self.target.y + (sin_beta*ball_vel_dir.x + cos_beta*ball_vel_dir.y)*self.radius

        self.circle2 = Circle(self.center2,self.radius)

        self.x_cent2.append(self.center2.x)
        self.y_cent2.append(self.center2.y)


    def plot(self):
        plt.figure()
        plt.plot(self.x_tar,self.y_tar,'ok',
                self.x_pt,self.y_pt,'og',
                self.x_cent1,self.y_cent1,'or',
                self.x_cent2,self.y_cent2,'ob',
                self.x_kub,self.y_kub,'oy',
                self.x_ball,self.y_ball,'om')
        plt.savefig('plot_path.png')


    def add_kub(self,kub):
        self.kub = kub

    def add_target(self):
        ball_pos = Vector2D(self.kub.state.ballPos.x,self.kub.state.ballPos.y)
        ball_vel = self.ball_vel_dir

        self.target = ball_pos - self.d*(ball_vel)
        self.x_tar.append(self.target.x)
        self.y_tar.append(self.target.y)

 

    def decide_tangent(self):
        kub_pos = Vector2D(self.kub.state.homePos[self.kub.kubs_id].x, self.kub.state.homePos[self.kub.kubs_id].y)
        ball_vel = self.ball_vel_dir
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

        self.x_pt.append(self.point_on_circle.x)
        self.y_pt.append(self.point_on_circle.y)

        self.x_pt.append(self.point_on_circle.x)
        self.y_pt.append(self.point_on_circle.y)

        tangent = self.point_on_circle - kub_pos
        self.tangent = tangent*(1/ tangent.abs(tangent))


    def add_point_on_arc(self):
        # if self.center_arc is not None:
        #     print("center already given")
        #     return
        # if self.target is None:
        #     print("target not given")
        #     return
        bot_pos = self.kub.get_pos()
        a1,b1,c1 = 2*(self.target.x-bot_pos.x), 2*(self.target.y-bot_pos.y), bot_pos.x**2+bot_pos.y**2-self.target.x**2-self.target.y**2
        a2,b2,c2 = 2*(self.kub.state.ballPos.x-bot_pos.x), 2*(self.kub.state.ballPos.y-bot_pos.y), bot_pos.x**2+bot_pos.y**2-self.kub.state.ballPos.x**2-self.kub.state.ballPos.y**2
        self.x_ball.append(self.kub.state.ballPos.x)
        self.y_ball.append(self.kub.state.ballPos.y)
        x = (b1*c2-b2*c1)/(a1*b2-a2*b1)
        y = (c1*a2-c2*a1)/(a1*b2-a2*b1)
        self.center_arc = Vector2D()
        self.center_arc.x = x
        self.center_arc.y = y
        self.radius_arc = dist(self.center_arc, self.target)

        print(self.radius_arc)

    def update_all(self):
        try:
            self.kub.state = getState(prev_state).stateB
        except rospy.ServiceException, e:
            print("Error ", e)


        self.i += 1
        kub_pos = Vector2D(self.kub.state.homePos[self.kub.kubs_id].x, self.kub.state.homePos[self.kub.kubs_id].y)
        self.x_kub.append(kub_pos.x)
        self.y_kub.append(kub_pos.y)

        self.add_target()
        self.calcTheta()
        
        # if self.bot_outside_circles() and not self.bot_inside_2alpha():
        self.center1 = self.center1 + (self.target - self.prev_target)
        self.circle1.center = self.center1
        self.center2 = self.center2 + (self.target - self.prev_target)
        self.circle2.center = self.center2
        self.x_cent1.append(self.center1.x)
        self.y_cent1.append(self.center1.y)
        self.x_cent2.append(self.center2.x)
        self.y_cent2.append(self.center2.y)

        if self.bot_outside_circles() and not self.bot_inside_2alpha():
            self.decide_tangent()


        self.prev_target = self.target

        if self.i%500==0 and self.i!=0:
            self.plot()


        

    def on_enter_setup(self):
        self.behavior_failed = False

    def execute_setup(self):
        # time.sleep(0.2)
        try:
            self.kub.state = getState(prev_state).stateB
        except rospy.ServiceException, e:
            print("Error ", e)
        

        if self.FIRST_CALL:
            self.add_ball_dir()
            self.FIRST_CALL = False
        self.add_target()
        self.prev_target = self.target
        self.calcTheta()
        self.calcCenter()

    def on_exit_setup(self):
        pass

    def on_enter_in_2_alpha(self):
        pass

    def execute_in_2_alpha(self):
        while True:
            
            k = 4
            kub_pos = Vector2D(self.kub.state.homePos[self.kub.kubs_id].x, self.kub.state.homePos[self.kub.kubs_id].y)
            direct_vel = (self.target-kub_pos) * MAX_BOT_SPEED *(1/ (self.target-kub_pos).abs(self.target-kub_pos))
            bot_target_vec = (self.target - kub_pos)*(1/(self.target - kub_pos).abs(self.target - kub_pos))
            ball_vel = Vector2D(self.kub.state.ballVel.x, self.kub.state.ballVel.y)
            vel_max_given = direct_vel - ball_vel
            vel = min(vel_max_given.abs(vel_max_given), k*kub_pos.dist(self.target))
            vel = vel*bot_target_vec + ball_vel
            
            # rotate = self.ball_vel_dir.angle()
            # vw = Get_Omega(self.kub.kubs_id,rotate,self.kub.state.homePos)

            # if abs(normalize_angle(self.kub.state.homePos[self.kub.kubs_id].theta-rotate))<ROTATION_FACTOR:
            #     self.kub.turn(0)
            # else:
            #     self.kub.turn(vw)


            self.kub.move(vel.x,vel.y)

            if self.target.dist(kub_pos) < 50:
                self.kub.move(0,0)
            
            if  self.target.dist(kub_pos) < 50:# and abs(normalize_angle(self.kub.state.homePos[self.kub.kubs_id].theta-rotate))<ROTATION_FACTOR:
                self.kub.move(0,0)
                self.kub.execute()
                self.behavior_completed = True
                break

            self.kub.execute()

            if not self.bot_inside_2alpha():
                self.behavior_failed = True
                break

            self.update_all()    

    def on_exit_in_2_alpha(self):
        pass


    def on_enter_outside_circles(self):
        kub_pos = Vector2D(self.kub.state.homePos[self.kub.kubs_id].x, self.kub.state.homePos[self.kub.kubs_id].y)
        
        if kub_pos.dist(self.center1) < kub_pos.dist(self.center2):
            self.to_move_circle = self.circle1
            self.decided_circle = 'circle1'
        else:
            self.to_move_circle = self.circle2
            self.decided_circle = 'circle2'


        self.update_all()



    def execute_outside_circles(self):
        prev_vel = 0
        while True:
            consts = np.array([2,0.05])
            kub_pos = Vector2D(self.kub.state.homePos[self.kub.kubs_id].x, self.kub.state.homePos[self.kub.kubs_id].y)
            direct_vel = self.tangent*self.max_vel

            # print 'TANGENT: '+ str(self.tangent.x) + ',' + str(self.tangent.y)
            ball_vel = Vector2D(self.kub.state.ballVel.x, self.kub.state.ballVel.y)
            vel_max_given = direct_vel - ball_vel
            dist = kub_pos.dist(self.target)
            vel_rad = 2*dist - 0.2 * prev_vel
            vel = min(vel_max_given.abs(vel_max_given),vel_rad)

            # rotate = self.ball_vel_dir
            # rotate = rotate.angle()
            # vw = Get_Omega(self.kub.kubs_id,rotate,self.kub.state.homePos)

            # if abs(normalize_angle(self.kub.state.homePos[self.kub.kubs_id].theta-rotate))<ROTATION_FACTOR:
            #     self.kub.turn(0)
            # else:
            #     self.kub.turn(vw)
            
            velf = vel*self.tangent + ball_vel
            

            if self.point_on_circle.dist(kub_pos)< 50:
                self.kub.move(0,0)
            else:
                self.kub.move(velf.x,velf.y)

            self.kub.execute()

            if  self.point_on_circle.dist(kub_pos)< 50: # and abs(normalize_angle(self.kub.state.homePos[self.kub.kubs_id].theta-rotate))<ROTATION_FACTOR:
                self.kub.execute()
                break


            if self.bot_inside_2alpha() and not self.bot_on_circle():
                
                self.behavior_failed = True
                break

            # print 'Dist: ' + str(kub_pos.dist(self.point_on_circle))

            prev_vel = velf.abs(velf)
            self.update_all()

            

    def on_exit_outside_circles(self):
        pass



    def on_enter_move_on_circle(self):                       

        self.center_arc = None
        self.update_all()
        self.add_point_on_arc()
        _GoOnArc_.init(self.kub,self.kub.state.ballPos,self.center_arc,self.radius_arc,False,self.target,False)        

    def execute_move_on_circle(self):
        first_call = True
        start_time = rospy.Time.now()
        start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)
        generatingfunction = _GoOnArc_.execute(start_time,self.DISTANCE_THRESH/2, self.ROTATION_FACTOR)
        for gf in generatingfunction:
            self.kub,target = gf	
            self.update_all()
            _GoOnArc_.TARGET = self.kub.state.ballPos
            _GoOnArc_.POINT = self.target

            # if not first_call:
            #     dist = kub_pos.dist(self.target)
            #     if dist == prev_dist:
            #         self.kub.move(0,0)
            #         self.kub.execute()
            #         self.behavior_completed = True
            #         break
            
            # print 'Dist on  : ' + str(kub_pos.dist(self.target))
            # print 'Dist ball : ' + str(kub_pos.dist(self.kub.state.ballPos))
                
            if self.at_target_point():
                self.kub.move(0,0)
                self.kub.execute()
                self.behavior_completed = True
                break

            if self.bot_outside_circles():
                self.behavior_failed = True
                break
            
        



    def on_exit_move_on_circle(self):
        self.kub.move(0,0)
        self.kub.execute()

    def on_enter_inside_circles(self):
        kub_pos = Vector2D(self.kub.state.homePos[self.kub.kubs_id].x, self.kub.state.homePos[self.kub.kubs_id].y)
        
        if kub_pos.dist(self.center1) < kub_pos.dist(self.center2):
            self.to_move_circle = self.circle1
            self.decided_circle = 'circle1'
        else:
            self.to_move_circle = self.circle2
            self.decided_circle = 'circle2'

        self.update_all()
        self.add_point_on_arc()
        _GoOnArc_.init(self.kub,self.kub.state.ballPos,self.center_arc,self.radius_arc,False,self.target,False)

    def execute_inside_circles(self):
        start_time = rospy.Time.now()
        start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)
        generatingfunction = _GoOnArc_.execute(start_time,self.DISTANCE_THRESH, self.ROTATION_FACTOR)
        for gf in generatingfunction:
            self.kub,target = gf
            self.update_all()
            self.center_arc = None
            _GoOnArc_.TARGET = self.kub.state.ballPos
            _GoOnArc_.POINT = self.target

            if self.at_target_point():
                self.kub.move(0,0)
                self.kub.execute()
                self.behavior_completed = True
                break

            if not vicinity_points(self.kub.state.ballPos,target):
                self.behavior_failed = True
                break

    def on_exit_inside_circles(self):
        pass

        

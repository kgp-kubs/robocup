from enum import Enum
import behavior
import _GoToPoint_
import rospy
from utils.functions import *
from utils.geometry import *
from utils.config import *
from math import *

class DribbleAndKick(behavior.Behavior):

    class State(Enum):
        setup = 1 
        course_approach = 2
        fine_approach = 3
        intercept = 4
        dribble_move = 5
        kick = 6

    def __init__(self,course_approch_thresh =  DISTANCE_THRESH/3,continuous=False):

        super(DribbleAndKick,self).__init__()

        self.name = "DribbleAndKick"

        self.power = 2.5

        self.target_point = None

        self.course_approch_thresh = 4*course_approch_thresh

        self.ball_dist_thresh = 1.3*BOT_BALL_THRESH

        self.behavior_failed = False
        


        self.add_state(DribbleAndKick.State.setup,
            behavior.Behavior.State.running)

        self.add_state(DribbleAndKick.State.course_approach,
            behavior.Behavior.State.running)
        
        self.add_state(DribbleAndKick.State.fine_approach,
            behavior.Behavior.State.running)

        # self.add_state(DribbleAndKick.State.intercept,
        #     behavior.Behavior.State.running)

        self.add_state(DribbleAndKick.State.dribble_move,
            behavior.Behavior.State.running)

        # self.add_state(DribbleAndKick.State.kick,
        #     behavior.Behavior.State.running)



        self.add_transition(behavior.Behavior.State.start,
            DribbleAndKick.State.setup,lambda:True,'immediately')

        self.add_transition(DribbleAndKick.State.setup,
            DribbleAndKick.State.course_approach,lambda:not self.at_target_point() and not self.ball_moving(),'course approach')

        self.add_transition(DribbleAndKick.State.setup,
            DribbleAndKick.State.fine_approach,lambda:self.at_target_point() and not self.ball_moving(),'fine approach')

        # self.add_transition(DribbleAndKick.State.setup,
        #     DribbleAndKick.State.intercept,lambda:self.ball_moving(), 'intercept')

        self.add_transition(DribbleAndKick.State.course_approach,
            DribbleAndKick.State.fine_approach,lambda:self.at_target_point() and not self.ball_moving(),'course-fine approach')

        self.add_transition(DribbleAndKick.State.fine_approach,
            DribbleAndKick.State.dribble_move,lambda:self.at_ball_pos() and self.ball_connected(),'dribble')
        
        self.add_transition(DribbleAndKick.State.dribble_move,
            behavior.Behavior.State.completed,lambda:self.at_ball_pos(),'complete')

        # self.add_transition(DribbleAndKick.State.dribble_move,DribbleAndKick.State.kick,
        #     lambda:self.near_the_goal() and self.ball_connected(), 'kick')



    def at_target_point(self):
        return vicinity_points(self.target_point,self.kub.get_pos(),thresh= self.course_approch_thresh)

    def ball_moving(self):
        #########  ball moving condition  #########
        return False

    def at_ball_pos(self):
        return ball_in_front_of_bot(self.kub)

    def near_the_goal(self):
        # if vicinity_points(self.kub.get_pos(), )  
        return False

    def ball_connected(self):
        return  kub_has_ball(self.kub.state, self.kub.kubs_id)

    def add_kub(self,kub):
        self.kub = kub

    def add_theta(self,theta):
        self.theta = theta
    

    
    def on_enter_setup(self):
        self.target_point = self.kub.state.ballPos
    
    def execute_setup(self):
        pass
        
    def on_exit_setup(self):
        pass



    def on_enter_course_approach(self):
        self.target_point = getPointBehindTheBall(self.kub.state.ballPos,self.theta)
        # self.target_point = self.kub.state.ballPos
        _GoToPoint_.init(self.kub, self.target_point, self.theta)
        # pass

    def execute_course_approach(self):
        start_time = rospy.Time.now()
        start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
        generatingfunction = _GoToPoint_.execute(start_time,self.course_approch_thresh,True)
        #self.behavior_failed = False
        for gf in generatingfunction:
            self.kub,target_point = gf
            self.target_point = getPointBehindTheBall(self.kub.state.ballPos,self.theta)
            # self.target_point = self.kub.state.ballPos
            if not vicinity_points(self.target_point,target_point,thresh=BOT_RADIUS*2.0):
                self.behavior_failed = True
                break

    def on_exit_course_approach(self):
        pass



    def on_enter_fine_approach(self):
        print "[on_enter_fine_approach]"
        self.target_point = self.kub.state.ballPos
        theta = self.kub.get_pos().theta
        _GoToPoint_.init(self.kub, self.target_point, theta)
        pass

    def execute_fine_approach(self):
        print "[execute_fine_approach]"
        start_time = rospy.Time.now()
        start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
        generatingfunction = _GoToPoint_.execute(start_time,self.ball_dist_thresh,False,True)
        # generatingfunction = _GoToPoint_.execute(start_time,self.ball_dist_thresh,True,True)

        for gf in generatingfunction:
            self.kub,ballPos = gf
            # self.kub.dribble(True)
            # self.kub.execute()
            # print "OK VMRO"
            
            if not vicinity_points(ballPos,self.kub.state.ballPos,thresh=BOT_RADIUS):
                self.behavior_failed = True
                print("FAILED")
                break

    def on_exit_fine_approach(self):
        pass



    def on_enter_dribble_move(self):
        if not self.ball_connected():
            self.behavior_failed = True

        print "[on_enter_dribble_move]"
        self.kub.dribble(True)
        self.kub.execute()
        self.target_point=Vector2D(0, 0)
        _GoToPoint_.init(self.kub, self.target_point, self.theta)


    def execute_dribble_move(self):
        print "[execute_dribble_move]"
        start_time = rospy.Time.now()
        start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
        generatingfunction = _GoToPoint_.execute(start_time,self.course_approch_thresh, False, True)
        #self.behavior_failed = False

        for gf in generatingfunction:
            self.kub,target_point = gf
            # self.target_point = getPointBehindTheBall(self.kub.state.ballPos,self.theta)
            # self.target_point = self.kub.state.ballPos
            if not vicinity_points(self.target_point,target_point,thresh=BOT_RADIUS*2.0):
                self.behavior_failed = True
                print("FAILED")
                break

        self.kub.execute()

    def on_exit_dribble_move(self):
        self.kub.kick(self.power)
        self.kub.execute()
        print("Kick")
        pass
        

        

        

               

        

        

        

        


          



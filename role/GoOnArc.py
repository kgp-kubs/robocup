from enum import Enum
import behavior
import _GoOnArc_
try:
    _GoToPoint_ = reload(_GoToPoint_)
except:
    import _GoToPoint_
import rospy
import time
from utils.functions import *
from utils.config import *


DISTANCE_THRESH = 10

class GoOnArc(behavior.Behavior):
    """docstring for GoOnArc"""
    ##
    ## @brief      Class for state.
    ##
    class State(Enum):
        #formality
        setup = 1
        #ringa ringa roses
        round_n_round = 2


    def __init__(self,continuous=False):

        #initiating behaviour init
        super(GoOnArc,self).__init__()

        #defining class varibles
        self.name = "GoOnArc"
        self.center = None  #A vector2d for center of circle
        self.target = None
        self.radius = None
        self.point = None
        self.behavior_failed = False
        self.DISTANCE_THRESH = DISTANCE_THRESH
        self.take_bigger_arc = False

        #adding states to fsm
        self.add_state(GoOnArc.State.setup,
            behavior.Behavior.State.running)
        self.add_state(GoOnArc.State.round_n_round,
            behavior.Behavior.State.running)

        #adding trnsitions to fsm
        self.add_transition(behavior.Behavior.State.start,
            GoOnArc.State.setup,lambda: True,'immediately')

        self.add_transition(GoOnArc.State.setup,
            GoOnArc.State.round_n_round,lambda: self.checklist(),'setup')

        self.add_transition(GoOnArc.State.round_n_round,
            behavior.Behavior.State.completed,lambda:self.at_target(),'complete')

        self.add_transition(GoOnArc.State.setup,
            behavior.Behavior.State.failed,lambda: self.behavior_failed,'failed')

        self.add_transition(GoOnArc.State.round_n_round,
            behavior.Behavior.State.failed,lambda: self.behavior_failed,'failed')


    def add_center(self,center):
        if self.center is not None:
            print("center already there")
            return
        self.center = center
        bot_pos = self.kub.get_pos()
        bot_pos = Vector2D(bot_pos.x,bot_pos.y)
        self.radius = bot_pos.dist(self.center)
        self.point = None

    def add_point_on_arc(self, point):
        if self.center is not None:
            print("center already given")
            return
        if self.target is None:
            print("target not given")
            return
        bot_pos = self.kub.get_pos()
        a1,b1,c1 = 2*(point.x-bot_pos.x), 2*(point.y-bot_pos.y), bot_pos.x**2+bot_pos.y**2-point.x**2-point.y**2
        a2,b2,c2 = 2*(self.target.x-bot_pos.x), 2*(self.target.y-bot_pos.y), bot_pos.x**2+bot_pos.y**2-self.target.x**2-self.target.y**2
        x = (b1*c2--b2*c1)/(a1*b2-a2*b1)
        y = (c1*a2-c2*a1)/(a1*b2-a2*b1)
        self.center = Vector2D()
        self.center.x = x
        self.center.y = y
        self.radius = dist(self.center, point)
        self.point = point
        print(self.center.x, self.center.y)


    def add_kub(self,kub):
        self.kub = kub

    def add_target(self,target):
        self.target = target

    def bigger_arc(self):
        self.take_bigger_arc = True

    def checklist(self):
        center_check = (self.center is None)
        target_check = (self.target is None)
        if center_check or target_check:
            print("something missing out of center or target")
            return False
        return True


    def at_target(self):
        bot_pos = self.kub.get_pos()
        bot_pos = Vector2D(bot_pos.x, bot_pos.y)
        return dist(self.target,bot_pos) < self.DISTANCE_THRESH

    #setup state run functions
    def on_enter_setup(self):
        pass
    def execute_setup(self):
        _GoOnArc_.init(self.kub,self.target,self.center,self.radius,False,self.point, self.take_bigger_arc)
        pass

    def on_exit_setup(self):
        pass

    #round_n_round run functions
    def on_enter_round_n_round(self):
        pass

    def terminate(self):
        super().terminate()

    def execute_round_n_round(self):
        print("Execute drive")
        start_time = rospy.Time.now()
        start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)
        generatingfunction = _GoOnArc_.execute(start_time,self.DISTANCE_THRESH)
        print("Datatype of gf:",type(generatingfunction))
        for gf in generatingfunction:
            self.kub,target = gf

            if not vicinity_points(self.target,target):
                self.behavior_failed = True
                break
        self.new_point = self.kub.get_pos()

    def on_exit_round_n_round(self):
        pass

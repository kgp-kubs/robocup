from enum import Enum
import behavior
import rospy
from role import GoToPoint, KickToPoint
import math
import kubs
from utils import math_functions
from utils.geometry import Vector2D
from utils.functions import *
from utils.state_functions import *
from utils.config import *
from velocity.run import *
from krssg_ssl_msgs.srv import *
from utils.math_functions import *
import krssg_ssl_msgs.msg
from role import _turnAround_, _GoToPoint_,_PassRecv_,_GoOnArc_
import time
rospy.wait_for_service('bsServer',)
getState = rospy.ServiceProxy('bsServer',bsServer)
start_time = None
DIST_THRESH = 75
dboxp = Vector2D((abs(OUR_DBOX_X)-(DBOX_HEIGHT/2)),0)
DRIBBLE_DIST = BOT_RADIUS*10
BOT_BALL_THRESH = 0.9*BOT_RADIUS
V_CLOSE=MAX_BOT_SPEED/30
D_CLOSE=BOT_RADIUS*6
class Pass_Recieve(behavior.Behavior):

    class State(Enum):
        reach_ball=1
        align=2
        dribble_kick=3
        goal=4
        setup = 5
        insidealpha = 6 
        outsidecircle = 7
        insidecircle = 8
########kub is changed in on_exit_kick() state
    def __init__(self, continuous=False):

        super(Pass_Recieve,self).__init__()
        self.name = "Pass_Recieve"
        self.behavior_failed = False
        self.DISTANCE_THRESH = BOT_RADIUS 
        self.theta = 0
        self.course_approach_thresh = BOT_RADIUS 
        self.power = 3.0
        self.name = "Dribble_Kick"
        self.go_at = None
        self.target_point = None
        self.dbox = dboxp    
        self.approachball = False
        self.turn = False
        self.goto_dribble = False
        self.approachdbox = False
        self.readytogoal = False
        self.theta = 0
        self.behavior_failed = False

        ######STATES##########
        self.add_state(Pass_Recieve.State.reach_ball,
            behavior.Behavior.State.running)
        self.add_state(Pass_Recieve.State.align,
            behavior.Behavior.State.running)
        self.add_state(Pass_Recieve.State.dribble_kick,
            behavior.Behavior.State.running)
        self.add_state(Pass_Recieve.State.goal,
            behavior.Behavior.State.running)
        self.add_state(Pass_Recieve.State.setup,
            behavior.Behavior.State.running)
        self.add_state(Pass_Recieve.State.insidealpha,
            behavior.Behavior.State.running)
        self.add_state(Pass_Recieve.State.outsidecircle,
            behavior.Behavior.State.running)
        self.add_state(Pass_Recieve.State.insidecircle,
            behavior.Behavior.State.running)
        
        ##########TRANSITIONS############
        self.add_transition(behavior.Behavior.State.start,
            Pass_Recieve.State.reach_ball,lambda: self.reachball(),'far_away_from_ball')

        self.add_transition(Pass_Recieve.State.dribble_kick,
            Pass_Recieve.State.goal,lambda: self.readytogoal,'Goaling')
        
        self.add_transition(Pass_Recieve.State.reach_ball,
            Pass_Recieve.State.align,lambda: self.alignnow(),'Aligning')

        self.add_transition(Pass_Recieve.State.align,
            Pass_Recieve.State.dribble_kick,lambda: self.goto_dribble,"dribbling")
        
        self.add_transition(Pass_Recieve.State.goal,
            Pass_Recieve.State.setup,lambda: True, 'pass complete')
        
        self.add_transition(Pass_Recieve.State.setup,
            behavior.Behavior.State.completed,lambda: kub_has_ball(self.kub.state,self.kub.kubs_id),'setup')

        self.add_transition(Pass_Recieve.State.setup,
           Pass_Recieve.State.insidealpha,lambda:self.bot_inside_alpha() and (not kub_has_ball(self.kub.state,self.kub.kubs_id)),'inside alpha')
        
        self.add_transition(Pass_Recieve.State.setup,
            Pass_Recieve.State.outsidecircle,lambda:self.outsideofcircle() and (not kub_has_ball(self.kub.state,self.kub.kubs_id)) ,'ball_outside_circle')
        
        self.add_transition(Pass_Recieve.State.setup,
            Pass_Recieve.State.insidecircle,lambda: self.insideofcircle() and (not kub_has_ball(self.kub.state,self.kub.kubs_id)),'inside circle')
        
        self.add_transition(Pass_Recieve.State.insidecircle,
            Pass_Recieve.State.setup,lambda: True,'back to setup')
        
        self.add_transition(Pass_Recieve.State.outsidecircle,
            Pass_Recieve.State.setup,lambda:True,'back to setup')
        
        self.add_transition(Pass_Recieve.State.insidealpha,
            Pass_Recieve.State.setup,lambda: True,'back to setup')
        
        self.add_transition(Pass_Recieve.State.reach_ball,
			behavior.Behavior.State.failed, lambda: self.behavior_failed, 'failed')
		
        self.add_transition(Pass_Recieve.State.align,
			behavior.Behavior.State.failed, lambda: self.behavior_failed, 'failed')
			
        self.add_transition(Pass_Recieve.State.dribble_kick,
			behavior.Behavior.State.failed, lambda: self.behavior_failed, 'failed')


        
    def add_kickto(self,point):
        self.kickto = point

    def add_theta(self,theta):
        self.theta = theta

    def add_kubs(self,passer,receiver):
        self.passer = passer
        self.receiver = receiver
        self.kub = self.passer

    def change_kub(self):
        if self.kub == self.passer:
            self.kub = self.receiver
        else:
            self.kub = self.passer
        
    def near_the_ball(self):
        if ball_in_front_of_bot(self.kub):
            return True
        return False
       
    def ball_in_front(self):
        if ball_in_front_of_bot(self.kub):
            return True
        return False
    
    def get_pos_asvec2d(self, point2d):
        return Vector2D(int(point2d.x),int(point2d.y))
    
    def reachball(self):
        ballpos = self.kub.state.ballPos
        theta = angle_diff(ballpos, self.kickto)
        go_at = getPointBehindTheBall(ballpos, theta, 2)
	self.approachball = True
        return not(self.turn) and not(self.goto_dribble) and go_at.dist(self.get_pos_asvec2d(self.kub.get_pos())) >= DISTANCE_THRESH*0.25 and self.approachball

    def alignnow(self):
        ballpos = self.kub.state.ballPos
        theta = angle_diff(ballpos, self.kickto)
        go_at = getPointBehindTheBall(ballpos, theta, 2)
        return not (go_at.dist(self.get_pos_asvec2d(self.kub.get_pos())) >= DISTANCE_THRESH*0.25) and self.turn and not(self.goto_dribble)
		
    def at_DBox(self):
        target = self.kickto()
        curPos = self.kub.get_pos()
        return target.dist(self.get_pos_asvec2d(curPos)) <= DISTANCE_THRESH and self.turndbox

    def getPointBehindTheBall(self, factor=3.5):
        point = self.kub.state.ballPos
        theta = self.theta
        x = point.x - (factor * BOT_RADIUS) * (math.cos(theta))
        y = point.y - (factor * BOT_RADIUS) * (math.sin(theta))
        return Vector2D(int(x), int(y))
    
    def add_alpha(self,alpha):
        self.alpha = alpha
        self.radius = BOT_BALL_THRESH/(2*math.sin(self.alpha))

    def kub_reset(self):
        self.kub.reset()
        self.kub.execute()

    def bot_inside_alpha(self):
        if self.kub.state.ballVel.x == 0 and self.kub.state.ballVel.y == 0:
            veldir = math.atan2(self.kub.state.ballPos.y-self.kub.get_pos().y,self.kub.state.ballPos.x-self.kub.get_pos().x)   
        else:
            veldir = math.atan2(self.kub.state.ballVel.y, self.kub.state.ballVel.x)
        point = getPointBehindTheBall(self.kub.state.ballPos,veldir)
        #angle = angle_diff(point,self.kub.get_pos())
        angle = math.atan2(point.y - self.kub.get_pos().y,point.x - self.kub.get_pos().x)
        if abs(veldir - angle) <= self.alpha:
            return True
        return False
    
    def insideofcircle(self):
        if self.kub.state.ballVel.x == 0 and self.kub.state.ballVel.y == 0:
            veldir = math.atan2(self.kub.state.ballPos.y-self.kub.get_pos().y,self.kub.state.ballPos.x-self.kub.get_pos().x)    
        else:
            veldir = math.atan2(self.kub.state.ballVel.y, self.kub.state.ballVel.x)
        v1 = Vector2D(self.radius*math.sin(self.alpha)*math.cos(veldir),self.radius*math.sin(self.alpha)*math.sin(veldir))
        v2 = Vector2D(-self.radius*math.cos(self.alpha)*math.sin(veldir),self.radius*math.cos(self.alpha)*math.cos(veldir))
        v3 = Vector2D(self.radius*math.cos(self.alpha)*math.sin(veldir),-self.radius*math.cos(self.alpha)*math.cos(veldir))
        ballPos =Vector2D(self.kub.state.ballPos.x,self.kub.state.ballPos.y)
        center1 = ballPos - v1 - v2
        center2 = ballPos - v1 - v3
        self.circle1 = Circle(center1,self.radius)
        self.circle2 = Circle(center2,self.radius)
        if center2.dist(self.kub.get_pos()) < self.radius:
            self.circle = self.circle2
            print(True)
            return True
        if center1.dist(self.kub.get_pos()) < self.radius:
            self.circle = self.circle1
            print(True)
            return True
        return False 

    def outsideofcircle(self):
        if self.bot_inside_alpha() or self.insideofcircle():
            return False
	print(self.insideofcircle())
        if self.kub.state.ballVel.x == 0 and self.kub.state.ballVel.y == 0:
            veldir = math.atan2(self.kub.state.ballPos.y-self.kub.get_pos().y,self.kub.state.ballPos.x-self.kub.get_pos().x)    
        else:
            veldir = math.atan2(self.kub.state.ballVel.y, self.kub.state.ballVel.x)
        v1 = Vector2D(self.radius*math.sin(self.alpha)*math.cos(veldir),self.radius*math.sin(self.alpha)*math.sin(veldir))
        v2 = Vector2D(-self.radius*math.cos(self.alpha)*math.sin(veldir),self.radius*math.cos(self.alpha)*math.cos(veldir))
        v3 = Vector2D(self.radius*math.cos(self.alpha)*math.sin(veldir),-self.radius*math.cos(self.alpha)*math.cos(veldir))
        ballPos =Vector2D(self.kub.state.ballPos.x,self.kub.state.ballPos.y)
        center1 = ballPos - v1 - v2
        center2 = ballPos - v1 - v3
        if center2.dist(self.kub.get_pos()) < center1.dist(self.kub.get_pos()):
            self.circle = self.circle2
        else:
            self.circle = self.circle1
        return True

    def at_target_point(self):
        return vicinity_points(self.target_point,self.kub.get_pos(),thresh= self.course_approch_thresh)


    def ball_in_vicinity(self):
        if ball_in_front_of_bot(self.kub):
            return True
        return False
    
    def at_ball_pos(self):
        error = 50
        return vicinity_points(self.kub.get_pos(),self.kub.state.ballPos,thresh=self.ball_dist_thresh+error) 

    def on_enter_reach_ball(self):
        self.theta = normalize_angle(angle_diff(self.kub.state.ballPos,self.kickto))
        self.target_point = getPointBehindTheBall(self.kub.state.ballPos, self.theta, 2)
        #self.target_point = self.kub.state.ballPos
        _GoToPoint_.init(self.kub, self.target_point,0)
	

    def execute_reach_ball(self):
        start_time = rospy.Time.now()
        start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
        generatingfunction = _GoToPoint_.execute(start_time,self.course_approach_thresh,True)
        for gf in generatingfunction:
            self.kub,target_point = gf
            self.target_point = getPointBehindTheBall(self.kub.state.ballPos, self.theta, 1.5)
            if not vicinity_points(self.target_point,target_point,thresh=BOT_RADIUS):
                self.behavior_failed = True
                break
        self.turn = True
        print("Reached the ball. Start ALIGN ... ")
        self.approachball = False

    def on_enter_align(self):
        print("Entering align: ", self.turn)
        ballpos = self.kub.state.ballPos
        self.theta = angle_diff(self.kub.get_pos(),ballpos)
        self.go_at = getPointBehindTheBall(ballpos, self.theta, 2)

    def execute_align(self):
		
        READY_TO_DRIBBLE = self.goto_dribble
        theta = self.theta

        rospy.wait_for_service('bsServer',)
        getState = rospy.ServiceProxy('bsServer',bsServer)

        prev_state = None   #HERE decalred None
		#What should it be?
        try:
                prev_state = getState(prev_state).stateB
        except rospy.ServiceException, e:
                print("Error :: ROS ", e)

        vw=0.0
        while not READY_TO_DRIBBLE: 

            try:
                prev_state = getState(prev_state).stateB
            except rospy.ServiceException, e:
                print("Error :: ROS ", e)
			# print(kub.state)
            self.kub.update_state(prev_state)
            if not(prev_state == self.kub.state):
                prev_state = self.kub.state
            print(self.kub.get_theta())
            print("Target Alignment : ",theta)
            totalAngle = theta
            MAX_w = (MAX_BOT_OMEGA+MIN_BOT_OMEGA)/1.2
            MIN_w = (MIN_BOT_OMEGA)

				# theta2 = totalAngle :-- target angular alignment ; theta1 = current_bot_theta; New x-axis: Bot_theta_line
				# rotating standard axes
            deltheta = totalAngle-normalize_angle(self.kub.get_theta()) 
            modtheta = min(abs(deltheta),abs((2*math.pi)-deltheta))  #shortest turn
            sign = (normalize_angle(deltheta))/(abs(normalize_angle(deltheta))) 

			#print "Remaining angle: ",modtheta,"   ",sign
				
            theta_lft = modtheta * sign    
            if abs(theta_lft)<ROTATION_FACTOR/2:
                vw = 0.0
                READY_TO_DRIBBLE=True
			
            else:
                READY_TO_DRIBBLE=False
                vw = 3.0*(theta_lft)/(math.pi)*MAX_w
					#vw = (theta_lft/2*math.pi)*MAX_w
            if abs(vw)<1*MIN_w and READY_TO_DRIBBLE==False:
                vw = 1*MIN_w*(1 if vw>0 else -1)
				
            if abs(vw) > MAX_w and READY_TO_DRIBBLE==False:
                vw = (vw/abs(vw))*MAX_w
						
            if vw!=0:
                print("TURNING")
						#print("vw Hoon Main =",vw)
						#print("BOT THETA:",data.homePos[BOT_ID].theta)
                print ("\nAngle Remaining :",theta_lft)
            else:
                print("DONE !!")
                break
					
            print ("Omega Return: ",vw)
            print (READY_TO_DRIBBLE)
            print ("_____________________________________________")
            self.kub.reset()
            self.kub.turn(vw)
			
            self.kub.execute()
		
        try:
            prev_state = getState(prev_state).stateB
        except rospy.ServiceException, e:
            print("Error :: ROS ", e)
			# print(kub.state)
        self.kub.update_state(prev_state)
        if not(prev_state == self.kub.state):
            prev_state = self.kub.state

        self.turn = False
        self.goto_dribble = True
        print("HO GYA ALIGN !! ---------------------------------------")

    def on_exit_align(self):
        print("EXITING ALIGN... goto_dribble:",self.goto_dribble)
        self.goto_dribble = True; #Now, Ready to dribble after aligning
        self.approachball = False
        self.turn = False
    
    def on_enter_dribble_kick(self):
        self.kub.dribble(True)
        self.kub.execute()
        pass

    def execute_dribble_kick(self):
        print("Execute dribble_to_point")
        start_time = rospy.Time.now()
        start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)
        self.theta = normalize_angle(angle_diff(self.kub.state.ballPos,self.kickto))
        self.dribbleto = Vector2D(self.kub.state.ballPos.x+ DRIBBLE_DIST* math.cos(self.theta),self.kub.state.ballPos.y+ DRIBBLE_DIST* math.sin(self.theta))
        _GoToPoint_.init(self.kub,self.dribbleto,self.theta)
        generatingfunction = _GoToPoint_.execute(start_time,self.DISTANCE_THRESH)
        print("Datatype of gf:",type(generatingfunction))
        for gf in generatingfunction:
            self.kub,target_point = gf

            if not vicinity_points(self.dribbleto,target_point):
                self.behavior_failed = True
                break
        self.kub.reset()
        self.kub.execute()
        self.goto_dribble = False
        self.kub.dribbler = False
        self.readytogoal = True
        self.kub.execute()

    def on_exit_reach_ball(self):
        rospy.loginfo("exit reach ball")
        print("I have reached the ball")
        self.turn = True

    def on_exit_dribble_kick(self):
        self.kub.dribbler = False
        self.kub.execute()
        self.kub.kick(self.power)
        self.kub.execute()
        self.goto_dribble = False
        self.go_at = None
        self.turn = False
        self.kub.reset()
        self.kub.execute()
    
    def on_enter_goal(self):
        pass
    def execute_goal(self):
        self.readytogoal = True
        self.kub_reset()
        self.change_kub()
        self.kub.execute()

    def on_exit_goal(self):
        pass

    def on_enter_setup(self):
        self.kub.dribble(True)
        self.kub.execute()
        pass
    def execute_setup(self):
        ball_vel = self.kub.state.ballVel
        if ball_vel.x == 0 and ball_vel.y == 0:
            bot_pos = self.kub.get_pos()
            ball_pos = self.kub.state.ballPos
            self.veldir = math.atan2(ball_pos.y-bot_pos.y, ball_pos.x-bot_pos.x)
        else:
            self.veldir = math.atan2(self.kub.state.ballVel.y, self.kub.state.ballVel.x)
        pass
        
    def on_exit_setup(self):
        pass

    def on_enter_insidealpha(self):
        _PassRecv_.init(self.kub,self.kub.state.ballPos,self.veldir)

    def execute_insidealpha(self):
        start_time = rospy.Time.now()
        start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
	
        generatingfunction = _PassRecv_.execute('inside_alpha', start_time, BOT_RADIUS*1.2)
        for gf in generatingfunction:
            self.kub,target_point= gf
            if vicinity_points(self.kub.state.ballPos, self.kub.get_pos(),thresh =  BOT_RADIUS):
                break
    
    def on_exit_insidealpha(self):
        pass
    
    def on_enter_outsidecircle(self):
        _PassRecv_.init(self.kub,self.kub.state.ballPos,self.veldir,self.circle.center,self.circle.radius)
        if self.kub.state.ballVel.x == 0 and self.kub.state.ballVel.y <= 0.01:
            veldir = math.atan2(self.kub.state.ballPos.y-self.kub.get_pos().y,self.kub.state.ballPos.x-self.kub.get_pos().x)    
        else:
            veldir = math.atan2(self.kub.state.ballVel.y, self.kub.state.ballVel.x)
        point=getPointBehindTheBall(self.kub.state.ballPos,veldir)
        _GoOnArc_.init(self.kub,self.kub.state.ballPos, self.circle.center, self.radius,True, point, False)
    
    def execute_outsidecircle(self):
        start_time = rospy.Time.now()
        start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
        generatingfunction = _PassRecv_.execute('outside_circle', start_time, BOT_RADIUS*1.2)
        for gf in generatingfunction:
	    if dist(self.circle.center,self.kub.get_pos())<=self.radius:
		break
	    self.kub,target_point = gf
            if vicinity_points(self.kub.state.ballPos,self.kub.get_pos(),thresh=BOT_RADIUS):
                break
        start_time = rospy.Time.now()
        start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
        generatingfunction = _GoOnArc_.execute(start_time, 20, math.pi/360)
        for gf in generatingfunction:
            _GoOnArc_.TARGET = self.kub.state.ballPos
            if self.kub.state.ballVel.x == 0 and self.kub.state.ballVel.y <= 0.01:
                veldir = math.atan2(self.kub.state.ballPos.y-self.kub.get_pos().y,self.kub.state.ballPos.x-self.kub.get_pos().x)    
            else:
                veldir = math.atan2(self.kub.state.ballVel.y, self.kub.state.ballVel.x)
            point=getPointBehindTheBall(self.kub.state.ballPos,veldir)
            _GoOnArc_.POINT = point
            _GoOnArc_.CENTER = self.circle.center
	    self.kub,target_point = gf
            if vicinity_points(self.kub.state.ballPos,self.kub.get_pos(),thresh=BOT_RADIUS):
                break

    def on_exit_outsidecircle(self):
        pass

    def on_enter_insidecircle(self):
        _PassRecv_.init(self.kub,self.kub.state.ballPos,self.veldir,self.circle.center,self.circle.radius)

    def execute_insidecircle(self):
        start_time = rospy.Time.now()
        start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
        generatingfunction = _PassRecv_.execute('inside_circle', start_time, BOT_RADIUS*1.2)
        for gf in generatingfunction:
            
	    if dist(self.circle.center,self.kub.get_pos())>self.radius:
		break
            self.kub,target_point = gf
            if vicinity_points(self.kub.state.ballPos,self.kub.get_pos(),thresh=BOT_RADIUS):
                break
    def on_exit_insidecircle(self):
        pass

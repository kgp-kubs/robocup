import behavior
import _GoToPoint_
import _GoOnArc_
from enum import Enum
import rospy
from utils.functions import *
from utils.geometry import *
from utils.config import *
from math import *
from velocity.run_w import *
import time
from krssg_ssl_msgs.srv import *
from utils import config, functions


rospy.wait_for_service('bsServer',)
getState = rospy.ServiceProxy('bsServer',bsServer)

prev_state = None
try:
    prev_state = getState(prev_state).stateB
except rospy.ServiceException, e:
    print("Error ", e)



class Main(behavior.Behavior):

    class State(Enum):
        main = 1
        Pass = 2
        Receive = 3
        Goal = 4

    def __init__(self,max_vel = MAX_BOT_SPEED/1.2,min_vel = MIN_BOT_SPEED*1.2):

        super(Main,self).__init__()

        self.behavior_completed = False

        self.receive_completed = False

        self.pass_completed = False

        self.goal_completed = False

        self.passing = False

        self.receive = False

        self.goaling = False

        self.FIRST_CALL = True

        self.add_state(Main.State.main,behavior.Behavior.State.running)

        self.add_state(Main.State.Receive,Main.State.main)

        self.add_state(Main.State.Pass,Main.State.main)

        self.add_state(Main.State.Goal,Main.State.main)

        self.add_transition(behavior.Behavior.State.start,Main.State.main,lambda: True,'immediately')

        self.add_transition(Main.State.main,Main.State.Pass,lambda: self.is_pass(),'Pass')

        self.add_transition(Main.State.main,Main.State.Receive,lambda: self.is_receive(),'Receive')

        self.add_transition(Main.State.main,Main.State.Goal,lambda: self.is_goal(),'Goal')

        self.add_transition(Main.State.main,behavior.Behavior.State.completed,lambda: self.behavior_completed,'Completed')


    def add_kub(self, kub):
        self.kub = kub

    def add_receiver(self, receiver):
        self.receiver = receiver

    def add_conn(self,conn):
        self.conn = conn

    def is_pass(self):
        return self.passing

    def is_goal(self):
        return self.goaling

    def is_receive(self):
        return self.receive

    def on_enter_main(self):
        kub_pos = Vector2D(self.kub.get_pos())
        recv_pos = Vector2D(self.kub.state.homePos[self.receiver])
        ball_pos = Vector2D(self.kub.state.ballPos)

        if kub_pos.dist(ball_pos) < recv_pos.dist(ball_pos):
            self.passing = True

    def execute_main(self):

        if self.goal_completed:
            self.goaling = False
            self.behavior_completed = True

        if self.pass_completed:
            ball_vel = Vector2D(self.kub.state.ballVel.x,self.kub.state.ballVel.y)
            if vicinity_points(self.kub.get_pos(),self.kub.state.ballPos,thresh=BOT_RADIUS*2) or ball_vel.abs(ball_vel) == 0:
                self.passing = True
                self.pass_completed = False
            else:
                self.passing = False
                self.pass_completed = False
                self.conn.send('kicked')

        if self.receive_completed:
            self.receive_completed = False
            self.FIRST_CALL = True
            if vicinity_points(self.kub.get_pos(),self.kub.state.ballPos,thresh=BOT_BALL_THRESH):   
                self.receive = True
            else:
                if self.kub.state.ballPos.y > 4500:
                    self.goaling = True
                else:
                    self.passing = True
                
                self.receive = False
                

        i = 0 
        if self.receive or self.passing or self.goaling or self.behavior_completed:
            pass
        else:
            while True:
                i+= 1
                if i<10000:
                    self.kub.move(0,0)
                    self.kub.execute()
                else:
                    break
           
            obj = self.conn.recv()
            self.receive = True

    def on_exit_main(self):
        pass   


class Pass(Main):

    class State(Enum):
        setup_pass = 1
        fine_receive = 2
        move_to_point = 3
        kick = 4

    def __init__(self):

        super(Pass, self).__init__()

        self.power = 3.5

        self.alpha = math.pi/6

        self.target_point = None

        self.distance = BOT_RADIUS*2

        self.distance1 = BOT_BALL_THRESH

        self.ball_dist_thresh = BOT_BALL_THRESH

        self.rotate = None

        self.goal = Vector2D(6000,0)

        self.pass_failed = False


        self.add_state(Pass.State.setup_pass, Main.State.Pass)
        self.add_state(Pass.State.fine_receive, Main.State.Pass)
        self.add_state(Pass.State.move_to_point, Main.State.Pass)
        self.add_state(Pass.State.kick, Main.State.Pass)

        self.add_transition(Main.State.Pass, Pass.State.setup_pass, lambda: True, 'immediately')
        self.add_transition(Pass.State.setup_pass, Pass.State.fine_receive, lambda: self.first_receive(), 'first_receive')
        self.add_transition(Pass.State.setup_pass, Pass.State.move_to_point, lambda: self.at_ball_pos(), 'ball connected')
        self.add_transition(Pass.State.fine_receive, Pass.State.move_to_point, lambda: self.at_ball_pos(), 'ball connected')
        self.add_transition(Pass.State.move_to_point, Pass.State.kick, lambda: self.at_target(), 'kick')
        self.add_transition(Pass.State.fine_receive,Pass.State.setup_pass, lambda: self.pass_failed, 'failed')
        self.add_transition(Pass.State.move_to_point,Pass.State.setup_pass, lambda: self.pass_failed, 'failed')
        self.add_transition(Pass.State.kick,Pass.State.setup_pass, lambda: self.pass_failed, 'failed')
        self.add_transition(Pass.State.kick, Main.State.main, lambda: self.pass_completed, 'completed')



    def first_receive(self):
        return not kub_has_ball(self.kub.state, self.kub.kubs_id)

    def ball_connected(self):
        return kub_has_ball(self.kub.state, self.kub.kubs_id)

    def at_ball_pos(self):
        return vicinity_points(self.kub.state.ballPos,self.kub.get_pos(),thresh= self.ball_dist_thresh*2) and abs(normalize_angle(self.kub.get_pos().theta-self.theta)) <= ROTATION_FACTOR

    def at_target(self):
        return vicinity_points(self.target_point,self.kub.get_pos(),thresh= self.ball_dist_thresh*2) and abs(normalize_angle(self.kub.get_pos().theta-self.theta)) <= ROTATION_FACTOR

    def calculate_theta(self,point):

        rev_pos = Vector2D(self.kub.state.homePos[self.receiver])
        vec_ball_bot = rev_pos - point
        dist = vec_ball_bot.abs(vec_ball_bot)
        
        cos_beta = sin(self.alpha)
        sin_beta = cos(self.alpha)
        point1 = Vector2D()
        point1.x = point.x + (cos_beta*vec_ball_bot.x - sin_beta*vec_ball_bot.y)*dist
        point1.y = point.y + (sin_beta*vec_ball_bot.x + cos_beta*vec_ball_bot.y)*dist


        cos_beta = sin(self.alpha)
        sin_beta = -cos(self.alpha)
        point2 = Vector2D()
        point2.x = point.x + (cos_beta*vec_ball_bot.x - sin_beta*vec_ball_bot.y)*dist
        point2.y = point.y + (sin_beta*vec_ball_bot.x + cos_beta*vec_ball_bot.y)*dist

        if point1.dist(self.goal) < point2.dist(self.goal):
            self.theta = vec_ball_bot.angle() + self.alpha
        else:
            self.theta = vec_ball_bot.angle() - self.alpha



    def on_enter_setup_pass(self):
        self.behavior_failed = False

    def execute_setup_pass(self):
        ball_pos = Vector2D(self.kub.state.ballPos)
        self.calculate_theta(ball_pos)

    def on_exit_setup_pass(self):
        pass

    def on_enter_fine_receive(self):
        kub_pos = Vector2D(self.kub.get_pos())
        ball_pos = Vector2D(self.kub.state.ballPos)
        self.theta = (ball_pos - kub_pos).angle()
        vec = Vector2D(cos(self.theta),sin(self.theta))
        self.target_point = getPointBehindTheBall(ball_pos,self.theta)
        _GoToPoint_.init(self.kub, self.target_point,self.theta)
        self.kub.dribble(True)
        self.kub.execute()

    def execute_fine_receive(self):
        start_time = rospy.Time.now()
        start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
        generatingfunction = _GoToPoint_.execute(start_time,self.ball_dist_thresh*1.3)
        for gf in generatingfunction:
            self.kub,target = gf
            
            if not vicinity_points(target,self.target_point,thresh=BOT_RADIUS*2):
                self.pass_failed = True
                return

        vec = Vector2D(cos(self.theta),sin(self.theta))
        ball_pos = Vector2D(self.kub.state.ballPos)
        self.target_point = ball_pos+vec*self.distance1
        _GoToPoint_.init(self.kub, self.target_point,self.theta)
        start_time = rospy.Time.now()
        start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
        generatingfunction = _GoToPoint_.execute(start_time,self.ball_dist_thresh*1.3)
        for gf in generatingfunction:
            self.kub,target = gf
            
            if not vicinity_points(target,self.target_point,thresh=BOT_RADIUS*2):
                self.pass_failed = True
                return

    def on_exit_fine_receive(self):
        pass

    def on_enter_move_to_point(self):
        rev_pos = Vector2D(self.kub.state.homePos[self.receiver])
        ball_pos = Vector2D(self.kub.state.ballPos)
        vec_ball_bot = rev_pos - ball_pos
        vec_ball_bot = vec_ball_bot*(1/vec_ball_bot.abs(vec_ball_bot))
        self.target_point = ball_pos + vec_ball_bot*self.distance
        self.calculate_theta(self.target_point)
        self.target_point = getPointBehindTheBall(self.target_point,self.theta)
        _GoToPoint_.init(self.kub, self.target_point,self.theta)
        self.kub.dribble(True)
        self.kub.execute()

    def execute_move_to_point(self):
        start_time = rospy.Time.now()
        start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)
        generatingfunction = _GoToPoint_.execute(start_time,self.ball_dist_thresh)
        for gf in generatingfunction:
            self.kub,target = gf
            if not vicinity_points(self.target_point,target,thresh=BOT_RADIUS*2):
                print 'Failed'
                self.pass_failed = True
                break

    def on_exit_move_to_point(self):
        self.kub.move(0,0)

    def on_enter_kick(self):
        pass

    def execute_kick(self):

        self.kub.kick(self.power)
        self.kub.execute()
        self.kub.dribble(False)
        self.kub.execute()
        time.sleep(0.2)
        try:
            self.kub.state = getState(prev_state).stateB
        except rospy.ServiceException, e:
            print("Error ", e)
        if not vicinity_points(self.kub.get_pos(),self.kub.state.ballPos,thresh=BOT_RADIUS):
            self.pass_completed = True
        else:
            self.pass_failed = True

    def on_exit_kick(self):
        pass



class Receive(Main):

    class State(Enum):
        setup_receive = 1
        in_2_alpha = 2
        outside_circles = 3
        inside_circles = 4
        move_on_circle = 5

    def __init__(self,max_vel = MAX_BOT_SPEED/1.2,min_vel = MIN_BOT_SPEED*1.2):

        super(Receive, self).__init__()
        
        self.alpha = math.pi/8

        self.d = BOT_BALL_THRESH*2.5

        self.radius = self.d/(2*sin(self.alpha))

        self.theta = None
        
        self.circle1 = None

        self.circle2 = None

        self.DISTANCE_THRESH = BOT_BALL_THRESH

        self.ROTATION_FACTOR = ROTATION_FACTOR

        self.max_vel = max_vel
        
        self.receive_failed = False

        self.goal_pos = Vector2D(6000,0)

        self.target = None



        self.add_state(Receive.State.setup_receive,Main.State.main)
        self.add_state(Receive.State.in_2_alpha,Main.State.main)
        self.add_state(Receive.State.outside_circles,Main.State.main)
        self.add_state(Receive.State.inside_circles,Main.State.main)
        self.add_state(Receive.State.move_on_circle, Main.State.main)


        self.add_transition(Main.State.Receive,Receive.State.setup_receive,lambda: True,'immediately')
        self.add_transition(Receive.State.setup_receive,Receive.State.in_2_alpha, lambda:self.bot_inside_2alpha() and self.bot_outside_circles() ,'In 2 alpha')
        self.add_transition(Receive.State.setup_receive,Receive.State.outside_circles, lambda:self.bot_outside_circles() and not self.bot_inside_2alpha() ,'Outside circles')
        self.add_transition(Receive.State.setup_receive,Receive.State.move_on_circle, lambda:self.bot_on_circle() ,'On circle')
        self.add_transition(Receive.State.outside_circles,Receive.State.move_on_circle, lambda:self.bot_on_circle() ,'On circle')
        self.add_transition(Receive.State.setup_receive,Receive.State.inside_circles, lambda: not self.bot_inside_2alpha() and not self.bot_outside_circles() ,'Inside circles')
        self.add_transition(Receive.State.outside_circles,Receive.State.setup_receive,lambda:self.receive_failed, 'Outside Circle failed')
        self.add_transition(Receive.State.inside_circles,Receive.State.setup_receive,lambda:self.receive_failed, 'Inside Circle failed')
        self.add_transition(Receive.State.in_2_alpha,Receive.State.setup_receive,lambda:self.receive_failed, 'In 2 alpha failed')
        self.add_transition(Receive.State.in_2_alpha,Main.State.main,lambda:self.receive_completed, 'In 2 alpha completed')
        self.add_transition(Receive.State.move_on_circle,Receive.State.setup_receive,lambda:self.receive_failed, 'Move on Circle failed')
        self.add_transition(Receive.State.move_on_circle,Main.State.main,lambda:self.at_target_point() or self.receive_completed, 'Completed')
        self.add_transition(Receive.State.inside_circles,Main.State.main,lambda:self.receive_completed or self.at_target_point(), 'Inside Circle completed')

    
    def bot_inside_2alpha(self):
        if self.theta > math.pi - self.alpha:
            return True
        return False

    def bot_outside_circles(self):
        kub_pos = Vector2D(self.kub.get_pos().x, self.kub.get_pos().y)

        if kub_pos.dist(self.circle1.center)>self.radius+50 and kub_pos.dist(self.circle2.center) > self.radius+50:
            return True

        return False

    def bot_on_circle(self):
        ERROR_RADIUS = 50
        kub_pos = Vector2D(self.kub.get_pos().x, self.kub.get_pos().y)

        if abs(kub_pos.dist(self.circle1.center)-self.radius)<=ERROR_RADIUS or abs(kub_pos.dist(self.circle2.center) - self.radius)<=ERROR_RADIUS:
            return True

        return False

    def at_target_point(self):
        kub_pos = Vector2D(self.kub.get_pos().x, self.kub.get_pos().y)
        ball_pos = Vector2D(self.kub.state.ballPos)
        if kub_pos.dist(ball_pos)<=BOT_BALL_THRESH*1.1:
            return True
        return False

    def add_ball_dir(self):
        # recv_pos = Vector2D(self.kub.state.homePos[self.receiver])

        ball_pos = Vector2D(self.kub.state.ballPos)
        ball_vel = Vector2D(self.kub.state.ballVel.x, self.kub.state.ballVel.y)
        ball_vel = self.goal_pos - ball_pos
        theta = functions.normalize_angle(self.kub.get_pos().theta)
        print 'Theta: ' + str(theta)
        ball_vel = Vector2D(cos(theta),sin(theta)) 
        self.ball_vel_dir = ball_vel*(1/ball_vel.abs(ball_vel))

    def calcTheta(self):          
        kub_pos = Vector2D(self.kub.get_pos().x, self.kub.get_pos().y)
        vec_ball_bot = kub_pos - self.target
        dot_temp = vec_ball_bot.dot(self.ball_vel_dir)
        final = dot_temp / vec_ball_bot.abs(vec_ball_bot)
        self.theta = math.acos(final)

    def calcCenter(self):
        ball_vel_dir = self.ball_vel_dir
        
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


    def add_target(self):
        ball_pos = Vector2D(self.kub.state.ballPos.x,self.kub.state.ballPos.y)
        ball_vel = self.ball_vel_dir

        self.target = ball_pos - self.d*(ball_vel)

    def decide_tangent(self):
        kub_pos = Vector2D(self.kub.get_pos().x, self.kub.get_pos().y)
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

        tangent = self.point_on_circle - kub_pos
        self.tangent = tangent*(1/ tangent.abs(tangent))

    def add_point_on_arc(self):
        bot_pos = self.kub.get_pos()
        a1,b1,c1 = 2*(self.target.x-bot_pos.x), 2*(self.target.y-bot_pos.y), bot_pos.x**2+bot_pos.y**2-self.target.x**2-self.target.y**2
        a2,b2,c2 = 2*(self.kub.state.ballPos.x-bot_pos.x), 2*(self.kub.state.ballPos.y-bot_pos.y), bot_pos.x**2+bot_pos.y**2-self.kub.state.ballPos.x**2-self.kub.state.ballPos.y**2

        x = (b1*c2-b2*c1)/(a1*b2-a2*b1)
        y = (c1*a2-c2*a1)/(a1*b2-a2*b1)
        self.center_arc = Vector2D()
        self.center_arc.x = x
        self.center_arc.y = y
        self.radius_arc = dist(self.center_arc, self.target)

    def update_all(self):
        try:
            self.kub.state = getState(prev_state).stateB
        except rospy.ServiceException, e:
            print("Error ", e)

        kub_pos = Vector2D(self.kub.get_pos())
        self.add_target()
        self.center1 = self.center1 + (self.target - self.prev_target)
        self.circle1.center = self.center1
        self.center2 = self.center2 + (self.target - self.prev_target)
        self.circle2.center = self.center2

        if self.bot_outside_circles() and not self.bot_inside_2alpha():
            self.decide_tangent()

        if self.theta < math.pi - self.alpha:
            if self.decided_circle == 'circle1':
                self.to_move_circle = self.circle1
            else:
                self.to_move_circle = self.circle2

        self.prev_target = self.target

    def on_enter_setup_receive(self):
        self.receive_failed = False

    def execute_setup_receive(self):
        # time.sleep(0.1)
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

    def on_exit_setup_receive(self):
        pass

    def on_enter_in_2_alpha(self):
        pass

    def execute_in_2_alpha(self):
        prev_vel = 0
        while True:
            
            k = 2
            kub_pos = Vector2D(self.kub.state.homePos[self.kub.kubs_id].x, self.kub.state.homePos[self.kub.kubs_id].y)
            vec_tar_kub = self.target-kub_pos
            bot_target_vec = vec_tar_kub*(1/vec_tar_kub.abs(vec_tar_kub))
            direct_vel = bot_target_vec*self.max_vel
            ball_vel = Vector2D(self.kub.state.ballVel.x, self.kub.state.ballVel.y)
            vel_max_given = direct_vel - ball_vel
            dist = kub_pos.dist(self.target)
            vel_rad = 2*dist - 0.08 * prev_vel
            vel = min(vel_max_given.abs(vel_max_given),vel_rad)
            
            
            velf = vel*bot_target_vec + ball_vel
            
            # rotate = self.ball_vel_dir.angle()
            # vw = Get_Omega(self.kub.kubs_id,rotate,self.kub.state.homePos)

            # if abs(normalize_angle(self.kub.state.homePos[self.kub.kubs_id].theta-rotate))<ROTATION_FACTOR:
            #     self.kub.turn(0)
            # else:
            #     self.kub.turn(vw)

            self.kub.move(velf.x,velf.y)

            if self.target.dist(kub_pos) < 50:
                self.kub.move(0,0)

            self.kub.execute()
            
            if  self.target.dist(kub_pos) < 50: #and abs(normalize_angle(self.kub.state.homePos[self.kub.kubs_id].theta-rotate))<ROTATION_FACTOR:
                self.kub.move(0,0)
                self.kub.execute()
                self.receive_completed = True
                break


            if not self.bot_inside_2alpha() or not self.bot_outside_circles():
                self.receive_failed = True
                break
            
            prev_vel = velf.abs(velf)
            self.update_all()
           

    def on_exit_in_2_alpha(self):
        pass

    def line(self,x,y):
        value = (y - self.target.y) - tan(functions.normalize_angle(self.ball_vel_dir.angle()))*(x - self.target.x)
        return value

    def on_enter_outside_circles(self):
        kub_pos = Vector2D(self.kub.get_pos().x, self.kub.get_pos().y)
        val1 = self.line(kub_pos.x,kub_pos.y)
        val2 = self.line(self.center1.x,self.center1.y)
        if val1*val2>0:
            self.to_move_circle = self.circle1
            self.decided_circle = 'circle1'
        else:
            self.to_move_circle = self.circle2
            self.decided_circle = 'circle2'

        self.decide_tangent()
        self.update_all()

    def execute_outside_circles(self):
        prev_vel = 0
        while True:
            kub_pos = Vector2D(self.kub.state.homePos[self.kub.kubs_id].x, self.kub.state.homePos[self.kub.kubs_id].y)
            direct_vel = self.tangent*self.max_vel

            # print 'TANGENT: '+ str(self.tangent.x) + ',' + str(self.tangent.y)
            ball_vel = Vector2D(self.kub.state.ballVel.x, self.kub.state.ballVel.y)
            vel_max_given = direct_vel - ball_vel
            dist = kub_pos.dist(self.target)
            vel_rad = 2*dist - 0.08 * prev_vel
            vel = min(vel_max_given.abs(vel_max_given),vel_rad)

            # rotate = self.ball_vel_dir
            # rotate = rotate.angle()
            # vw = Get_Omega(self.kub.kubs_id,rotate,self.kub.state.homePos)

            # if abs(normalize_angle(self.kub.state.homePos[self.kub.kubs_id].theta-rotate))<ROTATION_FACTOR:
            #     self.kub.turn(0)
            # else:
            #     self.kub.turn(vw)
            
            velf = vel*self.tangent + ball_vel
            

            if self.point_on_circle.dist(kub_pos)< 100:
                self.kub.move(0,0)
            else:
                self.kub.move(velf.x,velf.y)

            self.kub.execute()

            if  self.point_on_circle.dist(kub_pos)< 100: #and abs(normalize_angle(self.kub.state.homePos[self.kub.kubs_id].theta-rotate))<ROTATION_FACTOR:
                self.kub.execute()
                break


            if not self.bot_outside_circles() or self.bot_inside_2alpha():
                
                self.receive_failed = True
                break

            # print 'Dist: ' + str(kub_pos.dist(self.point_on_circle))

            prev_vel = velf.abs(velf)
            self.update_all()

    def on_exit_outside_circles(self):
        pass

    def on_enter_move_on_circle(self):
        kub_pos = Vector2D(self.kub.get_pos().x, self.kub.get_pos().y)                    
        
        self.kub.dribble(True)
        self.update_all()
        self.add_point_on_arc()
        _GoOnArc_.init(self.kub,self.kub.state.ballPos,self.center_arc,self.radius_arc,False,self.target,False) 

    def execute_move_on_circle(self):
        start_time = rospy.Time.now()
        start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)
        generatingfunction = _GoOnArc_.execute(start_time,self.DISTANCE_THRESH, self.ROTATION_FACTOR)
        for gf in generatingfunction:
            self.kub,target = gf	
            self.update_all()
            _GoOnArc_.TARGET = self.kub.state.ballPos
            _GoOnArc_.POINT = self.target
            
            if self.at_target_point():
                self.kub.move(0,0)
                self.kub.execute()
                self.receive_completed = True
                break

            if self.bot_outside_circles():
                self.kub.dribble(False)
                self.receive_failed = True


    
    def on_exit_move_on_circle(self):
        self.kub.move(0,0)
        self.kub.execute()

    def on_enter_inside_circles(self):
        kub_pos = Vector2D(self.kub.get_pos().x, self.kub.get_pos().y)
        
        if kub_pos.dist(self.center1) < kub_pos.dist(self.center2):
            self.to_move_circle = self.circle1
            self.decided_circle = 'circle1'
        else:
            self.to_move_circle = self.circle2
            self.decided_circle = 'circle2'

        self.update_all()
        self.kub.dribble(True)
        self.center_arc = None
        self.add_point_on_arc()
        _GoOnArc_.init(self.kub,self.kub.state.ballPos,self.center_arc,self.radius_arc,False,self.target,False)

    def execute_inside_circles(self):
        start_time = rospy.Time.now()
        start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)
        generatingfunction = _GoOnArc_.execute(start_time,self.DISTANCE_THRESH, self.ROTATION_FACTOR)
        for gf in generatingfunction:
            self.kub,target = gf
            self.update_all()
            _GoOnArc_.TARGET = self.kub.state.ballPos
            _GoOnArc_.POINT = self.target

            if self.at_target_point():
                self.receive_completed = True
                break

            if self.bot_outside_circles():
                self.kub.dribble(False)
                self.receive_failed = True
                break

    def on_exit_inside_circles(self):
        pass

class Goal(Main):

    class State(Enum):
        setup_goal = 1
        receive_ball = 2
        kick_goal = 3

    def __init__(self):
        
        super(Goal,self).__init__()
        
        self.power = 7

        self.goal = Vector2D(6000,0)

        self.ball_dist_thresh = BOT_BALL_THRESH

        self.goal_failed = False

        self.goal_completed = False

        self.add_state(Goal.State.setup_goal,Main.State.Goal)
        self.add_state(Goal.State.receive_ball,Main.State.Goal)
        self.add_state(Goal.State.kick_goal,Main.State.Goal)

        self.add_transition(Main.State.Goal,Goal.State.setup_goal,lambda :True, 'Immediately')
        self.add_transition(Goal.State.setup_goal,Goal.State.receive_ball,lambda :not self.at_ball_pos(),'Receive_Ball')
        self.add_transition(Goal.State.receive_ball,Goal.State.kick_goal,lambda :self.at_ball_pos(),'Kick_Goal')
        self.add_transition(Goal.State.receive_ball,Goal.State.setup_goal,lambda :self.goal_failed,'Goal_Failed')
        self.add_transition(Goal.State.kick_goal,Goal.State.setup_goal,lambda :self.goal_failed,'Goal_Failed')
        self.add_transition(Goal.State.kick_goal,Main.State.main,lambda :self.goal_completed,'Goal_Completed')

    def at_ball_pos(self):
        return vicinity_points(self.kub.state.ballPos,self.kub.get_pos(),thresh= self.ball_dist_thresh*2) and abs(normalize_angle(self.kub.get_pos().theta-self.theta)) <= ROTATION_FACTOR

    def calcTheta(self):
        self.theta = math.atan2(self.goal.y-self.kub.state.ballPos.y,self.goal.x-self.kub.state.ballPos.x)

    def on_enter_setup_goal(self):
        pass

    def execute_setup_goal(self):
        self.calcTheta()

    def on_exit_setup_goal(self):
        pass

    def on_enter_receive_ball(self):
        self.calcTheta()
        self.target = getPointBehindTheBall(self.kub.state.ballPos,self.theta)
        _GoToPoint_.init(self.kub, self.target,self.theta)
        self.kub.dribble(True)
        self.kub.execute()

    def execute_receive_ball(self):
        start_time = rospy.Time.now()
        start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
        generatingfunction = _GoToPoint_.execute(start_time,self.ball_dist_thresh*1.3)
        for gf in generatingfunction:
            self.kub,target = gf
            
            if not vicinity_points(target,self.target,thresh=BOT_RADIUS*2):
                self.goal_failed = True
                break

    def on_exit_receive_ball(self):
        self.kub.dribble(True)
        self.kub.execute()

    def on_enter_kick_goal(self):
        pass

    def execute_kick_goal(self):
        self.kub.kick(self.power)
        self.kub.execute()
        time.sleep(0.1)

        if not vicinity_points(self.kub.state.ballPos,self.target,thresh=BOT_RADIUS*2):
            self.goal_completed = True
        else:
            self.goal_failed = True

    def on_exit_kick_goal(self):
        pass




class PassReceive(Receive,Pass,Goal,Main):


    def __init__(self):

        super(PassReceive,self).__init__()


        





     









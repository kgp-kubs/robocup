class Intercept(behavior.Behavior):
   
    class State(Enum):
        setup = 1
        inside_2a = 2
        outside_circle = 3
        inside_circle = 4

    def _init_(self,course_approch_thresh =  DISTANCE_THRESH/3,continuous=False):

        super(Intercept,self)._init_()

        self.name = "Intercept"

        self.target_point = None
       

        self.ball_dist_thresh = 2*BOT_BALL_THRESH

        self.behavior_failed = False


        self.add_state(Intercept.State.setup,
            behavior.Behavior.State.running)

        self.add_state(Intercept.State.inside_2a,
            behavior.Behavior.State.running)
       
        self.add_state(Intercept.State.outside_circle,
            behavior.Behavior.State.running)

        self.add_state(Intercept.State.inside_circle,
            behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
            Intercept.State.setup,lambda: True,'immediately')

        self.add_transition(Intercept.State.setup,
           Intercept.State.inside_2a,lambda:self.kub_within_2a() ,'within 2a')

        self.add_transition(Intercept.State.setup,
            Intercept.State.outside_circle,lambda: self.kub_outside_circle() ,'outside circle')

        self.add_transition(Intercept.State.setup,
            Intercept.State.inside_circle,lambda: self.kub_inside_circle() ,'inside circle')

        self.add_transition(Intercept.State.outside_circle,
            Intercept.State.inside_circle,lambda: self.kub_along_circle(),'along circle')

        self.add_transition(Intercept.State.inside_2a,
            Intercept.State.setup,lambda: self.behavior_failed,'failed')

        self.add_transition(Intercept.State.outside_circle,
            Intercept.State.setup,lambda: self.behavior_failed,'failed')

        self.add_transition(Intercept.State.inside_circle,
            Intercept.State.setup,lambda: self.behavior_failed,'failed')

from transitions import Machine

import numpy as np
import RobotDART as rd
import dartpy  # OSX breaks if this is imported before RobotDART
import copy
from utils import create_grid, create_problems
from utils_1 import damped_pseudoinverse


class PITask:
    def __init__(self, target, dt, Kp , Ki ):
        self._target = target
        self._dt = dt
        self._Kp = Kp
        self._Ki = Ki
        self._sum_error = 0
    
    def set_target(self, target):
        self._target = target
    
    # function to compute error
    def error(self, tf):
        rot_error = rd.math.logMap(self._target.rotation() @ tf.rotation().T)
        lin_error = self._target.translation() - tf.translation()
        return np.r_[rot_error, lin_error]
    
    def update(self, current):
        error_in_world_frame = self.error(current)

        self._sum_error = self._sum_error + error_in_world_frame * self._dt

        return self._Kp * error_in_world_frame + self._Ki * self._sum_error
    
        
dt = 0.001 # you are NOT allowed to change this
simulation_time = 27.0 # you are allowed to change this
total_steps = int(simulation_time / dt)


box_positions = create_grid()

box_size = [0.04, 0.04, 0.04]

# Red Box
# Random cube position
red_box_pt = np.random.choice(len(box_positions))
box_pose = [0., 0., 0., box_positions[red_box_pt][0], box_positions[red_box_pt][1], box_size[2] / 2.0]
red_box = rd.Robot.create_box(box_size, box_pose, "free", 0.1, [0.9, 0.1, 0.1, 1.0], "red_box")

# Green Box
# Random cube position
green_box_pt = np.random.choice(len(box_positions))
while green_box_pt == red_box_pt:
    green_box_pt = np.random.choice(len(box_positions))
box_pose = [0., 0., 0., box_positions[green_box_pt][0], box_positions[green_box_pt][1], box_size[2] / 2.0]
green_box = rd.Robot.create_box(box_size, box_pose, "free", 0.1, [0.1, 0.9, 0.1, 1.0], "green_box")

# Blue Box
# Random cube position
box_pt = np.random.choice(len(box_positions))
while box_pt == green_box_pt or box_pt == red_box_pt:
    box_pt = np.random.choice(len(box_positions))
box_pose = [0., 0., 0., box_positions[box_pt][0], box_positions[box_pt][1], box_size[2] / 2.0]
blue_box = rd.Robot.create_box(box_size, box_pose, "free", 0.1, [0.1, 0.1, 0.9, 1.0], "blue_box")


# PROBLEM DEFINITION
# Choose problem
problems = create_problems()
problem_id = np.random.choice(len(problems))
problem = problems[problem_id]

print('We want to put the', problem[2], 'cube on top of the', problem[1], 'and the', problem[1], 'cube on top of the', problem[0], 'cube.')

######################## SOSTO MEXRI EDW ############################
class MyRobot:
    def __init__(self, dt):
        self.robot = rd.Franka(int(1. / dt))
        self.init_position = [0., np.pi / 4., 0., -np.pi / 4., 0., np.pi / 2., 0., 0.04, 0.04]
        self.robot.set_positions(self.init_position)
        self.x=0
        self.i=0
        # print("1")
        #print(self.close_finger)

        max_force = 5.
        self.robot.set_force_lower_limits([-max_force, -max_force], ["panda_finger_joint1", "panda_finger_joint2"])
        self.robot.set_force_upper_limits([max_force, max_force], ["panda_finger_joint1", "panda_finger_joint2"])
        #########################################################
        self.robot.set_actuator_types("servo") # you can use torque here
        # get end-effector pose
        self.eef_link_name = "panda_ee"        
        self.tf_desired = self.robot.body_pose(self.eef_link_name)        
        self.vec_desired = self.robot.body_pose_vec(self.eef_link_name)
        self.target_positions = copy.copy(self.robot.positions())
        self.y=0

        self.dt = dt
        self.Kp = 2. # Kp could be an array of 6 values
        self.Ki = 0.01 # Ki could be an array of 6 values
        self.controller = PITask(self.tf_desired, self.dt, self.Kp, self.Ki)
        #self.Jcontrol=PIJoint(self.target_positions, self.dt, self.Kp, self.Ki)
        
    def close(self):
       self.robot.set_commands([-0.2], ['panda_finger_joint1'])
       
    def open(self):
       self.robot.set_commands([0.4], ['panda_finger_joint1'])
      
    def condition(self):
       return True
    
    def update(self):
        tf = self.robot.body_pose(self.eef_link_name)
        vel = self.controller.update(tf)
        jac = self.robot.jacobian(self.eef_link_name) # this is in world frame
        jac_pinv = damped_pseudoinverse(jac)  # get pseudo-inverse
        cmd = jac_pinv @ vel

        self.robot.set_commands(cmd)   

    def target(self):
        return self.controller._target
    
    def reached_target(self):
        tf = self.robot.body_pose(self.eef_link_name)
        err = self.controller.error(tf)
        if np.linalg.norm(err) < 1e-3:
            return True
        return False
        
    def success(self):
       return True


    def on_enter_A(self):
        tf = dartpy.math.Isometry3()
        tf.set_translation([self.tf_desired.translation()[0]-0.2 , self.tf_desired.translation()[1], self.tf_desired.translation()[2] -0.1])
        tf.set_rotation(self.tf_desired.rotation())
        self.controller = PITask(tf, self.dt, self.Kp, self.Ki)

    def on_enter_B(self):          #3
        count=[2,3]
        print("Going for 1 ",problem[0])
        ###Topothesia
        self.tf2=dartpy.math.Isometry3()
        self.tf2.set_rotation(self.tf_desired.rotation())
        if problem[0]=='red':
         self.tf2.set_translation([box_positions[red_box_pt][0], box_positions[red_box_pt][1],count[self.i]* box_size[2]] )      
        elif problem[0]=='green':
         self.tf2.set_translation([box_positions[green_box_pt][0], box_positions[green_box_pt][1],count[self.i]* box_size[2]] )
        else:
         self.tf2.set_translation([box_positions[box_pt][0], box_positions[box_pt][1],count[self.i]* box_size[2] ])      
        self.i=self.i+1

        self.controller = PITask(self.tf2, self.dt, self.Kp, self.Ki)
        
    def on_enter_C(self):
        self.x=0        #4
        self.y=1
        self.tf2.set_translation([self.tf2.translation()[0], self.tf2.translation()[1], self.tf2.translation()[2] +0.01])
        self.controller = PITask(self.tf2, self.dt, self.Kp, self.Ki) 

    def on_enter_D(self):    
        self.y=0   #5
        self.tf2.set_translation([self.tf2.translation()[0], self.tf2.translation()[1], self.tf2.translation()[2] +0.1])
        self.controller = PITask(self.tf2, self.dt, self.Kp, self.Ki)            


    def on_enter_E(self):            #1
        
        print("Going for 2 ",problem[1])
        self.tf1 = dartpy.math.Isometry3()
        self.tf1.set_rotation(self.tf_desired.rotation())
        if problem[1]=='red':
         self.tf1.set_translation([box_positions[red_box_pt][0], box_positions[red_box_pt][1],1/2* box_size[2]] )      
        elif problem[1]=='green':
         self.tf1.set_translation([box_positions[green_box_pt][0], box_positions[green_box_pt][1],1/2* box_size[2]] )
        else:
         self.tf1.set_translation([box_positions[box_pt][0], box_positions[box_pt][1],1/2* box_size[2] ])
        
        self.controller = PITask(self.tf1, self.dt, self.Kp, self.Ki) 

    def on_enter_F(self):           #2
         self.x=1
         self.tf1.set_translation([self.tf1.translation()[0], self.tf1.translation()[1], self.tf1.translation()[2] +0.01])
         self.controller = PITask(self.tf1, self.dt, self.Kp, self.Ki) 


    def on_enter_G(self):             #6
        self.x=0
        self.y=0
        print("Going for 3 ",problem[2])
        self.tf3 = dartpy.math.Isometry3()
        self.tf3.set_rotation(self.tf_desired.rotation())
        if problem[2]=='red':
         self.tf3.set_translation([box_positions[red_box_pt][0], box_positions[red_box_pt][1],4* box_size[2]] )      
        elif problem[2]=='green':
         self.tf3.set_translation([box_positions[green_box_pt][0], box_positions[green_box_pt][1],4*box_size[2]] )
        else:
         self.tf3.set_translation([box_positions[box_pt][0], box_positions[box_pt][1],4*box_size[2] ])
        
        self.controller = PITask(self.tf3, self.dt, self.Kp, self.Ki)

    def on_enter_I(self):         
       self.tf3.set_translation([self.tf3.translation()[0], self.tf3.translation()[1], 1/2*box_size[2] ])
       self.controller = PITask(self.tf3, self.dt, self.Kp, self.Ki)   
    
    def on_enter_H(self): 
       self.x=1           #7
       self.tf3.set_translation([self.tf3.translation()[0], self.tf3.translation()[1], self.tf3.translation()[2]+0.01 ])
       self.controller = PITask(self.tf3, self.dt, self.Kp, self.Ki) 
    
    def on_enter_J(self):
       self.tf3.set_translation([self.tf3.translation()[0], self.tf3.translation()[1], self.tf3.translation()[2]+0.2 ])
       self.controller = PITask(self.tf3, self.dt, self.Kp, self.Ki) 

       



robot = MyRobot(dt)
goal_robot = rd.Robot.create_ellipsoid(dims=[0.15, 0.15, 0.15], pose=robot.vec_desired, color=[0., 1., 0., 0.5], ellipsoid_name="target")

# Create Graphics
gconfig = rd.gui.Graphics.default_configuration()
gconfig.width = 1280 # you can change the graphics resolution
gconfig.height = 960 # you can change the graphics resolution
graphics = rd.gui.Graphics(gconfig)

# Create simulator object
simu = rd.RobotDARTSimu(dt)
simu.set_collision_detector("fcl") # you can use bullet here
simu.set_control_freq(100)
simu.set_graphics(graphics)
graphics.look_at((0., 4.5, 2.5), (0., 0., 0.25))
simu.add_checkerboard_floor()
simu.add_robot(robot.robot)
simu.add_robot(red_box)
simu.add_robot(blue_box)
simu.add_robot(green_box)
simu.add_visual_robot(goal_robot)
#########################################################

FSM_states = ['A', 'B', 'C','D','E','F','G','H','I','J']

# And some transitions between states. We're lazy, so we'll leave out
# the inverse phase transitions (freezing, condensation, etc.).
FSM_transitions = [
    { 'trigger': 'arrived', 'source' : 'A', 'dest': 'E', 'conditions' : 'reached_target'},
    { 'trigger': 'arrived', 'source' : 'E', 'dest': 'F', 'conditions' : 'reached_target'},
    { 'trigger': 'arrived', 'source' : 'F', 'dest': 'B', 'conditions' : 'reached_target'},
    {'trigger' : 'arrived', 'source' : 'B', 'dest': 'C', 'conditions' : 'reached_target'},
    {'trigger' : 'arrived', 'source' : 'C', 'dest': 'D', 'conditions' : 'reached_target'},
    {'trigger' : 'arrived', 'source' : 'D', 'dest': 'G', 'conditions' : 'reached_target'},
    {'trigger' : 'arrived', 'source' : 'G', 'dest': 'I', 'conditions' : 'reached_target'},
    {'trigger' : 'arrived', 'source' : 'I', 'dest': 'H', 'conditions' : 'reached_target'},
    {'trigger' : 'arrived', 'source' : 'H', 'dest': 'J', 'conditions' : 'reached_target'},
    {'trigger' : 'arrived', 'source' : 'J', 'dest': 'B', 'conditions' : 'reached_target'},
    {'trigger' : 'arrived', 'source' : 'B', 'dest': 'C', 'conditions' : 'reached_target'},
    {'trigger' : 'arrived', 'source' : 'C', 'dest': 'A', 'conditions' : 'reached_target'}

]

machine = Machine(model=robot, states=FSM_states, transitions=FSM_transitions, initial='A')


for step in range(total_steps):
    if (simu.schedule(simu.control_freq())):
        goal_robot.set_base_pose(robot.target())
        
        
        robot.update()


        if robot.x==1:
           robot.close()
           #wait=1
        else:
           print("")
        # tick FSM
        if robot.y==1:
           robot.open()
           #wait=1
        else:
           print("")
        
        robot.arrived() 
        pass

    if (simu.step_world()):
        break


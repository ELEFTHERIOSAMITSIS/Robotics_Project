import numpy as np
import RobotDART as rd
import dartpy  # OSX breaks if this is imported before RobotDART
import copy
from utils_1 import damped_pseudoinverse, AdT, enforce_joint_limits
from utils import create_grid, create_problems
from utils_2 import angle_wrap_multi

class PIJoint:
    def __init__(self, target, dt, Kp = 10., Ki = 0.1):
        self._target = target
        self._dt = dt
        self._Kp = Kp
        self._Ki = Ki
        self._sum_error = 0.

    def set_target(self, target):
        self._target = target

    def update(self, current):
        error = angle_wrap_multi(self._target - current) # since we have angles, it's better to wrap into [-pi,pi)
        self._sum_error = self._sum_error + error * self._dt

        return self._Kp * error + self._Ki * self._sum_error
    # def close(self):
    #   self.target_positions = copy.copy(self.robot.positions())
    #   self.target_positions[7] = 0.01
    #   self.Jcontrol=PIJoint(self.target_positions, self.dt, self.Kp, self.Ki)
    #   cmd =self.Jcontrol.update(self.robot.positions())
    #   self.robot.set_commands(cmd)




dt = 0.001 # you are NOT allowed to change this
simulation_time = 20.0 # you are allowed to change this
total_steps = int(simulation_time / dt)


robot = rd.Franka(int(1. / dt))
init_position = [0., np.pi / 4., 0., -np.pi / 4., 0., np.pi / 2., 0.0, 0.0, 0.04]
robot.set_positions(init_position)
######################################################
#print(init_position)
print(robot.joint_names())
eef_link_name = "panda_leftfinger"
tf_desired =robot.body_pose(eef_link_name)
print(tf_desired)  
#tf_desired.set_translation([box_positions[red_box_pt][0],tf_desired.translation()[1],tf_desired.translation()[2]])
#print(tf_desired)  
#tf_desired = robot.body_pose(eef_link_name)
        
 ######################################################       
        
max_force = 5.
robot.set_force_lower_limits([-max_force, -max_force], ["panda_finger_joint1", "panda_finger_joint2"])
robot.set_force_upper_limits([max_force, max_force], ["panda_finger_joint1", "panda_finger_joint2"])
#########################################################
robot.set_actuator_types("servo") # you can use torque here

#########################################################
# DO NOT CHANGE ANYTHING IN HERE
# Create boxes
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
box_pose = [0, 0., 0., box_positions[green_box_pt][0], box_positions[green_box_pt][1], box_size[2] / 2.0]
green_box = rd.Robot.create_box(box_size, box_pose, "free", 0.1, [0.1, 0.9, 0.1, 1.0], "green_box")

# Blue Box
# Random cube position
box_pt = np.random.choice(len(box_positions))
while box_pt == green_box_pt or box_pt == red_box_pt:
    box_pt = np.random.choice(len(box_positions))
box_pose = [0., 0., 0., box_positions[box_pt][0], box_positions[box_pt][1], box_size[2] / 2.0]
blue_box = rd.Robot.create_box(box_size, box_pose, "free", 0.1, [0.1, 0.1, 0.9, 1.0], "blue_box")
#########################################################


#########################################################
# PROBLEM DEFINITION
# Choose problem
problems = create_problems()
problem_id = np.random.choice(len(problems))
problem = problems[problem_id]

print('We want to put the', problem[2], 'cube on top of the', problem[1], 'and the', problem[1], 'cube on top of the', problem[0], 'cube.')
#########################################################

#########################################################
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
simu.add_robot(robot)
simu.add_robot(red_box)
simu.add_robot(blue_box)
simu.add_robot(green_box)
#########################################################
Kp = 1.
Ki = 0.01
target_positions = copy.copy(robot.positions())
target_positions[7] = 0.01
controller1 = PIJoint(target_positions, dt, Kp, Ki)

while True:
    if simu.step_world():
        break
    v = controller1.update(robot.positions()[1])
    robot.set_commands([0.4], ['panda_finger_joint1'])
    #cmd = controller.update(robot.positions())
    #robot.set_commands(cmd)


import numpy as np
import RobotDART as rd

from utils import create_grid, create_problems

dt = 0.001 # you are NOT allowed to change this
simulation_time = 20.0 # you are allowed to change this
total_steps = int(simulation_time / dt)

#########################################################
# DO NOT CHANGE ANYTHING IN HERE
# Create robot
robot = rd.Franka(int(1. / dt))
init_position = [0., np.pi / 4., 0., -np.pi / 4., 0., np.pi / 2., np.pi / 4.0, 0.04, 0.04]
robot.set_positions(init_position)
######################################################
#print(init_position)
print(robot.body_names())
eef_link_name = "panda_finger_joint1"
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

for step in range(total_steps):
    if (simu.schedule(simu.control_freq())):
        # Control here!
        pass

    if (simu.step_world()):
        break

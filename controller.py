"""lab3 controller."""
# Copyright University of Colorado Boulder 2021
# CSCI 3302 "Introduction to Robotics" Lab 3 Base Code.

from controller import Robot, Motor
import math


# TODO: Fill out with correct values from Robot Spec Sheet (or inspect PROTO definition for the robot)
MAX_SPEED = 2.84 # [rad/s]
MAX_SPEED_MS = 0.22 # [m/s]
AXLE_LENGTH = 0.16 # [m]



MOTOR_LEFT = 0 # Left wheel index
MOTOR_RIGHT = 1 # Right wheel index

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# The Turtlebot robot has two motors
part_names = ("left wheel motor", "right wheel motor")


# Set wheels to velocity control by setting target position to 'inf'
# You should not use target_pos for storing waypoints. Leave it unmodified and 
# use your own variable to store waypoints leading up to the goal
target_pos = ('inf', 'inf') 
robot_parts = []

for i in range(len(part_names)):
        robot_parts.append(robot.getDevice(part_names[i]))
        robot_parts[i].setPosition(float(target_pos[i]))

# Odometry
pose_x     = 3.30
pose_y     = 8.72
pose_theta = 0

# Rotational Motor Velocity [rad/s]
vL = 0
vR = 0



# checkpoints information
start = (3.30 , 8.72) # initial (x,y) for robot odometry 

goal = (3.9 , 3.2)
checkpoints = [ (4.8 , 7.7) , (5.5 , 6.8) , (5.5 , 5.5) , (4.0 , 4.6) , goal ]


current_waypoint = 0
# checkpoints(current_waypoint) gets the tupple of the waypoint we're heading towards


# TODO
# Create you state and goals (waypoints) variable here
# You have to MANUALLY figure out the waypoints, one sample is provided for you in the instructions

while robot.step(timestep) != -1:

    # STEP 2.1: Calculate sources of error
    
    
    p = math.dist((pose_x, pose_y), checkpoints[current_waypoint])
    a = math.atan2(checkpoints[current_waypoint][1] - pose_y, checkpoints[current_waypoint][0] - pose_x) - pose_theta
    n = math.atan2(checkpoints[current_waypoint +1][1] - checkpoints[current_waypoint][1], checkpoints[current_waypoint + 1][0] - checkpoints[current_waypoint][0]) - pose_theta
       
    print(checkpoints[current_waypoint])
    #goal_theta - pose_theta  
    if a < -3.1415: a += 6.283
    if n < -3.1415: n += 6.283
  
    print(p, a, n)

    # STEP 2.2: Feedback Controller
    dx_r =  p/(a**2 + 1)
    
    dTheta = a**2 * a/abs(a) 
    
    print(dTheta)
    # STEP 1: Inverse Kinematics Equations (vL and vR as a function dX and dTheta)
    # Note that vL and vR in code is phi_l and phi_r on the slides/lecture
    vL = ((dx_r - ((dTheta * AXLE_LENGTH) / 2)) / (MAX_SPEED_MS / MAX_SPEED))
    vR = ((dx_r + ((dTheta * AXLE_LENGTH) / 2)) / (MAX_SPEED_MS / MAX_SPEED))
    
    # STEP 2.3: Proportional velocities
    if (vL/vR <= 1):
        vL = (vL/vR) * MAX_SPEED
        vR = MAX_SPEED
    else:
        vL = MAX_SPEED
        vR = (vR/vL) * MAX_SPEED

    # STEP 2.4: Clamp wheel speeds
    if ( robot_parts[MOTOR_LEFT].getMaxVelocity() > MAX_SPEED):
        robot_parts[MOTOR_LEFT].setVelocity(MAX_SPEED)
    if (robot_parts[MOTOR_RIGHT].getMaxVelocity() > MAX_SPEED):
        robot_parts[MOTOR_RIGHT].setVelocity(MAX_SPEED)     
        
    # pass

    print(current_waypoint)
    # condition to change current_waypoint
    if p <= 0.02:
        current_waypoint = current_waypoint + 1

    if current_waypoint >= 6 : 
    # stop motion
        vL = 0
        vR = 0

         
    
    # TODO
    # Use Your Lab 2 Odometry code after these 2 comments. We will supply you with our code next week 
    # after the Lab 2 deadline but you free to use your own code if you are sure about its correctness
    
    # NOTE that the odometry should ONLY be a function of 
    # (vL, vR, MAX_SPEED, MAX_SPEED_MS, timestep, AXLE_LENGTH, pose_x, pose_y, pose_theta)
    # Odometry code. Don't change speeds (vL and vR) after this line

    
    pose_x += (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.cos(pose_theta)

    pose_y += (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.sin(pose_theta)

    pose_theta += (vL-vR)/AXLE_LENGTH/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0

    print("X: %f Y: %f Theta: %f" % (pose_x, pose_y, pose_theta))
    

    ########## End Odometry Code ##################
    
    ########## Do not change ######################
    # Bound pose_theta between [-pi, 2pi+pi/2]
    # Important to not allow big fluctuations between timesteps (e.g., going from -pi to pi)
    if pose_theta > 6.28+3.14/2: pose_theta -= 6.28
    if pose_theta < -3.14: pose_theta += 6.28
    ###############################################

    # TODO
    # Set robot motors to the desired velocities
    robot_parts[MOTOR_LEFT].setVelocity(vL)
    robot_parts[MOTOR_RIGHT].setVelocity(vR)

    


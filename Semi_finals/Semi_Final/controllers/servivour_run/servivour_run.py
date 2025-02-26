from controller import Robot, Camera
import numpy as np
import math

robot = Robot()
# get the time step of the current world.
TIME_STEP = 4
MAX_SPEED = 6.28

gyro = robot.getDevice("gyro")
gyro.enable(TIME_STEP)
    
    
left_motor = robot.getDevice('left_motor')
right_motor = robot.getDevice('right_motor')

left_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)

right_motor.setPosition(float('inf'))
right_motor.setVelocity(0.0)

# Speeds
wheel_speed = 2.0  # m/s
wheel_base = 0.053  # Distance between e-puck wheels in meters
turn_time = 3.84 # 3.7721 Time for 90-degree turn
cell_time = 6.66
turn_speed = 2
fast_turn = 6
turn_angle = 90
turn_back_angle = 180

camera1 = robot.getDevice('camera1')
camera1.enable(TIME_STEP)
camera2 = robot.getDevice('camera2')
camera2.enable(TIME_STEP)

visited = set()


MAZE_SIZE = 20
cells = np.zeros((MAZE_SIZE, MAZE_SIZE, 5), dtype=int)  # 3D array for walls [N, E, S, W]

# Directions (North, East, South, West)
DIRECTIONS = [(-1, 0), (0, 1), (1, 0), (0, -1)]

# Robot initial position and direction
global robot_position, robot_direction
robot_position = [19, 10]  # (x, y)
robot_direction = 0  # 0 = North, 1 = East, 2 = South, 3 = West

def stop() : #Stop the robot for a 0.5 seconds.
    start_time1 = robot.getTime()
    while robot.step(TIME_STEP) != -1:
        if robot.getTime() - start_time1 > 0.2:
            break  

def right_turn() : #Perform a 90 degree right turn.
    angle_z = 0.0
    left_motor.setVelocity(turn_speed)
    right_motor.setVelocity(-turn_speed)
    while robot.step(TIME_STEP) != -1:
        dt = TIME_STEP / 1000.0  # Convert to seconds
        gyro_values = gyro.getValues()  # Returns [wx, wy, wz] in rad/s
        angle_z += gyro_values[2] * dt
        angle_z_deg = angle_z * (180.0 / 3.14159)
        #print(f" Z: {angle_z_deg:.2f}°")
        if  angle_z_deg <= -(turn_angle-15):
            left_motor.setVelocity(0.3)
            right_motor.setVelocity(-0.3)
        elif  angle_z_deg <= -(turn_angle-5):
            left_motor.setVelocity(0.1)
            right_motor.setVelocity(-0.1)
        if  angle_z_deg <= -(turn_angle):
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)            
            break
            
    stop()


def turn_back() : #Turn back
    angle_z = 0.0
    left_motor.setVelocity(turn_speed)
    right_motor.setVelocity(-turn_speed)
    while robot.step(TIME_STEP) != -1:
        dt = TIME_STEP / 1000.0  # Convert to seconds
        gyro_values = gyro.getValues()  # Returns [wx, wy, wz] in rad/s
        angle_z += gyro_values[2] * dt
        angle_z_deg = angle_z * (180.0 / 3.14159)
        #print(f" Z: {angle_z_deg:.2f}°")
        if  angle_z_deg <= -(turn_back_angle-15):
            left_motor.setVelocity(0.3)
            right_motor.setVelocity(-0.3)
        elif  angle_z_deg <= -(turn_back_angle-5):
            left_motor.setVelocity(0.1)
            right_motor.setVelocity(-0.1)
        if  angle_z_deg <= -turn_back_angle: 
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)           
            break
            
    stop() 
        
        
def left_turn() : #Perform a 90 degree left turn.
    angle_z = 0.0
    left_motor.setVelocity(-turn_speed)
    right_motor.setVelocity(turn_speed)
    while robot.step(TIME_STEP) != -1:
        dt = TIME_STEP / 1000.0  # Convert to seconds
        gyro_values = gyro.getValues()  # Returns [wx, wy, wz] in rad/s
        angle_z += gyro_values[2] * dt
        angle_z_deg = angle_z * (180.0 / 3.14159)
        #print(f" Z: {angle_z_deg:.2f}°")
        if  angle_z_deg >= turn_angle-15:
            left_motor.setVelocity(-0.3)
            right_motor.setVelocity(0.3)
        elif  angle_z_deg >= turn_angle-5:
            left_motor.setVelocity(-0.1)
            right_motor.setVelocity(0.1)
        if  angle_z_deg >= turn_angle: 
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)           
            break
            
    stop()

def forward() : #Go forward at high speed
    left_motor.setVelocity(10)
    right_motor.setVelocity(10)

def forwardSlow() : #Go forward at low speed
    left_motor.setVelocity(5)
    right_motor.setVelocity(5)

def brake() : #Stop rotating motors
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    
def go_straight() : #Go straight until meet a wall
    while robot.step(TIME_STEP) != -1:
        forward()
        #print(list_ps[1].getValue())
        if list_ps[1].getValue() > 295 :
            brake()
            stop()
            break

def move_to_next_cell() : #Go forward by one cell
    forward() 
    start_time1 = robot.getTime()
    while robot.step(TIME_STEP) != -1:
        if robot.getTime() - start_time1 > cell_time/4 or list_ps[1].getValue() > 295:
            brake()
            break 
    if list_ps[1].getValue() > 135 and list_ps[1].getValue() < 250:
        forward()
        while robot.step(TIME_STEP) != -1:
            if list_ps[1].getValue() > 295 :
                brake()
                break 
      
    stop()


def enter() :
    forward()
    start_time2 = robot.getTime()
    while robot.step(TIME_STEP) != -1:
        if robot.getTime() - start_time2 > 1.5*cell_time/4 or list_ps[1].getValue() > 295:
            brake()
            break 
    if list_ps[1].getValue() > 135 and list_ps[1].getValue() < 250:
        forward()
        while robot.step(TIME_STEP) != -1:
            if list_ps[1].getValue() > 295 :
                brake()
                stop()
                break
                
                    
def find_robot_direction(x, y):
    global robot_direction, robot_position

    if robot_direction == 0:  # Facing North
        if x == -1 and y == 0:
            move_to_next_cell()
            robot_position[0] -= 1
            
        elif x == 0 and y == 1:
            right_turn()
            move_to_next_cell()
            robot_direction = 1
            robot_position[1] += 1
        elif x == 1 and y == 0:
            turn_back()
            robot_direction = 2
            if list_ps[1].getValue() < 150:
                move_to_next_cell()
                robot_position[0] += 1
        elif x == 0 and y == -1:
            left_turn()
            move_to_next_cell()
            robot_direction = 3
            robot_position[1] -= 1
        elif x == 2 and y == 2:
            pass

    elif robot_direction == 1:  # Facing East
        if x == 0 and y == 1:
            move_to_next_cell()
            robot_position[1] += 1
        elif x == -1 and y == 0:
            left_turn()
            move_to_next_cell()
            robot_direction = 0
            robot_position[0] -= 1
            
        elif x == 1 and y == 0:
            right_turn()
            move_to_next_cell()
            robot_direction = 2
            robot_position[0] += 1
        elif x == 0 and y == -1:
            turn_back()
            robot_direction = 3
            if list_ps[1].getValue() < 150:
                move_to_next_cell()
                robot_position[1] -= 1
        elif x == 2 and y == 2:
            left_turn()

    elif robot_direction == 2:  # Facing South
        if x == 0 and y == -1:
            right_turn()
            move_to_next_cell()
            robot_direction = 3
            robot_position[1] -= 1
        elif x == 1 and y == 0:
            move_to_next_cell()
            robot_position[0] += 1
        elif x == 0 and y == 1:
            left_turn()
            move_to_next_cell()
            robot_direction = 1
            robot_position[1] += 1
        elif x == -1 and y == 0:
            turn_back()
            robot_direction = 0
            if list_ps[1].getValue() < 150:
                move_to_next_cell()
                robot_position[0] -= 1
                
        elif x == 2 and y == 2:
            turn_back()

    elif robot_direction == 3:  # Facing West
        if x == -1 and y == 0:
            right_turn()
            move_to_next_cell()
            robot_direction = 0
            robot_position[0] -= 1
            
        elif x == 1 and y == 0:
            left_turn()
            move_to_next_cell()
            robot_direction = 2
            robot_position[0] += 1
        elif x == 0 and y == -1:
            move_to_next_cell()
            robot_position[1] -= 1
        elif x == 0 and y == 1:
            turn_back()
            robot_direction = 1
            if list_ps[1].getValue() < 150:
                move_to_next_cell()
                robot_position[1] += 1
        elif x == 2 and y == 2:
            right_turn()
    stop()
    

list_ps = []
for ind in ['left_', 'front_', 'right_']: 
    sensor_name = ind + 'sensor'
    list_ps.append(robot.getDevice(sensor_name))
    list_ps[-1].enable(TIME_STEP)


#removed (19,10) from stack
stack = [(19, 9), (19, 8), (19, 7), (18, 7), (18, 8), (17, 8), (16, 8), (16, 9), (16, 10), (17, 10), (17, 9), (18, 9), (18, 10), (18, 11), (17, 11), (16, 11), (15, 11), (14, 11), (14, 12), (14, 13), (14, 14), (13, 14), (13, 13), (12, 13), (12, 12), (11, 12), (11, 11), (12, 11), (12, 10), (11, 10), (10, 10), (9, 10), (8, 10), (8, 9), (9, 9), (9, 8), (9, 7), (8, 7), (9, 7), (9, 8), (9, 9), (8, 9), (8, 10), (9, 10), (9, 11), (8, 11), (7, 11), (6, 11), (6, 12), (7, 12), (7, 13), (8, 13), (8, 14), 
(7, 14), (8, 14), (8, 13), (7, 13), (7, 12), (6, 12), (6, 11), (7, 11), (8, 11), (9, 11), (9, 10), (10, 10), (11, 10), (12, 10), (12, 11), (11, 11), (11, 12), (12, 12), (12, 13), (13, 13), (13, 14), (14, 14), (14, 13), (15, 13), (15, 14), (16, 14), (16, 15), (15, 15), (14, 15), (14, 16), (15, 16), (15, 17), (14, 17), (13, 17), (12, 17), (12, 16), (11, 16), (11, 17), (11, 18), (11, 19), (10, 19), (9, 19), (8, 19), (8, 18), (7, 18), (7, 17), (7, 16), (6, 16), (5, 16), (4, 16), (4, 15), (3, 15), (3, 14),
(2, 14), (1, 14), (1, 13), (1, 12), (0, 12), (0, 11), (1, 11), (1, 10), (1, 9), (0, 9), (0, 8), (1, 8), (1, 7), (0, 7), (0, 6), (0, 5), (0, 4), (0, 3), (0, 2), (0, 1), (0, 0), (1, 0), (2, 0), (3, 0), (4, 0), (5, 0), (5, 1), (5, 2), (6, 2), (7, 2), (8, 2), (9, 2), (10, 2), (10, 1), (10, 0), (11, 0), (12, 0), (13, 0), (14, 0), (15, 0), (15, 1), (16, 1), (16, 2), (17, 2), (17, 1), (18, 1), (19, 1), (19, 2), (19, 3), (19, 4), (18, 4), (18, 3), (17, 3), (17, 4), (17, 5), (18, 5), (19, 5), (19, 6), (18, 6),
(19, 6), (19, 5), (18, 5), (17, 5), (17, 4), (17, 3), (18, 3), (18, 4), (19, 4), (19, 3), (19, 2), (19, 1), (18, 1), (17, 1), (17, 2), (16, 2), (16, 1), (15, 1), (15, 0), (14, 0), (13, 0), (12, 0), (11, 0), (10, 0), (10, 1), (10, 2), (9, 2), (8, 2), (7, 2), (6, 2), (5, 2), (5, 1), (5, 0), (4, 0), (3, 0), (2, 0), (1, 0), (0, 0), (0, 1), (0, 2), (0, 3), (0, 4), (0, 5), (0, 6), (0, 7), (1, 7), (1, 8), (0, 8), (0, 9), (1, 9), (1, 10), (1, 11), (0, 11), (0, 12), (1, 12), (1, 13),
(1, 14), (2, 14), (3, 14), (3, 15), (4, 15), (4, 16), (5, 16), (6, 16), (7, 16), (7, 17), (7, 18), (8, 18), (8, 19), (9, 19), (10, 19), (11, 19), (11, 18), (11, 17), (11, 16), (12, 16), (12, 17), (13, 17), (14, 17), (15, 17), (15, 16), (14, 16), (14, 15), (15, 15), (16, 15), (16, 14), (15, 14), (15, 13), (14, 13), (14, 12), (14, 11), (15, 11), (16, 11), (17, 11), (18, 11), (18, 10), (18, 9), (17, 9), (17, 10), (16, 10), (16, 9), (16, 8), (17, 8), (18, 8), (18, 7), (19, 7), (19, 8), (19, 9), (19, 10)]

reversed_stack = stack[::-1]
while robot.step(TIME_STEP) != -1:
    (x,y) = robot_position
    if reversed_stack:
       
        prev_x, prev_y = reversed_stack.pop()
        dx, dy = prev_x - x, prev_y - y
        find_robot_direction(dx, dy)

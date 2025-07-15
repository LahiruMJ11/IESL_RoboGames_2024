"""In our code we place e_puck robot at an arbitrary place(middle of a cell and parallel to the walls).
   Then robot will execute Depth First Search to find the Red color. Then it will go to yellow, pink, brown
   and green respectively. """

from controller import Robot, Camera
import numpy as np
import math

robot = Robot()
# get the time step of the current world.
TIME_STEP = 8
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
turn_speed = 3
fast_turn = 6
turn_angle = 90
turn_back_angle = 180

camera1 = robot.getDevice('camera1')
camera1.enable(TIME_STEP)
camera2 = robot.getDevice('camera2')
camera2.enable(TIME_STEP)

visited = set()


MAZE_SIZE = 20
cells = np.zeros((MAZE_SIZE, MAZE_SIZE, 4), dtype=int)  # 3D array for walls [N, E, S, W]

# Directions (North, East, South, West)
DIRECTIONS = [(-1, 0), (0, 1), (1, 0), (0, -1)]

# Robot initial position and direction
robot_position = [19, 10]  # (x, y)
robot_direction = 0  # 0 = North, 1 = East, 2 = South, 3 = West

# Define green color range (tunable)
GREEN_THRESHOLD = 100  # Minimum green intensity to be considered "green"
GREEN_RATIO = 1.5  # Green should be significantly higher than red & blue

def detect_color(camera):
    """Detects specific colors from the left and right 1/4 of the middle height of the camera image."""
    image = camera.getImage()
    width = camera.getWidth()
    height = camera.getHeight()
    

    # Define the middle height (single row in the center)
    middle_row = height // 2

    # Convert image to RGB array for easier processing
    image_data = np.zeros((height, width, 3), dtype=int)
    for y in range(height):
        for x in range(width):
            image_data[y, x, 0] = camera.imageGetRed(image, width, x, y)   # R
            image_data[y, x, 1] = camera.imageGetGreen(image, width, x, y) # G
            image_data[y, x, 2] = camera.imageGetBlue(image, width, x, y)  # B

    # Define color thresholds for the specific colors you want to detect
    color_ranges = {
    "Red": ((190, 255), (0, 45), (0, 45)),           # R is high, G and B are low (Red)
    "Yellow": ((190, 255), (190, 255), (100, 160)),     # R and G are high, B is low (Yellow)
    "Green": ((0, 145), (80, 255), (0, 145)),         # G is high, R and B are low (Green)
    "Orange": ((200, 255), (45, 120), (0, 50)),}    # R: high, G: moderate, B: low (Orange)
    #check orange colour

    # Create masks for each color
    color_masks = {color: np.zeros(width, dtype=bool) for color in color_ranges}

    # Loop through each pixel in the middle row and apply color segmentation for each color
    for color_name, ((min_r, max_r), (min_g, max_g), (min_b, max_b)) in color_ranges.items():
        mask = (image_data[middle_row, :, 0] >= min_r) & (image_data[middle_row, :, 0] <= max_r) & \
               (image_data[middle_row, :, 1] >= min_g) & (image_data[middle_row, :, 1] <= max_g) & \
               (image_data[middle_row, :, 2] >= min_b) & (image_data[middle_row, :, 2] <= max_b)
        color_masks[color_name] = mask
        
    # Define the left and right parts (1/3 of the width from both sides)
    left_part_end = width // 3
    right_part_start = width - (width // 3)

    detected_colors = []
   
    # Check if any of the left or right parts match the color
    for color_name, mask in color_masks.items():
        # Left part check
        left_part_match = np.sum(mask[:left_part_end]) > left_part_end / 2
        # Right part check 
        right_part_match = np.sum(mask[right_part_start:]) > left_part_end / 2 

        if left_part_match or right_part_match:
            detected_colors.append(color_name)

    # Return the detected colors
    if detected_colors:
        # print detected colors
        print(f"Color Detected: {detected_colors}")  
        return detected_colors[0]

        
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
        if  angle_z_deg <= -(turn_angle-10):
            left_motor.setVelocity(0.5)
            right_motor.setVelocity(-0.5)
        if  angle_z_deg <= -(turn_angle):            
            break
            
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
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
        if  angle_z_deg <= -(turn_back_angle-10):
            left_motor.setVelocity(0.5)
            right_motor.setVelocity(-0.5)
        if  angle_z_deg <= -turn_back_angle:            
            break
            
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
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
            left_motor.setVelocity(-0.5)
            right_motor.setVelocity(0.5)
        if  angle_z_deg >= turn_angle:            
            break
            
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
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
        forwardSlow()
        print(list_ps[1].getValue())
        if list_ps[1].getValue() > 295 :
            brake()
            color = detect_color(camera)
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
    color = detect_color(camera2)  
    stop()     
    


#def encode_position():
#    for sensor in list_ps:
#        sensor.enable(TIME_STEP)
#    robot.step(TIME_STEP * 10)  # Give sensors time to stabilize
#    """Encode position as a tuple of sensor readings."""
#    return (math.floor(list_ps[0].getValue()), 
#            math.floor(list_ps[3].getValue()), 
#           math.floor(list_ps[2].getValue()))
            

def wall_mapping(x, y):
    max_x = MAZE_SIZE - 1
    max_y = MAZE_SIZE - 1
    
    if robot_direction == 0:  # North
        if list_ps[1].getValue() > 130:  # Front
            cells[x][y][0] = 1
            if x > 0:
                cells[x - 1][y][2] = 1
        else:
            cells[x][y][0] = 0
            if x > 0:
                cells[x - 1][y][2] = 0
        if list_ps[2].getValue() > 130:  # East
            cells[x][y][1] = 1
            if y < max_y:
                cells[x][y + 1][3] = 1
        else:
            cells[x][y][1] = 0
            if y < max_y:
                cells[x][y + 1][3] = 0
        if list_ps[0].getValue() > 130:  # West
            cells[x][y][3] = 1
            if y > 0:
                cells[x][y - 1][1] = 1
        else:
            cells[x][y][3] = 0
            if y > 0:
                cells[x][y - 1][1] = 0
    
    elif robot_direction == 1:  # East
        if list_ps[1].getValue() > 130:  # East
            cells[x][y][1] = 1
            if y < max_y:
                cells[x][y + 1][3] = 1
        else:
            cells[x][y][1] = 0
            if y < max_y:
                cells[x][y + 1][3] = 0
        if list_ps[2].getValue() > 130:  # South
            cells[x][y][2] = 1
            if x < max_x:
                cells[x + 1][y][0] = 1
        else:
            cells[x][y][2] = 0
            if x < max_x:
                cells[x + 1][y][0] = 0
        if list_ps[0].getValue() > 130:  # North
            cells[x][y][0] = 1
            if x > 0:
                cells[x - 1][y][2] = 1
        else:
            cells[x][y][0] = 0
            if x > 0:
                cells[x - 1][y][2] = 0
    
    elif robot_direction == 2:  # South
        if list_ps[1].getValue() > 130:  # South
            cells[x][y][2] = 1
            if x < max_x:
                cells[x + 1][y][0] = 1
        else:
            cells[x][y][2] = 0
            if x < max_x:
                cells[x + 1][y][0] = 0
        if list_ps[2].getValue() > 130:  # West
            cells[x][y][3] = 1
            if y > 0:
                cells[x][y - 1][1] = 1
        else:
            cells[x][y][3] = 0
            if y > 0:
                cells[x][y - 1][1] = 0
        if list_ps[0].getValue() > 130:  # East
            cells[x][y][1] = 1
            if y < max_y:
                cells[x][y + 1][3] = 1
        else:
            cells[x][y][1] = 0
            if y < max_y:
                cells[x][y + 1][3] = 0
    
    elif robot_direction == 3:  # West
        if list_ps[1].getValue() > 130:  # West
            cells[x][y][3] = 1
            if y > 0:
                cells[x][y - 1][1] = 1
        else:
            cells[x][y][3] = 0
            if y > 0:
                cells[x][y - 1][1] = 0
        if list_ps[2].getValue() > 130:  # North
            cells[x][y][0] = 1
            if x > 0:
                cells[x - 1][y][2] = 1
        else:
            cells[x][y][0] = 0
            if x > 0:
                cells[x - 1][y][2] = 0
        if list_ps[0].getValue() > 130:  # South
            cells[x][y][2] = 1
            if x < max_x:
                cells[x + 1][y][0] = 1
        else:
            cells[x][y][2] = 0
            if x < max_x:
                cells[x + 1][y][0] = 0

def move_next_pos(x, y, robot_direction, visited, stack):
    """Explore the maze using DFS + Backtracking."""
    global robot_position

    # Mark the current cell as visited
    visited.add((x, y))
    #print(f"Current Position: {robot_position}, Visited: {visited}")

    # Check all 4 directions (N, E, S, W)
    for i in range(4):
        dx, dy = DIRECTIONS[i]
        nx, ny = x + dx, y + dy  # New position

        # Check if the new position is within bounds and unvisited
        if (0 <= nx < MAZE_SIZE) and (0 <= ny < MAZE_SIZE) and (nx, ny) not in visited:
            if cells[x][y][i] == 1:  # If there's a wall, skip
                continue
            else:
                # Move to the new cell
                find_robot_direction(dx, dy)
                print(robot_position)
                if (x,y) not in stack:
                    stack.append((x,y))
                stack.append((nx, ny))  # Push to stack for backtracking
                #print(stack)
                return  # Move to the new cell and continue exploration

    # If no available move, backtrack
    if stack:
       
        prev_x, prev_y = stack.pop()
        #print(stack)
        # Calculate the direction to backtrack
        dx, dy = prev_x - x, prev_y - y
        find_robot_direction(dx, dy)
        print(robot_position)
        #visited.add((prev_x, prev_y))
        
        
        
def explore():
    global robot_position, robot_direction
    visited = set()
    stack = [(robot_position[0], robot_position[1])]

    while robot.step(TIME_STEP) != -1:
        # Update wall configuration for the current cell
        wall_mapping(robot_position[0], robot_position[1])

        # Explore the maze using DFS
        move_next_pos(robot_position[0], robot_position[1], robot_direction, visited, stack)

        # Check if all cells have been visited
        if len(visited) == MAZE_SIZE * MAZE_SIZE:
            while (len(stack)!=0):
       
                prev_x, prev_y = stack.pop()
                print(stack)
                x = robot_position[0]
                y = robot_position[1]
        # Calculate the direction to backtrack
                dx, dy = prev_x - x, prev_y - y
                find_robot_direction(dx, dy)
                print('w')
            find_robot_direction(2, 2)
            print("Exploration complete.")
            break

    
    
def find_robot_direction(x, y):
    global robot_direction, robot_position

    if robot_direction == 0:  # Facing North
        if x == -1 and y == 0:
            move_to_next_cell()
            robot_position[0] -= 1
            print('f')
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
            print('j')
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
                print('d')
        elif x == 2 and y == 2:
            turn_back()

    elif robot_direction == 3:  # Facing West
        if x == -1 and y == 0:
            right_turn()
            move_to_next_cell()
            robot_direction = 0
            robot_position[0] -= 1
            print('k')
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


def print_cells_matrix():
    """Prints the cells matrix showing wall configurations."""
    for x in range(MAZE_SIZE):
        for y in range(MAZE_SIZE):
            print(f"({x},{y}): N={cells[x][y][0]}, E={cells[x][y][1]}, S={cells[x][y][2]}, W={cells[x][y][3]}")
        print()

list_ps = []
for ind in ['left_', 'front_', 'right_']: 
    sensor_name = ind + 'sensor'
    list_ps.append(robot.getDevice(sensor_name))
    list_ps[-1].enable(TIME_STEP)
    
# Start exploration
#explore()
stop()
detect_color(camera1)
# Call the function to print the matrix
#right_turn()
#move_to_next_cell()
#left_turn()
#move_to_next_cell()
#right_turn()
#move_to_next_cell()
#left_turn()
#turn_back()
#while robot.step(TIME_STEP) != -1:
#    print(list_ps[1].getValue())
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

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

left_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)

right_motor.setPosition(float('inf'))
right_motor.setVelocity(0.0)

# Speeds
wheel_speed = 2.0  # m/s
wheel_base = 0.053  # Distance between e-puck wheels in meters
turn_time = 2.225  # Time for 90-degree turn

camera = robot.getDevice('camera')
camera.enable(TIME_STEP)

visited = set()
stack = []

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
        "Yellow": ((190, 255), (190, 255), (0, 45)),     # R and G are high, B is low (Yellow)
        "Pink": ((190, 255), (0, 45), (190, 255)),       # R is high, B is high, G is low (Pink)
        "Brown": ((170, 200), (120, 140), (45, 65)),      # R: moderate, G: Bit-low, B: low (Brown)
        "Green": ((0, 45), (190, 255), (0, 45)),         # G is high, R and B are low (Green)
    }

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
        
def stop() : #Stop the robot for a 2 seconds.
    start_time1 = robot.getTime()
    while robot.step(TIME_STEP) != -1:
        if robot.getTime() - start_time1 > 1:
            break  

def right_turn() : #Perform a 90 degree right turn.
    left_motor.setVelocity(1)
    right_motor.setVelocity(-1)
    start_time1 = robot.getTime()
    while robot.step(TIME_STEP) != -1:
        if robot.getTime() - start_time1 >= turn_time:            
            break
            
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    color = detect_color(camera)
    stop()

def turn_back() : #Turn back
    left_motor.setVelocity(1)
    right_motor.setVelocity(-1)
    start_time1 = robot.getTime()
    while robot.step(TIME_STEP) != -1:
        if robot.getTime() - start_time1 >= 2*turn_time:            
            break  
        
def left_turn() : #Perform a 90 degree left turn.
    left_motor.setVelocity(-1)
    right_motor.setVelocity(1)
    start_time1 = robot.getTime()
    while robot.step(TIME_STEP) != -1:
        if robot.getTime() - start_time1 >= turn_time:            
            break
            
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    color = detect_color(camera)
    stop()

def forward() : #Go forward at high speed
    left_motor.setVelocity(6.28)
    right_motor.setVelocity(6.28)

def forwardSlow() : #Go forward at low speed
    left_motor.setVelocity(3.14)
    right_motor.setVelocity(3.14)

def brake() : #Stop rotating motors
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    
def go_straight() : #Go straight until meet a wall
    while robot.step(TIME_STEP) != -1:
        forward()
        if list_ps[5].getValue() > 116 :
            brake()
            color = detect_color(camera)
            stop()
            break

def move_to_next_cell() : #Go forward by one cell
    forwardSlow() 
    start_time1 = robot.getTime()
    while robot.step(TIME_STEP) != -1:
        if robot.getTime() - start_time1 > 7.936/2 or list_ps[5].getValue() > 116:
            brake()
            break
    if list_ps[5].getValue() > 90 :
        go_straight()     
    


def encode_position():
    for sensor in list_ps:
        sensor.enable(TIME_STEP)
    robot.step(TIME_STEP * 10)  # Give sensors time to stabilize
    """Encode position as a tuple of sensor readings."""
    return (math.floor(list_ps[0].getValue()), 
            math.floor(list_ps[3].getValue()), 
            math.floor(list_ps[2].getValue()))

def explore():
    """Explore the maze using Depth First Search and find the red color."""
    global stack
    stack.append((encode_position(), "START"))
    
    while robot.step(TIME_STEP) != -1:
        color = detect_color(camera)
        if color == "Red" :
            print("Color Pattern Navigation Started.")
            break

        position = encode_position()
        if position in visited:
            if not stack:
                print("Exploration complete. Red color not found.")
                break
            _, direction = stack.pop()
            if direction == "LEFT":
                right_turn()
            elif direction == "RIGHT":
                left_turn()
            elif direction == "BACK":
                left_turn()
                left_turn()
            continue

        visited.add(position)
        # Prioritize directions: Left, Forward, Right, Back
        if detect_color(camera) == "Red" :
            print("Color Pattern Navigation Started.")
            break
            
        if list_ps[3].getValue() < 100:  # Check left
            left_turn()
            stack.append((position, "LEFT"))
            move_to_next_cell()
        elif list_ps[0].getValue() < 100:  # Check forward
            move_to_next_cell()
            stack.append((position, "FORWARD"))
        elif list_ps[2].getValue() < 100:  # Check right
            left_turn()
            if detect_color(camera) == "Red" :
                print("Color Pattern Navigation Started.")
                break
            
            turn_back()
            stack.append((position, "RIGHT"))
            move_to_next_cell()
        else:  # Backtrack
            if stack:
                _, direction = stack.pop()
                if direction == "LEFT":
                    right_turn()
                elif direction == "RIGHT":
                    left_turn()
                elif direction == "FORWARD":
                    left_turn()
                    left_turn()
               
def red_yellow(): #Hard coded to go from red to yellow
    left_turn()
    go_straight()
    left_turn()
    go_straight()
    right_turn() 
    go_straight()
    right_turn()
    go_straight()
    left_turn()
    go_straight()
    left_turn()
    go_straight() 
    right_turn()
    go_straight()
    left_turn()
    go_straight()
    left_turn()
    go_straight()
    right_turn()
    go_straight()
       
def yellow_pink(): #Hard coded to go from yellow to pink
    right_turn()
    right_turn()
    go_straight()
    left_turn()
    go_straight()
    right_turn()
    go_straight()
    right_turn()
    go_straight()
    left_turn()
    go_straight()
    right_turn()
    go_straight()
    right_turn()
    go_straight()
    left_turn()
    go_straight()
    left_turn()
    go_straight()
    right_turn()
    go_straight()
    right_turn()
    go_straight()
     
def pink_brown(): #Hard coded to go from pink to brown
    right_turn()
    right_turn()  
    move_to_next_cell()
    right_turn()
    go_straight()
    right_turn()
                
def brown_green(): #Hard coded to go from brown to green
    right_turn()
    right_turn()
    go_straight()
    right_turn()
    go_straight()
    left_turn()
    go_straight()
    left_turn()
    go_straight()    
    left_turn()
    go_straight() 
    right_turn()   
    move_to_next_cell()
    right_turn()
    go_straight()
    left_turn()
    go_straight()
    left_turn()
    go_straight()
    right_turn()
    go_straight()
    left_turn()
    go_straight()               

list_ps = []
for ind in [0, 1, 2, 5, 6, 7]: 
    sensor_name = 'ps' + str(ind)
    list_ps.append(robot.getDevice(sensor_name))
    list_ps[-1].enable(TIME_STEP)
    
# Start exploration
explore()
stop()
red_yellow()
yellow_pink()
pink_brown()
brown_green()
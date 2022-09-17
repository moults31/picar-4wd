import picar_4wd as fc
import time
import numpy as np
import sys
import math
import argparse
import multiprocessing as mp
import heapq
from enum import Enum
import heapq
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure

# User modules
disable_camera = False
try:
    import lab1part2_object_detector
except ModuleNotFoundError as e:
    print("Could not import lab1part2_object_detector.")
    print("Proceeding without object detection functionality.")
    print("Original error:")
    print(e)
    disable_camera = True

# Define enum for holding driving direction, in relation to the destination, which can be defined
# as infinity in the direction of the starting position of the car.
class DrivingDirection (Enum):
    towards_destination = 1
    right = 2
    left = 3
    away_from_destination = 4

# Set speed of car
speed = 5
starting_point = (24,10)
# Set starting direction of car as toward destination
direction = DrivingDirection.towards_destination

# Initialise counter for measuring distance
distance_counter = 0
forward_timer = 0

# Global for storing individual car tuning
driver = None

# Initialise array representing 20 * 20 occupany squares of approx 20cm
# when 7 represents unmapped areas, 0 represents clear and 1 represents obstacle
array_shape = (50, 50)
fill_value = 7
map = np.full(array_shape, fill_value)

# Intialise car location in array
car_position = [24, 10]
x_coords = []
y_coords = []
def route_map(route):
    global array_shape
    global fill_value
    route_map = np.full(array_shape, fill_value)
    for i in (range(0,len(route))):
        x = route[i][0]
        y = route[i][1]
        route_map[x][y]=8
    return route_map

def heuristic(a, b):
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

def a_star_algorithm(array, start, target):
    #Identifying the neighbors of the starting point
    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    #The close set for points that were already checked
    close_set = set()
    came_from = {}
    gscore = {start:0}
    #the distance between start and end
    fscore = {start:heuristic(start, target)}
    open_list = []
    heapq.heappush(open_list, (fscore[start], start))
    while open_list:
        current = heapq.heappop(open_list)[1]
        if current == target:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            return data
        #add current to the close set
        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:
                    if array[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    # array bound y walls
                    continue
            else:
                # array bound x walls
                continue

            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue

            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in open_list]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, target)
                heapq.heappush(open_list, (fscore[neighbor], neighbor))
    return []

# Execute turn of car
def turn(turning_direction):

    global distance_counter
    global driver

    # Set time for turning action for a period in seconds which gives a 90 degree turn angle.
    # Different timers needed for left and right turns to maintain consistent turning angle
    # Tunes are stored here per individual car
    if driver == 'Rob':
        turn_left_timer = 0.9
        turn_right_timer = 1
    elif driver == 'Kinga':
        turn_left_timer = 1
        turn_right_timer = 1.07
    elif driver == 'Zac':
        turn_left_timer = 2.05
        turn_right_timer = 1.35
    else:
        turn_left_timer = 1
        turn_right_timer = 1

    # Execute turn in direction received in function call and wait for specific time
    # before stopping
    if turning_direction == 'right':
        fc.turn_right(speed)
        time.sleep(turn_right_timer)

    else:
        fc.turn_left(speed)
        time.sleep(turn_left_timer)

    # Stop turn
    fc.stop()
    #updatePositionTurning(turning_direction)

    # Reset distance counter as this is used to ensure car moves a certain distance
    # forward after each turn before attempting a turn towards destination
    distance_counter = 0
    return

def updatePositionMovingForward(distance):
    global direction
    global car_position
    if direction == DrivingDirection.towards_destination:
        car_position[0] = car_position[0] - distance

    elif direction == DrivingDirection.right:
        car_position[1] = car_position[1] + distance

    elif direction == DrivingDirection.left:
        car_position[1] = car_position[1] - distance

    else:
        car_position[0] = car_position[0] + distance

# Move car forward and update distance counter each time function is called
def move_forward():
    global distance_counter
    fc.forward(speed)
    # Sleep while car travels 5 cm
    #time.sleep(0.16)
    distance_counter += 1
    print("Distance counter ", distance_counter)
    # Car moves 16cm forward each time
    updatePositionMovingForward(1)
    return

# Check ultrasonic scan. Assessing left, centre and right parts of scan for obstacles
# return blocked state object
def check_scan(scan_list, blocked_state):
    if scan_list[0:3] != [ 2, 2, 2]:
        blocked_state['left'] = True
        #print("Blocked left")
    else:
        blocked_state['left'] = False

    if scan_list[3:7] != [2, 2, 2, 2]:
        blocked_state['centre'] = True
        #print("Blocked centre")
    else:
        blocked_state['centre'] = False

    if scan_list[7:10] != [2, 2, 2]:
        blocked_state['right'] = True
        #print ("Blocked right")
    else:
        blocked_state['right'] = False
    return blocked_state

def print_map(x,y):
    map_file_name = "map.png"
    print(f"Saving map to {map_file_name}...")
    global array_shape
    fig, ax = plt.subplots(figsize=(50,50))

    ax.imshow(map, cmap=plt.get_cmap('tab20'))
    ax.scatter(x,y, marker = ".", color = "red", s = 300)
    ax.scatter(target[1],target[0], marker = ".", color = "blue", s = 300)
    ax.plot(y_coords,x_coords, color = "green")

    plt.savefig("map.png")
    print ("Done!")

# Decide on the action based on the blocked state and the direction of the car
# in relation to the destination
def decide_on_action(blocked_state, route, new_car_position):
    print("Car position ", new_car_position)
    print("Going to point", route[-1])

    # Use global variable
    global direction
    print("Car direction", direction)
    next_point=route[-1]
    #y
    y=new_car_position[0]-next_point[0]
    #x
    x=new_car_position[1]-next_point[1]

    #UP
    if (direction == DrivingDirection.towards_destination) and (y == 1 and x == 0):
        print("y1 x0")
        if blocked_state['centre']:
            if not blocked_state['left']:
                turn('left')
                direction = DrivingDirection.left
                print("Up- Turning left y1x0")
                move_forward()
                return

            elif not blocked_state['right']:
                turn('right')
                direction = DrivingDirection.right
                print("Up-Turning right y1x0")
                move_forward()
                return

            else:
                turn('right')
                turn('right')
                print("Forward-Turning 180")
                direction = DrivingDirection.away_from_destination
                move_forward()
                return
        else:
            move_forward()
            print("Up - default y1x0")
            return

    elif (direction == DrivingDirection.towards_destination) and (y == 1 and x == -1):
        print("y1 x-1")
        if blocked_state['centre']:
            if not blocked_state['right']:
                turn('right')
                direction = DrivingDirection.right
                print("Forward-Turning right")
                move_forward()
                return

            elif not blocked_state['left']:
                turn('left')
                direction = DrivingDirection.left
                print("Forward-Turning left")
                move_forward()
                return
            else:
                turn('right')
                turn('right')
                print("Forward-Turning 180")
                direction = DrivingDirection.away_from_destination
                move_forward()
                return
        else:
            move_forward()
            print("Up - Default y1 x-1")
            return

    elif (direction == DrivingDirection.towards_destination) and (y == 1 and x == 1):
        print("y1 x1")
        if blocked_state['centre']:
            if not blocked_state['left']:
                turn('left')
                direction = DrivingDirection.left
                print("Forward-Turning left")
                move_forward()
                return

            elif not blocked_state['right']:
                turn('right')
                direction = DrivingDirection.right
                print("Forward-Turning right")
                move_forward()
                return

            else:
                turn('right')
                turn('right')
                print("Forward-Turning 180")
                direction = DrivingDirection.away_from_destination
                move_forward()
                return
        else:
            move_forward()
            print("Up - y1 x1")
            return

    elif (direction == DrivingDirection.towards_destination) and (y == 0 and x == -1):
        print(" Up - y0 x-1")
        if blocked_state['right']:
            if not blocked_state['centre']:
                direction = DrivingDirection.towards_destination
                move_forward()
                print("Forward- forward y0x-1")
                return

            elif not blocked_state['left']:
                turn('left')
                direction = DrivingDirection.left
                print("Forward-Turning left")
                move_forward()
                return

            else:
                turn('right')
                turn('right')
                print("Forward-Turning 180")
                direction = DrivingDirection.away_from_destination
                move_forward()
                return
        else:
            turn('right')
            direction = DrivingDirection.right
            print("Forward-Turning right")
            move_forward()
            return

    elif (direction == DrivingDirection.towards_destination) and (y == 0 and x == 1):
        print("y0 x1")
        if blocked_state['left']:
            if not blocked_state['centre']:
                direction = DrivingDirection.towards_destination
                move_forward()
                print("Up - forward y0x1")
                return

            elif not blocked_state['right']:
                turn('right')
                direction = DrivingDirection.right
                print("Up - Turning right")
                move_forward()
                return

            else:
                turn('right')
                turn('right')
                print("Up - Turning 180")
                direction = DrivingDirection.away_from_destination
                move_forward()
                return
        else:
            turn('left')
            direction = DrivingDirection.left
            print("Up - Turning left")
            move_forward()
            return

    elif (direction == DrivingDirection.towards_destination) and (y == 0 and x == 0):
        print("Up - return y0 x0")
        return

    elif (direction == DrivingDirection.towards_destination) and (y == -1 and x == 0):
        print("Up y-1 x0")
        turn('right')
        turn('right')
        print("Up - Turning 180")
        direction = DrivingDirection.away_from_destination
        move_forward()
        return

    elif (direction == DrivingDirection.towards_destination) and (y == -1 and x == -1):
        print("Up y-1 x-1")
        if blocked_state['right']:
            if not blocked_state['left']:
                turn('left')
                direction = DrivingDirection.left
                print("Up - Turning left")
                move_forward()
                return

            elif not blocked_state['centre']:
                direction = DrivingDirection.towards_destination
                move_forward()
                print("Up - forward not blocked y-1 x-1")
                return
            else:
                turn('right')
                turn('right')
                print("Up - Turning 180")
                direction = DrivingDirection.away_from_destination
                move_forward()
                return
        else:
            turn('right')
            direction = DrivingDirection.right
            print("Up - Turning right")
            move_forward()
            return

    elif (direction == DrivingDirection.towards_destination) and (y == -1 and x == 1):
        print("Up y-1 x1")
        if blocked_state['left']:
            if not blocked_state['right']:
                turn('right')
                direction = DrivingDirection.right
                print("Up - Turning right")
                move_forward()
                return

            elif not blocked_state['centre']:
                direction = DrivingDirection.towards_destination
                print("Up - forward ")
                move_forward()
                return
            else:
                turn('right')
                turn('right')
                print("Up - Turning 180")
                direction = DrivingDirection.away_from_destination
                move_forward()
                return
        else:
            turn('left')
            direction = DrivingDirection.left
            print("Up - Turning left")
            move_forward()
            return

    #RIGHT
    if (direction == DrivingDirection.right) and (y == 1 and x == 0):
        print("Right y1 x0")
        if blocked_state['left']:
            if not blocked_state['centre']:
                direction = DrivingDirection.right
                move_forward()
                print("Right -forward y1x0")
                return

            elif not blocked_state['right']:
                turn('right')
                direction = DrivingDirection.away_from_destination
                print("Right - Turning right")
                move_forward()
                return

            else:
                turn('right')
                turn('right')
                print("Right - Turning 180")
                direction = DrivingDirection.left
                move_forward()
                return
        else:
            turn('left')
            direction = DrivingDirection.towards_destination
            print("Right - Turning left")
            move_forward()
            return

    elif (direction == DrivingDirection.right) and (y == 1 and x == -1):
        print("Right y1 x-1")
        if blocked_state['centre']:
            if not blocked_state['left']:
                turn('left')
                direction = DrivingDirection.towards_destination
                print("Right -Turning left")
                move_forward()
                return

            elif not blocked_state['right']:
                turn('right')
                direction = DrivingDirection.away_from_destination
                print("Right - Turning right")
                move_forward()
                return

            else:
                turn('right')
                turn('right')
                print("Right- Turning 180")
                direction = DrivingDirection.left
                move_forward()
                return
        else:
            move_forward()
            return

    elif (direction == DrivingDirection.right) and (y == 1 and x == 1):
        print("Right y1 x1")
        if blocked_state['left']:
            if not blocked_state['centre']:
                direction = DrivingDirection.right
                move_forward()
                print("Right - Forward y1x1")
                return

            elif not blocked_state['right']:
                turn('right')
                direction = DrivingDirection.away_from_destination
                print("Right - Turning right")
                move_forward()
                return

            else:
                turn('right')
                turn('right')
                print("Right - Turning 180")
                direction = DrivingDirection.left
                move_forward()
                return
        else:
            turn('left')
            direction = DrivingDirection.towards_destination
            print("Right - Turning left")
            move_forward()
            return

    elif (direction == DrivingDirection.right) and (y == 0 and x == -1):
        print("Right y0 x-1")
        if blocked_state['centre']:
            if not blocked_state['left']:
                turn('left')
                direction = DrivingDirection.towards_destination
                print("Right - Turning left")
                move_forward()
                return

            elif not blocked_state['right']:
                turn('right')
                direction = DrivingDirection.away_from_destination
                print("Right - Turning right")
                move_forward()
                return

            else:
                turn('right')
                turn('right')
                print("Right - Turning 180")
                direction = DrivingDirection.left
                move_forward()
                return
        else:
            move_forward()
            return

    elif (direction == DrivingDirection.right) and (y == 0 and x == 1):
        print("Right y0 x1")
        turn('right')
        turn('right')
        print("Right - Turning 180")
        direction = DrivingDirection.left
        move_forward()
        return

    elif (direction == DrivingDirection.right) and (y == 0 and x == 0):
        print("Right y0 x0 - return")
        return

    elif (direction == DrivingDirection.right) and (y == -1 and x == 0):
        print("Right y-1 x0")
        if blocked_state['right']:
            if not blocked_state['centre']:
                move_forward()
                return

            elif not blocked_state['left']:
                turn('left')
                direction = DrivingDirection.towards_destination
                print("Right - Turning left")
                move_forward()
                return

            else:
                turn('right')
                turn('right')
                print("Right - Turning 180")
                direction = DrivingDirection.left
                move_forward()
                return
        else:
            turn('right')
            direction = DrivingDirection.away_from_destination
            print("Right - Turning right")
            move_forward()
            return

    elif (direction == DrivingDirection.right) and (y == -1 and x == -1):
        print("Right y-1 x-1")
        if blocked_state['centre']:
            if not blocked_state['right']:
                turn('right')
                direction = DrivingDirection.away_from_destination
                print("Right - TTurning right")
                move_forward()
                return

            elif not blocked_state['left']:
                turn('left')
                direction = DrivingDirection.towards_destination
                print("Right - TTurning left")
                move_forward()
                return

            else:
                turn('right')
                turn('right')
                print("Right - Turning 180")
                direction = DrivingDirection.left
                move_forward()
                return
        else:
            move_forward()
            return

    elif (direction == DrivingDirection.right) and (y == -1 and x == 1):
        print("Right y-1 x1")
        turn('right')
        turn('right')
        print("Left -Turning 180")
        direction = DrivingDirection.left
        move_forward()

    #LEFT
    if (direction == DrivingDirection.left) and (y == 1 and x == 0):
        print("Left y1 x0")
        if blocked_state['right']:
            if not blocked_state['centre']:
                direction = DrivingDirection.left
                print("Left -Forward y1 x0")
                move_forward()
                return

            elif not blocked_state['left']:
                turn('left')
                direction = DrivingDirection.away_from_destination
                print("Left -Turning left y1 x0")
                move_forward()
                return

            else:
                turn('right')
                turn('right')
                print("Left -Turning 180 y1 x0")
                direction = DrivingDirection.right
                move_forward()
                return
        else:
            turn('right')
            direction = DrivingDirection.towards_destination
            print("Left -Turning right y1 x0")
            move_forward()
            return

    elif (direction == DrivingDirection.left) and (y == 1 and x == -1):
        print("Left y1 x-1")
        if blocked_state['right']:
            if not blocked_state['centre']:
                direction = DrivingDirection.left
                move_forward()
                print("Left -Forward y1 x-1")
                return

            elif not blocked_state['left']:
                turn('left')
                direction = DrivingDirection.away_from_destination
                print("Left -Turning left y1 x-1")
                move_forward()
                return

            else:
                turn('right')
                turn('right')
                print("Left -Turning 180 y1 x-1")
                direction = DrivingDirection.right
                move_forward()
                return
        else:
            turn('right')
            direction = DrivingDirection.towards_destination
            print("Left -Turning right y1 x-1")
            move_forward()
            return

    elif (direction == DrivingDirection.left) and (y == 1 and x == 1):
        print("Left y1 x1")
        if blocked_state['centre']:
            print("Left y1 x1 entered loop")
            if not blocked_state['right']:
                turn('right')
                direction = DrivingDirection.towards_destination
                print("Left -Turning right y1 x1")
                move_forward()
                return

            elif not blocked_state['left']:
                turn('left')
                direction = DrivingDirection.away_from_destination
                print("Left -Turning left y1 x1")
                move_forward()
                return

            else:
                turn('right')
                turn('right')
                print("Left -Turning 180 y1 x1")
                direction = DrivingDirection.right
                move_forward()
                return
        else:
            print("Left -Default y1 x1 else loop")
            move_forward()
            direction == DrivingDirection.left
            print("Left -Default y1 x1")
            return

    elif (direction == DrivingDirection.left) and (y == 0 and x == -1):
        print("Left y0 x-1")
        print("Default y0 x1")
        turn('right')
        turn('right')
        print("Left -Turning 180 y0 x1")
        direction = DrivingDirection.right
        move_forward()
        return

    elif (direction == DrivingDirection.left) and (y == 0 and x == 1):
        print("Left y0 x1")
        if blocked_state['centre']:
            if not blocked_state['right']:
                turn('right')
                direction = DrivingDirection.towards_destination
                print("Left -Turning right y0x1")
                move_forward()
                return

            elif not blocked_state['left']:
                turn('left')
                direction = DrivingDirection.away_from_destination
                print("Left -Turning left y0x1")
                move_forward()
                return

            else:
                turn('right')
                turn('right')
                print("Turning 180 y0x1")
                direction = DrivingDirection.right
                move_forward()
                return
        else:
            move_forward()
            print("Default y0x1")
            return

    elif (direction == DrivingDirection.left) and (y == 0 and x == 0):
        print("Left y0 x0")
        print('Route is done y0x0')
        return

    elif (direction == DrivingDirection.left) and (y == -1 and x == 0):
        print("Left y-1 x0")
        if blocked_state['left']:
            if not blocked_state['centre']:
                move_forward()
                return

            elif not blocked_state['right']:
                turn('right')
                direction = DrivingDirection.towards_destination
                print("Turning right")
                move_forward()
                return

            else:
                turn('right')
                turn('right')
                print("Turning 180")
                direction = DrivingDirection.right
                move_forward()
                return
        else:
            turn('left')
            direction = DrivingDirection.away_from_destination
            print("Turning left")
            move_forward()
            return

    elif (direction == DrivingDirection.left) and (y == -1 and x == -1):
        print("Left y-1 x-1")
        if blocked_state['left']:
            if not blocked_state['centre']:
                move_forward()
                return

            elif not blocked_state['right']:
                turn('right')
                direction = DrivingDirection.towards_destination
                print("Turning right")
                move_forward()
                return

            else:
                turn('right')
                turn('right')
                print("Turning 180")
                direction = DrivingDirection.right
                move_forward()
                return
        else:
            turn('left')
            direction = DrivingDirection.away_from_destination
            print("Turning left")
            move_forward()
            return

    elif (direction == DrivingDirection.left) and (y == -1 and x == 1):
        print("Left y-1 x1")
        if blocked_state['centre']:
            if not blocked_state['right']:
                turn('right')
                direction = DrivingDirection.towards_destination
                print("Turning right")
                move_forward()
                return

            elif not blocked_state['left']:
                turn('left')
                direction = DrivingDirection.away_from_destination
                print("Turning left")
                move_forward()
                return

            else:
                turn('right')
                turn('right')
                print("Turning 180")
                direction = DrivingDirection.right
                move_forward()
                return
        else:
            move_forward()
            return

    #DOWN
    if (direction == DrivingDirection.away_from_destination) and (y == 1 and x == 0):
        turn('right')
        turn('right')
        print("Turning 180")
        direction = DrivingDirection.towards_destination
        return

    elif (direction == DrivingDirection.away_from_destination) and (y == 1 and x == -1):
        if blocked_state['left']:
            turn('right')
            turn('right')
            print("Turning 180")
            direction = DrivingDirection.towards_destination
            return
        else:
            turn('left')
            direction = DrivingDirection.right
            print("Turning left")
            move_forward()
            return

    elif (direction == DrivingDirection.away_from_destination) and (y == 1 and x == 1):
        if blocked_state['right']:
            turn('right')
            turn('right')
            print("Turning 180")
            direction = DrivingDirection.towards_destination
            return
        else:
            turn('right')
            direction = DrivingDirection.left
            print("Turning right")
            move_forward()
            return

    elif (direction == DrivingDirection.away_from_destination) and (y == 0 and x == -1):
        if blocked_state['left']:
            if not blocked_state['centre']:
                direction = DrivingDirection.away_from_destination
                return

            elif not blocked_state['right']:
                turn('right')
                direction = DrivingDirection.left
                print("Turning right")
                return

            else:
                turn('right')
                turn('right')
                print("Turning 180")
                direction = DrivingDirection.towards_destination
                return
        else:
            turn('left')
            direction = DrivingDirection.right
            print("Turning left")
            move_forward()
            return

    elif (direction == DrivingDirection.away_from_destination) and (y == 0 and x == 1):
        if blocked_state['right']:
            if not blocked_state['centre']:
                direction = DrivingDirection.away_from_destination
                return

            elif not blocked_state['left']:
                turn('left')
                direction = DrivingDirection.right
                print("Turning left")
                return

            else:
                turn('right')
                turn('right')
                print("Turning 180")
                direction = DrivingDirection.towards_destination
                return
        else:
            turn('right')
            direction = DrivingDirection.left
            print("Turning right")
            move_forward()
            return

    elif (direction == DrivingDirection.away_from_destination) and (y == 0 and x == 0):
        return
    elif (direction == DrivingDirection.away_from_destination) and (y == -1 and x == 0):
        if blocked_state['centre']:
            if not blocked_state['right']:
                turn('right')
                direction = DrivingDirection.left
                print("Turning right")
                return

            elif not blocked_state['left']:
                turn('left')
                direction = DrivingDirection.right
                print("Turning left")
                return

            else:
                turn('right')
                turn('right')
                print("Turning 180")
                direction = DrivingDirection.towards_destination
                return
        else:
            move_forward()
            return

    elif (direction == DrivingDirection.away_from_destination) and (y == -1 and x == -1):
        if blocked_state['centre']:
            if not blocked_state['right']:
                turn('right')
                direction = DrivingDirection.left
                print("Turning right")
                return

            elif not blocked_state['left']:
                turn('left')
                direction = DrivingDirection.right
                print("Turning left")
                return

            else:
                turn('right')
                turn('right')
                print("Turning 180")
                direction = DrivingDirection.towards_destination
                return
        else:
            move_forward()
            return

    elif (direction == DrivingDirection.away_from_destination) and (y == -1 and x == 1):
        if blocked_state['centre']:
            if not blocked_state['left']:
                turn('left')
                direction = DrivingDirection.right
                print("Turning left")
                return

            elif not blocked_state['right']:
                turn('right')
                direction = DrivingDirection.left
                print("Turning right")
                return

            else:
                turn('right')
                turn('right')
                print("Turning 180")
                direction = DrivingDirection.towards_destination
                return
        else:
            move_forward()
            return

def updateMap(blocked_state):
    global direction
    global map
    global car_position

    if map[car_position[0], car_position[1]] != 1:
        map[car_position[0], car_position[1]] = 8

    # Inital filter on driving direction
    if direction == DrivingDirection.towards_destination:
        if blocked_state['left']:
            map[car_position[0],car_position[1]-1] = 1
        else:
            if map[car_position[0],car_position[1]-1] != 1:
                map[car_position[0],car_position[1]-1] = 0

        if blocked_state['centre']:
            map[car_position[0]-1, car_position[1]] = 1
        else:
            if map[car_position[0]-1, car_position[1]] != 1:
                map[car_position[0]-1, car_position[1]] = 0

        if blocked_state['right']:
            map[car_position[0], car_position[1]+1] = 1
        else:
            if map[car_position[0], car_position[1]+1] != 1:
                map[car_position[0], car_position[1]+1] = 0

    elif direction == DrivingDirection.right:
        if blocked_state['left']:
            map[car_position[0]-1,car_position[1]] = 1
        else:
            if map[car_position[0]-1,car_position[1]] != 1:
                map[car_position[0]-1,car_position[1]] = 0

        if blocked_state['centre']:
            map[car_position[0], car_position[1]+1] = 1
        else:
            if map[car_position[0], car_position[1]+1] != 1:
                map[car_position[0], car_position[1]+1] = 0

        if blocked_state['right']:
            map[car_position[0]+1, car_position[1]] = 1
        else:
            if map[car_position[0]+1, car_position[1]] != 1:
                map[car_position[0]+1, car_position[1]] = 0

    elif direction == DrivingDirection.left:
        if blocked_state['left']:
            map[car_position[0]+1,car_position[1]] = 1
        else:
            if map[car_position[0]+1,car_position[1]] != 1:
                map[car_position[0]+1,car_position[1]] = 0

        if blocked_state['centre']:
            map[car_position[0], car_position[1]-1] = 1
        else:
            if map[car_position[0], car_position[1]-1] != 1:
                map[car_position[0], car_position[1]-1] = 0

        if blocked_state['right']:
            map[car_position[0]-1, car_position[1]] = 1
        else:
            if map[car_position[0]-1, car_position[1]] != 1:
                map[car_position[0]-1, car_position[1]] = 0

    else: # If direction away frpm destination
        if blocked_state['left']:
            map[car_position[0],car_position[1]+1] = 1
        else:
            if map[car_position[0],car_position[1]+1] != 1:
                map[car_position[0],car_position[1]+1] = 0

        if blocked_state['centre']:
            map[car_position[0]-1, car_position[1]] = 1
        else:
            if map[car_position[0]-1, car_position[1]] != 1:
                map[car_position[0]-1, car_position[1]] = 0

        if blocked_state['right']:
            map[car_position[0], car_position[1]-1] = 1
        else:
            if map[car_position[0], car_position[1]-1] != 1:
                map[car_position[0], car_position[1]-1] = 0


def main(model: str, camera_id: int, width: int, height: int, num_threads: int,
        enable_edgetpu: bool, stationary_run: bool,
        obj_det_thresh: float, _driver: str,
        destX: int, destY: int, startX: int, startY: int) -> None:
    print("Sys max size ", sys.maxsize)
    np.set_printoptions(threshold=sys.maxsize)

    # Apply driver id for car tune
    global driver
    driver = _driver

    # Apply input params for starting position and destination
    global car_position
    # Initate dictionary to hold detected obstacle location in front of car
    blocked_state = {
        'left': False,
        'centre': False,
        'right': False
    }
    #starting point - this needs to update as the car moves
    if not startX:
        startX=int(input(f"Enter the x coordinate of the starting point between 0 and {array_shape[0]}\n"))
        while (startX>array_shape[0]) or (startX<=0):
            startX=int(input(f"The number needs to be netween 0 and {array_shape[0]}. Please enter again\n"))
    if not startY:
        startY=int(input(f"Enter the y coordinate of the starting point between 0 and {array_shape[1]}\n"))
        while (startY>array_shape[1]) or (startY<=0):
            startY=int(input(f"The number needs to be netween 0 and {array_shape[1]}. Please enter again\n"))

    car_position[:] = (startX,startY)
    starting_point = (startX, startY)
    print(f"Starting point is {starting_point}")

    #Asking user to enter the coordinates for the destination
    if not destX:
        destX=int(input(f"Enter the x coordinate of the target between 0 and {array_shape[0]}\n"))
        while (destX>array_shape[0]) or (destX<=0):
            destX=int(input(f"The number needs to be netween 0 and {array_shape[0]}. Please enter again\n"))
    if not destY:
        destY=int(input(f"Enter the y coordinate of the target between 0 and {array_shape[1]}\n"))
        while (destY>array_shape[1]) or (destY<=0):
            destY=int(input(f"The number needs to be netween 0 and {array_shape[1]}. Please enter again\n"))

    global target
    #Target is set by the user - doesn't change while the car is driving
    target = (destX,destY)
    print(f"Target point is {target}")

    # Use multiprocessing manager to allow shared list between processes
    with mp.Manager() as manager:
        # Create object detector object which will allow us to process
        # video frames one at a time
        if not disable_camera:
            object_detector = lab1part2_object_detector.Object_detector(
                model, camera_id, width, height,
                num_threads, enable_edgetpu
            )

        # List of strings for names of objects detected in camera stream
        detected_objects = manager.list()
        already_serviced_objects = list()

        # Wrap main loop in try block so we can safely close
        # object detection thread if we hit an exception
        try:
            # Create and start the object detection child thread
            if not disable_camera:
                obj_det_proc = mp.Process(
                    target=object_detector.run_blocking,
                    args=(detected_objects, obj_det_thresh,)
                )
                obj_det_proc.start()

                # Wait a bit to make sure the video window is up and running before the car starts moving
                OBJ_DET_WARMUP_TIME_SEC = 9
                time.sleep(OBJ_DET_WARMUP_TIME_SEC)

            # Start loop to perform scan and take respective actions
            while True:
                # Act based on detected objects.
                if len(detected_objects) > 0:
                    print(detected_objects)

                if 'stop sign' in detected_objects:
                    if 'stop sign' not in already_serviced_objects:
                        already_serviced_objects.append('stop sign')
                        print("Pausing for 5 sec...")
                        time.sleep(5)
                        print("Continuing")
                else:
                    if 'stop sign' in already_serviced_objects:
                        already_serviced_objects.remove('stop sign')

                # Get ultrasonic scan input
                scan_list = fc.scan_step(25)
                if not scan_list:
                    continue
                # Wait for full scan to be received from the sensor
                if len(scan_list) != 10:
                    continue

                # Check for obstacles
                blocked_state = check_scan(scan_list, blocked_state)
                # Update map
                updateMap(blocked_state)

                new_car_position = (car_position[0],car_position[1])
                x_coords.append(car_position[0])
                y_coords.append(car_position[1])
                route = a_star_algorithm(map, new_car_position, target)
                print(route)
                if len(route) == 0:
                    print("########################")
                    print("##### WE MADE IT!! #####")
                    print("########################")
                    fc.stop()
                    print_map(starting_point[1],starting_point[0])
                    break
                routemap=route_map(route)

                print("____________________________________________________")
                print(f"route (distance {len(route)}):\n{route}")
                print(f"pos: {car_position} dest: {target}")
                print(f"blocked: {[direction if state else '' for direction, state in blocked_state.items()]}")

                if not stationary_run:
                    decide_on_action(blocked_state,route,new_car_position)

                print("____________________________________________________")

        finally:
            # Send the stop signal to the child process and make sure it terminates
            if not disable_camera:
                print("Sending stop signal to object detection process")
                object_detector.should_stop = True
                obj_det_proc.terminate()
                obj_det_proc.join()
                print("Object detection process closed safely.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '--model',
        help='Path of the object detection model.',
        required=False,
        default='efficientdet_lite0.tflite')
    parser.add_argument(
        '--cameraId', help='Id of camera.', required=False, type=int, default=0)
    parser.add_argument(
        '--frameWidth',
        help='Width of frame to capture from camera.',
        required=False,
        type=int,
        default=640)
    parser.add_argument(
        '--frameHeight',
        help='Height of frame to capture from camera.',
        required=False,
        type=int,
        default=480)
    parser.add_argument(
        '--numThreads',
        help='Number of CPU threads to run the model.',
        required=False,
        type=int,
        default=1)
    parser.add_argument(
        '--enableEdgeTPU',
        help='Whether to run the model on EdgeTPU.',
        action='store_true',
        required=False,
        default=False)
    parser.add_argument(
        '--stationary_run',
        help='Whether to stay stationary for the entire run',
        action='store_true',
        required=False,
        default=False)
    parser.add_argument(
        '--detectThresh',
        help='Threshold for object detection, int, out of 100',
        required=False,
        type=int,
        default=50)
    parser.add_argument(
        '--driver',
        help='Who is driving the car??',
        required=False,
        type=str,
        default='default')
    parser.add_argument(
        '--destX',
        help='X coordinate of destination',
        required=False,
        type=int,
        default=None)
    parser.add_argument(
        '--destY',
        help='X coordinate of destination',
        required=False,
        type=int,
        default=None)
    parser.add_argument(
        '--startX',
        help='X coordinate of starting position',
        required=False,
        type=int,
        default=None)
    parser.add_argument(
        '--startY',
        help='X coordinate of destination',
        required=False,
        type=int,
        default=None)
    args = parser.parse_args()

    print("If you want to quit, please press q")

    try:
        main(
            args.model, int(args.cameraId), args.frameWidth, args.frameHeight, int(args.numThreads),
            bool(args.enableEdgeTPU), bool(args.stationary_run),
            float(args.detectThresh/100.0), args.driver,
            args.destX, args.destY, args.startX, args.startY
        )
    finally:
        fc.stop()

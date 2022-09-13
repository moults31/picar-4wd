import picar_4wd as fc
import time
import numpy as np
import sys
import math
from enum import Enum
import heapq


# Define enum for holding driving direction, in relation to the destination, which can be defined
# as infinity in the direction of the starting position of the car.
class DrivingDirection (Enum):
    towards_destination = 1
    right = 2
    left =3
    away_from_destination = 4

# Set speed of car
speed = 5

# Set starting direction of car as toward destination
direction = DrivingDirection.towards_destination

# Initialise counter for measuring distance
distance_counter = 0
forward_timer = 0

# Initialise array representing 20 * 20 occupany squares of approx 20cm 
# when 7 represents unmapped areas, 0 represents clear and 1 represents obstacle
array_shape = (25, 25)
fill_value = 7
map = np.full(array_shape, fill_value)

# Intialise car location in array
car_position = [24, 10]

def route_map(route):
    route_map = np.full((25, 25), 7)
    for i in (range(0,len(route))):
        x = route[i][0]
        y = route[i][1]
        route_map[x][y]=0
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
    return False     

# Execute turn of car
def turn(turning_direction):

    global distance_counter

    # Set time for turning action for a period in seconds which gives a 90 degree turn angle.
    # Different timers needed for left and right turns to maintain consistent turning angle
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

# Decide on the action based on the blocked state and the direction of the car
# in relation to the destination
def decide_on_action(blocked_state, route, new_car_position):
    print("Car position ", new_car_position)
    # Use global variable
    global direction

    next_point=route[-1]
    #y
    y=new_car_position[0]-next_point[0]
    #x
    x=new_car_position[1]-next_point[1]

    #UP
    if (direction == DrivingDirection.towards_destination) and (y == 1 and x == 0):
        if blocked_state['centre']:
            if not blocked_state['left']:
                turn('left')
                direction = DrivingDirection.left
                print("Turning left")
                move_forward()
                return

            elif not blocked_state['right']:
                turn('right')
                direction = DrivingDirection.right
                print("Turning right")
                move_forward()
                return
        
            else:
                turn('right')
                turn('right')
                print("Turning 180")
                direction = DrivingDirection.away_from_destination
                move_forward()
                return
        else:
            move_forward()
            return

    elif (direction == DrivingDirection.towards_destination) and (y == 1 and x == -1):
        if blocked_state['centre']:
            if not blocked_state['right']:
                turn('right')
                direction = DrivingDirection.right
                print("Turning right")
                move_forward()
                return

            elif not blocked_state['left']:
                turn('left')
                direction = DrivingDirection.left
                print("Turning left")
                move_forward()
                return
            else:
                turn('right')
                turn('right')
                print("Turning 180")
                direction = DrivingDirection.away_from_destination
                move_forward()
                return
        else:
            move_forward()
            return

    elif (direction == DrivingDirection.towards_destination) and (y == 1 and x == 1):
        if blocked_state['centre']:
            if not blocked_state['left']:
                turn('left')
                direction = DrivingDirection.left
                print("Turning left")
                move_forward()
                return

            elif not blocked_state['right']:
                turn('right')
                direction = DrivingDirection.right
                print("Turning right")
                move_forward()
                return
        
            else:
                turn('right')
                turn('right')
                print("Turning 180")
                direction = DrivingDirection.away_from_destination
                move_forward()
                return
        else:
            move_forward()
            return
            
    elif (direction == DrivingDirection.towards_destination) and (y == 0 and x == -1):
        if blocked_state['right']:
            if not blocked_state['center']:
                direction = DrivingDirection.towards_destination
                move_forward()
                return

            elif not blocked_state['left']:
                turn('left')
                direction = DrivingDirection.left
                print("Turning left")
                move_forward()
                return
        
            else:
                turn('right')
                turn('right')
                print("Turning 180")
                direction = DrivingDirection.away_from_destination
                move_forward()
                return
        else:
            turn('right')
            direction = DrivingDirection.right
            print("Turning right")
            move_forward()
            return

    elif (direction == DrivingDirection.towards_destination) and (y == 0 and x == 1):
        if blocked_state['left']:
            if not blocked_state['center']:
                direction = DrivingDirection.towards_destination
                move_forward()
                return

            elif not blocked_state['right']:
                turn('right')
                direction = DrivingDirection.right
                print("Turning right")
                move_forward()
                return
        
            else:
                turn('right')
                turn('right')
                print("Turning 180")
                direction = DrivingDirection.away_from_destination
                move_forward()
                return
        else:
            turn('left')
            direction = DrivingDirection.left
            print("Turning left")
            move_forward()
            return

    elif (direction == DrivingDirection.towards_destination) and (y == 0 and x == 0):
        return
        
    elif (direction == DrivingDirection.towards_destination) and (y == -1 and x == 0):
        turn('right')
        turn('right')
        print("Turning 180")
        direction = DrivingDirection.away_from_destination
        move_forward()
        return

    elif (direction == DrivingDirection.towards_destination) and (y == -1 and x == -1):
        if blocked_state['right']:
            if not blocked_state['left']:
                turn('left')
                direction = DrivingDirection.left
                print("Turning left")
                move_forward()
                return

            elif not blocked_state['center']:
                direction = DrivingDirection.towards_destination
                move_forward()
                return
            else:
                turn('right')
                turn('right')
                print("Turning 180")
                direction = DrivingDirection.away_from_destination
                move_forward()
                return
        else:
            turn('right')
            direction = DrivingDirection.right
            print("Turning right")
            move_forward()
            return

    elif (direction == DrivingDirection.towards_destination) and (y == -1 and x == 1):
        if blocked_state['left']:
            if not blocked_state['right']:
                turn('right')
                direction = DrivingDirection.right
                print("Turning right")
                move_forward()
                return

            elif not blocked_state['center']:
                direction = DrivingDirection.towards_destination
                move_forward()
                return
            else:
                turn('right')
                turn('right')
                print("Turning 180")
                direction = DrivingDirection.away_from_destination
                move_forward()
                return
        else:
            turn('left')
            direction = DrivingDirection.left
            print("Turning left")
            move_forward()
            return
    
    #RIGHT
    if (direction == DrivingDirection.right) and (y == 1 and x == 0):
        if blocked_state['left']:
            if not blocked_state['center']:
                direction = DrivingDirection.right
                move_forward()
                return

            elif not blocked_state['right']:
                turn('right')
                direction = DrivingDirection.away_from_destination
                print("Turning right")
                move_forward()
                return

            else:
                turn('right')
                turn('right')
                print("Turning 180")
                direction = DrivingDirection.left
                move_forward()
                return
        else:
            turn('left')
            direction = DrivingDirection.towards_destination
            print("Turning left")
            move_forward()
            return

    elif (direction == DrivingDirection.right) and (y == 1 and x == -1):
        if blocked_state['centre']:
            if not blocked_state['left']:
                turn('left')
                direction = DrivingDirection.towards_destination
                print("Turning left")
                move_forward()
                return

            elif not blocked_state['right']:
                turn('right')
                direction = DrivingDirection.away_from_destination
                print("Turning right")
                move_forward()
                return
        
            else:
                turn('right')
                turn('right')
                print("Turning 180")
                direction = DrivingDirection.left
                move_forward()
                return
        else:
            move_forward()
            return

    elif (direction == DrivingDirection.right) and (y == 1 and x == 1):
        if blocked_state['left']:
            if not blocked_state['center']:
                direction = DrivingDirection.right
                move_forward()
                return

            elif not blocked_state['right']:
                turn('right')
                direction = DrivingDirection.away_from_destination
                print("Turning right")
                move_forward()
                return
        
            else:
                turn('right')
                turn('right')
                print("Turning 180")
                direction = DrivingDirection.left
                move_forward()
                return
        else:
            turn('left')
            direction = DrivingDirection.towards_destination
            print("Turning left")
            move_forward()
            return

    elif (direction == DrivingDirection.right) and (y == 0 and x == -1):
        if blocked_state['centre']:
            if not blocked_state['left']:
                turn('left')
                direction = DrivingDirection.towards_destination
                print("Turning left")
                move_forward()
                return

            elif not blocked_state['right']:
                turn('right')
                direction = DrivingDirection.away_from_destination
                print("Turning right")
                move_forward()
                return
        
            else:
                turn('right')
                turn('right')
                print("Turning 180")
                direction = DrivingDirection.left
                move_forward()
                return
        else:
            move_forward()
            return

    elif (direction == DrivingDirection.right) and (y == 0 and x == 1):
        turn('right')
        turn('right')
        print("Turning 180")
        direction = DrivingDirection.left
        move_forward()
        return

    elif (direction == DrivingDirection.right) and (y == 0 and x == 0):
        return

    elif (direction == DrivingDirection.right) and (y == -1 and x == 0):
        if blocked_state['right']:
            if not blocked_state['center']:
                move_forward()
                return
        
            elif not blocked_state['left']:
                turn('left')
                direction = DrivingDirection.towards_destination
                print("Turning left")
                move_forward()
                return

            else:
                turn('right')
                turn('right')
                print("Turning 180")
                direction = DrivingDirection.left
                move_forward()
                return
        else:
            turn('right')
            direction = DrivingDirection.away_from_destination
            print("Turning right")
            move_forward()
            return

    elif (direction == DrivingDirection.right) and (y == -1 and x == -1):
        if blocked_state['centre']:
            if not blocked_state['right']:
                turn('right')
                direction = DrivingDirection.away_from_destination
                print("Turning right")
                move_forward()
                return
        
            elif not blocked_state['left']:
                turn('left')
                direction = DrivingDirection.towards_destination
                print("Turning left")
                move_forward()
                return

            else:
                turn('right')
                turn('right')
                print("Turning 180")
                direction = DrivingDirection.left
                move_forward()
                return
        else:
            move_forward()
            return

    elif (direction == DrivingDirection.right) and (y == -1 and x == 1):
        turn('right')
        turn('right')
        print("Turning 180")
        direction = DrivingDirection.left
        move_forward()
    
    #LEFT
    if (direction == DrivingDirection.left) and (y == 1 and x == 0):
        if blocked_state['right']:
            if not blocked_state['center']:
                direction = DrivingDirection.left
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
            turn('right')
            direction = DrivingDirection.towards_destination
            print("Turning right")
            move_forward()
            return

    elif (direction == DrivingDirection.left) and (y == 1 and x == -1):
        if blocked_state['right']:
            if not blocked_state['center']:
                direction = DrivingDirection.left
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
            turn('right')
            direction = DrivingDirection.towards_destination
            print("Turning right")
            move_forward()
            return

    elif (direction == DrivingDirection.left) and (y == 1 and x == 1):
        if blocked_state['center']:
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

    elif (direction == DrivingDirection.left) and (y == 0 and x == -1):
        turn('right')
        turn('right')
        print("Turning 180")
        direction = DrivingDirection.right
        move_forward()
        return

    elif (direction == DrivingDirection.left) and (y == 0 and x == 1):
        if blocked_state['center']:
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

    elif (direction == DrivingDirection.left) and (y == 0 and x == 0):
        return

    elif (direction == DrivingDirection.left) and (y == -1 and x == 0):
        if blocked_state['left']:
            if not blocked_state['center']:
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
        if blocked_state['left']:
            if not blocked_state['center']:
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
        if blocked_state['center']:
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
        pass
        #tylko do góry
    elif (direction == DrivingDirection.away_from_destination) and (y == 1 and x == -1):
        pass
        #do góry i w prawo
    elif (direction == DrivingDirection.away_from_destination) and (y == 1 and x == 1):
        pass
        #do góry i w lewo
    elif (direction == DrivingDirection.away_from_destination) and (y == 0 and x == -1):
        pass
        #tylko w prawo
    elif (direction == DrivingDirection.away_from_destination) and (y == 0 and x == 1):
        pass
        #tylko w lewo
    elif (direction == DrivingDirection.away_from_destination) and (y == 0 and x == 0):
        pass
        #koniec - stop
    elif (direction == DrivingDirection.away_from_destination) and (y == -1 and x == 0):
        pass
        #tylko w gół
    elif (direction == DrivingDirection.away_from_destination) and (y == -1 and x == -1):
        pass
        #w dół i w prawo
    elif (direction == DrivingDirection.away_from_destination) and (y == -1 and x == 1):
        pass
        #w dół i lewo

def updateMap(blocked_state):
    global direction
    global map
    global car_position

    if map[car_position[0], car_position[1]] != 1:
        map[car_position[0], car_position[1]] = 0

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
          

def main():
    print("Sys max size ", sys.maxsize)
    np.set_printoptions(threshold=sys.maxsize)

    # Initate dictionary to hold detected obstacle location in front of car
    blocked_state = {
        'left': False,
        'centre': False,
        'right': False
    }

    # Start loop to perform scan and take respective actions
    while True:
        # Get ultrasonic scan input 
        scan_list = fc.scan_step(25)
        #print(scan_list)
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
        route = a_star_algorithm(map, new_car_position, (18,10))
        routemap=route_map(route)
        decide_on_action(blocked_state,route,new_car_position)

        print(map)
        print(routemap)
        

if __name__ == "__main__":
    print("If you want to quit.Please press q")
    try: 
        main()
    finally: 
        fc.stop()
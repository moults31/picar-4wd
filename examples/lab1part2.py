import picar_4wd as fc
import time
import numpy as np
import sys
import math
import argparse
from enum import Enum
from threading import Thread

# User modules
import lab1part2_object_detector

# Enum for holding driving direction, in relation to the destination, which can be defined
# as infinity in the direction of car at its starting position.
class DrivingDirection (Enum):
    towards_destination = 1
    right = 2
    left =3
    away_from_destination = 4

# Globals

# Set speed of car
speed = 1

# Set starting direction of car as toward destination
direction = DrivingDirection.towards_destination

# Initialise counter for measuring distance
distance_counter = 0

# Execute turn of car
def turn(turning_direction):

    global distance_counter

    # Set time for turning action for a period in seconds which gives a 90 degree turn angle.
    # Different timers needed for left and right turns to maintain consistent turning angle
    turn_left_timer = 0.9
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

    # Reset distance counter as this is used to ensure car moves a certain distance
    # forward after each turn before attempting a turn towards destination
    distance_counter = 0
    return

# Move car forward and update distance counter each time function is called
def move_forward():
    global distance_counter
    fc.forward(speed)
    # Sleep while car travels 5 cm
    #time.sleep(0.16)
    distance_counter += 1
    print("Distance counter ", distance_counter)
    # Car moves 16cm forward each time
    # updatePositionMovingForward(1)
    return

# Check ultrasonic scan. Assessing left, centre and right parts of scan for obstacles
# return blocked state object
def check_scan(scan_list, blocked_state):
    if scan_list[0:3] != [ 2, 2, 2]:
        blocked_state['left'] = True
        print("Blocked left")
    else:
        blocked_state['left'] = False

    if scan_list[3:7] != [2, 2, 2, 2]:
        blocked_state['centre'] = True
        print("Blocked centre")
    else:
        blocked_state['centre'] = False
    
    if scan_list[7:10] != [2, 2, 2]:
        blocked_state['right'] = True
        print ("Blocked right")
    else:
        blocked_state['right'] = False
    return blocked_state

# Decide on the action based on the blocked state and the direction of the car
# in relation to the destination
def decide_on_action(blocked_state):

    # Use global variable
    global direction

    # Inital filter on driving direction
    if direction == DrivingDirection.towards_destination:

        if blocked_state['centre']:

            # Check if left or right appears to be clear and
            # if so attempt turn in that direction
            if not blocked_state['left']:
                turn('left')
                direction = DrivingDirection.left
                print("Turning left")
                return

            elif not blocked_state['right']:
                turn('right')
                direction = DrivingDirection.right
                print("Turning right")
                return
        
            else:
                # If no preferred direction then attempt to turn left
                turn('left')
                direction = DrivingDirection.left
                print("Turning left - default action")
                return

        else:
            #If clear in front then drive forward
            move_forward()


    elif direction == DrivingDirection.right:

        # Want to turn left towards destination if possible and car has travelled at least 
        # about a car length forward
        if not blocked_state['left'] and distance_counter >= 2:
            turn('left')
            direction = DrivingDirection.towards_destination
            print("Turning left")
            return
        
        # If cannot turn left continue if possible and if not then turn
        if blocked_state['centre']:

            if not blocked_state['right']:
                turn('right')
                direction = DrivingDirection.away_from_destination
                print("Turning right")
                return
        
            else:
                turn('left')
                direction = DrivingDirection.towards_destination
                print("Turning left - default action")
                return

        else:
            #If clear in front then drive forward
            move_forward()

    elif direction == DrivingDirection.left:
        print("In driving direction left")

        # Want to turn right towards destination if possible and car has travelled at least 
        # about a car length forward
        if not blocked_state['right'] and distance_counter >= 2:
            turn('right')
            direction = DrivingDirection.towards_destination
            print("Turning right")
            return

        # If cannot turn right continue if possible and if not then turn
        if blocked_state['centre']:

            if not blocked_state['left']:
                turn('left')
                direction = DrivingDirection.away_from_destination
                print("Turning left")
                return
        
            else:
                turn('right')
                direction = DrivingDirection.towards_destination
                print("Turning right - default action")
                return

        else:
            #If clear in front then drive forward
            move_forward()

    elif direction == DrivingDirection.away_from_destination:
        print("In driving direction - away from destination")

        # Want to turn right or left back towards destination if possible and car has travelled at least 
        # about a car length forward
        if not blocked_state['right'] and distance_counter >= 2:
            turn('right')
            direction = DrivingDirection.left
            print("Turning right")
            return

        if not blocked_state['left'] and distance_counter >= 2:
            turn('left')
            direction = DrivingDirection.right
            print("Turning left")
            return

        # If cannot turn right continue if possible and if not then turn
        if blocked_state['centre']:
            turn('right')
            direction = DrivingDirection.towards_destination
            print("Turning right - default action")
            return

        else:
            #If clear in front then drive forward
            move_forward()
    else:
        return 
        

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

def main(model: str, camera_id: int, width: int, height: int, num_threads: int,
        enable_edgetpu: bool, stationary_run: bool, obj_det_thresh: float) -> None:

    print("Sys max size ", sys.maxsize)
    np.set_printoptions(threshold=sys.maxsize)
    #np.set_printoptions(threshold=220)

    # Initate dictionary to hold detected obstacle location in front of car
    blocked_state = {
        'left': False,
        'centre': False,
        'right': False
    }

    # Create object detector object which will allow us to process
    # video frames one at a time
    object_detector = lab1part2_object_detector.Object_detector(
        model, camera_id, width, height,
        num_threads, enable_edgetpu
    )

    # List of strings for names of objects detected in camera stream
    detected_objects = []

    # Process some frames on the main thread before entering main loop
    # to make sure the video window is up and running before the car starts moving
    WARMUP_FRAMES = 20
    for frame in range(WARMUP_FRAMES):
        object_detector.process_frame(detected_objects)

    # Calculate the period for object detector frame processing,
    # to achieve the target FPS. If we run at the maximum possible FPS
    # (Around 6 or 7) the main thread will be cpu-starved
    obj_det_period = 1 / object_detector.target_fps


    # Wrap main loop in try block so we can safely close
    # object detection thread if we hit an exception
    try:
        # Create and start the object detection child thread
        obj_det_thread = Thread(
            target=object_detector.run_periodic_blocking,
            args=(obj_det_period, detected_objects, obj_det_thresh,)
        )
        obj_det_thread.start()

        # Start main loop to perform scan and take respective actions
        while True:
            # Act based on detected objects.
            # Performed at the start of the loop because we should still act
            # if the previous iteration got "continued"
            # Note 1: could this be pulled into a helper function, or merged with decide_on_action?
            # Note 2: could this be pulled into a helper function, or merged with decide_on_action?
            print(detected_objects)
            if 'stop sign' in detected_objects:
                    fc.stop
                    break

            # Get ultrasonic scan input 
            scan_list = fc.scan_step(35)
            print(scan_list)
            if not scan_list:
                continue
            # Wait for full scan to be received from the sensor
            if len(scan_list) != 10:
                continue

            # Check for obstacles
            blocked_state = check_scan(scan_list, blocked_state)
            print(scan_list)
            print ("Blocked state ", blocked_state)

            # Update map
            # if not stationary_run:
                # updateMap(blocked_state)

            # Check for objects
            obj_det_running = False

            #Decide on actions based on obstacles and current driving direction
            if not stationary_run:
                # print("Car position ", car_position)
                decide_on_action(blocked_state)

            print(map)
            #print(map[car_position[0]-25:car_position[0]+25, car_position[1]-25:car_position[1]+25])
            #plt.imshow(map, interpolation='nearest')
            #plt.show()

    finally:
        object_detector.should_stop = True
        obj_det_thread.join()
        print("Object detection thread closed safely.")



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
    args = parser.parse_args()

    print("If you want to quit.Please press q")

    try: 
        main(
            args.model, int(args.cameraId), args.frameWidth, args.frameHeight, int(args.numThreads),
            bool(args.enableEdgeTPU), bool(args.stationary_run), float(args.detectThresh/100.0)
        )
    finally: 
        fc.stop()
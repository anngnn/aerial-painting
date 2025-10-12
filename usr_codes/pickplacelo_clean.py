
'''
---------
Flyer API
---------

flyer.id            : returns an integer unique id for the flyer

flyer.state()       : requires nothing
                    : returns a numpy array of the 12 part system state if a state is available. Otherwise, returns a numpy array of len 1 (np.zeros(1)) ([0]).
                    : state is: x, y, z, xdot, ydot, zdot, roll, pitch, yaw, rolldot, pitchdot, yawdot 
                    : The returned state may not be a 'fresh' or 'new' state. 

flyer.waypoint(s)   : requires a 12 part numpy array system state in the form: x, y, z, xdot, ydot, zdot, roll, pitch, yaw, rolldot, pitchdot, yawdot
                    : returns nothing

flyer.select_controller(str)    : requires a string from the following list of options: 'pid' 'lqr'
                                : sets which controller the flyer will run to get to the waypoint
                                : returns nothing.

flyer.log(str)      : requires a string message to be logged (similar to print())
                    : returns nothing

flyer.delay(float)  : requires a float time value (in seconds)
                    : flyer user code will block for the specified time
                    : returns nothing

flyer.send_msg(msg) : requires a struct-packed message (max length of 1024 bytes)
                    : Attempts to broadcast a message to peer robots
                    : returns True if success, False if failed

flyer.recv_msg()    : Attempts to return the buffer of received messages (up to 100 messages of 1024 bytes each)
                    : Returns False if failed.
                    : If successful, retruns a list of struct-packed messages that the user must unpack.

flyer.arm()         : Arm the flight controller. Best done AFTER an intial setpoint is set so to avoid bounding box errors

flyer.disarm()      : Disarm the flight controller

flyer.mode(mode)    : Set the flyer mode to 
                        # 1500 == horizon
                        # 1000 == angle
                        # 1800 == RATE 


DEVELOPERS NOTE: arm(), disarm(), and mode() EACH result in a 200 ms blocking delay.

'''
import numpy as np
import time
import struct

'''
UP, RIGHT, DOWN, UP, RIGHT2, DOWN, TWIST_YAW, UP
'''
def usr(flyer):
    list_waypoints = []     # list of all waypoints
    time_at_waypoint = []   # time to spend at each waypoint

    START = time.time()


    flyer.select_controller('lqr')

    flyer.log(['user code started'])

    # get the first position
    while True:
        state = flyer.state()
        if len(state) > 1:
            first_pos = np.copy(state)
            break
    
    flyer.log(['The user code is running. The flyer is localized to: ' + str(first_pos)])
    
    def make_wp(base_setpoint, unit_vector, distance):
        unit_vec = np.array(unit_vector,dtype='int')
        vec = (distance/np.linalg.norm(unit_vec))*unit_vec

        setpoint = base_setpoint + np.array([vec[0], vec[1], vec[2], 0, 0, 0, 0, 0, 0, 0, 0, 0])
        return setpoint
    
    def make_gradual_wp(base_setpoint, unit_vector, distance, step_size, going_down):
        remaining_dist = distance
        prev_setpt = base_setpoint
        multiplier = -1 if going_down else 1

        while remaining_dist > 0:
            step = min(step_size, remaining_dist)
            curr_setpoint = make_wp(prev_setpt, unit_vector, step*multiplier)
            list_waypoints.append(curr_setpoint)
            prev_setpt = curr_setpoint
            remaining_dist -= step
    
    # set the first waypoint
    setpoint = first_pos
    setpoint[2] = setpoint[2] + 0.5
    setpoint[3:] = 0
    flyer.waypoint(setpoint)
    current_waypoint = np.copy(setpoint)

    list_waypoints.append(setpoint)
    time_at_waypoint.append(3)


    # ABOVE droxel
    setpoint2 = make_wp(setpoint, [1,0,0], 0.5)
    list_waypoints.append(setpoint2)
    time_at_waypoint.append(3)

    # DOWN to pick
    idx_before_pick = len(list_waypoints)
    make_gradual_wp(setpoint2, [0,0,1], 0.55, 0.05, going_down=True)
    for _ in range (len(list_waypoints) - idx_before_pick):
        time_at_waypoint.append(1)
    pick_waypoint = list_waypoints[-1]

    # UP after pick
    setpoint4 = make_wp(pick_waypoint, [0,0,1], 0.55)
    list_waypoints.append(setpoint4)
    time_at_waypoint.append(3)

    # ABOVE droxel placement
    setpoint5 = make_wp(setpoint4, [0,1,0], 0.5)
    list_waypoints.append(setpoint5)
    time_at_waypoint.append(3)

    # DOWN to place
    idx_before_place = len(list_waypoints)
    make_gradual_wp(setpoint5, [0,0,1], 0.54, 0.05, going_down=True)
    for _ in range(len(list_waypoints) - idx_before_place):
        time_at_waypoint.append(1)
    place_waypoint = list_waypoints[-1]

    # TWIST YAW to release droxel
    setpoint7 = place_waypoint + np.array([0, 0, 0, 0, 0, 0, 0, 0, -3*np.pi/4, 0, 0, 0])
    list_waypoints.append(setpoint7)
    time_at_waypoint.append(1.5)

    # UP to release droxel
    idx_before_release = len(list_waypoints)
    make_gradual_wp(setpoint7, [0, 0, 1], 0.56, 0.05, going_down=False)
    for _ in range(len(list_waypoints) - idx_before_release):
        time_at_waypoint.append(1)


    # BACK to setpoint2 (above droxel)
    setpoint9 = make_wp(setpoint2, [1, 1, 0], -0.5)
    list_waypoints.append(setpoint9)
    time_at_waypoint.append(5)

    # arm the flyer
    flyer.arm()
    flyer.run_controller()

    stepped = False

    current_waypoint_idx = 0
    current_waypoint = list_waypoints[current_waypoint_idx].copy()
    flyer.waypoint(current_waypoint)
    time_of_last_switch = time.time() - START

    while True:

        flyer.delay()

        # log our state
        state = flyer.state()
        if len(state) > 1:
            rounded_time = np.round(time.time() - START, 3)
            rounded_state = np.round(state,2)
            rounded_state_list = rounded_state.tolist()
            volts = flyer.get_volts()
            watts = flyer.get_watts()
            log_list = [rounded_time] + rounded_state_list + [volts, watts]
            flyer.log(log_list)

        current_time = time.time() - START
        time_at_current = current_time - time_of_last_switch

        if time_at_current >= time_at_waypoint[current_waypoint_idx]:
            if current_waypoint_idx < len(list_waypoints) - 1: 
                current_waypoint_idx += 1
                current_waypoint = list_waypoints[current_waypoint_idx].copy()
                flyer.waypoint(current_waypoint)
                time_of_last_switch = current_time
                print(f'New waypoint: {current_waypoint_idx}')
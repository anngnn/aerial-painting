
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
UP, ABOVE_PICK, DOWN, UP, ABOVE_PLACE, DOWN, TWIST_YAW, UP
'''


def usr(flyer):
    global current_waypoint_index
    current_waypoint_index = 0

    START = time.time()


    flyer.select_controller('lqr')

    flyer.log(['user code started'])

    #get the first position
    while True:
        state = flyer.state()
        if len(state) > 1:
            first_pos = np.copy(state)
            break
    
    flyer.log(['The user code is running. The flyer is localized to: ' + str(first_pos)])
    
    list_waypoints = []
    
    def make_wp(base_setpoint, unit_vector, distance):
        unit_vec = np.array(unit_vector,dtype='int')
        vec = (distance/np.linalg.norm(unit_vec))*unit_vec

        setpoint = base_setpoint + np.array([vec[0], vec[1], vec[2], 0, 0, 0, 0, 0, 0, 0, 0, 0])
        return setpoint

    def move_vertical_increment(base_setpoint, dist, step_size):
        small_dist = dist / step_size
        curr_setpoint = base_setpoint
        for _ in range(step_size):
            setpoint = make_wp(curr_setpoint, [0,0,1], small_dist)
            list_waypoints.append(setpoint)
            curr_setpoint = setpoint
        return curr_setpoint  # Return the final setpoint

    #set the first waypoint
    setpoint = first_pos
    setpoint[2] = setpoint[2] + 0.5
    setpoint[3:] = 0
    flyer.waypoint(setpoint)
    list_waypoints.append(np.copy(setpoint)) 

    #ABOVE droxel
    setpoint2 = make_wp(setpoint, [1,0,0], 0.5)
    list_waypoints.append(setpoint2)

    #DOWN to pick
    setpoint3_final = move_vertical_increment(setpoint2, -0.46, 5)

    #UP after pick
    setpoint4 = make_wp(setpoint3_final, [0,0,1], 0.55)
    list_waypoints.append(setpoint4)

    #ABOVE droxel placement
    setpoint5 = make_wp(setpoint4, [0,1,0], 0.5)
    list_waypoints.append(setpoint5)

    #DOWN to place
    setpoint6 = move_vertical_increment(setpoint5, -0.55, 5)

    #TWIST YAW to release droxel(shorter time at this setpoint)
    setpoint7 = setpoint6 + np.array([0, 0, 0, 0, 0, 0, 0, 0, -np.pi/2, 0, 0, 0])
    list_waypoints.append(setpoint7)

    #UP to release droxel
    setpoint8 = make_wp(setpoint7, [0,0,1], 0.65)
    list_waypoints.append(setpoint8)

    #BACK to setpoint2 (above droxel)
    setpoint9 = setpoint2
    setpoint9 = make_wp(setpoint9, [1,1,0], -0.5)
    list_waypoints.append(setpoint9)

    #arm the flyer
    flyer.arm()
    flyer.run_controller()

    current_time = time.time() - START
    time_at_each_setpoint = 5 #seconds
    time_of_last_switch = 0

    stepped = False

    while True:

        # print('looping')

        flyer.delay()

        #log our state
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
        
        time_of_last_switch = 0
        if current_waypoint_index == 6:
            delay_for_this_waypoint = 1  # 1 second for yaw twist
        else:
            delay_for_this_waypoint = 5  # 5 seconds for all others

        # Check if it's time to switch waypoints
        if (current_time - time_of_last_switch >= delay_for_this_waypoint):
            if current_waypoint_index < len(list_waypoints):
                print('Switching to waypoint', current_waypoint_index)
                current_waypoint = np.copy(list_waypoints[current_waypoint_index])
                flyer.waypoint(current_waypoint)
                time_of_last_switch = current_time
                current_waypoint_index += 1
            

        

        


        
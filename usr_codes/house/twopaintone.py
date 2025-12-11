
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
    
    # def make_wp(base_setpoint, unit_vector, distance):
    #     unit_vec = np.array(unit_vector,dtype='int')
    #     vec = (distance/np.linalg.norm(unit_vec))*unit_vec

    #     setpoint = base_setpoint + np.array([vec[0], vec[1], vec[2], 0, 0, 0, 0, 0, 0, 0, 0, 0])
    #     return setpoint

    def make_wp(base_setpoint, unit_vector, distance, 
                xdot=0, ydot=0, zdot=0, 
                roll=0, pitch=0, yaw=0, 
                rolldot=0, pitchdot=0, yawdot=0):
        unit_vec = np.array(unit_vector, dtype='float')
        vec = (distance / np.linalg.norm(unit_vec)) * unit_vec
        
        setpoint = base_setpoint + np.array([
            vec[0], vec[1], vec[2],
            xdot, ydot, zdot,
            roll, pitch, yaw,
            rolldot, pitchdot, yawdot
        ])
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
    
    # Mission definition: each entry is (direction, distance, time, kwargs)
    # Use 'gradual' key for gradual movements
    mission = [
        # get y to FRONT side
        [[0, 1, 0], -0.35, 3],

        # DOT1
        # ABOVE point to paint
        [[1, 0, 0], 0.50, 5],
        [[0, 1, 0], 0.40, 3],
        [[1, 0, 0], -0.05, 3],
        # DOWN to paint (gradual)
        [[0, 0, -1], 0.46, 1, 0.02],
        # UP after painting
        [[0, 0, 1], 0.05, 3],

        # DOT2
        # X
        [[1, 0, 0], -0.025, 3],
        [[0, 0, -1], 0.05, 1, 0.02],
        [[0, 0, 1], 0.05, 3],
        # DOT3
        [[1, 0, 0], -0.025, 3],
        [[0, 0, -1], 0.05, 1, 0.02],
        [[0, 0, 1], 0.05, 3],


        # DOT4
        # XY
        [[1, 0, 0], -0.025, 3],
        [[0, 1, 0], 0.025, 3],
        [[0, 0, -1], 0.05, 1, 0.02],
        [[0, 0, 1], 0.05, 3],
        # DOT5
        # X
        [[0, 1, 0], 0.025, 3],
        [[0, 0, -1], 0.05, 1, 0.02],
        [[0, 0, 1], 0.05, 3],


        # DOT6
        [[0, 1, 0], 0.025, 3],
        [[1, 0, 0], 0.025, 3],
        [[0, 0, -1], 0.05, 1, 0.02],
        [[0, 0, 1], 0.05, 3],

        # DOT7
        [[1, 0, 0], 0.025, 3],
        [[0, 0, -1], 0.05, 1, 0.02],
        [[0, 0, 1], 0.05, 3],


        ##### EXITING #####
        # UP enough height to exit
        [[0, 0, 1], 0.15, 3],
        # FORWARD Y to EXIT
        [[0, 1, 0], 1.5, 1, 0.3],

    ]

    # set the first waypoint
    setpoint = first_pos
    setpoint[2] = setpoint[2] + 0.35
    setpoint[3:] = 0
    flyer.waypoint(setpoint)
    current_waypoint = np.copy(setpoint)

    list_waypoints.append(setpoint)
    time_at_waypoint.append(17)


    # Build waypoints from mission definition
    current_base = setpoint
    for step in mission:
        dir_vec = step[0]
        dist = step[1]
        wp_time = step[2]
        
        if len(step) == 4:  # Gradual movement
            step_magnitude = abs(step[3]) # Use magnitude for calculation (0.3)
            move_direction = np.sign(step[3]) # Use the sign for the actual move (-1)
            remaining_magnitude = abs(dist) # Use magnitude for loop (1.40)
            
            while remaining_magnitude > 0:
                step = min(step_magnitude, remaining_magnitude)
                current_step_value = step * move_direction # step value is 0.3 * -1 = -0.3

                current_base = make_wp(current_base, dir_vec, current_step_value) # Pass the signed value
                list_waypoints.append(current_base)
                time_at_waypoint.append(wp_time)
                remaining_magnitude -= step # remaining_magnitude = 1.40 - 0.3 = 1.10. This correctly decreases!
                
        else:  # Normal movement
            current_base = make_wp(current_base, dir_vec, dist)
            list_waypoints.append(current_base)
            time_at_waypoint.append(wp_time)

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
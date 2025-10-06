
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
    
    def make_wp(base_setpoint, unit_vector, distance):
        unit_vec = np.array(unit_vector,dtype='int')
        vec = (distance/np.linalg.norm(unit_vec))*unit_vec

        setpoint = base_setpoint + np.array([vec[0], vec[1], vec[2], 0, 0, 0, 0, 0, 0, 0, 0, 0])
        return setpoint
    
    #set the first waypoint
    setpoint = first_pos
    setpoint[2] = setpoint[2] + 0.5 #+ (0.15*(np.random.rand()-0.5)) #meters
    setpoint[3:] = 0
    flyer.waypoint(setpoint)
    current_waypoint = np.copy(setpoint)

    #ABOVE droxel
    setpoint2 = make_wp(setpoint, [1,0,0], 0.5)

    #DOWN to pick
    # setpoint3_1 = make_wp(setpoint2, [0,0,1], -0.1)
    # setpoint3_2 = make_wp(setpoint3_1, [0,0,1], -0.1)
    # setpoint3_3 = make_wp(setpoint3_2, [0,0,1], -0.1)
    # setpoint3_4 = make_wp(setpoint3_3, [0,0,1], -0.1)
    # setpoint3_5 = make_wp(setpoint3_4, [0,0,1], -0.1)
    # setpoint3 = make_wp(setpoint3_5, [0,0,1], -0.05)

    #UP after pick
    # setpoint4 = make_wp(setpoint3, [0,0,1], 0.55)

    #ABOVE droxel placement
    # setpoint5 = make_wp(setpoint4, [0,1,0], 0.5)
    setpoint5 = make_wp(setpoint2, [0,1,0], 0.5)

    #DOWN to place
    # setpoint6_1 = make_wp(setpoint5, [0,0,1], -0.1)
    # setpoint6_2 = make_wp(setpoint6_1, [0,0,1], -0.1)
    # setpoint6_3 = make_wp(setpoint6_2, [0,0,1], -0.1)
    # setpoint6_4 = make_wp(setpoint6_3, [0,0,1], -0.1)
    # setpoint6_5 = make_wp(setpoint6_4, [0,0,1], -0.1)
    # setpoint6 = make_wp(setpoint6_5, [0,0,1], -0.02)

    setpoint6_1 = make_wp(setpoint5, [0,0,1], -0.05)
    setpoint6_2 = make_wp(setpoint6_1, [0,0,1], -0.05)
    setpoint6_3 = make_wp(setpoint6_2, [0,0,1], -0.05)
    setpoint6_4 = make_wp(setpoint6_3, [0,0,1], -0.05)
    setpoint6_5 = make_wp(setpoint6_4, [0,0,1], -0.05)
    setpoint6_6 = make_wp(setpoint6_5, [0,0,1], -0.05)
    setpoint6_7 = make_wp(setpoint6_6, [0,0,1], -0.05)
    setpoint6_8 = make_wp(setpoint6_7, [0,0,1], -0.05)
    setpoint6_9 = make_wp(setpoint6_8, [0,0,1], -0.05)
    setpoint6 = make_wp(setpoint6_9, [0,0,1], -0.03)


    #TWIST YAW to release droxel(shorter time at this setpoint)
    setpoint7 = setpoint6 + np.array([0, 0, 0, 0, 0, 0, 0, 0, -np.pi/2, 0, 0, 0])

    #UP to release droxel
    setpoint8 = make_wp(setpoint7, [0,0,1], 0.65)

    #BACK to setpoint2 (above droxel)
    setpoint9 = setpoint2
    setpoint9 = setpoint2
    setpoint9 = make_wp(setpoint9, [1,1,0], -0.5)

    #arm the flyer
    flyer.arm()
    flyer.run_controller()

    current_time = time.time() - START
    time_at_each_setpoint = 3 #seconds

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
        if int(current_time) % time_at_each_setpoint == 0 and current_time - time_of_last_switch > 1 and not stepped:
            print('new setpoint.')
            if np.all(current_waypoint == setpoint):
                current_waypoint = np.copy(setpoint2)
                flyer.waypoint(current_waypoint)
                time_at_each_setpoint = 40 #seconds

                time_of_last_switch = current_time

            elif np.all(current_waypoint == setpoint2):
            #     current_waypoint = np.copy(setpoint3_1)
            #     flyer.waypoint(current_waypoint)
            #     time_of_last_switch = current_time
            # elif np.all(current_waypoint == setpoint3_1):
            #     current_waypoint = np.copy(setpoint3_2)
            #     flyer.waypoint(current_waypoint)
            #     time_of_last_switch = current_time
            # elif np.all(current_waypoint == setpoint3_2):
            #     current_waypoint = np.copy(setpoint3_3)
            #     flyer.waypoint(current_waypoint)
            #     time_of_last_switch = current_time
            # elif np.all(current_waypoint == setpoint3_3):
            #     current_waypoint = np.copy(setpoint3_4)
            #     flyer.waypoint(current_waypoint)
            #     time_of_last_switch = current_time
            # elif np.all(current_waypoint == setpoint3_4):
            #     current_waypoint = np.copy(setpoint3_5)
            #     flyer.waypoint(current_waypoint)
            #     time_of_last_switch = current_time
            # elif np.all(current_waypoint == setpoint3_5):
            #     current_waypoint = np.copy(setpoint3)
            #     flyer.waypoint(current_waypoint)
            #     time_of_last_switch = current_time



            # elif np.all(current_waypoint == setpoint3):
            #     current_waypoint = np.copy(setpoint4)
            #     flyer.waypoint(current_waypoint)
            #     time_of_last_switch = current_time
            # elif np.all(current_waypoint == setpoint4):
                current_waypoint = np.copy(setpoint5)
                flyer.waypoint(current_waypoint)
                time_at_each_setpoint = 3
                  
                time_of_last_switch = current_time
            elif np.all(current_waypoint == setpoint5):
                current_waypoint = np.copy(setpoint6_1)
                flyer.waypoint(current_waypoint)
                time_at_each_setpoint = 1  # Shorter time for yaw twist

                time_of_last_switch = current_time
            elif np.all(current_waypoint == setpoint6_1):
                current_waypoint = np.copy(setpoint6_2)
                flyer.waypoint(current_waypoint)
                time_at_each_setpoint = 1  # Shorter time for yaw twist

                time_of_last_switch = current_time
            elif np.all(current_waypoint == setpoint6_2):
                current_waypoint = np.copy(setpoint6_3)
                time_at_each_setpoint = 1  # Shorter time for yaw twist

                flyer.waypoint(current_waypoint)
                time_of_last_switch = current_time
            elif np.all(current_waypoint == setpoint6_3):
                current_waypoint = np.copy(setpoint6_4)
                flyer.waypoint(current_waypoint)
                time_at_each_setpoint = 1  # Shorter time for yaw twist

                time_of_last_switch = current_time
            elif np.all(current_waypoint == setpoint6_4):
                current_waypoint = np.copy(setpoint6_5)
                flyer.waypoint(current_waypoint)
                time_at_each_setpoint = 1  # Shorter time for yaw twist

                time_of_last_switch = current_time
            elif np.all(current_waypoint == setpoint6_5):
                current_waypoint = np.copy(setpoint6_6)
                flyer.waypoint(current_waypoint)
                time_at_each_setpoint = 1  # Shorter time for yaw twist

                time_of_last_switch = current_time
            
            elif np.all(current_waypoint == setpoint6_6):
                current_waypoint = np.copy(setpoint6_7)
                flyer.waypoint(current_waypoint)
                time_at_each_setpoint = 1  # Shorter time for yaw twist

                time_of_last_switch = current_time
            
            elif np.all(current_waypoint == setpoint6_7):
                current_waypoint = np.copy(setpoint6_8)
                flyer.waypoint(current_waypoint)
                time_at_each_setpoint = 1  # Shorter time for yaw twist

                time_of_last_switch = current_time
            
            elif np.all(current_waypoint == setpoint6_8):
                current_waypoint = np.copy(setpoint6_9)
                flyer.waypoint(current_waypoint)
                time_at_each_setpoint = 1  # Shorter time for yaw twist

                time_of_last_switch = current_time
            
            elif np.all(current_waypoint == setpoint6_9):
                current_waypoint = np.copy(setpoint6)
                flyer.waypoint(current_waypoint)
                time_at_each_setpoint = 1  # Shorter time for yaw twist

                time_of_last_switch = current_time

            elif np.all(current_waypoint == setpoint6):
                current_waypoint = np.copy(setpoint7)
                flyer.waypoint(current_waypoint)
                time_at_each_setpoint = 0.5  # Shorter time for yaw twist

                time_of_last_switch = current_time
            elif np.all(current_waypoint == setpoint7):
                current_waypoint = np.copy(setpoint8)
                flyer.waypoint(current_waypoint)
                time_of_last_switch = current_time
            elif np.all(current_waypoint == setpoint8):
                current_waypoint = np.copy(setpoint9)
                flyer.waypoint(current_waypoint)
                time_at_each_setpoint = 5  # Shorter time for yaw twist

                time_of_last_switch = current_time

            # stepped = True

            

        

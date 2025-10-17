
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
    
    #set the first waypoint
    setpoint = first_pos
    setpoint[2] = setpoint[2] + 0.5 #+ (0.15*(np.random.rand()-0.5)) #meters
    setpoint[3:] = 0
    flyer.waypoint(setpoint)
    current_waypoint = np.copy(setpoint)

    #step is 0.5m in length, at unit vector 1,1,1
    unit_vec = np.array([1,1,0],dtype='int')
    distance = 0 #m
    vec = (distance/np.linalg.norm(unit_vec))*unit_vec

    setpoint2 = setpoint + np.array([vec[0], vec[1], vec[2], 0, 0, 0, 0, 0, 0, 0, 0, 0])

    #arm the flyer
    flyer.arm()
    flyer.run_controller()

    current_time = time.time() - START
    time_at_each_setpoint = 10 #seconds
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
                time_of_last_switch = current_time

            

        

        


        
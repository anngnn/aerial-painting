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

def usr(flyer):


    flyer.select_controller('lqr')

    flyer.log(['user code started'])


    
    #get the first position
    while True:
        state = flyer.state()
        if len(state) > 1:
            first_pos = np.copy(state)
            break
    
    flyer.log(['The user code is running. The flyer is localized to: ' + str(first_pos)])

    setpoint = np.zeros(12)
    setpoint[0:3] = first_pos[0:3]
    flyer.waypoint(setpoint)
    

    interval = 10 #seconds
    start = time.time()
    armed = False
    time_of_last_change = start

    # flyer.arm()



    i = 0
    while True:

        now = time.time()
        if not int(now) % interval and now - time_of_last_change > 1:
            if armed:
                armed = False
                flyer.disarm()
                time_of_last_change = now
            elif not armed:
                armed = True
                flyer.arm()
                time_of_last_change = now



        flyer.delay()
        state = flyer.state()
        if len(state) > 1:
            print('%d, %0.3f, %0.3f, %0.3f' % (i, state[0], state[1], state[2]))
            state = np.round(state,3)
            flyer.log([state[0], state[1], state[2]])
            i += 1
            
        
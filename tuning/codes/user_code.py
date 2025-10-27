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


import time
import numpy as np
import struct
import random

def usr(flyer):
    # thse dont do anything?
    loop_counter = 0
    rec_limit = random.randint(11,20)

    first_pos = np.zeros(12, dtype=float)

    #get the first position
    while True:
        state = flyer.state()
        if len(state) > 1:
            first_pos = np.copy(state)
            break

    flyer.log('The user code is running. The flyer is localized to: ' + str(first_pos))

    flyer.mode(1500)

    flyer.select_controller('lqr')


    print('CALLING ARM!!!')   
    flyer.arm()

    print("SHOULD BE ARMED")

    start_time = time.time()
    waypoint_timer = start_time
    time_at_waypoint = 20 #s

    xyz_zero = np.array([1,1,1,0,0,0,0,0,0,0,0,0])

    waypoint_1 = (first_pos + np.array([0,0,0.4,0,0,0,0,0,0,0,0,0])) * xyz_zero
    waypoint_2 = (waypoint_1 + np.array([0.0,-0.4,0.0,0,0,0,0,0,0,0,0,0]))
    waypoint_3 = waypoint_2 + np.array([0.0,0.0,-0.4,0,0,0,0,0,0,0,0,0])

    waypoints = [waypoint_1, waypoint_2, waypoint_3]
    waypoint_pointer = 0

    while True:




        now = time.time()

        # if now - start_time > 10:
        #     flyer.disarm()
        #     flyer.delay()
        #     print("SHOULD BE DONE")
        #     return


        pos = flyer.state()
        if len(pos) > 1:
            x = pos[0]
            y = pos[1]
            z = pos[2]
            # flyer.log(str(x) +', ' + str(y) + ', ' + str(z))
        flyer.waypoint(waypoints[waypoint_pointer])



        if now - waypoint_timer > time_at_waypoint:
            flyer.log('new waypoint')
            waypoint_pointer += 1
            if waypoint_pointer >= len(waypoints):
                waypoint_pointer = 2
                return
            waypoint_timer = now

        


        flyer.delay()

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

def jerk(flyer, time):
    flyer.arm()
    flyer.log(['arming now'])
    flyer.delay(time)
    flyer.log(['disarming now'])
    flyer.disarm()


def usr(flyer):

    # heres what this code is trying to do:
    # start the drone with LQR
    # get the current position
    # make that position the set point
    # arm it so we can tell that the drone code is working

    start_time = time.time()


    flyer.select_controller('lqr')
    flyer.log(['user code started'])


    
    #get the first position
    while True:
        state = flyer.state()
        if len(state) > 1:
            first_pos = np.copy(state)
            break
    
    flyer.log(['The user code is running. The flyer is localized to: ]' + str(first_pos)])
    
    #set the first waypoint
    setpoint = first_pos

    flyer.waypoint(setpoint)
    # jerk(flyer, 0.25)

    # setpoint = first_pos
    # setpoint[2] = setpoint[2] + 0.4 #meters
    # setpoint[3:] = 0
    # flyer.waypoint(setpoint)
    # flyer.arm()
    

    
    done = 0

    num_msgs = 80

    if flyer.id in [9]:
        for i in range(num_msgs):
            #flyer.log(['attempting to send message ' + str(i)])
            # flyer.delay(0.1)
            print(i)
            flyer.send_msg(struct.pack('ff', flyer.id, i))
            flyer.log(['message ' + str(i) + "--------------------------------------------------------"])
            flyer.delay(0.05)
            #flyer.log(['tr ied to reveive a message'])

    if flyer.id in [14, 10, 11]:
        while True: 


            flyer.send_msg(struct.pack('ff', flyer.id, 42))
            
            flyer.delay(0.05)
            msgs = flyer.recv_msg()
            flyer.log([' '])
            flyer.log(['message received'])
            if msgs:
                for m in msgs:
                    flyer.log([struct.unpack('ff', m[0:8])])
                    temp = struct.unpack('ff', m[0:8])
                    if temp[0] == 9:
                        print(temp[1])
                        done = temp[1]
                    else:
                        print('noise from: ', temp[0])
            else:
                print('no message received1')
                flyer.log(['no message received!!!!!!!!!!!!!!!!!!'])

            msgs = flyer.recv_msg()
            flyer.log([' '])
            flyer.log(['message received'])
            if msgs:
                for m in msgs:
                    flyer.log([struct.unpack('ff', m[0:8])])
                    temp = struct.unpack('ff', m[0:8])
                    if temp[0] == 9:
                        print('2: ' + str(temp[1]))
                        done = temp[1]
                    else:
                        print('2 noise from: ', temp[0])
            else:
                print("no message received2")
                flyer.log(['no message received2!!!!!!!!!!!!!!!!!!'])

            if done == 79:
                break
            
    run_time = time.time() - start_time
    flyer.log([run_time])
    print(run_time)
    # jerk(flyer, 0.25)

"""
        if msgs:
            flyer.log([' '])
            flyer.log(['message received'])
            for m in msgs:
                
                #number = -1.0
                #number = struct.unpack('f', msgs[0:4])
                flyer.log([struct.unpack('f', m[0:4])])
        else:
            flyer.log(['no message received!!!!!!!!!!!!!!!!!!'])

    flyer.delay(2)
    flyer.log(['the rest of these were in the queue.'])
    msgs = flyer.recv_msg()

    if msgs:
        for m in msgs:
            flyer.log([struct.unpack('f', m[0:4])])
            
    else:
        flyer.log(['jk there was nothing here'])

    # alr we're done - log that
    flyer.log(['you can stop now'])

"""
    # setpoint[2] = setpoint[2] - 0.2 #meters
    # setpoint[3:] = 0
    # flyer.waypoint(setpoint)

# want to make sure they can fly and receive at the same time

# game plan
# 0. make sure each drone gets all the data
# 0.5 
# 1. fix the formatting so its easy to see
# 2. do it while they are flying

"""
user code handler fxn to flyerget time
shared data 
c code to write time associated with each call
"""

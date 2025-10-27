
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
from typing import Any, List
from statemachine import StateMachine, State
import math
from math import pi, sin, cos

#Define the origin and exit node in virtual space
VIRTUAL_ORIGIN = np.array([0.0,0.0,0.0])
VIRTUAL_EXIT_NODE = np.array([1.0, 0.0, 0.0])
VIRTUAL_EXIT_DESTINATION = np.array([1.0, 0.0, -1.0])

CHARGING = np.array([0.0,0.0,0.0])
PRECHARGING = np.array([0.0,0.0,0.0])
# foo = np.array([-1.525, -1.321, 0.095]) #6

SCALE = 0.75#meters. This is the distance between nodes in the cube #we know 0.75 and 6s travel time works (3/6/2025) - start to see downwash issues at 0.6m, given 25deg roll, 45deg pitch
TRANSLATE = np.array([-0.1,-1.25,0.65]) #This is the origin of the shape in real space
ROLL = 0.436 #25 deg #0.262 #15 deg
PITCH = -0.785 #45 deg

COMM_RADIUS = 1.8
BITMAP_CENTER_OFFSET = 8

TRAVEL_TIME = 6 #seconds #the time allotted to travel between setpoints
SKIP_TIME = 1.05*TRAVEL_TIME
HOLD_TIME = 4#seconds. #the time allotted to hold at a setpoint
TIME_STEP = int(HOLD_TIME + TRAVEL_TIME) #seconds. The TOTAL time in between in-shape movements. TIME_STEP = HOLD_TIME + TAVEL_TIME

MAX_VELO = 0.95 #m/s
SHAPE_VELO = SCALE / TRAVEL_TIME #0.75/6 = 0.125 m/s
DT = 0.2

ALGORITHM = 2

STARTING_SHAPE = [[0, 0, 0], [0, 0, 1], [0, 1, 1], [0, 1, 0], [1, 1, 0], [1, 1, 1], [1, 0, 1], [1, 0, 0]]

VELO = SCALE/TRAVEL_TIME #velocity (in m/s) needed to travel 1 SCALE distance in TRAVEL_TIME seconds.

global fast_pointer

def usr(flyer):

    global fast_pointer

    print('started')
    flyer.log(['started'])

    #Let's initialize some things.
    #first, initialize the time, so we know when we started.
    #second, let's make sure the flyer is going to use the lqr controller in the horizon (default) mode
    START = time.time()
    flyer.select_controller('lqr')
    flyer.mode(1000)

    print('looking for first pos')
    
    #get the first position (we do nothing until then)
    while True:
        state = flyer.state()
        if len(state) > 1:
            first_pos = np.copy(state)
            break
        flyer.delay()
    state = np.copy(first_pos)
    current_velo = np.zeros(3, dtype=float)
    yaw = 0

    print('first pos found')

    PRESHAPE = np.array([state[0], state[1], 0.5])
    LANDER = flyer.get_lander()[0:3] 
    CHARGING = LANDER + np.array([0.0, 0.0, 0.015])
    PRECHARGING = LANDER + np.array([0,0,0.2])
    PENALTY_BOX = LANDER + np.array([0.3, 0.05, 0.5]) #we tend to overshoot this in y, so we offset it slightly to help the flyer out a bit
    print("CHARGING: " + str(CHARGING))
    print("PRECHARGING: " + str(PRECHARGING))
    
    #let the user log know we have our first position.
    flyer.log(['The user code is running. The flyer is localized to: ' + str(first_pos)])

    # flyer.arm()

    #create a state machine to govern the sculpting behavior.
    stm = FlyerStateMachine(flyer.id)

    #Timing and destination variables to govern when we can move to next waypoint, etc.
    launch_order = { 30: 0, #5
                     6: 1,   
                     7: 2,
                     9: 0,
                    10: 1,
                    11: 2,
                    13: 6,
                    14: 7,
                     5: 0}
    print(flyer.id)
    TIME_TO_LEAVE = 3 + launch_order[flyer.id]*TIME_STEP #seconds
    TEMP_TTL = False
    # if flyer.id == 6:
    #     TIME_TO_LEAVE = 3 + 1*TIME_STEP #seconds
    # elif flyer.id == 10:
    #     TIME_TO_LEAVE = 3 + 0*TIME_STEP #seconds
    last_time_outside_of_radius = 0
    ERROR_RAD = 0.3 #meters
    TIME_WITHIN_ERROR = 0.5 #seconds

    print('defining a function')        

    #Variables to help us track if we are MOVING between waypoints or HOLDING at a waypoint
    HOLDING = False
    MOVING = False
    ready_to_exit = False
    time_of_new_state_data = time.time()
    prev_state_data_time = time_of_new_state_data
    moving_start_time = time.time()

    #create a waypoint manager to help the robot move in the cube (by dragging in between cube points, etc.)
    wm = WaypointManager(MAX_VELO, first_pos[0:3], time_step=DT)
    wm.set_velo(SHAPE_VELO, SHAPE_VELO, SHAPE_VELO)
    flyer_waypoint = np.zeros(12)

    time_landing_started = time.time()
    print('at loop start')
    flyer.log(['at loop start'])
    flyer.log(['time', 'loop duration', '# msgs', 'state', 'TTL'])

    first_time = True
    takeoff_chute_cleared = True

    loop_frequency = 42
    loop_start = time.time()
    loop_end = time.time() + 1

    out_time = 0

    fast_pointer = 1

    while True:        
        
        flyer.delay()

        loop_start = time.time()

        #get the current time
        NOW = time.time() - START

        #Receive any messages
        msgs = flyer.recv_msg()

        #Process the messages
        if msgs:
            num_of_messages = len(msgs)
            process_time_start = time.time()
            if hasattr(flyer,'bot'): #if there is a bot (meaning we're in the shape), then have that do the message unpacking.
                flyer.bot.unpack_message(msgs, flyer.log, NOW)
            process_time_end = time.time()

            # flyer.log([np.round(NOW, 3), np.round(loop_frequency, 4), num_of_messages, "%0.6f" %(process_time_end - process_time_start), np.round(wm.time_step,3), np.round(wm.dt,3)])
        else:
            # flyer.log([np.round(NOW, 3), np.round(loop_frequency, 4), 0, "%0.6f" %(0),  np.round(wm.time_step,3), np.round(wm.dt,3)])
            num_of_messages = 0

        #get the current robot state (x,y,z,vx,vy,vz,theta,phi,psi,r,p,y)
        #and its position (x,y,z) in virtual space.
        temp_state = flyer.state()
        if len(temp_state) > 1:
            state = np.copy(temp_state)
            pos = convert_reality_to_virtual(state[0:3])
            time_of_new_state_data = time.time()
            current_velo = state[3:6]
            yaw = state[8]


        #Pack any messages for sending
        if hasattr(flyer, 'bot'): #if there is a bot (meaning we're in the shape), then have that do the outbound message packing, etc.       
            outbound, outbound_msg = flyer.bot.pack_message() #call a pack message function here
        else:
            outbound = False

        #Send any messages
        if outbound:
            # print(str(flyer.bot.id) + ': sending: ' + str(flyer.bot.msg_type))
            flyer.send_msg(outbound_msg)
            pass



        ###############################################################################################################################################
        #STATE MACHINE
        ###############################################################################################################################################
        #Execute some behavior (based on our state)
        #CHARGING STATE
        if stm.current_state.name == 'charging':

            # print(NOW, TIME_TO_LEAVE)
            # flyer.log([NOW, TIME_TO_LEAVE])

            #it doesn't take long to pop up, so wait a beat to make sure above drones clear out.
            if first_time:
                pop_up_delay = 3
            else:
                pop_up_delay = 0
            

            
            #if it is time to go back to the shape, we do so.
            if NOW > TIME_TO_LEAVE + pop_up_delay:

                PRESHAPE = np.array([state[0], state[1], state[2] + 0.5])
                
                setpoint = np.zeros(12)
                setpoint[0:3] = PRESHAPE
                flyer.waypoint(setpoint)
                # print(str(NOW) + " taking off from: " + str(np.round(state[0:3],3)) + " to " + str(np.round(setpoint[0:3],3)))
                #flyer.log([str(NOW) + " taking off from: " + str(np.round(state[0:3],3)) + " to " + str(np.round(setpoint[0:3],3))])
                flyer.arm()
                flyer.run_controller()

                #update the waypoint manager so it is pointing to our current waypoint destination.
                wm.set_NEW_destination(setpoint[0:3])
                
                #transition to takeoff state
                stm.launch()
                # print('going to takeoff')
                if first_time:
                    TIME_TO_LEAVE = TIME_TO_LEAVE + TIME_STEP
                else:
                    TEMP_TTL = NOW + 3 #this makes sure we pop vertically off the charging station before going horizontal to the cube.
                    takeoff_chute_cleared = False
                    

                first_time = False

        #TAKOFF STATE - launching OFF the ground
        elif stm.current_state.name == 'takeoff':

            if TEMP_TTL: #we will have a temporary time to leave (temp TTL) IFF we are leaving the charging station.
                go_time = TEMP_TTL
                # print('temp TTL go time: ' + str(go_time))

                # check the z position. If we are up out of the charging station far enough, shift over to get out of the way of landing robots.
                if not takeoff_chute_cleared:
                    zee = state[2]
                    if zee > LANDER[2] + 0.10:
                        # print('THE ZEE CODE RAN')
                        # print('\n\n\n\n')
                        PRESHAPE = PRESHAPE + np.array([-0.25, 0, 0])
                        setpoint[0:3] = PRESHAPE
                        flyer.waypoint(setpoint)
                        wm.set_NEW_destination(setpoint[0:3])
                        takeoff_chute_cleared = True



            else:
                go_time = TIME_TO_LEAVE

            if NOW > go_time:

                if TEMP_TTL:
                    TEMP_TTL = False
                

                #head to the origin of the shape.
                xyz = convert_virtual_to_reality(VIRTUAL_ORIGIN)

                #use the waypoint manager to go to this new destination setpoint
                wm.set_NEW_destination(xyz)
                #get a new waypoint from the waypoint manager (waypoint will be ON THE WAY to the destination setpoint set above)
                flyer_waypoint[0:3] = wm.get_waypoint()

                #set it on the robot
                flyer.waypoint(flyer_waypoint)

                #record the time we started moving
                moving_start_time = time.time()
                # print(str(NOW) + " about to enter the preshape from: " + str(np.round(state[0:3],3)) + " to " + str(np.round(xyz,3)))
                #flyer.log(str(NOW) + " about to enter the preshape from: " + str(np.round(state[0:3],3)) + " to " + str(np.round(xyz,3)))
                TIME_TO_LEAVE = TIME_TO_LEAVE + TIME_STEP
                stm.set_heading_to_shape(True)
                stm.go_to_travel()


            

        elif stm.current_state.name == 'travel': #traveling to a setpoint (that is likely NOT within the shape or the scope of the default behavior, etc.)


            #see if we are out of time to get there AND we are going to the shape
            if time.time()-moving_start_time >= SKIP_TIME and stm.get_heading_to_shape():

                #we are out of time (OR) we reached the setpoint already, so we can enter the shape 
                wm.skip_to_end()
                flyer_waypoint[0:3] = wm.get_waypoint()
                flyer.waypoint(flyer_waypoint)


                reached = True


            else:
                #get a waypoint from the waypoint manager
                flyer_waypoint[0:3] = wm.get_waypoint()

                #set it on the robot
                flyer.waypoint(flyer_waypoint)
                
                reached = wm.is_destination_reached()

                if stm.get_heading_to_penalty_box():
                    print('on my way to the box %0.4f' % (NOW))

            if reached:

                #if we were moving to the shape
                if stm.get_heading_to_shape():

                    MOVING = False
                    HOLDING = True
                    #DEVELOPERS NOTE: We ALREADY updated our time to leave value prior to entering 'travel',
                    #so TIME_TO_LEAVE should be the time to leave the position we are traveling to.
                    # Thats why the next line is commented out  
                    ####TIME_TO_LEAVE = TIME_TO_LEAVE + TIME_STEP
                    stm.enter_shape()
                    # print('entered the shape at its origin', NOW)
                    #flyer.log(['entered the shape at its origin', NOW])
                    ready_to_exit = False
                    flyer.bot = Robot(flyer.id, VIRTUAL_ORIGIN, VIRTUAL_EXIT_NODE, VIRTUAL_EXIT_DESTINATION, ALGORITHM) #create a robot object to handle the execution of sculpting algorithms
                    flyer.bot.set_shape = STARTING_SHAPE
                    flyer.bot.determine_change_readiness()

                    wm.set_velo(SHAPE_VELO, SHAPE_VELO, SHAPE_VELO)

                    fast_pointer = 1

                elif stm.get_heading_to_charge():

                    stm.begin_landing()
                    # print('starting the landing sequence', NOW)
                    # flyer.log(['starting the landing sequence', NOW])

                    stm.upon_charge_entry()

                    time_landing_started = time.time()

                elif stm.get_heading_to_penalty_box():
                    # flyer.log(["REACHED THE BOX. %0.4f" %(NOW)])
                    temp_velo = SHAPE_VELO
                    wm.set_velo(temp_velo, temp_velo, temp_velo)
                    if TEMP_TTL:
                        if NOW > TEMP_TTL:
                            stm.upon_penalty_box_arrival()
                            wm.set_NEW_destination(PRECHARGING)
                            #get a new waypoint from the waypoint manager (waypoint will be ON THE WAY to the destination setpoint set above)
                            flyer_waypoint[0:3] = wm.get_waypoint()
                            #set it on the flyer
                            flyer.waypoint(flyer_waypoint)
                            TEMP_TTL = False








            
            

        #SHAPE STATE - traveling within the shape (default behavior, detect, primary changes, and secondary changes all executed here)
        elif stm.current_state.name == 'shape':

            #GET HUMAN INPUT HERE
            human_input = False

            #Execute the INNER state machine (i.e. the SHAPE STATE MACHINE)
            #To get our next pos
            next_pos, exiting = flyer.bot.run(params = [NOW, human_input])

            if exiting:
                ready_to_exit = True

            if HOLDING:
                if (ready_to_exit and NOW > TIME_TO_LEAVE):
                    # #Transition to the landing state IF we have reached the exit node and it is time to go.
                    # exit_node = np.zeros(12)
                    # exit_node[0:3] = convert_virtual_to_reality(VIRTUAL_EXIT_NODE)
                    # reached_setpoint, last_time_outside_of_radius = setpoint_reached(exit_node, state, NOW, last_time_outside_of_radius)
                    # if reached_setpoint and TIME_TO_LEAVE < NOW:

                    # flyer.waypoint(CHARGING_STATE)
                    # stm.exit_shape()
                    # destination = np.copy(CHARGING_STATE)

                    # print('exiting the shape', NOW)
                    # flyer.log(['exiting the shape', NOW])
                    out_time = time.time()

                    # #LONGCUT
                    # temp_velo = SHAPE_VELO * 3
                    # wm.set_velo(temp_velo, temp_velo, temp_velo)

                    # wm.set_NEW_destination(PENALTY_BOX)
                    # stm.set_heading_to_penalty_box(True)
                    # TIME_TO_LEAVE = TIME_TO_LEAVE + TIME_STEP #*0.5 #DEVELOPERS NOTE: We sould leave the charging station after 1/2 of a time step (in order to go there and back in 1 time step)
                    # TEMP_TTL = NOW + 3
                    # #END LONGCUT

                    #RETURN TO SHAPE SHORTCUT
                    #head to the origin of the shape.
                    xyz = convert_virtual_to_reality(VIRTUAL_ORIGIN)
                    #use the waypoint manager to go to this new destination setpoint
                    wm.set_NEW_destination(xyz)
                    TIME_TO_LEAVE = TIME_TO_LEAVE + TIME_STEP
                    stm.set_heading_to_shape(True)
                    #END SHORTCUT

                    #get a new waypoint from the waypoint manager (waypoint will be ON THE WAY to the destination setpoint set above)
                    flyer_waypoint[0:3] = wm.get_waypoint()
                    #set it on the flyer
                    flyer.waypoint(flyer_waypoint)

                    
                    stm.go_to_travel()
                    if hasattr(flyer, 'bot'):
                        if hasattr(flyer.bot, 'shape_stm'):
                            del flyer.bot.shape_stm
                        del flyer.bot
                    
                    moving_start_time = time.time()
                    ready_to_exit = False

                elif NOW > TIME_TO_LEAVE:
                    next_pos_reality = np.round(convert_virtual_to_reality(next_pos),3)
                    # print('leaving for the pos: ' + str(next_pos) + ' ' + str(next_pos_reality) + ' %0.2f, %0.0f' % (NOW, TIME_TO_LEAVE))
                    #flyer.log(['leaving for the pos: ' + str(next_pos) + ' ' + str(next_pos_reality) + ' %0.2f, %0.0f' % (NOW, TIME_TO_LEAVE)])
                    # flyer.log(['next pos virtual = ', next_pos])
                    next_pos_reality = convert_virtual_to_reality(next_pos)
                    wm.set_NEW_destination(next_pos_reality)
                    #get a new waypoint from the waypoint manager (waypoint will be ON THE WAY to the destination setpoint set above)
                    flyer_waypoint[0:3] = wm.get_waypoint()
                    #set it on the flyer
                    flyer.waypoint(flyer_waypoint)
                    
                    HOLDING = False
                    MOVING = True
                    moving_start_time = time.time()
                    TIME_TO_LEAVE = TIME_TO_LEAVE + TIME_STEP



            if MOVING:
                
                if time.time()-moving_start_time < SKIP_TIME: #if we haven't reached the destination (and we still have time), just keep going towards it.
                    #get a new waypoint from the waypoint manager (waypoint will be ON THE WAY to the destination setpoint set above)
                    flyer_waypoint[0:3] = wm.get_waypoint()
                    #set it on the flyer
                    flyer.waypoint(flyer_waypoint)

                    reached =  wm.is_destination_reached()

                else:
                    wm.skip_to_end()
                    flyer_waypoint[0:3] = wm.get_waypoint()
                    flyer.waypoint(flyer_waypoint)

                    reached = True #we need this to be true so just move can execute. Hopefully we catch up to where we need to be soon!!
                    print("%f > %f" %(time.time()-moving_start_time, SKIP_TIME))
                    

                if reached:
                    MOVING = False
                    HOLDING = True
                    flyer.bot.just_moved(pos, params = [flyer])
                    # print('executed just moved', pos, np.round(state[0:3],3))
                    # flyer.log(['executed just moved', pos, np.round(state[0:3],3)])

            

            
                    
        #LANDING STATE - traveling from the PRECHARGING position (above the charging station) to the charging position
        elif stm.current_state.name == 'landing':

            dest = wm.get_CURRENT_destination()

            error_xy = np.linalg.norm(state[0:2] - dest[0:2])
            velo = np.linalg.norm(current_velo)

            # flyer.log(['land seq dest', dest[0], dest[1], dest[2]])


            #when we first enter the 'landing' state, our destination will be precharging. We only want to shift to charging if our error and velocity are small.
            if np.all(wm.get_CURRENT_destination() == PRECHARGING):
                if error_xy <= 0.05 and velo < 0.2:
                    wm.set_velo(velo_z = SHAPE_VELO)

                    wm.set_NEW_destination(CHARGING)
                    #get a new waypoint from the waypoint manager (waypoint will be ON THE WAY to the destination setpoint set above)
                    flyer_waypoint[0:3] = wm.get_waypoint()
                    #set it on the flyer
                    flyer.waypoint(flyer_waypoint)
                    # flyer.log(['change precharging to charging', error_xy, velo])
                else:
                    # flyer.log(["xy error: %0.4f" %(error_xy), "velo: %0.2f" %(velo)])
                    pass
            


            #get a waypoint from the waypoint manager
            flyer_waypoint[0:3] = wm.get_waypoint()

            #set it on the robot
            flyer.waypoint(flyer_waypoint)


            #We can cut off our motors anywhere between the PRECHARGING and CHARGING positions. (within a height of 6 cm)
            if np.all(wm.get_CURRENT_destination() == CHARGING):
                # error_xy = np.linalg.norm(state[0:2] - wm.get_CURRENT_destination()[0:2])
                x_error = abs(state[0] - wm.get_CURRENT_destination()[0])
                y_error = abs(state[1] - wm.get_CURRENT_destination()[1])
                # z_error = abs(state[2] - wm.get_CURRENT_destination()[2])
                z = state[2] - LANDER[2]
                velo = np.linalg.norm(current_velo)
                # print([velo, x_error, y_error, z, yaw])
                if ((x_error <= 0.01 and y_error <= 0.025) or (x_error <= 0.025 and y_error <= 0.01)) and velo < 0.1 and z <= 0.06:
                    flyer.disarm()

                    stm.landed()

                    # print('landed', NOW)
                    # flyer.log(np.round([velo, x_error, y_error, z, yaw],4))
                    # flyer.log(['landed', NOW, "%0.4f" % (time.time() - out_time)])

                    temp_velo = SHAPE_VELO * 3
                    wm.set_velo(temp_velo, temp_velo, temp_velo)
                else:
                    test1 = "True" if x_error <= 0.01 and y_error <=0.025 else "False"
                    test2 = "True" if y_error <= 0.01 and x_error <=0.025 else "False"
                    test3 = "True" if velo < 0.1 else "False"
                    test4 = "True" if z <= 0.06 else "False"
                    
                    # flyer.log(np.round([velo, x_error, y_error, z, yaw, test1, test2, test3, test4],4))
                    # flyer.log(["%0.3f %0.3f %0.3f %0.3f" % (velo, x_error, y_error, z), test1, test2, test3, test4])


        #prepare for next loop
        prev_state_data_time = time_of_new_state_data

        loop_end = time.time()
        loop_frequency = loop_end - loop_start

        flyer.log([np.round(loop_end - START,3), np.round(loop_end - loop_start,6), num_of_messages, stm.current_state.name, TIME_TO_LEAVE])

    while True:
        print("USER CODE SLEEPING")
        time.sleep(2)



            

        



    
# stmex.TransitionNotAllowed
        

        #log our state
        # state = flyer.state()
        # if len(state) > 1:
        #     rounded_time = np.round(time.time() - START, 3)
        #     rounded_state = np.round(state,2)
        #     rounded_state_list = rounded_state.tolist()
        #     volts = flyer.get_volts()
        #     watts = flyer.get_watts()
        #     log_list = [rounded_time] + rounded_state_list + [volts, watts]
        #     flyer.log(log_list)

        
def get_R(roll, pitch):
    '''
    Accepts a roll and pitch angle in radians.
    Returns a rotation matrix 
    '''
    Rx = np.array([[1,   0,  0],
                   [0,  cos(roll), -sin(roll)],
                   [0, sin(roll), cos(roll)]])
    Ry = np.array([[cos(pitch), 0, sin(pitch)],
                   [0,      1,      0],
                   [-sin(pitch), 0, cos(pitch)]])
    Rz = np.array([[cos(0), -sin(0), 0],
                   [sin(0), cos(0), 0],
                   [0, 0, 1]])
    
    R = np.matmul(Rz, np.matmul(Ry,Rx))
    return R


def convert_virtual_to_reality(point):
    '''
    point = a 3 part numpy array of a position in virtual space
    returns the corresponding 3 part numpy array of a position in real space.
    '''
    # real_point = np.matmul(get_R(ROLL, PITCH),point)
    # real_point = SCALE*real_point + TRANSLATE
    # return real_point

    xystep = 0.75
    z_level = 1.0
    virtual_point_list = [np.array([0.0,0.0,0.0]), np.array([0.0,0.0,1.0]), np.array([0.0,1.0,1.0]), np.array([0.0,1.0,0.0]), np.array([1.0,1.0,0.0]), np.array([1.0,1.0,1.0]), np.array([1.0,0.0,1.0]), np.array([1.0,0.0,0.0])]
    real_point_list = [np.array([TRANSLATE[0] + (0 * xystep),TRANSLATE[1] + (0 * xystep),z_level]),
                       np.array([TRANSLATE[0] + (0 * xystep),TRANSLATE[1] + (1 * xystep),z_level]),
                       np.array([TRANSLATE[0] + (0 * xystep),TRANSLATE[1] + (2 * xystep),z_level]),
                       np.array([TRANSLATE[0] + (0 * xystep),TRANSLATE[1] + (3 * xystep),z_level]),
                       np.array([TRANSLATE[0] + (1 * xystep),TRANSLATE[1] + (3 * xystep),z_level]),
                       np.array([TRANSLATE[0] + (1 * xystep),TRANSLATE[1] + (2 * xystep),z_level]),
                       np.array([TRANSLATE[0] + (1 * xystep),TRANSLATE[1] + (1 * xystep),z_level]),
                       np.array([TRANSLATE[0] + (1 * xystep),TRANSLATE[1] + (0 * xystep),z_level])]

    for i in range(0,len(virtual_point_list)):
        if np.all(point == virtual_point_list[i]):
            real_point = real_point_list[i]

            break

    return real_point

def convert_reality_to_virtual(point):
    '''
    point = a 3 part numpy array of a position in real space
    returns the correspoinding 3 part numpy array of a position in virtual space
    '''

    # virtual_point = (point - TRANSLATE)/SCALE
    # virtual_point = np.matmul(get_R(ROLL,PITCH).T, virtual_point)
    # virtual_point = np.round(virtual_point,0)
    # return virtual_point

    virtual_point = np.array([0,0,0])

    xystep = 0.75
    z_level = 1.0
    virtual_point_list = [np.array([0.0,0.0,0.0]), np.array([0.0,0.0,1.0]), np.array([0.0,1.0,1.0]), np.array([0.0,1.0,0.0]), np.array([1.0,1.0,0.0]), np.array([1.0,1.0,1.0]), np.array([1.0,0.0,1.0]), np.array([1.0,0.0,0.0])]
    real_point_list = [np.array([TRANSLATE[0] + (0 * xystep),TRANSLATE[1] + (0 * xystep),z_level]),
                       np.array([TRANSLATE[0] + (0 * xystep),TRANSLATE[1] + (1 * xystep),z_level]),
                       np.array([TRANSLATE[0] + (0 * xystep),TRANSLATE[1] + (2 * xystep),z_level]),
                       np.array([TRANSLATE[0] + (0 * xystep),TRANSLATE[1] + (3 * xystep),z_level]),
                       np.array([TRANSLATE[0] + (1 * xystep),TRANSLATE[1] + (3 * xystep),z_level]),
                       np.array([TRANSLATE[0] + (1 * xystep),TRANSLATE[1] + (2 * xystep),z_level]),
                       np.array([TRANSLATE[0] + (1 * xystep),TRANSLATE[1] + (1 * xystep),z_level]),
                       np.array([TRANSLATE[0] + (1 * xystep),TRANSLATE[1] + (0 * xystep),z_level])]

    distance = np.inf
    for i in range(0,len(real_point_list)):
        d = np.linalg.norm(point - real_point_list[i])
        if d < distance:
            distance = d
            virtual_point = virtual_point_list[i]
        

    return virtual_point

class WaypointManager():
    def __init__(self, max_velo, destination, time_step = 0.2):
        self.max_velo = np.array([max_velo, max_velo, max_velo])
        self.actual_velo = np.copy(self.max_velo)
        self.time_step = time_step
        self.dt= time_step
        self.destination = np.copy(destination)
        self.waypoint = np.copy(destination)
        self.destination_reached = False

        self.time_of_last_set = time.time()

    def set_NEW_destination(self, next_destination, current_dest = np.zeros(1)):
        '''
        destinations should be np.array(x,y,z)
        Find a unit vector and distance between our current destination and the next destination.
        Set new destination to current destination

        optionally, a user can set a new "current_dest" which is the starting point from which the vectors to the new destination are set.
        ''' 
        # print('setting new destination')
        # print(self.destination)
        # print('to')
        # print(next_destination)
        if len(current_dest) > 1:
            current_destination = current_dest
        else:
            current_destination = np.copy(self.destination)
        # print(current_destination, next_destination)
        self.destination_vector = next_destination - current_destination
        self.destination_vector_norm = np.linalg.norm(self.destination_vector)
        self.unit_destination_vector = self.destination_vector/self.destination_vector_norm
        self.destination = np.copy(next_destination)
        self.waypoint = np.copy(current_destination) #this SHOULD be the O.G. destination already, but just to be sure, we overwrite it here.
        self.destination_reached = False
    
    def get_CURRENT_destination(self):
         return np.copy(self.destination)
    
    def is_destination_reached(self):
        return self.destination_reached

    def get_waypoint(self):

        alpha = 0.9 #1 -> 100% OLD value

        if not self.destination_reached:

            clock = time.time()

            self.dt = clock - self.time_of_last_set
            if self.dt > 1:
                pass
            else:
                self.time_step = alpha * self.time_step + (1-alpha)*self.dt
            self.time_of_last_set = clock

            velo = np.copy(self.actual_velo)

            #find how far we could go with our given maximum velocity. If we are out of range, we just keep moving the waypoint. If we are within range, then we set the waypoint to the destination 
            max_distance_we_could_move = np.linalg.norm(velo*self.time_step*self.unit_destination_vector)
            dist2 = np.linalg.norm(self.destination - self.waypoint) #distance between our current waypoint and our final and destination
            
            #if the destination and the waypoint are the same, return the destination
            if dist2 == 0:
                self.destination_reached = True
                return self.destination
            #else, if the max distance we could move is LESS than the actual distance between our current waypoint and the destination,
                #then find a waypoint between our current waypoint and our destination IAW our max velocity and timestep.
            elif max_distance_we_could_move < dist2:            
                self.waypoint = self.waypoint + velo*(self.time_step)*self.unit_destination_vector
                return self.waypoint
            #else, (this is when the distance we could move is FURTHER than the actual distance between our current waypoint and the destination)
                #then we just set the waypoint to the destination
            else:
                self.waypoint = np.copy(self.destination)
                self.destination_reached = True
                return self.waypoint
            
        else:
            return self.destination
        
    def skip_to_end(self): 
        '''
        #used for "stepping" immediately to the destination 
        also sets the destination reached variable to True
        '''
        print('OUT OF TIME! SKIP TO END: ' + str(self.destination))
        self.waypoint = np.copy(self.destination)
        self.destination_reached = True
        return self.waypoint
    
    def set_velo(self,velo_x=False, velo_y=False, velo_z=False):

        if velo_x:
            if velo_x > self.max_velo[0]:
                self.actual_velo[0] = self.max_velo[0]

            elif velo_x < self.max_velo[0] and velo_x > 0:
                self.actual_velo[0] = velo_x

        if velo_y:
            if velo_y > self.max_velo[1]:
                self.actual_velo[1] = self.max_velo[1]

            elif velo_y < self.max_velo[1] and velo_y > 0:
                self.actual_velo[1] = velo_y

        if velo_z:
            if velo_z > self.max_velo[2]:
                self.actual_velo[2] = self.max_velo[2]

            elif velo_z < self.max_velo[2] and velo_z > 0:
                self.actual_velo[2] = velo_z

    

    
    


class FlyerStateMachine(StateMachine):
    '''
    The state machine for the flyer executing continuous sculpting in 3D
    Houses all states and transitions and behavior when states transition
    python docs on statemachine: https://python-statemachine.readthedocs.io/en/latest/readme.html
    '''

    #Here are all the states in the state machine.
    charging = State('charging', initial=True)
    takeoff = State('takeoff') #This is the state where we are flying from the charging station TO the first setpoint.
    travel = State('travel') #This is used any time the robot is generically going to a setpoint in space (i.e., to or  from the charging station)
    shape = State('shape', enter="upon_shape_entry") #this is the state where we run the default behavior
    landing = State('landing') #this is the state where we are flying from the shape TO the charging station
    

    #Here are all the events/transitions in the state machine
    launch = charging.to(takeoff)
    enter_shape = travel.to(shape) #when we transition into the shape, we create a sub-state machine to govern in-shape states and transitions.
    exit_shape = shape.to(landing) #When we transition out of the shape, we kill the in-shape state machine so we can start fresh next time we enter the shape.
    landed = landing.to(charging)
    go_to_travel = (shape.to(travel) | takeoff.to(travel))
    begin_landing = travel.to(landing)
    end_landing = landing.to(charging)



    def __init__(self, id, model: Any = None, state_field: str = "state", start_value: Any = None, rtc: bool = True, allow_event_without_transition: bool = False, listeners: List[object] | None = None):
        super().__init__(model, state_field, start_value, rtc, allow_event_without_transition, listeners)
        self.id = id 
        self.heading_to_shape = False
        self.heading_to_charge = False
        self.heading_to_penalty_box = False

    def set_heading_to_shape(self, val):
        if val in [True, False]:
            self.heading_to_shape = val
            return True
        else:
            return False
    
    def get_heading_to_shape(self):
        return self.heading_to_shape
    
    def upon_shape_entry(self):
        self.set_heading_to_shape(False)

    def set_heading_to_charge(self, val):
        if val in [True,False]:
            self.heading_to_charge = val
            return True
        else:
            return False

    def get_heading_to_charge(self):
        return self.heading_to_charge
       
    def upon_charge_entry(self):
        self.set_heading_to_charge(False)

    def set_heading_to_penalty_box(self, val):
        if val in [True,False]:
            self.heading_to_penalty_box = val
            return True
        else:
            return False
        
    def get_heading_to_penalty_box(self):
        return self.heading_to_penalty_box
    
    def upon_penalty_box_arrival(self):
        self.set_heading_to_penalty_box(False)
        self.set_heading_to_charge(True)

          


class ShapeStateMachine(StateMachine):
    '''
    The state machine for the flyer executing algorithms WITHIN THE SHAPE
    Houses all the states and transitions and behavior when states transition
    python docs on statemachine: https://python-statemachine.readthedocs.io/en/latest/readme.html
    '''

    #Here are all the states in the state machine
    default = State('default', initial=True)
    ncp = State('at ncp')
    prescribed = State('prescribed')
    stay = State('stay')
    passback = State('passback')
    sub_ncp = State('sub ncp')
    file_out = State('file out')
    alg2_sender = State('alg2 sender')
    changebot = State('change bot')

    #Here are all the events.transitions in the state machine
    go_to_stay = (default.to(stay) | prescribed.to(stay) | sub_ncp.to(stay) | stay.to(stay) | file_out.to(stay) | alg2_sender.to(stay))
    default_to_ncp = default.to(ncp)
    ncp_to_prescribed = ncp.to(prescribed)
    default_to_sub_ncp = default.to(sub_ncp)
    file_out_to_sub_ncp = file_out.to(sub_ncp)
    sub_ncp_to_default = sub_ncp.to(default)
    default_to_passback = default.to(passback)
    default_to_file_out = default.to(file_out)
    return_to_default = (ncp.to(default) | prescribed.to(default) | stay.to(default) | passback.to(default) | file_out.to(default) | default.to.itself() | alg2_sender.to(default) | passback.to(default) | changebot.to(default))
    become_alg2sender = (default.to(alg2_sender) | stay.to(alg2_sender))
    become_passback = (stay.to(passback) | prescribed.to(passback) | file_out.to(passback) | sub_ncp.to(passback))
    become_changebot = (stay.to(changebot) | default.to(changebot) | ncp.to(changebot))
    

    def __init__(self, id, model: Any = None, state_field: str = "state", start_value: Any = None, rtc: bool = True, allow_event_without_transition: bool = False, listeners: List[object] | None = None):
        super().__init__(model, state_field, start_value, rtc, allow_event_without_transition, listeners)
        self.id = id
        return
    

class ProcessedMessage():
    '''
    Stores all of the relevant (unpacked and parsed) information for a receied message
    '''
    def __init__(self, id, type):
        self.sender_id = id
        self.msg_type = type
        self.sender_position = np.array([0,0,0],dtype=int)
        self.sender_next_pos = np.array([0,0,0],dtype=int)
        self.sender_point_of_interest = np.array([0,0,0], dtype=int)
        self.sender_changed_box = np.array([0,0,0],dtype=int)
        self.sender_nodes_visited = []
        self.sender_shape = []
        self.sender_last_shape_change_id = 0.0


############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################

class Robot():
    '''
    SCULPTING ALGORITHMS ARE HOUSED HERE!!
    This governs a robot moving through a virtual grid graph world where the length of an edge in the grid is always 1 and nodes are at integer increments of 1.
    A scaling and translation/rotation matrix must be used to convert these positions into real positions in space (and back again)
    '''

    def __init__(self, id, pos, exit_node, exit_destination, alg):
        '''
        Initialize the robot with parameters and variables it will need to execute the continuous sculpting algorithms
        Initialize nodes and boxes visited as well (by calling initialize data).
        This is called on entry to the "shape" state (i.e., when a robot enters the shape at the entry node.)
        '''


        #First, just initialize some things we will need to reverence and use throughout the execution of algorithms
        self.id = id
        self.step_size = 1
        self.pos = np.copy(pos) #TODO will need to seed this with the actual entry node.
        self.next_pos = np.array([0,0,0], dtype=int)
        self.last_node = np.copy(self.pos)
        self.origin = np.copy(self.pos)
        self.exit_node = np.copy(exit_node)
        self.death_row = np.copy(exit_destination) #where the robot should go to when exiting the shape.
        self.alg = alg
        
        #ADDERS
        #These help us create saddles that are regular or inverted so we can loop through all the points in saddle more easily.
        #DEVELOPERS NOTE: the first and last point of each saddle adder is repeated.
        self.regular_adder = np.array([[0, 0, 0],
                                        [0, 0, 1],
                                        [0, 1, 1],
                                        [0, 1, 0],
                                        [1, 1, 0],
                                        [1, 1, 1],
                                        [1, 0, 1],
                                        [1, 0, 0],
                                        [0, 0, 0]], dtype=int)
        
        self.inverted_adder = np.array([[0, 0, 0],
                                        [0, 0, 1],
                                        [1, 0, 1],
                                        [1, 0, 0],
                                        [1, 1, 0],
                                        [1, 1, 1],
                                        [0, 1, 1],
                                        [0, 1, 0],
                                        [0, 0, 0]], dtype=int)
        
        #initialize information about the box that we are in (the saddle inversion, saddle points, etc.)
        self.__update_box_info()        

        #initalize boxes and nodes visited, etc.
        self.initialize_data(origin=self.pos) #our location IS the origin, because we enter at the origin.
        self.visited_in_saddle = 1 #The number of positions we have visited within the saddle

        #initalize variables that will help us pack messages easier.
        self.msg_type = 99
        self.broadcast = 0 #0 is not broadcast, 1 is broadcast
        self.desired_recipient = np.array([0,0,0],dtype=int)
        self.time_of_last_shape_change = 0.0
        self.change_counter = 0

        #initialize a list where we will store messages that we then have to parse and deal with.
        self.messages = []

        #External nodes visited = list of nodes we have visited OR the memory of others we are relaying
        #it is external because it is what is sent to others.
        self.external_nodes_visited = self.nodes_visited.copy()

        #initialize our readiness for accepting and processing human-input for change
        self.change_readiness = 'none' #'none' for not possible to change, 'add' for addition, 'sub' for stubtraction.
        self.change_readiness_box = np.zeros(3,dtype=int)
        self.changed_box = np.zeros(3,dtype=int) #the box that has been most recently added or subtracted.
        self.ncp_bot_pos = np.zeros(3,dtype=int)
        self.last_change_type = 'none'

        #initialize a statemachine for moving within the shape.
        self.shape_stm = ShapeStateMachine(self.id)

        #used for passback robots in algorithm 3
        self.prescribed_destination = np.zeros(3,dtype=int)
        self.passback_msg_received = False
        self.passback_first_movement = True
        self.responding_to_change_bot = False

        self.msg_counter = 0

       
    def initialize_data(self, origin):
        '''
        initailize nodes visited, boxes visited, the shape, etc. based on our entry position
        '''

        #Create a regular (non-inverted saddle)        
        saddle = np.array([[0, 0, 0],
                            [0, 0, 1],
                            [0, 1, 1],
                            [0, 1, 0],
                            [1, 1, 0],
                            [1, 1, 1],
                            [1, 0, 1],
                            [1, 0, 0]], dtype=int)
        
        #move that saddle so its first node is at the origin, and scale it to the right step size (assumed 1)
        saddle = self.step_size*saddle + origin

        #loop through that temporary saddle to find an index pointer to the robot's position within that saddle.
        for i,s in enumerate(saddle):
            if np.all(s==self.pos):
                pointer = i

        #Use the pointer and saddle to create a list of visited nodes and boxes (basically hard coded to start.)
        Vn = saddle[:pointer+1,:].tolist()
        Vb = [[0,0,0]]
        self.nodes_visited = Vn
        self.boxes_visited = Vb
        self.parent_boxes = Vb
        self.saddle_pointer = pointer

        #Assume the shape is the saddle (Knowledge will be expanded later using set_shape.)
        self.shape = saddle.tolist() 

    def run(self, params = []):
        '''
        This is the bot main page. All bot logic goes through/starts here.
        '''


        #parse any given parameters:
        if len(params) > 0:
            now = params[0] #current time
            human_input = params[1] #boolean t/f of if a human interacted w. us or not.
        else:
            human_input = False
        
        if human_input:
            print("HUMAN INPUT SET WHEN NOT SUPPOSED TO BE SET.")
            human_input = False

        #the string identifyer for our curreint in-shape state
        state_id = self.shape_stm.current_state.id

        #First, process any human inputs (if there are any to process)
        if human_input:
            #if we are an adder, we initiate an add.
            if self.change_readiness == 'add':
                #get the box that we were tracking as the box we would add if a human input occurred
                self.added_box = np.copy(self.change_readiness_box)
                #add that box to our knowledge of the shape
                self.__append_shape(self.added_box)
                #Log the current time as the time of the change
                self.time_of_last_shape_change = now
                #Set the message type to an adder
                self.msg_type = 1
                #Set broadcast to 1:
                self.broadcast = 1
                #Set the recipient as our position (as ncp)
                self.desired_recipient = np.copy(self.pos)
                #set the changed box as the added box
                self.changed_box = np.copy(self.added_box)
                self.last_change_type = 'add'
                #Transition to the ncp state
                self.shape_stm.default_to_ncp(self)
            #if we area a subtracter, we initiate a subtraction
            elif self.change_readiness == 'sub':
                #get the box that we were tracking as the box we would remove if a human input occurred
                #(This is the box that we are IN)
                self.removed_box = np.copy(self.change_readiness_box)
                #Remove that box from our knowledge of the shape.
                self.__remove_shape(self.removed_box)
                #Log the current time as the time of the change
                self.time_of_last_shape_change = now
                #Set the message type to a PRE-subtracter
                self.msg_type = 3
                #Set broadcast to 1
                self.broadcast = 1
                #Set the recipient as the removed box
                self.desired_recipient = np.copy(self.removed_box)
                #Set the changed box as the removed box
                self.changed_box = np.copy(self.removed_box)
                self.last_change_type = 'sub'
                #DEVELOPERS NOTE: DON't transition to the file out state (do this upon receipt of a actual subtracter message)
                #This makes it more consistent for all robots to transition at the same time/upon the same trigger
                         

        #Second, process any messages (if there are any to process)
        if len(self.messages) > 0:
            for msg in self.messages:
                self.process_message(msg)

        #######################################################
                ### EXECUTE BEHAVIOR BASED ON STATE ###
        #######################################################

        #DEFAULT BEHAVIOR STATE
        if state_id == 'default':
            next_pos, exiting = self.default_behavior()

        #NCP STATE
        elif state_id == 'ncp':
            #determine if the added box is inverted
            added_box_inverted = find_if_box_is_inverted(self.added_box)
            #Create a prescribed bot to help move through the prescribed fill-in process
            self.pbot = PrescribedBot(self.id, self.pos, self.added_box, added_box_inverted, self.step_size)
            #find the next pos based on that prescribed order
            next_pos, _ = self.pbot.run()
            self.next_pos = next_pos
            exiting = False
            #auto transition to the prescribed state
            self.shape_stm.ncp_to_prescribed()
        
        #PRESCRIBED STATE (FOR FILL IN)
        elif state_id == 'prescribed':
            #find the next pos based on that prescribed order
            next_pos = self.pbot.run()
            self.next_pos = next_pos
            exiting = False

        #STAY STATE 
        elif state_id == 'stay':
            self.next_pos = np.copy(self.pos)
            exiting = False

        #SUB NCP STATE
        elif state_id == 'sub ncp':
            next_pos, exiting = self.default_behavior()
            #auto transition to the default state
            self.shape_stm.sub_ncp_to_default()
            

        #FILE OUT STATE
        elif state_id == 'file out': 
            #create a file out bot if we don't have one (i.e., we just transitioned here)
            if not hasattr(self,'fbot'):
                self.fbot = FileOutBot(self.id, self.pos, self.removed_box, self.ncp_bot_pos, self.step_size)
                #we save the memory of nodes visited that was sent to us from the msg type 2.
                #This shouldn't already be overwitten with our own memory since 'just moved' hasn't happend yet
                #Specifically, we processed the inbound msg_type == 2, saved the message's memory into our external nodes visited
                #THEN we transitioned to 'file out' and now this code is executing. We haven't moved yet.
                #And, by saving it in the fbot, we don't have to worry about it being overwritten again before we need it.
                self.fbot.saved_memory = self.external_nodes_visited.copy()

            next_pos = self.fbot.run()
            self.next_pos = next_pos
            exiting = False

        #ALG 2 SENDER STATE
        elif state_id == 'alg2 sender':
            done = self.alg2send()
            next_pos = np.copy(self.pos)
            exiting = False

            if done:
                #prepare a go message for broadcast
                self.msg_type = 5 #GO msg
                self.broadcast = 1
                self.external_nodes_visited = self.nodes_visited.copy()

        #PASSBACK STATE
        elif state_id == 'passback':
            #if we didn't receive a passback message telling us where to go, we are likely in 1 of 2 cases
            if not self.passback_msg_received:
                #if we are at the exit node, exit.
                if np.all(self.pos == self.exit_node):
                    next_pos = np.copy(self.death_row)
                    self.next_pos = next_pos
                    exiting = True
                #otherwise, go into our parent box (closest node)
                #triggers when fill-in robots don't get a follow message from the robot in front of them
                #since the robot in front of them never filled in the box.
                elif self.passback_first_movement:
                    next_pos = self.enter_parent() #self.next_pos saved here too
                    exiting = False
                else:
                    print('I HAVE NO WHERE TO GO AS A PASSBACK ROBOT!')
            else:
                exiting = False
                self.next_pos = np.copy(self.prescribed_destination)

            #save all the relative information to make a passback message
            self.prep_passback_message()



        #CHANGE BOT STATE
        elif state_id == 'change bot':
            #execute the default behavior
            next_pos, exiting = self.default_behavior()

            #save all the relative information to make a change bot message
            self.prep_cb_message()



            


            






        return next_pos, exiting
    
    def just_moved(self, pos, params = []):

        global fast_pointer
        '''
        #this code updates memory and causes transitions of state
        #it is executed each time a robot arrives at a new node (at the end of flyer movement)
        '''
        #Set our position as the new position we just moved to.
        self.set_pos(pos)

        #Updcate visited nodes
        if self.pos.tolist() not in self.nodes_visited:
            self.nodes_visited.append(self.pos.tolist()) #update visited nodes list here!

        fast_pointer += 1

        if len(params) > 0:
            flyer = params[0]
            # flyer.log(['just moved' , pos])
            # flyer.log(self.nodes_visited)

        #update information about the box that we are in (identify it, see if it is inverted, etc.)
        self.__update_box_info() #update box info here! (also updates inversion, self.saddle is set here too.)

        #Add that box to the list of visited boxes and parent boxes, accordingly.
        box = self.box.tolist()
        self.__update_box_lists(box)

        #####
        #Determine where we are within our saddle.
        #####
        saddle = np.copy(self.saddle)
        found = False
        self.visited_in_saddle = 0
        #Loop to find our place in the saddle
        for j,s in enumerate(saddle):
            if np.all(s==self.pos) and not found: #find the saddle position that we are currently in (s == pos)
                k = j + 1
                #k now points to the next position in the saddle. If it points beyond the length of the saddle, we break so to avoid an error
                if k == len(saddle[:,0]):
                    break
                self.saddle_pointer = j
                found = True
            #We also need to log the number of steps we have taken through the saddle, so we know when we have completed the saddle.
            if s.tolist() in self.nodes_visited:
                self.visited_in_saddle += 1
        #need to adjust for extra node at end of saddle (first saddle node is repeated at end.)
        #this way, we have an accaurate count of how many nodes we have visited within the saddle.
        self.visited_in_saddle = self.visited_in_saddle - 1     


        #######
        #quick error check to make sure we found our place in the saddle (should only run if there's a bug!)
        #######
        if not found:
            print('ERROR finding saddle info when just moved!')
            print(saddle)
            print(self.pos)

        #revert to sending basic messages only with our own memory in them.
        self.reset_comms()

        

        ###################################################################################
        ############## THE REMAINING BEHAVIOR DEPENDS ON OUR CURRENT STATE ################
        ###################################################################################
        state = self.shape_stm.current_state.id
        if state == 'default':
            #determine our change readiness based on our new location.
            self.determine_change_readiness()

        elif state == 'file out':
            end, last_out = self.fbot.step()
            #if we are at the sub_ncp, then we adjust our memory before we transition to at sub_ncp
            if np.all(self.pos == self.ncp_bot_pos):

                #make our memory that which we stored in the fbot
                self.nodes_visited = self.fbot.saved_memory.copy()

                #make sure our current position is in the new memory (it should be, but just in case)
                p = self.pos.tolist()
                if p not in self.nodes_visited:
                    self.nodes_visited.append(p)

                #Go through and recreate our boxes visited and parent boxes
                self.boxes_visited = []
                self.parent_boxes = []
                for node in self.nodes_visited:
                    box = get_box_from_node(np.array(node), self.step_size).tolist()
                    self.__update_box_lists(box)

                #see if we are the last out
                last_out = self.fbot.last_out

                #Delete the fbot
                del self.fbot

                #if we are the last out, we kick off secondary changes (either algorithm)
                #and we transition to a state based on which change we kick off
                if last_out:
                    if self.alg == 2:
                        self.shape_stm.go_to_stay()
                        #kick off promotion message sending
                        self.msg_type = 11
                        self.broadcast = 1

                    elif self.alg == 3:
                        self.shape_stm.become_passback()
                        self.passback_first_movement = True
                        #Kick off the start of secondary changes
                        self.msg_type = 20
                        self.broadcast = 1
                        self.external_nodes_visited = self.nodes_visited.copy()


                #otherwise, we transition to just being at the sub_ncp (which will auto transition to default)
                else:
                    #transition to sub_ncp
                    self.shape_stm.file_out_to_sub_ncp()

                

        elif state == 'prescribed':
            end = self.pbot.step()
            #if we reached the end of our prescribed order, then we transition to stay (if alg 2)
            if end: 
                if self.alg == 2:
                    self.shape_stm.go_to_stay()
                    del self.pbot
                    #Kick off promotion message sending
                    self.msg_type = 11
                    self.broadcast = 1

                elif self.alg == 3:
                    self.shape_stm.become_passback()
                    self.passback_first_movement = True
                    del self.pbot
                    #Kick off the start of secondary changes
                    self.msg_type = 20
                    self.broadcast = 1
                    self.external_nodes_visited = self.nodes_visited.copy()

        elif state == 'passback':
            self.passback_msg_received = False 
            self.passback_first_movement = False
            self.responding_to_change_bot = False


    def reset_comms(self):
        self.msg_type = 99
        self.broadcast = 1
        self.external_nodes_visited = self.nodes_visited.copy()    
        self.msg_counter = 0

    def pack_message(self):
        '''
        Populates an outgoing message in the standard format
        returns True, msg if successful, False, False if nothing to send.
        '''

        #first part is id, msg type, broadcast 
        first_part = struct.pack('3B', self.id, self.msg_type, self.broadcast)

        #Next is the robot position in compact form
        compact_pos = compact_format(self.pos)

        #next is the robot next position in compact format
        compact_next_pos = compact_format(self.next_pos)

        #next is the position of the recipient/(was also POI in lw sim)
        compact_recipient = compact_format(self.desired_recipient)

        #next is the new poi (if I decide to use it)
        compact_poi = compact_format([0,0,0])

        #next is the change ID (now a signed int, not a float anymore)
        change_id = struct.pack('b', self.change_counter)

        #next is the drone's knowledge of the latest chagned box
        compact_change_box = compact_format(self.changed_box)

        #next part is the visited nodes
        packed_visited = create_bitmap(self.external_nodes_visited)
        
        #last is the shape
        packed_shape = create_bitmap(self.shape)


        msg = first_part + compact_pos + compact_next_pos + compact_recipient + compact_poi + change_id + compact_change_box + packed_visited + packed_shape

        return True, msg

        
    def unpack_message(self, msgs, log, now):
        '''
        Accepts the messages directly in the format from flyer.recv_msg and the position of the flyer in virtual space.
        Looks at each message and parses it accordingly.
        '''
        #DEVELOPERS NOTE: Some of the existing code in the lightweight simulator has been removed, so may need to check back there on how to deal with other messages.

        self.messages = []

        for msg in msgs:

            save_msg = False

            #unpack the beginning to see what we're dealing with 
            preamble = struct.unpack('3B', msg[:3])
            msgtype = preamble[1]
            broadcast = preamble[2]
            sender = pos_from_compact_format(msg[3:5])

            #if the message is out side of range, ignore it.
            if math.dist(sender, self.pos) > COMM_RADIUS:
                continue
        
            #if the message was a broadcast 1 (i.e., its for everyone/repeater)
            elif broadcast == 1:
                save_msg = True

            #if the message was a direct message
            elif broadcast == 0:
                #find the recipient and see if it is meant for us.
                recipient = pos_from_compact_format(msg[7:9])
                if np.all(self.pos == recipient):
                    save_msg = True

            #if it is intended for us, parse all of its data and save it. We will act on it later.
            if save_msg:
                m = ProcessedMessage(preamble[0],msgtype)
                m.sender_position = np.copy(sender)
                m.sender_next_pos = np.array(pos_from_compact_format(msg[5:7]))
                m.sender_point_of_interest = np.array(pos_from_compact_format(msg[7:9]))
                m.sender_changed_box = np.array(pos_from_compact_format(msg[12:14]))
                m.sender_nodes_visited = unpack_bitmap(msg[14:78])
                m.sender_shape = unpack_bitmap(msg[78:142])
                m.sender_last_shape_change_id = struct.unpack('b',msg[11:12])
                self.messages.append(m)

                # log([now, 'msg rec', m.sender_id, m.sender_position[0], m.sender_position[1], m.sender_position[2], m.sender_next_pos[0], m.sender_next_pos[1], m.sender_next_pos[2]])



            ########################################################################## 
            #SPECIAL CASES:
            ########################################################################## 
            #if we are transmitting a memory message, we need to listen for when 
            #downstream robots start sending it so we know when to stop sending it.
            if self.shape_stm.current_state.id == 'alg2 sender':
                if np.all(self.next_pos == self.sender):
                    #transition to stay (we have successfully passed the memory message)
                    self.shape_stm.go_to_stay()
                    #comms reset (so we stop broadcasting memory message)
                    self.reset_comms()

        return
    
    def process_message(self, msg:ProcessedMessage):
        '''
        Accepts a list of pre-processed messages so we can react to them accordingly.
        Returns the transition that the in-shape state machine should execute (if any)
        '''
        '''
        Message codes:
        1 = adder message (the shape has been added to.)
        2 = subtracter message (a part of the shape has been removed)
        5 = GO message (make stay bots go.)
        10 = memory message (passing along new memory IAW alg 2)
        11 = promotion message (find and promote a robot at the SCSN, make all robots stay.)
        20 = alg3 kickoff message
        21 = change bot message
        22 = passback message
        99 = standard message
        '''

        # print(str(self.id) + ': processing ' + str(msg.msg_type))


        if msg.msg_type == 1: #this is an adder message. 
            #Check to see if the timing is AFTER our last known shape change.
            #if it is, then it is NEW information for us, and we have to update our knowledge
            if msg.sender_last_shape_change_id > self.time_of_last_shape_change:
                #next, double check that their shape is bigger than ours (since this is an ADDER message, it should be.)
                if len(self.shape) < len(msg.sender_shape):

                    #First, record the received time of shape cahnge as the time_of_last_shape_change so we pass along the right information
                    self.time_of_last_shape_change = msg.sender_last_shape_change_id

                    #Make their shape, our shape.
                    self.set_shape(msg.sender_shape)

                    #Store the box that has been added to the shape.
                    self.added_box = np.copy(msg.sender_changed_box)

                    #Determine if we are upstream or downstream of the sender's point of interest (the ncp)
                    self.determine_upstream_or_downstream(msg.sender_point_of_interest, 'add')

                    #Setup to repeat the message through the swarm.
                    self.msg_type = 1
                    self.broadcast = 1
                    self.desired_recipient = np.copy(msg.sender_point_of_interest)
                    self.changed_box = np.copy(msg.sender_changed_box)
                    self.last_change_type = 'add'
                    self.external_nodes_visited = self.nodes_visited.copy() #send our own memory

        elif msg.msg_type == 2: #This is a subtracter message (it comes only AFTER a sub ncp has been established.)
            #Check to see if the timing is EQUAL TO OR AFTER our last known shape change.
            #DEVELOPERS NOTE: it may be equal to if we already received a pre-subtraction message.
            if msg.sender_last_shape_change_id >= self.time_of_last_shape_change:
                #Next, make sure we are in the default state. 
                #If we are not in the default state, we responded to this before OR we will ignore it because we are meant to be default    
                if self.shape_stm.current_state.id == 'default':
                    #First, record the received time of shape change as the time_of_last_shape_chagne so we pass along the right information
                    #we do this again in case we never did get the msg type 3 (i.e., it already changed to 2 in the swarm before the 3 reached us.)
                    self.time_of_last_shape_change = msg.sender_last_shape_change_id

                    #store the ncp for this subtraction
                    sub_ncp = np.copy(msg.sender_point_of_interest)
                    self.ncp_bot_pos = sub_ncp

                    #Make their shape, our shape.
                    self.set_shape(msg.sender_shape)

                    #Store the box that has been removed from the shape.
                    self.removed_box = np.copy(msg.sender_changed_box)

                    #Determine if we are upstream or downstream of the removed box (esp. the file out node)
                    self.determine_upstream_or_downstream(sub_ncp, 'sub')

                    #Set up to repeat the message through the swarm
                    self.msg_type = 2
                    self.broadcast = 1
                    self.desired_recipient = np.copy(msg.sender_point_of_interest)
                    self.changed_box = np.copy(msg.sender_changed_box)
                    self.last_change_type = 'sub'
                    self.external_nodes_visited = msg.sender_nodes_visited.copy() #send the memory of the robot at sub ncp

        elif msg.msg_type == 3: #This is a PRE subtracter message. It is used to find the ncp for subtraction
            #Check to see if the timing is AFTER our last known shape change
            #If it is, then this is NEW information for us, and we have to update our knowledge.
            if msg.sender_last_shape_change_id > self.time_of_last_shape_change:
                #Next, double check that their shape is smaller than ours (since this is a PRE-SUBTRACTER message, it should be.)
                if len(self.shape) > len(msg.sender_shape):

                    #First, record the received time of shape change as the time_of_last_shape_chagne so we pass along the right information
                    self.time_of_last_shape_change = msg.sender_last_shape_change_id

                    #Make their shape our shape.
                    self.set_shape(msg.sender_shape)

                    #Check if at ncp for subtraction
                    removed_box = np.copy(msg.sender_changed_box)
                    at_ncp = self.check_subtraction_ncp(removed_box)

                    
                    if at_ncp:
                        #transition to the subtraction ncp state (if at ncp)
                        self.shape_stm.default_to_sub_ncp()
                        
                        #begin transmitting an actual subtraction message
                        self.msg_type = 2
                        self.broadcast = 1
                        #set the recipient to our pose so others repeat it as the sub ncp
                        self.desired_recipient = np.copy(self.pos)
                        self.changed_box = np.copy(msg.sender_changed_box)
                        self.last_change_type = 'sub'
                        self.external_nodes_visited = self.nodes_visited.copy() #send our own memory

                    else:
                        #Set up to repeat the message through the swarm
                        self.msg_type = 3
                        self.broadcast = 1
                        self.desired_recipient = np.copy(msg.sender_point_of_interest)
                        self.changed_box = np.copy(msg.sender_changed_box)
                        self.last_change_type = 'sub'
                        self.external_nodes_visited = self.nodes_visited.copy() #send our own memory

        elif msg.msg_type == 5: #this is a GO message (returns robots to default)
            self.shape_stm.return_to_default()
            self.msg_type = 5
            self.broadcast = 1
            self.external_nodes_visited = self.nodes_visited.copy()

        
        elif msg.msg_type == 11:# This is a promotion message (trying to promote the robot at the SCSN for alg 2)
            
            #determine if we are at the SCSN or not
            is_scsn = self.getSCSN()

            #Become an alg2 sender if we are at the scsn
            if is_scsn:
                self.shape_stm.become_alg2sender()

            else:
                #prepare to repeat the promotion message. Go to stay
                self.shape_stm.go_to_stay()
                self.msg_type = 11
                self.broadcast = 1
                self.external_nodes_visited = self.nodes_visited.copy()
                                  

        elif msg.msg_type == 10: #memory message (for algorithm 2)
            new_memory = msg.sender_nodes_visited.copy()
            self.alg2receive(new_memory)
            #now prepare to become a alg2 sender
            _ = self.alg2send()  #DEVELOPERS NOTE: run alg2send here so we start sending msg type 10 right away (even before our next run of self.run())
            self.shape_stm.become_alg2sender()



        
        elif msg.msg_type == 20: #alg3 kickoff message

            #determine if we are at the SCSN or not
            is_scsn = self.getSCSN()

            #become the change robot if we are at the scsn
            if is_scsn:
                self.shape_stm.become_changebot()
            else:
                #prepare to repeat the kickoff message. Go to default or passback
                self.msg_type = 20
                self.broadcast = 1
                self.external_nodes_visited = self.nodes_visited.copy()

                #if the last change was an add
                if self.last_change_type == 'add':
                    #then, we need to know if we were just in default. If so, we stay default.
                    if self.shape_stm.current_state.id == 'default':
                        self.shape_stm.return_to_default()
                    #otherwise, we are definitely downstream (filling in, fileing out, or staying) and we should be passback
                    else:
                        self.shape_stm.become_passback()
                        self.passback_first_movement = True
                #if the last chagne was a subtraction
                elif self.last_change_type == 'sub':
                    #then, we need to know if we were just in the 'stay' state. 
                    #if not, then we know we were downstream of the ncp (default) or filing out or at the sub ncp.
                    if self.shape_stm.current_state.id != 'stay':
                        #in all of those cases, we become passback
                        self.shape_stm.become_passback()
                        self.passback_first_movement = True
                    #otherwise, if we are at stay, then we have to determine if we are upstream or downstream of the removed box entry point (the SCSN)
                    elif self.shape_stm.current_state.id == 'stay':
                        #we can check this by seeing if we have visited ANY node in the changed box.
                        box_inverted = find_if_box_is_inverted(self.changed_box)
                        box_list = find_nodes_from_LL(self.changed_box, self.step_size, box_inverted).tolist()
                        downstream = False
                        for node in box_list:
                            if node in self.nodes_visited:
                                downstream = True
                                break

                        if downstream:
                            self.shape_stm.become_passback()
                            self.passback_first_movement = True

                        else:
                            self.shape_stm.return_to_default()


        elif msg.msg_type == 21: #change bot message
            self.__receive_cb_message(msg.sender_position, msg.sender_point_of_interest)
            

        elif msg.msg_type == 22 and not self.responding_to_change_bot: #passback message
            #respond to a passback message ONLY if we are a passback robot.
            if self.shape_stm.current_state.id == 'passback':
                #flag that we received a destination
                self.passback_msg_received = True
                #set our destination to our sender's location.
                self.prescribed_destination = np.copy(msg.sender_position)

        elif msg.msg_type == 99: #standard message
            self.msg_counter += 1



        return

    
    def get_pos(self):
        return self.pos
    
    def set_pos(self,pos):
        #accepts a 3d x,y,z numpy array.
        self.last_node = np.copy(self.pos) #record our last node
        self.pos = pos
        return
    
    def set_shape(self, known_shape):
        self.shape = known_shape

    def __append_shape(self, box_node):
        box_inv = find_if_box_is_inverted(box_node)
        nodes = find_nodes_from_LL(box_node, self.step_size, box_inv)
        for n in nodes:
            if n.tolist() not in self.shape:
                self.shape.append(n.tolist())

    def __remove_shape(self, box_node):
        box_inv = find_if_box_is_inverted(box_node)
        nodes = find_nodes_from_LL(box_node, self.step_size, box_inv)
        for n in nodes:
            if n.tolist() in self.shape:
                self.shape.pop(self.shape.index(n.tolist()))
    
    def default_behavior(self, verbose = False):

        global fast_pointer

        exiting = False

        #in general, this subroutine finds the next pos (as an array)
        next_pos_found = False

        #exit node criterion:
        if np.all(self.pos==self.exit_node): #effectively pos == self.exit_node
            next_pos = self.death_row
            exiting = True

        # #SHORTCUT FROM ENTRY TO EXIT NODE - FOR TESTING ONLY
        # elif np.all(self.pos==self.origin):
        #     next_pos = self.exit_node
        #SHORTCUT for FAST user code
        elif True:
            next_pos = np.array(STARTING_SHAPE[fast_pointer])
        
        #Otherwise:
        else:          

            #######
            # create list of plausible next edges/nodes
            #######
            possible_edges = identify_potential_edges(self.pos) #stores the possible nodes (as an array) that the robot could move to.
            if verbose:
                print('new loop')
                print('id: ' + str(self.id))
                print('pos: ' + str(self.pos))
                print('possible edges: ' + str(possible_edges))
                print('shape: ' + str(self.shape))

            #######
            ## remove visited and out-of-shape options
            #######
            #First, convert the possible destinations to a list (from a numpy array)
            possible_edge_list = possible_edges.tolist()
            #Next, loop through thae ARRAY, turning each possible destination node into a list and seeing if it is visited or in the shape.
            #Adjust the LIST of possible next edges by removing them if they are visited or not in the shape.
            for n in possible_edges:
                n_list = n.tolist()

                #If neighbor visited
                if n_list in self.nodes_visited:
                    possible_edge_list.pop(possible_edge_list.index(n_list))
                    if verbose:
                        print('popped ' + str(n_list) + ' for being visited.')
                    continue #go to the next one
                
                #If neighbor not in shape
                if n_list not in self.shape: 
                    possible_edge_list.pop(possible_edge_list.index(n_list))
                    if verbose:
                        print('popped ' + str(n_list) + ' for being out of shape.')
            if verbose:
                print('possible edge list reduced: ' + str(possible_edge_list))


            #######
            #Test what the saddle would have us do.
            #get a full glimpse of the saddle in order
            #######
            #otherwise, use k to determine where we should go next IF we follow the saddle.
            saddle_destination = np.copy(self.saddle[self.saddle_pointer+1]) #store our saddle destination locally
            saddle_destination_vertical = test_for_vertical_movement(self.pos, saddle_destination) #test to see if saddle destination is vertical movement

            #######
            #Determine if we have completed the saddle or not (assume we haven't unless the # visited >= 8 saddle points)
            #######
            saddle_complete = False
            if self.visited_in_saddle >= 8:
                saddle_complete = True
            if verbose:
                print('saddle complete: ' + str(saddle_complete))
            
            #######
            #Employ default behavior rules
            #######
            #RULE 1: if new box and vertical movement and saddle would be horizontal, take that edge
            for n in possible_edge_list:
                destination = np.array(n)
                box_of_destination = get_box_from_node(destination, self.step_size)
                vertical_test = test_for_vertical_movement(self.pos, destination)
                if box_of_destination.tolist() not in self.boxes_visited and vertical_test and not saddle_destination_vertical and not saddle_complete:
                    next_pos = destination
                    next_pos_found = True
                    if verbose:
                        print('rule 1')
                    break
            
            #RULE 2: if new box and not vertical movement and saddle would be vertical, take that edge
            if not next_pos_found:
                for n in possible_edge_list:
                    destination = np.array(n)
                    box_of_destination = get_box_from_node(destination, self.step_size)
                    vertical_test = test_for_vertical_movement(self.pos, destination)                      
                    if box_of_destination.tolist() not in self.boxes_visited and not vertical_test and saddle_destination_vertical and not saddle_complete:
                        
                        next_pos_found = True
                        next_pos = destination
                        break

                if next_pos_found and verbose:
                    print('rule 2')

            #RULE 3: if destination is within self.box, follow it if it is in accordance with saddle
            if not next_pos_found:
                valid_destination_inbox = False
                for n in possible_edge_list:
                    destination = np.array(n)
                    box_of_destination = get_box_from_node(destination, self.step_size)
                    if np.all(box_of_destination == self.box): #current box and destination box are the same
                        valid_destination_inbox = True
                
                #if we saw that at least one valid destination was within our box, then we should evaluate them in order of the saddle
                if valid_destination_inbox and saddle_destination.tolist() in possible_edge_list:
                    next_pos = np.copy(saddle_destination)
                    next_pos_found= True
                    if verbose:
                        print('rule 3')               

            #RULE 4: if destination is within parent box, follow it.
            if not next_pos_found:
                for n in possible_edge_list:
                    destination = np.array(n)
                    box_of_destination = get_box_from_node(destination, self.step_size)
                    if verbose:
                        print('looking like #4')
                        print(self.boxes_visited)
                        print(self.parent_boxes)
                        print('current pos: ', self.pos)
                        print('current box: ' ,self.box)
                    if len(self.parent_boxes) == 1:
                        parent_box = self.parent_boxes[-1]
                        if verbose:
                            print('case 1: there is only 1 thing in the parent box list')
                    else:
                        parent_box = self.parent_boxes[-2]
                        if verbose:
                            print('case 2: there are > 1 parent boxes in the list.')
                    if verbose:
                        print('identified parent box:', parent_box)
                        print('box of destination: ' + str(box_of_destination))
                    if np.all(box_of_destination == parent_box):
                        next_pos = destination
                        next_pos_found = True
                        if verbose:
                            print('rule 4')
                        break            
            if verbose:
                print('next pos: ' + str(next_pos)) 

        #try to set the next pos here. If we can't, then the algorithm didn't get a next_pos (probably because the robot didn't get to the right position)
        # try:
        self.next_pos = next_pos       
        # except:
        #     self.next_pos = np.copy(self.pos)

        return next_pos, exiting      

        

    def __update_box_info(self):
        '''
        Determine the box that a robot is in given its current position in the virtual world
        Also determines if that box is inverted and creates a saddle of points for that box for quick reference later.
        '''
        self.box = get_box_from_node(self.pos, self.step_size)
        self.inv = find_if_box_is_inverted(self.box)
        saddle = self.box * np.ones((9,3))
        if not self.inv:
            saddle = saddle + self.regular_adder
        elif self.inv:
            saddle = saddle + self.inverted_adder
        self.saddle = np.copy(saddle)

    def __update_box_lists(self, box):
        '''
        accepts a box as a list and updates the parent box and boxes visited lists accordingly.
        '''
        #Add that box to the list of visited boxes and parent boxes, accordingly.
        if box not in self.boxes_visited:
            self.boxes_visited.append(box) #update boxes visited list here!
        if box not in self.parent_boxes:
            self.parent_boxes.append(box) #if we are in a new box, add it to the parent list/queue
        elif box in self.parent_boxes:
            index = self.parent_boxes.index(box)
            self.parent_boxes = self.parent_boxes[0:index+1] #if we are in an old box, get me everything up to (and including) the current box (cuts off anything after that)!

        

    def determine_change_readiness(self):
        '''
        Determines if the flyer is at an NCP for addition, or in the middle of the shape ready for subtraction.
        basically ncp = node from which fill in will begin for addition (or file out begins for subtraction)
        Sets the flyers change readiness designation accordingly.
        '''

        we_are_adder = False

        #Case where we are NOT in an inverted cube:
        if not self.inv:

            #Check if we are at an ncp for addition based on our saddle location
            #we will create a list of possible points that could be added to the shape, and then evaluate them below.
            #In position 0 for a non inverted cube, we may be involved in adding boxes in -x or -y
            if self.saddle_pointer == 0:
                possible_added_box_point_1 = self.pos + self.step_size*np.array([-1,0,0])
                possible_added_box_point_2 = self.pos + self.step_size*np.array([0,-1,0])
                pabp_list = [possible_added_box_point_1, possible_added_box_point_2]

            #In position 1 OR 5 for a non inverted cube, we may be involved in adding boxes in +z only
            elif self.saddle_pointer == 1 or self.saddle_pointer == 5:
                possible_added_box_point_1 = self.pos + self.step_size*np.array([0,0,1])
                pabp_list = [possible_added_box_point_1]

            #In position 2 for a non inverted cube, we may be involved in adding boxes in -x or +y
            elif self.saddle_pointer == 2:
                possible_added_box_point_1 = self.pos + self.step_size*np.array([-1,0,0])
                possible_added_box_point_2 = self.pos + self.step_size*np.array([0,1,0])
                pabp_list = [possible_added_box_point_1, possible_added_box_point_2]

            #In position 3 OR 7 for a non inverted cube, we may be involved in adding boxes in -z only
            elif self.saddle_pointer == 3 or self.saddle_pointer == 7:
                possible_added_box_point_1 = self.pos + self.step_size*np.array([0,0,-1])
                pabp_list = [possible_added_box_point_1]

            #In position 4 for a non inverted cube, we may be involved in adding boxes in +x or +y
            elif self.saddle_pointer == 4:
                possible_added_box_point_1 = self.pos + self.step_size*np.array([1,0,0])
                possible_added_box_point_2 = self.pos + self.step_size*np.array([0,1,0])
                pabp_list = [possible_added_box_point_1, possible_added_box_point_2]

            #In position 6 for a non inverted cube, we may be involved in adding boxes in +x or -y
            elif self.saddle_pointer == 6:
                possible_added_box_point_1 = self.pos + self.step_size*np.array([1,0,0])
                possible_added_box_point_2 = self.pos + self.step_size*np.array([0,-1,0])
                pabp_list = [possible_added_box_point_1, possible_added_box_point_2]

        #Case where we ARE in an inverted cube:
        elif self.inv:

            #Check if we are at an ncp for addition based on our saddle location
            #we will create a list of possible points that could be added to the shape, and then evaluate them below.
            #In position 0 for a inverted cube, we may be involved in adding boxes in -x or -y
            if self.saddle_pointer == 0:
                possible_added_box_point_1 = self.pos + self.step_size*np.array([-1,0,0])
                possible_added_box_point_2 = self.pos + self.step_size*np.array([0,-1,0])
                pabp_list = [possible_added_box_point_1, possible_added_box_point_2]

            #In position 1 OR 5 for a inverted cube, we may be involved in adding boxes in +z only
            elif self.saddle_pointer == 1 or self.saddle_pointer == 5:
                possible_added_box_point_1 = self.pos + self.step_size*np.array([0,0,1])
                pabp_list = [possible_added_box_point_1]

            #In position 2 for a inverted cube, we may be involved in adding boxes in +x or -y
            elif self.saddle_pointer == 2:
                possible_added_box_point_1 = self.pos + self.step_size*np.array([1,0,0])
                possible_added_box_point_2 = self.pos + self.step_size*np.array([0,-1,0])
                pabp_list = [possible_added_box_point_1, possible_added_box_point_2]

            #In position 3 OR 7 for a inverted cube, we may be involved in adding boxes in -z only
            elif self.saddle_pointer == 3 or self.saddle_pointer == 7:
                possible_added_box_point_1 = self.pos + self.step_size*np.array([0,0,-1])
                pabp_list = [possible_added_box_point_1]

            #In position 4 for a inverted cube, we may be involved in adding boxes in +x or +y
            elif self.saddle_pointer == 4:
                possible_added_box_point_1 = self.pos + self.step_size*np.array([1,0,0])
                possible_added_box_point_2 = self.pos + self.step_size*np.array([0,1,0])
                pabp_list = [possible_added_box_point_1, possible_added_box_point_2]

            #In position 6 for a inverted cube, we may be involved in adding boxes in -x or +y
            elif self.saddle_pointer == 6:
                possible_added_box_point_1 = self.pos + self.step_size*np.array([-1,0,0])
                possible_added_box_point_2 = self.pos + self.step_size*np.array([0,1,0])
                pabp_list = [possible_added_box_point_1, possible_added_box_point_2]

                   


        #check the points in the possible added point list to see if we are an adder for either of them.
        for pabp in pabp_list:

            #if the possible point is in the shape, continue (we are not an ncp for addition by default)
            possible_point = pabp.tolist()
            if possible_point in self.shape:
                continue

            #this is the box that could possibly be added to the shape
            possible_added_box = get_box_from_node(pabp, self.step_size)

            #this will create a seach grid and then call check_addition_ncp to see if it is a ncp or not
            is_ncp = self.addition_setup(possible_added_box) 

            #if this is at a node where we would fill in the new box from, then record its readiness
            if is_ncp:
                self.change_readiness = 'add'
                self.change_readiness_box = np.copy(possible_added_box)
                we_are_adder = True
                break
            
        #If we are not an adder, then we are a subtracter (unless our box is the origin box)
        if not we_are_adder:
            if np.all(self.box == self.origin):
                self.change_readiness = 'none'
                self.change_readiness_box = np.zeros(3)
            else:
                self.change_readiness = 'sub'
                self.change_readiness_box = np.copy(self.box)




        return

    def getSCSN(self):
        '''
        Determines if a flyer is at the SCSN (returns True) or not (returns False)
        Uses the last saved change box from self.changed_box
        '''
        #Check to see if we are at the scsn for the changed box.
        is_scsn = False
        skip_next_test = False
        #if we have visited any node in the changed box, we are not at the scsn (true for both subtraction and addition)
        box_inverted = find_if_box_is_inverted(self.changed_box)
        box_nodes = find_nodes_from_LL(self.changed_box, self.step_size, box_inverted).tolist()
        for n in box_nodes:
            if n in self.nodes_visited:
                skip_next_test = True
                break
        
        #if we didn't flag 'skip next test', then we didn't visit any nodes in the removed box, and we should do
        #a thorough check to see how many nodes we've visited around the chagned box and if we plan(ed) to move into it.
        if not skip_next_test:
            #it doesn't matter if the change was an add or a subtract, the scsn is the node that would be the ncp IF the box had been added.
            is_scsn = self.addition_setup(self.changed_box)

        return is_scsn


    def addition_setup(self, added_box):
        '''
        Constructs a search grid around the added box to determine if the robot is at a possible ncp (node critical point???) for that box
        basically ncp = node from which fill in will begin.
        returns TRUE if at ncp, FALSE otherwise
        '''

        #set up a search grid around the added box nodes to find the lowest value edge count adjacent to the new box.
        #or the robot that has visited only 1-2 values in the search grid. This becomes our ncp!
        search_grid_adder = np.array([[2, 0, 0],
                                      [2, 1, 0],
                                      [2, 0, 1],
                                      [2, 1, 1],
                                      [0, 2, 0],
                                      [1, 2, 0],
                                      [0, 2, 1],
                                      [1, 2, 1],
                                      [-1, 0, 0],
                                      [-1, 1, 0],
                                      [-1, 0, 1],
                                      [-1, 1, 1],
                                      [0, -1, 0],
                                      [1, -1, 0],
                                      [0, -1, 1],
                                      [1, -1, 1],
                                      [0, 0, 2],
                                      [1, 0, 2],
                                      [0, 1, 2],
                                      [1, 1, 2],
                                      [0, 0, -1],
                                      [1, 0, -1],
                                      [0, 1, -1],
                                      [1, 1, -1]])
        search_grid_adder = self.step_size * search_grid_adder
        search_grid = (added_box + search_grid_adder).tolist()


        #have the robot check if it is at the ncp or not.
        is_ncp = self.check_addition_ncp(search_grid, added_box)

        return is_ncp


    def check_addition_ncp(self,search_grid, added_box):

        #count how many times we've been through the search grid. Looking for 1-2 times max.
        counter = 0
        for v in self.nodes_visited:
            if v in search_grid:
                counter += 1
                if counter > 2:
                    return False #return false - we know we've visited more than 2, so we can't be the ncp.
            
        #if we have visited either 1 or 2 points in the search grid...
        if counter <= 2:
            saddle = np.copy(self.saddle)

            #find the next pos based on saddle order
            #log our saddle info, including if it's a vertical movement or not.
            saddle_destination = np.copy(self.saddle[self.saddle_pointer+1])
            saddle_destination_vertical = test_for_vertical_movement(self.pos, saddle_destination)

            #find the saddle for the newly added box.
            new_saddle = added_box * np.ones((9,3))
            inv = find_if_box_is_inverted(np.copy(added_box))
            if not inv:
                new_saddle = new_saddle + self.regular_adder
            elif inv:
                new_saddle = new_saddle + self.inverted_adder

            #find the nearest point in the new saddle
            new_saddle_list = new_saddle.tolist()
            nearest_point = []
            for point in new_saddle_list:
                if math.dist(self.pos, point) == self.step_size:
                    nearest_point = point
                    break
                    
            #check if moving to the new saddle will be vertical or horizontal movement for me.
            new_saddle_destination_vertical = test_for_vertical_movement(self.pos, np.array(nearest_point))

            #2 cases. either we would be moving horizontal and the new saddle is vertical
            # OR we would be moving vertical, and the new saddle is horizontal
            if not saddle_destination_vertical and new_saddle_destination_vertical:
                return True
            elif saddle_destination_vertical and not new_saddle_destination_vertical:
                return True

        return False
    
    def check_subtraction_ncp(self, removed_box):
        '''
        Accepts a numpy x,y,z array of the lower left corner of the removed cube
        Determines if the flyer is at the ncp for the subtraction
        Returns True if at ncp, False otherwise
        '''

        at_ncp = False

        #create an array of points in the removed shape. (assume lower left is identified as removed box from csv input file)
        removed_box_is_inverted = find_if_box_is_inverted(removed_box)
        removed_box_array = find_nodes_from_LL(removed_box, self.step_size, removed_box_is_inverted)
        removed_box_list = removed_box_array.tolist()

        #check if we are in the removed box list.
        if self.pos.tolist() in removed_box_list:
            pass #we are NOT at the ncp by default.
        else:
            #If we're not in the removed box, we have to figure out if we're at the ncp by counting our visited nodes within it.
            counter = 0
            #loop through the nodes in the removed box.
            for n in removed_box_list:
                if n in self.nodes_visited: #if the node from the removed box is in the visited list, add to counter. If not, bail
                    counter+= 1
                else:
                    #If any 1 of the removed box nodes is not visited, our counter will never be 8, so might as well break here and save time
                    break
            
            #if the counter is 8, then we know we've visited everything in the removed box
            #We just need to check if our last node was in the removed box
            if counter == 8:
                if self.last_node.tolist() in removed_box_list:
                    at_ncp = True

        return at_ncp
    
    def determine_upstream_or_downstream(self, sender, change):
        '''
        Accepts an x,y,z array of the sender's position of interest and a change (either 'add' or 'sub')
        Determines if the robot is upstream or downstream of this change.
        Executes the transition the in-shape state machine should take (if any)
        'sender' is the ncp for add and the ncp for 'sub'
        '''

        if change == 'add':
            poi = sender.tolist()
            #upstream bots:
            if poi not in self.nodes_visited:
                #transition to default state
                self.shape_stm.return_to_default()
                if self.shape_stm.current_state.id != 'default':
                    print("WE HAVE A PROBLEM. WE SHOULD BE IN DEFAULT STATE BUT WE ARE NOT")
                    print(self.shape_stm.current_state.id)
            #at ncp
            elif np.all(self.pos == sender):
                #transition to ncp (which will auto transition to a prescribed fill-in later)
                self.shape_stm.default_to_ncp()
            #downstream bots:
            elif poi in self.nodes_visited:
                #transition to stay
                self.shape_stm.go_to_stay()

        elif change == 'sub': 
            #bots in removed box
            #create an array of points in the removed shape. 
            removed_box_is_inverted = find_if_box_is_inverted(self.removed_box)
            removed_box_array = find_nodes_from_LL(self.removed_box, self.step_size, removed_box_is_inverted)
            removed_box_list = removed_box_array.tolist()
            if self.pos.tolist() in removed_box_list:
                #Transition to file out
                self.shape_stm.default_to_file_out()
            else:
                poi = sender.tolist()
                #upstream bots 
                if poi not in self.nodes_visited: #upstream, not in removed box
                    #transition to stay
                    self.shape_stm.go_to_stay()
                #at ncp
                elif np.all(self.pos == sender):
                    self.shape_stm.file_out_to_sub_ncp() #execute default behavior, memory should have already been updated
                #downstream bots
                elif poi in self.nodes_visited: #Downstream 
                    self.shape_stm.return_to_default()

        return 
        
    def alg2send(self):
        '''
        Code that runs for the robot sending the memory message.
        '''
        
        #get the next position by running the default behavior
        # (if it fails, run again so we can print and see where it fails)
        try:
            next_pos = self.default_behavior(verbose=False)
        except:
            next_pos = self.default_behavior(verbose=True)

        #save our nexst position
        self.next_pos = next_pos

        #check if change is complete (if we are about to exit the shape.)
        if np.all(self.next_pos == self.death_row):
            change_complete = True
        #If not, we need to ready a memory message.
        else:
            #find the receiver, and prep a msg for it
            self.msg_type = 10
            self.broadcast = 0
            self.desired_recipient = np.copy(self.next_pos)
            self.external_nodes_visited = self.nodes_visited.copy()
            change_complete = False

        return change_complete
    
    def alg2receive(self, new_memory:list):
            
        #give the receiver a new memory.
        self.nodes_visited = new_memory.copy()
        #Go through and recreate our boxes visited and parent boxes
        self.boxes_visited = []
        self.parent_boxes = []
        for node in self.nodes_visited:
            box = get_box_from_node(np.array(node), self.step_size).tolist()
            self.__update_box_lists(box)

        #act like we just arrived to add our pos and such to memory as well.
        self.just_moved(self.pos)
                            
        return

    def prep_cb_message(self):
        self.msg_type = 21
        self.broadcast = 1
        self.external_nodes_visited = self.nodes_visited.copy()
        self.desired_recipient = np.copy(self.next_pos)

    def prep_passback_message(self):
        try:
            #find the recipient. (it's our last node)
            if np.all(self.pos == self.nodes_visited[-1]):
                recipient = np.array(self.nodes_visited[-2])
            else:
                recipient = np.array(self.nodes_visited[-1])
        except:
            print(str(self.id) + ': I am having an issue finding my recipient. ' + str(self.nodes_visited))
            print(str(self.id) + ': I am at: ' + str(self.pos))

        if not np.all(recipient == self.last_node) and not np.all(self.last_node == self.pos):
            # print('here')
            recipient = self.last_node
        
        #build the message.
        self.msg_type = 22
        self.broadcast = 0
        self.external_nodes_visited = self.nodes_visited.copy()
        self.desired_recipient = np.copy(recipient)

           
    def __receive_cb_message(self, cbpos, cbdest):
        '''
        Accepts the change bot position (cbpos) and destination (cbdest)
        checks for destination swaps or destination double swaps!
        '''

        # print('evaluating')
        # print(cbpos)
        # print(cbdest)
        # print(self.pos)
        # print(self.next_pos)
        
        #now see if any of the passback robots are going where the change bot is gonna go.
        if np.all(self.next_pos == cbdest) and not np.all(self.pos == cbdest):
            #if so, we need to do something about it.
            self.__destination_swap(cbpos, cbdest)
            
        #We also have to handle the case for type II nodes where there may be 3 exits
        #this is only the case if the cb is at a corner node from us.
        else:
            # print('seeing if double swap is in order')
            neighbors = identify_26_neighbors(self.pos).tolist()
            if cbpos.tolist() in neighbors:
                self.__desitination_double_swap(cbpos, cbdest)


    def __destination_swap(self, cbpos, cbdest):
        '''
        Accepts the change bot position (cbpos) and destination (cbdest)
        Sets next pos IFF a destination swap is supposed to occur.
        '''

        #change bot info.
        change_bot_location = cbpos
        change_bot_destination = cbdest
        change_bot_box = get_box_from_node(change_bot_location, self.step_size)
        change_bot_inv = find_if_box_is_inverted(change_bot_box)

        #get our saddle.
        saddle_list = self.saddle.tolist()

        #2 cases: We were gonna SPAN and we were gonna NON-SPAN
        #case 1: we were gonna NON-SPAN
        if self.next_pos.tolist() in saddle_list:
            #this means we have to go out of our box into that of the change bot
            cb_saddle = change_bot_box * np.ones((9,3))
            if not change_bot_inv:
                cb_saddle = cb_saddle + self.regular_adder
            elif change_bot_inv:
                cb_saddle = cb_saddle + self.inverted_adder
            
            #find the position closest to us in the cb_saddle
            min_distance = np.inf
            next_pos = False
            for p in cb_saddle:
                dist = np.linalg.norm(self.pos - p)
                if dist < min_distance:
                    next_pos = np.copy(p)
                    min_distance = dist
            

            self.responding_to_change_bot = True
            self.next_pos = next_pos

            print('case 1. going from our box into change bot box.')

        #Case 2: we were gonna SPAN
        elif self.next_pos.tolist() not in saddle_list:
            #this means we have to go within our box.
            saddle_destination = np.copy(self.saddle[self.saddle_pointer+1])
            self.responding_to_change_bot = True
            self.next_pos = saddle_destination
            print('case 2. going from other box into my box.')
        print(str(self.id) +": I did a destination swap. Going: " + str(self.pos) + str(' to ' + str(self.next_pos)))
        return next_pos
    
    def __desitination_double_swap(self, cbpos, cbdest):
        '''
        Accepts the change bot position (cbpos) and destination (cbdest)
        Sets next pos IFF a destination double swap is supposed to occur.
        '''

        #change bot info.
        change_bot_location = cbpos
        change_bot_destination = cbdest
        box_of_cb_destination = get_box_from_node(change_bot_destination, self.step_size)
        change_bot_box = get_box_from_node(cbpos, self.step_size)
        change_bot_inv = find_if_box_is_inverted(change_bot_box)

        #make sure the change bot is in a different box than ours.
        if not np.all(change_bot_box == self.box):
            
            #make sure the change bot is going to take a spanning edge to a box that is not ours.
            if not np.all(change_bot_box == box_of_cb_destination) and not np.all(self.box == box_of_cb_destination):

                #make sure that my current next pos is in the cb box
                next_pos_box = get_box_from_node(self.next_pos, step_size=self.step_size)
                if np.all(next_pos_box == change_bot_box):

                    #this means we have to go within our box.
                    saddle_destination = np.copy(self.saddle[self.saddle_pointer+1])
                    print(str(id) + ' : THE SUPER SELECTIVE DOUBLE SWAP HAS OCCURRED! : ' + str(self.pos))
                    self.responding_to_change_bot = True
                    self.next_pos = saddle_destination

            #make sure the change bot is going to take a spanning edge to my box.
            elif not np.all(change_bot_box == box_of_cb_destination) and np.all(self.box == box_of_cb_destination):

                #make sure that my current next pos is not in our box
                next_pos_box = get_box_from_node(self.next_pos, step_size=self.step_size)
                if not np.all(next_pos_box == self.box):

                    #and make sure that my current next pos is not in the Change bot box.
                    if not np.all(next_pos_box == change_bot_box):

                        saddle_pos = np.copy(self.saddle[self.saddle_pointer+1])

                        #check to see if the cb is going to where our saddle destination/spot would be.
                        if np.all(saddle_pos == change_bot_destination):

                            #this means that we have to span into the cb box.

                            #this means we have to go out of our box into that of the change bot
                            cb_saddle = change_bot_box * np.ones((9,3))
                            if not change_bot_inv:
                                cb_saddle = cb_saddle + self.regular_adder
                            elif change_bot_inv:
                                cb_saddle = cb_saddle + self.inverted_adder
                            
                            #find the position closest to us in the cb_saddle
                            min_distance = np.inf
                            next_pos = False
                            for p in cb_saddle:
                                dist = np.linalg.norm(self.pos - p)
                                if dist < min_distance:
                                    next_pos = np.copy(p)
                                    min_distance = dist
                            

                            self.responding_to_change_bot = True
                            self.next_pos = next_pos

                            print(str(id) + ' : THE SUPER SELECTIVE DOUBLE SWAP HAS OCCURRED (type 2)! : ' + str(self.pos))


        return

    def enter_parent(self):

        #find parent box.
        if len(self.parent_boxes) == 1:
            parent_box = self.parent_boxes[-1]

        else:
            parent_box = self.parent_boxes[-2]
        parent_saddle = np.array(parent_box) * np.ones((9,3))

        #by definition, parent will be the OPPOSITE of our inv.
        if self.inv:
            parent_saddle = parent_saddle + self.regular_adder
        elif not self.inv:
            parent_saddle = parent_saddle + self.inverted_adder
        
        #find the position closest to us in the parent box
        min_distance = np.inf
        next_pos = False
        for p in parent_saddle:
            dist = np.linalg.norm(self.pos - p)
            if dist < min_distance:
                next_pos = np.copy(p)
                min_distance = dist
        
        self.next_pos = next_pos

        return next_pos

        
                
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
    


        

############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
        
class PrescribedBot():
    def __init__(self, id, pos, added_box, inv, step_size):
        self.id = id
        self.pos = pos
        self.added_box = added_box
        self.added_box_nodes = find_nodes_from_LL(added_box, step_size, inv).tolist()
        self.prescribed_order = []
        self.order_index = 0
        self.inv = inv
        self.step_size = step_size

        self.get_order()

    def get_order(self):

        #first, we need to enter the box, so find the closest neighbor
        neighbors = identify_potential_edges(self.pos)
        for n in neighbors:
            if n.tolist() in self.added_box_nodes:
                self.prescribed_order.append(n.tolist())
                point_of_entry = n.tolist()
                break

        #next, find where we are in the saddle
        index_of_entry = self.added_box_nodes.index(point_of_entry)

        #add everything AFTER the index of entry
        self.prescribed_order = self.prescribed_order + self.added_box_nodes[index_of_entry+1:]

        #add everything BEFORE the index of entry
        self.prescribed_order = self.prescribed_order + self.added_box_nodes[:index_of_entry]

        # print(self.id, self.pos, type(self.prescribed_order[0])) 

        return

    def run(self):

        next_pos = np.array(self.prescribed_order[self.order_index])
        return next_pos

    def step(self):
        end = False
        self.order_index += 1

        if self.order_index == len(self.prescribed_order):
            end = True
        
        return end
    
class FileOutBot():
    def __init__(self, id, pos, removed_box, poi, step_size):
        self.id = id
        self.pos = pos
        removed_box_is_inverted = find_if_box_is_inverted(removed_box)
        removed_box_array = find_nodes_from_LL(removed_box, step_size, removed_box_is_inverted)
        self.removed_box = removed_box_array
        self.removed_box_is_inverted = removed_box_is_inverted
        self.poi = poi
        self.prescribed_order = []
        self.order_index = 0
        self.step_size = step_size
        self.one_step_out = False
        self.saved_memory = []
        self.last_out = False

        self.get_order()

    def get_order(self):
        
        #find the saddle point closest to the poi (where robots will file out of the saddle)
        min_distance = np.inf
        exit_point_index = np.inf
        for i,p in enumerate(self.removed_box):
            d = np.linalg.norm(p - self.poi)
            if d < min_distance:
                min_distance = d
                exit_point = np.copy(p)
                exit_point_index = i        

        #Loop to find our place in the saddle
        for j,s in enumerate(self.removed_box):
            if np.all(s==self.pos): #find the saddle position that we are currently in (s == pos)
                saddle_index = j
                break

        try:
            #if the our current index is less than the exit point index:
            if saddle_index < exit_point_index:
                self.prescribed_order = self.removed_box[saddle_index+1:exit_point_index+1]
                self.prescribed_order = np.vstack((self.prescribed_order, self.poi))
                # print('case 1: ' + str(self.pos) + ', ' + str(self.prescribed_order.tolist()))
            elif saddle_index > exit_point_index:
                top_part = self.removed_box[saddle_index+1:]
                bottom_part = self.removed_box[:exit_point_index+1]
                self.prescribed_order = np.vstack((top_part,bottom_part))
                self.prescribed_order = np.vstack((self.prescribed_order, self.poi))
                # print('case 2: ' + str(self.pos) + ', ' + str(self.prescribed_order.tolist()))
            elif saddle_index == exit_point_index:
                self.prescribed_order = [np.copy(self.poi)]    
                self.one_step_out = True
                # print('case 3: ' + str(self.pos) + ', ' + str(self.prescribed_order.tolist()))
        except:
            print('ERROR IN FBOT FINDING PRESCRIBED ORDER')
            print(self.id)
            print(self.pos)
            print(self.poi)
            print(self.removed_box)

        # print(self.id, self.pos, type(self.prescribed_order[0]))      
            
        #identify if we are going to be the last robot out of the removed shape
        if len(self.prescribed_order) == 8:
            self.last_out = True

    def run(self):

        next_pos = self.prescribed_order[self.order_index]
        return next_pos

    def step(self):

        end = False

        self.order_index += 1

        if self.order_index == len(self.prescribed_order) or self.one_step_out:
            end = True
        
        return end, self.last_out  

############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################

###########
## FIND BOX FROM X,Y,Z COORDINATE
############
def get_box_from_node(given_node , step_size):
    '''
    node MUST BE AN ARRAY. RETURNS An ARRAY!
    uses modulus math to return lower left corner of cube
    assumes cube has side lengths of 1 and lower left corners are 2 sides away from one another
    '''
    node = np.copy(given_node)
    mod_of_node = np.mod(node,2*step_size)
    for i,item in enumerate(mod_of_node):
        if item == step_size:
            node[i] = node[i] - step_size

    box = np.copy(node)
    return box

###########
## Find if box is inverted
###########
def find_if_box_is_inverted(given_box):
    '''
    accepts a box lower left corrdinate as an array and then returns inverted or not based on modulus math
    '''
    box = np.copy(given_box)
    mod_box = np.copy(np.mod(box/2,2))
    total = np.sum(mod_box)
    if total % 2:
        inverted = True
    else:
        inverted = False
    return inverted

###########
## Identify the possible routes a robot can take away from a node in a cube
###########
def identify_potential_edges(pos):
    '''
    accepts a 3 part numpy array of a position in virtual space
    returns a numpy array of the possible nodes that a robot can travel to from the given pos
    '''

    #in general, a robot can travel in +/- x, +/- y, +/- z from a given node.
    #this enumerates those primitives
    edge_adder = np.array([[1,0,0],
                            [-1,0,0],
                            [0,1,0],
                            [0,-1,0],
                            [0,0,1],
                            [0,0,-1]])
        
        
    #initialize array of 6 locations with current location
    #i.e., we create an array containing 6 of our current [x,y,z] positions
    possible_edges = np.ones((6,3))
    possible_edges = pos * possible_edges

    #add the edge adder to get valid destinations from current pos
    possible_edges = possible_edges + edge_adder

    return possible_edges

###########
## TEST for vertical movement
##########
def test_for_vertical_movement(a1, a2):
    '''
    accepts 2 numpy arrays and determines if the result is vertical movement
    basically if difference in z in the two points is non-zero, its vertical
    '''
    diff = a2 - a1
    if diff[2] == 0:
        return False
    else:
        return True
    
###########
## Identify 26 neighbors (for determining if we are the NCP)
##########
def identify_26_neighbors(pos):

    adder = np.array([[1,0,0],
                        [1,1,0],
                        [0,1,0],
                        [-1,1,0],
                        [-1,0,0],
                        [-1,-1,0],
                        [0,-1,0],
                        [1,-1,0],
                        [0,0,1],
                        [1,0,1],
                        [1,1,1],
                        [0,1,1],
                        [-1,1,1],
                        [-1,0,1],
                        [-1,-1,1],
                        [0,-1,1],
                        [1,-1,1],
                        [0,0,-1],
                        [1,0,-1],
                        [1,1,-1],
                        [0,1,-1],
                        [-1,1,-1],
                        [-1,0,-1],
                        [-1,-1,-1],
                        [0,-1,-1],
                        [1,-1,-1]])
        
    #initialize array of 4 locations with current location
    possible_edges = np.ones((26,3))
    possible_edges = pos * possible_edges

    #add the edge adder to get valid destinations from current pos
    possible_edges = possible_edges + adder

    return possible_edges

###########
## FIND LIST OF NODES FROM BOX LL
##########
def find_nodes_from_LL(lower_left, step_size, inv):

    adder = np.array([[0, 0, 0],
                        [0, 0, 1],
                        [0, 1, 1],
                        [0, 1, 0],
                        [1, 1, 0],
                        [1, 1, 1],
                        [1, 0, 1],
                        [1, 0, 0]])
        
    inverted_adder = np.array([[0, 0, 0],
                                [0, 0, 1],
                                [1, 0, 1],
                                [1, 0, 0],
                                [1, 1, 0],
                                [1, 1, 1],
                                [0, 1, 1],
                                [0, 1, 0]])
    
    if inv:
        adder = step_size * inverted_adder
    elif not inv:
        adder = step_size * adder

    new = lower_left + adder

    return new

##########################################################################
### BITMAP MANAGEMENT
##########################################################################
#TODO REMOVE
def convert_to_bitmap(nodes):
    '''
    Accepts a list of nodes (e.g., [[x,y,z], [x2,y2,z2]])
    And returns a bitmap where the 1s are the occupied nodes and the zeros are empty.
    '''
    bitmap = np.zeros((8,16,16),dtype=int) #8 layers, each layer is 16x16. So its (z (sets), x (down rows), y (across columns))

    for node in nodes:
        x = int(node[0] + BITMAP_CENTER_OFFSET)
        y = int(node[1] + BITMAP_CENTER_OFFSET)
        z = int(node[2])
        bitmap[z,x,y] = 1

    # print(bitmap[0])

    return bitmap

#TODO REMOVE
def convert_bitmap_to_virtual(bitmap):
    '''
    Accepts a numpy bitmap where the 1s are occupied nodes nad the zeros are empty
    Returns a list of nodes (e.g., [[x,y,z], [x2,y2,z2]])
    '''
    shape = bitmap.shape
    nodes = []
    for z in range(0,shape[0]):
        for y in range(0,shape[2]):
            for x in range(0,shape[1]):
                if bitmap[z,x,y] == 1:
                    nodes.append([x-BITMAP_CENTER_OFFSET,y-BITMAP_CENTER_OFFSET,z])
    
    return nodes

#TODO REMOVE           
def convert_bitmap_to_struct(bitmap):
    '''
    Accepts a numpy bitmap of a 3D shape where the 1s are occupied nodes nad the zeros are empty
    Returns the packed message using struct.pack where each row of the bitmap is an unsigned short (H)
    '''

    integers = [] #list to hold each row converted into an integer, so we can pack them all as unsigned shorts (H)

    #Get the overall size of the bitmap
    Z, X, Y = bitmap.shape

    #Loop through each row in the bitmap:
    for z in range(0,Z):
        for x in range(0,X):
            row = bitmap[z,x,:].tolist()

            #convert the row to a binary string ()
            row_as_binary_string= ''.join(map(str, row))
            row_as_integer = int(row_as_binary_string,2) #the ',2' tells it that its base 2
            integers.append(row_as_integer)
    
    #now we have a list of 128 integers (8 levels in z, 16 rows in each level)
    #we will pack that into a struct of 128 unisgned shorts (2 bytes for each short, so thats 256 bytes)
    msg = struct.pack('128H',*integers)

    return msg

#TODO REMOVE
def convert_struct_to_bitmap(msg):
    '''
    Accepts a struct.packed message representing a bitmap
    Converts it to a numpy bitmap where the 1s are occupied nodes nad the zeros are empty
    '''

    #empty bitmap which we will populate
    bitmap = np.zeros((8,16,16),dtype=int) #8 layers, each layer is 16x16. So its (z (sets), x (down rows), y (across columns))

    #Unpack the struct into a tuple of integers
    integers = struct.unpack('128H',msg)

    Z = 0
    X = 0

    #loop through each integer. Convert it to a numpy array (1D)
    for i,value in enumerate(integers):

        value_as_binary = format(value,'016b') #the zero says we want leading zeros
        value_as_array = np.array(list(value_as_binary),dtype=int)
        
        #write the 1D array into the corresponding row in the empty bitmap
        bitmap[Z,X,:] = value_as_array

        #increment the pointers (this is consistent with the order in which the original msg was structured.)
        X = (i%16) + 1

        if X == 16: #at this point, we've reached the last row in the Z layer and we must go up a layer.
            Z += 1
            X = 0

    return bitmap



def create_bitmap(list_of_nodes):
    '''
    Accepts a list of nodes (numpy array of ints) of positions  in virtual space.
    Turns it into a bit map.

    Assumes each value is between -2 and 5, inclusive. (i.e., 4 cubes x 4 cubes x 4 cubes max.)

    returns a packed struct that is ready to be transmitted in a message.

    '''

    bitmap = np.zeros((8,8,8), dtype=int)

    #populate teh bitmap by looping through each node in the list.
    for node in list_of_nodes:

        #get the x,y,z value of the node, shifted over by 2 to be between 0 and 7
        x,y,z = node
        x += 2
        y += 2
        z += 2

        x = int(x)
        y = int(y)
        z = int(z)

        #set the value at that x,y,z to be 1
        bitmap[z,y,x] = 1

    #now that the bitmap is populated, pack it in bytes.
    first_byte = True
    for z_layer in bitmap:
        for y_row in z_layer:
            byte = 0x00
            for i,value in enumerate(y_row):
                if value == 1:
                    byte = byte | (1 << (7-i))

            # print_8_bit_binary(byte)

            if first_byte:
                first_byte = False
                byte_object = struct.pack("B", byte)
            else:
                byte_object += struct.pack("B", byte)

    return byte_object

def unpack_bitmap(byte_object):
    '''
    Does the opposite of "create_bitmap"
    It takes a bytes object received from a struct and it unpacks it into a list of nodes (for shape and visited msgs)

    '''
    bitmap = np.zeros((8,8,8), dtype=int)
    byte_counter = 0
    node_list = []

    for z,z_layer in enumerate(bitmap):
        for y,y_row in enumerate(bitmap):
            # print("%02X" % byte_object[byte_counter], byte_counter)
            data = byte_object[byte_counter]
            byte_counter += 1
            # print(bin(data), int(data), byte_counter)
            for x,value in enumerate(y_row):
                if(data & (1 << (7-x))):
                    bitmap[z,y,x] = 1
                    node_list.append([x-2,y-2,z-2])
    return node_list

def compact_format(virtual_pos):
    ''''
    Takes a numpy_array of a virtaul position (i.e., 2,1,1) and convertes it into two bytes in "compact format".

    Compact format:
    0 1 2 3  4 5 6 7      8 9 A B  C D E F

    Bit 0: unused
    Bit 1: x sign
    Bit 2: y sign
    Bit 3: z sign

    Bits 4-7 x magnitude (as unsigned integer so 1011 = 11) 
    Bits 8-B y magnitude (as unsigned integer)
    Bits C-F z magnitude (as unsigned integer)

    '''

    # print('Converting: ' + str(virtual_pos))

    #get the x, y, and z coordinates separately
    x = int(virtual_pos[0])
    y = int(virtual_pos[1])
    z = int(virtual_pos[2])

    #set up a 16 bit variable for the compact format
    cf = 0b0000000000000000

    #set the sign bits for x, y, and z:
    #only set if negative. if positive or zero, sign bits stay zero
    if x < 0: 
        cf = cf | (0x0001 << 14)
    if y < 0: 
        cf = cf | (0x0001 << 13)
    if z < 0: 
        cf = cf | (0x0001 << 12)

    #set the magnitude for x, y, and z
    xmag = abs(x) & 0xf
    cf = cf | (xmag << 8)

    ymag = abs(y) & 0xf
    cf = cf | (ymag << 4)

    zmag = abs(z) & 0xf
    cf = cf | zmag

    # print("converted: ")
    # print_16_bit_binary(cf)
    hi = (cf >> 8) & 0xff
    lo = cf & 0xff
    cf = struct.pack('bb', hi,lo)
    # print(cf)
    return cf
    
def pos_from_compact_format(cf):
    '''
    Does the reverse of "compact format"

    Takes a two byte representation and converts it to a numpy array of integers for x, y, z virtual position
    '''

    # print('Converting: ')
    # print(cf)
    hi,lo = struct.unpack('bb',cf)
    cf = (hi << 8) | lo
    # print_16_bit_binary(cf)

    pos = np.array([0,0,0], dtype=int)

    #Get the magnitude of x, y, and z
    xmag = int((cf >> 8) & 0xf)
    ymag = int((cf >> 4) & 0xf)
    zmag = int(cf & 0xf)

    pos[0] = xmag
    pos[1] = ymag
    pos[2] = zmag

    #get the signs of x, y, and z
    if ((cf >> 14) & 0x1):
        pos[0] = -1*pos[0]
    if ((cf >> 13) & 0x1):
        pos[1] = -1*pos[1]
    if ((cf >> 12) & 0x1):
        pos[2] = -1*pos[2]


    
    # print(pos)
    return pos
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

MAX_VELO = 0.1
DT = 0.2

def usr(flyer):

    disarm_delay = 5

    START = time.time()

    def now():
        return time.time() - START


    flyer.select_controller('lqr')

    flyer.log(['max velo: ', str(MAX_VELO), 'disarm delay: ', str(disarm_delay)])


    
    #get the first position
    while True:
        state = flyer.state()
        if len(state) > 1:
            first_pos = np.copy(state)
            break
    
    
    #set the first waypoint
    setpoint = np.copy(first_pos)
    setpoint[2] = setpoint[2] + 0.4 #meters
    setpoint[3:] = 0
    flyer.waypoint(first_pos)

    

    temp_point = np.copy(setpoint)
    temp_point[2] = first_pos[2]+0.030
    setpoint_options = {'up':setpoint[0:3], 'down':temp_point[0:3]}
    current_setpoint_key = 'up'

    time_of_last_switch = now()

    wm = WaypointManager(MAX_VELO, first_pos[0:3], time_step=DT)
    flyer_waypoint = np.zeros(12)

    #set the new setpoint in the waypoint manager per the new key
    wm.set_NEW_destination(setpoint_options[current_setpoint_key])

    primed_for_switch = False
    needs_arming = False
    #arm the flyer
    flyer.arm()
    flyer.run_controller()
    perform_arm_check = True
    fail_counter = 0
    fail_limit = 50

    current_pos = np.zeros(3)
    last_pos = first_pos[0:3]
    current_velo = np.zeros(3)
    current_state = np.zeros(12)
    landed = False

    step_point = np.copy(setpoint)
    step_point[2] = first_pos[2] - 0.50

    flyer.log(['time', 'landing duration', 'success', 'xy error', 'velo', 'yaw'])

    TIME_OF_NEXT_SWITCH = 0

    while True:

        # print('looping')

        
        
        flyer.delay()

        current_time = now()

        #log our state
        state = flyer.state()
        if len(state) > 1:
            current_pos = state[0:3]
            current_velo = state[3:6]

            if perform_arm_check:
                if int(100*current_velo[2]) > 0 and current_pos[2] > last_pos[2] + 0.005:
                    perform_arm_check = False
                    fail_counter = 0
                else:
                    fail_counter += 1
                    if fail_counter > fail_limit:
                        flyer.disarm()
                        print('Flyer failed to arm when commanded.')
                        print(int(100*current_velo[2]), current_pos[2], last_pos[2])
                        flyer.log(['Flyer failed to arm when commanded.'])
                        break


        #Indicates it is time to change setpoints
        if current_time > TIME_OF_NEXT_SWITCH and primed_for_switch:
            print(current_time)
            primed_for_switch = False
            
            #change the setpoint keys
            if current_setpoint_key == 'up':
                current_setpoint_key = 'down'
                #set the new setpoint in the waypoint manager per the new key
                wm.set_NEW_destination(setpoint_options[current_setpoint_key])
            elif current_setpoint_key == 'down':
                current_setpoint_key = 'up'
                needs_arming = True
                landed = False
                #set the new setpoint in the waypoint manager per the new key
                wm.set_NEW_destination(setpoint_options[current_setpoint_key],current_dest=current_pos)
                

            

            #update the time of the last setpoint change so we don't immediately bounce back next loop through.
            time_of_last_switch = current_time

            if needs_arming:
                flyer.arm()
                flyer.run_controller()
                needs_arming = False

        #get a new waypoint from the waypoint manager
        flyer_waypoint[0:3] = wm.get_waypoint()

        #set it on the robot
        flyer.waypoint(flyer_waypoint)

        if wm.is_destination_reached() and current_setpoint_key == 'down' and not landed:
            error_xy = np.linalg.norm(current_pos[0:2] - wm.get_CURRENT_destination()[0:2])
            velo = np.linalg.norm(current_velo)
            if error_xy < 0.01 and velo < 0.1:

                #everything checks out, so we are going to try to land.
                #set the setpoint far below our landing pos (so we cut the motors without disarming)
                flyer.waypoint(step_point)

                #wait a half a second (getting pos if we have one)
                wait_start = now()
                while now() - wait_start < 0.5:
                    state = flyer.state()
                    if len(state) > 1:
                        current_pos = state[0:3]
                        current_velo = state[3:6]
                        current_state = np.copy(state)

                error_xy = np.linalg.norm(current_pos[0:2] - wm.get_CURRENT_destination()[0:2])
                velo = np.linalg.norm(current_velo)

                print_success = 0
                yaw = current_state[8]
                if abs(yaw) > 0.1 or error_xy > 0.01: #~5deg
                    print('failed landing. Will try again.')
                else:
                    print('successful landing!')
                    flyer.disarm()
                    landed = True
                    print('landed')
                    print(error_xy, velo)
                    print(current_state)
                    print('-fin-')
                    print_success = 1

                #'time', 'landing duration', 'success', 'xy error', 'velo', 'yaw'])
                flyer.log([np.round(now(),3), np.round(now()-time_of_last_switch,3), print_success, np.round(error_xy,6), np.round(velo,6), np.round(yaw,3)])

                primed_for_switch = True
                TIME_OF_NEXT_SWITCH = now() + disarm_delay # this is the time spend 'down'/unarmed.
            else:
                print(error_xy, velo)
        elif wm.is_destination_reached() and not primed_for_switch and current_setpoint_key == 'up':
            primed_for_switch = True
            TIME_OF_NEXT_SWITCH = now() + 5 #will 'hover' for 1s before switching to go back down.

        
        last_pos = np.copy(current_pos)


        

            

            




class WaypointManager():
    def __init__(self, max_velo, destination, time_step = 0.2):
        self.max_velo = max_velo
        self.time_step = time_step
        self.destination = np.copy(destination)
        self.waypoint = np.copy(destination)
        self.destination_reached = False

    def set_NEW_destination(self, next_destination, current_dest = np.zeros(1)):
        '''
        destinations should be np.array(x,y,z)
        Find a unit vector and distance between our current destination and the next destination.
        Set new destination to current destination

        optionally, a user can set a new "current_dest" which is the starting point from which the vectors to the new destination are set.
        ''' 
        # print('setting new destination')
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

        if not self.destination_reached:

            if self.waypoint[2] < 0:
                velo = self.max_velo * 1
            else:
                velo = self.max_velo

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


    
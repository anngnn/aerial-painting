
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
    print("HOVER CODE STARTED")

    START = time.time()

    flyer.mode(1000)

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
    # setpoint[0] = setpoint[0] - 0.2 #meters
    # setpoint[1] = setpoint[1] - 0.2 #meters
    setpoint[2] = setpoint[2] + 0.8 #meters
    setpoint[3:] = 0
    flyer.waypoint(setpoint)

    # flyer.delay(5)

    #arm the flyer
    flyer.arm()
    # flyer.run_controller()

    while True:

        # print('looping')

        
        
        flyer.delay()


        #Receive any messages
        msgs = flyer.recv_msg()

        #Process the messages
        if msgs:
            pass

        #Pack any messages for sending    
        outbound, outbound_msg = pack_message(flyer.id) #call a pack message function here


        #Send any messages
        if outbound:
            flyer.send_msg(outbound_msg)
            pass

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


def pack_message(id):
    '''
    Populates an outgoing message in the standard format
    returns True, msg if successful, False, False if nothing to send.
    '''

    #first part is id, msg type, broadcast 
    first_part = struct.pack('3B', id, 99, 1)

    #Next is the robot position in compact form
    compact_pos = struct.pack('bb', 0,0)

    #next is the robot next position in compact format
    compact_next_pos = struct.pack('bb', 0,0)

    #next is the position of the recipient/(was also POI in lw sim)
    compact_recipient = struct.pack('bb', 0,0)

    #next is the new poi (if I decide to use it)
    compact_poi = struct.pack('bb', 0,0)

    #next is the change ID (now a signed int, not a float anymore)
    change_id = struct.pack('b', 0)

    #next is the drone's knowledge of the latest chagned box
    compact_change_box = struct.pack('bb', 0,0)

    #next part is the visited nodes
    packed_visited = create_bitmap([])
    
    #last is the shape
    packed_shape = create_bitmap([])


    msg = first_part + compact_pos + compact_next_pos + compact_recipient + compact_poi + change_id + compact_change_box + packed_visited + packed_shape

    return True, msg

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
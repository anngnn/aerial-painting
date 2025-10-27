'''
This code was developed by Andrew Curtis in the year 2024.
It was developed at Northwestern University, Evanston, IL, USA.

This file is home to the classes that other processes use to manage shared data.
The intent is that all managing of shared data (building it, getting it, setting it) happens in the classes here.

The processes kicked off by the bootloader use these classes by COMPOSITION (not inheritance)
'''

#import 3rd party libraries
#NONE

#import os files (files NU created)
#NONE

#import native libraries
import multiprocessing as mp
import numpy as np
import struct


class SharedData():
    '''
    Class to fabricate the shared data. ALL shared data is constructed in this class. If it's not in this class, it's not shared.
    '''

    def __init__(self, id):
        '''
        This function creates MultiProcessing objects for ALL of the data that is going to be shared between processes.

        DEVELOPER NOTE: If you want to add, remove, or change the data that is shared between processes,
        then you have to create a MP object here AND add it to the list.
        '''

        #Create shared memory variables for STATE
        #State is x, y, z, xdot, ydot, zdot, roll, pitch, yaw, rolldot, pitchdot, yawdot
        self.state = mp.Array('d', 12, lock=False) #d for double float, 12 entities
        self.timing_info = mp.Array('d', 4, lock=False) #d for double float, 4 entities
        #Also need an indicator for when the data is fresh. 
        self.state_is_fresh = mp.Value('I',1,lock=False)
        #Need a lock. Can't use the built-in locks (above) because we want to call it non-blocking
        self.state_lock = mp.Lock()


        #Create shared memory variable for SAFETY
        #This is an int with coded values from 0 to 65535
        self.safety = mp.Value('I',1,lock=True) #I for unsigned int, 1 entity
        self.safety.value = 0 #initialize to zero

        #Create shared memory variable for FLIGHT CONTROLLER COMMANDS
        #Commands are AETR (roll, pitch, throttle, yaw), ARM, and MODE
        self.commands = mp.Array('I',6,lock=False) #I for unsigned int, 6 entities
        #Also need an indicator for when the data is fresh.
        self.commands_are_fresh = mp.Value('I',1,lock=False)
        #Need a lock. Cna't use built-in locks (for variables above) because we may need non-blocking calls
        self.commands_lock = mp.Lock()
        i_have_lock = self.commands_lock.acquire(block=True)
        #Initialize commands and fresh value
        if i_have_lock:
            self.commands[0] = 1500 #roll (A)
            self.commands[1] = 1500 #pitch (E)
            self.commands[2] = 900 #Throttle (T)
            self.commands[3] = 1500 #yaw (R)
            self.commands[4] = 1000 #arm
            self.commands[5] = 1000 #mode
            self.commands_are_fresh.value = 1
            self.commands_lock.release()

        #Create shared memory variables for DESIRED STATE (SETPOINT)
        #State is x, y, z, xdot, ydot, zdot, roll, pitch, yaw, rolldot, pitchdot, yawdot
        self.desired_state = mp.Array('d', 12, lock=False) #d for double float, 12 entities
        #Need a lock. Can't use the built-in locks (above) because we want to call it non-blocking.
        self.desired_state_lock = mp.Lock()
        #DEVELOPERS NOTE: There is no need to know if setpoint is fresh. The controllers don't care.
        #That is up to the user to keep track of.

        #Create shared memory variables for LANDER/CHARGING STATION
        #Lander position is X,Y,Z,YAW
        self.lander_position = mp.Array('d',4,lock=False)
        #Need a lock. Cna't use built-in locks (for variables above) because we may need non-blocking calls
        self.lander_lock = mp.Lock()
        i_have_lock = self.lander_lock.acquire(block=True)
        #Initialize the lander data to zeros.
        if i_have_lock:
            self.lander_position[0] = 0.0
            self.lander_position[1] = 0.0
            self.lander_position[2] = 0.0
            self.lander_position[3] = 0.0
            self.lander_lock.release()

        #Create shared memory variables for CONTROLLER TYPE and whether it is ACTIVE/ARMED
        self.type_of_controller = mp.Value('I',1,lock=True) #I for unsigned int, 1 entry
        self.type_of_controller.value = 0 #initialize to zero
        self.controller_active = mp.Value('I',1,lock=True) #I for unsigned int, 1 entry
        self.controller_active.value = 0 #initialize to zero (not active)
        self.arm_status = mp.Value('I',1,lock=True) #status displaying whether armed (1) or not
        self.arm_status.value = 0 #initialize to zero (not armed)

        #Create shared memory variables for ROBOT-TO-ROBOT MESSAGING and NETWORKING
        self.MAX_MSG_NUM = 10 #maximum number of messages that can be stored in the read buffer
        self.MSG_LEN = 1016 #The raw message length (how long each single message can be in bytes)
        header = 3 + 1 + 4 # 3 bytes for 'p2p' header, 1 byte for robot id, 4 bytes for msg length.
        self.EXPECTED_LEN = self.MSG_LEN + header #the message + X bytes for the header
        #-->So, the read buffer is an array of chars (i.e. bytes) that is (1024)*100 empty bytes
                #should have a length of: 102400
        self.read_buffer = mp.Array('c',\
                                    b'\0' + (b'\0' * self.MAX_MSG_NUM * self.MSG_LEN) + chr(id).encode('utf-8'),\
                                    lock=False)
        self.num_msgs = mp.Value('I',1,lock=False) #number of messages in the read buffer
        self.num_msgs.value = 0 #initialize to zero
        self.read_buffer_lock = mp.Lock()
        #-->So, the send buffer is an array of chars (i.e., bytes) that is intialized to 1+1024 empty bytes.
                #should have length of: 1025
        self.send_buffer = mp.Array('c',\
                                    b'\0' * (self.EXPECTED_LEN),\
                                    lock=False)
        self.send_msg_length = mp.Value('I', 1) #the length of the message we are going to send out.
        self.send_msg_length.value = 0
        #DEVELOPERS NOTE: Given that there is a time delay as to how often messages can be sent, I don't think we
        #need to have locks on the send buffer (its written, then immediately read while the writer can't write again for Xms)

        #Create places to store the process ids (pids) of the read and send processes
        self.read_buffer_pid = mp.Value('I',1,lock=True) #I for unsigned int, 1 entry
        self.send_buffer_pid = mp.Value('I',1,lock=True) #I for unsigned int, 1 entry
        

        #Create a flag that the Flight Controller handler can set when it connects with a board
        #so the bootloader knows it is active.
        self.board_connected = mp.Value('I',1,lock=True)
        self.board_connected.value = 0 #initialize to zero

        #Create a value for the battery voltage (we'll monitor this and make sure its above a threshold)
        self.battery_voltage = mp.Value('d',1,lock=True) #d for double float, 1 entry
        self.battery_voltage.value = -1.0 #initialize to less-than zero
        self.battery_power = mp.Value('d',1,lock=True) #d for double float, 1 entry
        self.battery_power.value = -1.0 #initialize to less-than zero
        self.low_voltage_detected = mp.Value('I',1,lock=True) #I for unsigned int, 1 entry
        self.low_voltage_detected.value = 0 #intialize to zero


        #Store the id (may need it later when packing messages)
        self.id = id

        #Store the time at which a user code was started (i.e., time zero t=0)
        #and the most recent optitrack time difference
        self.time_zero = mp.Value('d',1,lock=True) #d for double float, 1 entry
        self.time_zero.value = 0
        self.optitime_diff = mp.Value('d',1,lock=True) #d for double float, 1 entry
        self.optitime_diff.value = 0

        self.global_time = mp.Value('d',1,lock=True) #d for double float, 1 entry
        self.global_time.value = 0

        #Create a value to flag if the user code is running or not
        self.user_code_running = mp.Value('I',1,lock=True) #I for unsigned int, 1 entry, w. lock
        self.user_code_running.value = 0 #initialize to 0 = not running

        #This is the time zero for when the robot was turned on.
        self.robot_start_time = mp.Value('d',1,lock=True) #d for double float, 1 entry with lock
        self.robot_start_time.value = 0
        

        










class SharedDataManager():
    '''
    Class to MANAGE the shared data.
    Process managers (the code that runs in each process kicked off by bootloader) ARE COMPOSED OF (i.e., "have")
    a SharedDataManager object which handles the getting, setting, and lock management of the shared data.

    This is basically a collection of getters and setters for all the data values defined in SharedData()
    DEVELOPERS NOTE: If you add new shared data, best practice is to create getters and setters here and then use
    the data_manager objects to access and write the shared data instead of accessing the shared data directly
    This way there's better lock management and everything is consistent.

    '''

    def __init__(self, shared_data: SharedData) -> None:
        '''
        Object is initialized by each process manager with the data that has been shared across the processes
        It's stored by the process manager for easy access
        '''
        self.shared_data = shared_data #"shared_data" MUST be a SharedData type

        #Dictionary to map controller type codes to human-readable controller type names
        self._controller_types = {0: 'none', 1:'pid', 2: 'lqr'}
        self._reverse_controller_types = {'none':0, 'pid':1, 'lqr':2} #This is a poor man's bidict. 

        self.accel_filter_alpha = 0.01 #Alpha*NEW + (1-Alpha)OLD

    def get_state(self, clear_fresh_flag = False, blocking = False, get_timing_info = False):
        '''
        If state_lock is available, copy the state locally and return it. Otherwise, return FALSE
        Will wait here for the lock to be available if blocking == True
        ONLY sets the data to *NOT* fresh IFF clear_fresh_flag == True
        (This is done so that only ONE process can mark the data as used (i.e., the controller class/process))

        RETURNS np.array(x, y, z, xdot, ydot, zdot, roll, pitch, yaw, rolldot, pitchdot, yawdot)
        RETURNS list of timing information IFF get_timing_info is True
        '''
        state = np.zeros(12)
        timing_info = []
        i_have_lock = self.shared_data.state_lock.acquire(block = blocking)
        if i_have_lock:
            #State is x, y, z, xdot, ydot, zdot, roll, pitch, yaw, rolldot, pitchdot, yawdot
            state = np.copy(self.shared_data.state)
            
            #set state as NOT fresh if flagged to do so.
            if clear_fresh_flag:
                self.shared_data.state_is_fresh.value = 0

            if get_timing_info:
                timing_info = np.copy(self.shared_data.timing_info).tolist()
            
            self.shared_data.state_lock.release() #DEVELOPERS NOTE: It is HIGHLY important that the lock is released.

            if get_timing_info:
                return state, timing_info
            else:
                return state
        else:
            return np.zeros(1)
        
    def is_state_fresh(self):
        '''
        checks to see if the state data is fresh (so user can only pull fresh/new data)
        DEVELOPERS NOTE: I don't care about locking here because ONLY the localization process is writing "fresh"
        and ONLY the controllers will set it to not fresh. More importantly, only the controller cares if the data
        is fresh, everything else can pull not-fresh data (for logging, etc.)
        '''
        if self.shared_data.state_is_fresh.value == 1:
            return True
        else:
            return False

        
    def set_state(self, state, blocking = False, timing_info = []):
        '''
        Write data to the shared data state (if lock is available. Otherwise, return FALSE)
        Will wait here for the lock to be available if blocking == True
        Also, set the shared data fresh flag to 1 so we know if the data is fresh
        timing_info = list of timing and message recipt information associated with the current state

        Assumes 'state' is a 12-part array in the form:
        x, y, z, xdot, ydot, zdot, roll, pitch, yaw, rolldot, pitchdot, yawdot
        '''
        i_have_lock = self.shared_data.state_lock.acquire(block = blocking)
        if i_have_lock:
            
            #write the state
            for i,value in enumerate(self.shared_data.state):
                self.shared_data.state[i] = state[i]

            #write the corresponding timing data
            if timing_info != []:
                for i, value in enumerate(self.shared_data.timing_info):
                    self.shared_data.timing_info[i] = timing_info[i]

            
            #mark data as fresh
            self.shared_data.state_is_fresh.value = 1
            
            #release lock
            self.shared_data.state_lock.release() #DEVELOPERS NOTE: It is HIGHLY important that the lock is released.
            return True
        else:
            return False

        
    def get_safety(self):
        '''
        Identify the current safety code status and return it.

        DEVELOPERS NOTE:
        Since safety is a simple value (with an automatic lock) and we are only reading it (not incrementing it)
        I actually don't need to explicity acquire a lock here. the MultiProcessing library does this automatically 
        for simple reads and writes.
        details here: https://stackoverflow.com/questions/73403370/do-i-need-lock-on-process-shared-variable-when-reassigning-not-incrementing
        If you wanted to be explicit about the lock, something like this would work:
        #The with automatically releases the lock after its indented code is complete
            with self.shared_data.safety.get_lock():
                val = self.shared_data.safety.value
            return val
        
        '''
        return self.shared_data.safety.value
    
    def set_safety(self, safety_value, override = False):
        '''
        Set the current safety value to the provided value IFF that value is less than the current safety value
        ASSUMES safety_value is an integer.
        Safety codes:
        0 - normal operation. Take no action. do nothing. life is good.
        1 - immediate board reboot. Often triggered by the localization system timeout, bounding box/roll over error, or ESTOP
        2 - simple shutdown. The code has come to its designed completion, and the board should be reset.
        3 - immediate land (something went wrong, so we're just going to lower the drone down safetly)
            This is triggered when there's an error in the user code (i.e., a bug) 
        4 - immediate land due to low battery voltage.
        5 - drone is attempting to connect to the flight controller board. Other processes may be stalled.
        6 - fetching logs
        7 - clearing logs
            ...
        We LOCK so we can make sure we don't override a higher priority safety error.
        Returns TRUE if new safety code is written. False otherwise
        busy_override allows us to override the safety value IFF the value is 6 or 7.
        '''
        ret = False
        with self.shared_data.safety.get_lock():
            old_val = self.shared_data.safety.value
            #only write the new safety error code IF the current code is 0 OR the current code is > the received code
            if self.shared_data.safety.value == 0:
                self.shared_data.safety.value = safety_value
                ret = True
            elif self.shared_data.safety.value > safety_value:
                #in this case, we have a safey value to set that is LESS than the current one. This could mean four things:
                # 1.  a higher priority error (i.e., 1 < 2) in which case we SHOULD overwrite the code
                # 2.  a process is trying to set safety to zero even though another process threw an error in which case we SHOULD NOT overwrite the error code
                        #DEVELOPERS NOTE: UNLESS THERE IS AN OVERRIDE FROM BOOTLOADER DURING RESET WHEN WE WANT TO ZERO THE ERROR ANYWAY.
                # 3.  the error code is a 5 and someone is trying to reset it back to zero (i.e., initalization is complete) in which case we SHOULD overwrite
                # 4.  the drone is sending or removing logs. the FC thinks all is good (wants to set it to 0) but logs aren't done being processed yet.
                        #DEVELOPERS NOTE: This makes sure the FC doesn't set it to 0 while the logs are still being processed.
                if (old_val in [6,7] and override) and safety_value == 0: #case 4 above
                    self.shared_data.safety.value = safety_value
                    ret = True
                if (old_val in [5,8] or override) and safety_value == 0: #case 3 above
                    self.shared_data.safety.value = safety_value
                    ret = True
                elif safety_value != 0:
                    self.shared_data.safety.value = safety_value #case 1 above
                    ret = True
                else:#Case 2 above
                    pass

            if ret and old_val != safety_value:
                print("New safety value set to: " + str(safety_value) + ', old val: ' + str(old_val))
        return ret
    
    def get_commands(self, clear_fresh_flag = False, blocking = False):
        '''
        If commands_lock is available, copy the commands locally and return it. Otherwise, return FALSE
        Will wait here for the lock to be available if blocking == True
        ONLY sets the data to *NOT* fresh IFF clear_fresh_flag == True
        (This is done so that only ONE process can mark the data as used (i.e., the controller class/process))
        RETURNS commands, success T/F
        '''
        commands = np.zeros(6)
        i_have_lock = self.shared_data.commands_lock.acquire(block = blocking)
        if i_have_lock:
            commands = np.copy(self.shared_data.commands)
            
            #set commands as NOT fresh if flagged to do so.
            if clear_fresh_flag:
                self.shared_data.commands_are_fresh.value = 0
            
            self.shared_data.commands_lock.release() #DEVELOPERS NOTE: It is HIGHLY important that the lock is released.
            return commands, True
        else:
            return False, False
        
    def are_commands_fresh(self):
        '''
        checks to see if the commands are fresh (so user can only pull fresh/new data)
        DEVELOPERS NOTE: I don't care about locking here because ONLY the controller process is writing "fresh"
        and ONLY the FC interface will set it to not fresh. More importantly, only the FC interface cares if the data
        is fresh, everything else can pull not-fresh data (for logging, etc.)
        '''
        if self.shared_data.commands_are_fresh == 1:
            return True
        else:
            return False

        
    def set_AETR_commands(self, cmds, blocking = False):
        '''
        Write AETR data to the shared data commands (if lock is available. Otherwise, return FALSE)
        Will wait here for the lock to be available if blocking == True
        Also, set the shared data fresh flag to 1 so we know if the data is fresh

        Assumes 'cmds' is a 4-part array in the form:
        AETR (roll, pitch, throttle, yaw)
        '''
        i_have_lock = self.shared_data.commands_lock.acquire(block = blocking)
        if i_have_lock:
            
            #write the commands
            for i,value in enumerate(self.shared_data.commands[0:4]):
                self.shared_data.commands[i] = cmds[i]
            
            #mark data as fresh
            self.shared_data.commands_are_fresh.value = 1
            
            #release lock
            self.shared_data.commands_lock.release() #DEVELOPERS NOTE: It is HIGHLY important that the lock is released.
            return True
        else:
            return False
        
    def set_ARM(self, arm, blocking = False):
        '''
        Write ARM data to the shared data commands (if lock is available. Otherwise, return FALSE)
        Will wait here for the lock to be available if blocking == True
        Also, set the shared data fresh flag to 1 so we know if the data is fresh

        Assumes arm is a 0 = DISARM, 1 = ARM:
        '''
        if arm == 1:
            arm_val = 1800
            self.shared_data.arm_status.value = 1
        else:
            arm_val = 1000
            self.shared_data.arm_status.value = 0

        i_have_lock = self.shared_data.commands_lock.acquire(block = blocking)
        if i_have_lock:
            
            #write arm
            self.shared_data.commands[4] = arm_val
                
            #mark data as fresh
            self.shared_data.commands_are_fresh.value = 1
            
            #release lock
            self.shared_data.commands_lock.release() #DEVELOPERS NOTE: It is HIGHLY important that the lock is released.
            return True
        else:
            return False

    def get_ARM(self):
        return self.shared_data.arm_status.value

    def set_MODE(self, mode, blocking = False):
        '''
        Write MODE data to the shared data commands (if lock is available. Otherwise, return FALSE)
        Will wait here for the lock to be available if blocking == True
        Also, set the shared data fresh flag to 1 so we know if the data is fresh

        Assumes mode is:
        #aux 2 modes:
        # 1500 == horizon
        # 1000 == angle
        # 1800 == RATE (default, no self leveling.)
        '''
        if mode in [1000, 1500, 1800]:
            i_have_lock = self.shared_data.commands_lock.acquire(block = blocking)
            if i_have_lock:
                
                #write arm
                self.shared_data.commands[5] = mode
                    
                #mark data as fresh
                self.shared_data.commands_are_fresh.value = 1
                
                #release lock
                self.shared_data.commands_lock.release() #DEVELOPERS NOTE: It is HIGHLY important that the lock is released.
                return True
            else:
                return False
        else:
            return False
        
    def get_MODE(self, blocking = False):
        '''
        Will read the mode value and return it.
        Returns None if unsuccessful.
        '''
        i_have_lock = self.shared_data.commands_lock.acquire(block = blocking)
        if i_have_lock:

            mode = self.shared_data.commands[5]

            #release lock
            self.shared_data.commands_lock.release() #DEVELOPERS NOTE: It is HIGHLY important that the lock is released.
            return mode
        else:
            return None




        
    def get_desired_state(self, blocking=False):
        '''
        If desired_state_lock is available, copy the desired state locally and return it. Otherwise, return FALSE
        Will wait here for the lock to be available if blocking == True       
        RETURNS np.array(x, y, z, xdot, ydot, zdot, roll, pitch, yaw, rolldot, pitchdot, yawdot), TRUE/FALSE
        '''
        desired_state = np.zeros(12)
        i_have_lock = self.shared_data.desired_state_lock.acquire(block = blocking)
        if i_have_lock:
            #State is x, y, z, xdot, ydot, zdot, roll, pitch, yaw, rolldot, pitchdot, yawdot
            desired_state = np.copy(self.shared_data.desired_state)
            
            self.shared_data.desired_state_lock.release() #DEVELOPERS NOTE: It is HIGHLY important that the lock is released.
            return desired_state, True
        else:
            return False, False
        
    def set_desired_state(self, state, blocking = False):
        '''
        Write data to the shared data desired state (if lock is available. Otherwise, return FALSE)
        Will wait here for the lock to be available if blocking == True

        Assumes 'state' is a 12-part array in the form:
        x, y, z, xdot, ydot, zdot, roll, pitch, yaw, rolldot, pitchdot, yawdot
        '''
        i_have_lock = self.shared_data.desired_state_lock.acquire(block = blocking)
        if i_have_lock:
            
            #write the state
            for i,value in enumerate(self.shared_data.desired_state):
                self.shared_data.desired_state[i] = state[i]
                        
            #release lock
            self.shared_data.desired_state_lock.release() #DEVELOPERS NOTE: It is HIGHLY important that the lock is released.
            return True
        else:
            return False
        
    def get_controller_type(self):
        '''
        Returns the human-readable controller type based on the shared memory value code
        No explicit blocking - just relies on built-in read-write blocking
        '''
        ctype = self.shared_data.type_of_controller.value #get the shared memory value 
        return self._controller_types[ctype] #return the human-readable version
    
    def set_controller(self, ctype):
        '''
        accepts EITHER the human-readable controller type OR the corresponding controller type integer code
        sets the shared memory and returns nothing
        '''
        if type(ctype) == str:
            code = self._reverse_controller_types[ctype]
        else:
            code = ctype

        self.shared_data.type_of_controller.value = code
        return
    
    def write_send_buffer(self, msg):
        '''
        Accepts a struct-packed message, formats it, and adds it to the send buffer so it is ready to send
        '''
        packed_msg, length = pack_payload(msg, self.shared_data.MSG_LEN, self.shared_data.id)
        self.shared_data.send_buffer[:self.shared_data.EXPECTED_LEN] = packed_msg
        self.shared_data.send_msg_length.value = length #We store the length of the message,
        #so when we access the buffer to send the message we only send what we need (and not the padded zeros)
        #However, we want to pad the zeros when writting the buffer so the buffer is completely overwritten each buffer write
        return

    def pack_buffer(self, msg):
        '''
        Accepts a struct-packed message and adds it to the read buffer.
        returns nothing
        '''
        #get the read buffer lock (block until we have it)
        i_have_lock = self.shared_data.read_buffer_lock.acquire(block=True)
        if i_have_lock:
            num_msgs = self.shared_data.num_msgs.value % self.shared_data.MAX_MSG_NUM #get the number of messages 
            #This will roll over to 0 once we reach 100 messages, so if there's more than 100 msgs, we will just overwrite the oldest ones.
            index = num_msgs * self.shared_data.MSG_LEN #multiply by the length of each message to get the index
            self.shared_data.read_buffer[index:index+self.shared_data.MSG_LEN] = msg #write between the index and the index + msg length
            num_msgs += 1 #increment number of messages
            self.shared_data.num_msgs.value = num_msgs #update the shared num_msgs variable

            #return the lock
            self.shared_data.read_buffer_lock.release() #DEVELOPERS NOTE: It is HIGHLY important that the lock is released.
        return


    def get_read_buffer(self):
        '''
        Attempts to get the read buffer and return a list of messages.
        Returns FALSE if the read buffer is locked (aka a message is being received.)
        '''
        #get the read buffer lock (DO NOT block until we have it)
        i_have_lock = self.shared_data.read_buffer_lock.acquire(block=False)
        if i_have_lock:
            msgs = []

            #get the number of messages. If more than 100, set it to 100 so we only loop through the entire buffer (and not beyond)
            num_msgs = self.shared_data.num_msgs.value
            if num_msgs > self.shared_data.MAX_MSG_NUM:
                num_msgs = self.shared_data.MAX_MSG_NUM

            #Loop through and add the messages to a list
            for i in range(0,num_msgs):
                index = i * self.shared_data.MSG_LEN
                
                #add the message data to the msgs list
                msgs.append(self.shared_data.read_buffer[index:index+self.shared_data.MSG_LEN])

                #clear that part of the buffer
                # self.shared_data.read_buffer[index:index+self.shared_data.MSG_LEN] = b'\0' * self.shared_data.MSG_LEN

                #TODO make message length 1024 and put the first byte as an ID if it is a new message (to distinguish from zero'd lines)

            #clear the number of messages ()
            self.shared_data.num_msgs.value = 0

            #return the lock
            self.shared_data.read_buffer_lock.release() #DEVELOPERS NOTE: It is HIGHLY important that the lock is released.
            
            return msgs
        
        else:
            return False
    
    def set_send_pid(self, pid):
        '''
        Accepts a pid integer and writes it to the shared data value for the SEND process
        '''
        self.shared_data.send_buffer_pid.value = pid

    def get_send_pid(self):
        '''
        return the process id of the send process (the one that is communicating peer to peer)
        '''
        return self.shared_data.send_buffer_pid.value

    def set_recv_pid(self, pid):
        '''
        Accepts a pid integer and writes it to the shared data value for the RECV process
        '''
        self.shared_data.read_buffer_pid.value = pid

    def get_recv_pid(self):
        '''
        return the process id of the recv process (the one that is communicating peer to peer)
        '''
        return self.shared_data.read_buffer_pid.value
    
    def get_board_connected(self):
        '''
        Return TRUE if board is connected. Else return False
        '''
        if self.shared_data.board_connected.value == 1:
            return True
        else:
            return False
        
    def set_board_connected(self, val):
        '''
        Set the board connected value to val
        '''
        self.shared_data.board_connected.value = val
        return
    
    def get_battery_voltage(self):
        '''
        return the battery voltage written in shared data
        '''
        return self.shared_data.battery_voltage.value
    
    def set_battery_voltage(self, volts):
        '''
        sets the battery voltage to the passed 'volts' value
        '''
        self.shared_data.battery_voltage.value = volts
        return True
    
    def get_battery_power(self):
        '''
        return the battery power written in shared data
        '''
        return self.shared_data.battery_power.value
    
    def set_battery_power(self, power):
        '''
        sets the battery power to the passed 'power' value
        '''
        self.shared_data.battery_power.value = power
        return True
    
    def get_low_voltage_warning(self):
        '''
        returns True if a low voltage has been detected. False otherwise
        '''
        flag = self.shared_data.low_voltage_detected.value
        if flag == 1:
            return True
        else:
            return False
        
    def set_low_voltage_warning(self):
        '''
        Sets the low voltage warning to 1
        '''
        self.shared_data.low_voltage_detected.value = 1
        return
    
    def get_controller_active(self):
        '''
        return the controller active boolean (1 or 0)
        '''
        val = self.shared_data.controller_active.value
        if val == 1:
            return True
        else:
            return False
    
    def set_controller_active(self, value):
        '''
        sets the controller to the active state (1) or inactive state (0)
        '''
        if value in [0,1]:
            self.shared_data.controller_active.value = value
            return True
        else:
            return False
        
    def set_start_time(self, t):
        '''
        Sets the floating point time received in a 'start' message as t=0 for a given user code run
        '''
        self.shared_data.time_zero.value = t
        return
    
    def get_start_time(self):
        '''
        returns time zero (the time at which user code kicked off.)
        '''
        t = self.shared_data.time_zero.value
        return t
    
    def set_robot_start_time(self, t):
        '''
        Sets the floating point time set when bootloader initializes. Represents local t=0 for that robot turning on.
        '''
        self.shared_data.robot_start_time.value = t
        return
    
    def get_robot_start_time(self):
        '''
        returns global time zero (the time at which the robot turned on and started running bootloader.)
        '''
        t = self.shared_data.robot_start_time.value
        return t
    
    def set_optitime_diff(self, t):
        '''
        Records the time value (floating point number) received from optitrack
        '''
        self.shared_data.optitime_diff.value = t
        return
    
    def get_optitime_diff(self):
        '''
        Returns the most recent optitrack recorded time difference
        Use it to find global time: global = local + diff
        '''
        t = self.shared_data.optitime_diff.value
        return t
    
    def get_global_time(self):
        '''
        returns the global time value that keeps updating in the ccode, via c interface
        '''
        t = self.shared_data.global_time.value
        return t
    
    def set_global_time(self, time_from_c):
        '''
        allows the c interface to set the shared data global time
        '''
        self.shared_data.global_time.value = time_from_c
        #print("Got" + str(time_from_c) + " from c interface --------------")
        return
    
    def get_user_code_running(self):
        '''
        Returns True if user code is running, False if not
        '''
        val = self.shared_data.user_code_running.value
        if val == 1:
            return True
        else:
            return False
        
    def set_user_code_running(self, val):
        '''
        Val should be a 1 or a 0. 1 = user code is running, 0 it is not running.
        Sets 1 or 0 in shared memory. Returns nothing
        '''
        self.shared_data.user_code_running.value = val
        return
    

    def get_lander_pos(self, blocking = False):
        '''
        Blocks IFF blocking == True
        if successful, RETURNS array of length 4 with the lander pose
        else, RETURNS an array of length 1 and value 0
        '''
        i_have_lock = self.shared_data.lander_lock.acquire(block = blocking)
        if i_have_lock:
            lander_pos = np.zeros(4)
            lander_pos = np.copy(self.shared_data.lander_position)
                       
            self.shared_data.lander_lock.release() #DEVELOPERS NOTE: It is HIGHLY important that the lock is released.
            return lander_pos
        else:
            return np.zeros(1)
        
        
    def set_lander_pos(self, lander_pos, blocking = False):
        '''
        Write lander position data to the shared data lander_position (if lock is available. Otherwise, return FALSE)
        Will wait here for the lock to be available if blocking == True
        
        Assumes lander_pos is a 4-part array in the form:
         (roll, pitch, throttle, yaw)

        Returns True if successful
        '''
        print('wainting for the lock to set on the lander pos...')
        i_have_lock = self.shared_data.lander_lock.acquire(block = blocking)
        if i_have_lock:
            
            #write the commands
            for i,value in enumerate(self.shared_data.lander_position):
                self.shared_data.lander_position[i] = lander_pos[i]
            
            #release lock
            self.shared_data.lander_lock.release() #DEVELOPERS NOTE: It is HIGHLY important that the lock is released.
            return True
        else:
            return False

        
    



def pack_payload(payload, MSG_LEN, id):
    """This function packs a payload and a position into a packet.

    Parameters:
        payload (bytes): The payload to send.
        MSG_LEN : the length of the payload in bytes (likely 1016)
        id : the integer id of the robot
    Returns:
        bytes: The packed message of length EXPECTED_LEN (1024) 
        This way the ENITRE send buffer is written with a new message (a shorter message will )
        The length of the header (8 bytes) + the length of the payload (so we only send the data, not padded zeros)
    """

    #calculate the trailing zeros that will fill the 1024 bytes of the message payload
    n_msg_bytes = min(len(payload), MSG_LEN)
    n_padding = max(0, MSG_LEN - len(payload))


    header = 'p2p'.encode('utf-8') #3 bytes
    sender = chr(id).encode('utf-8') #1 byte
    msg_length = struct.pack('I', n_msg_bytes)
    padding = b'\x00' * n_padding #This way the send buffer gets completely overwritten (all 1024 bytes)

    #concatenate the id, length with the payload in bytes
    msg = header + sender + msg_length + payload + padding
    
    return msg, 8 + n_msg_bytes
#import os files (files NU created)
from lib.shared_data_management import SharedDataManager, SharedData

#Native imports
import sys
import time
import numpy as np
import traceback
import struct
import subprocess
import mmap
import scipy
import multiprocessing as mp

SHARED_MEMORY_NAME = "MySharedMemory"
DATA_SIZE = 150 #total number of bytes shared between c and python.
MASS = 0.120 #g


'''
This class is going to interface with the c code that serves BOTH as the Localizer and the Controller.
Thus the name: Locaroller

DEVELOPERS NOTE: On the c side, floating point values (or anything that is made up of 4 bytes)
needs to be stored at a memory address that is mod 4 (ends in 0, 4, etc.). (called word-aligned)
To make sure this is always the case, I stack all of the floats first, followed by 2byte ints, followed by 1 byte ints.

For this to work, the python code and the c code will need to share the following things:                   NUM BYTES
01. System state: x,y,z,dx,dy,dz,roll,pitch,yaw,r,p,y (12 floats * 4 bytes each = 48 bytes)                      48
02. Desired state: x,y,z,yaw (4 floats * 4 bytes each = 16 bytes)                                                16
03. KS Matrix components (12 floats * 4 bytes/float = 48 bytes)                                                  48
04. User code start time (as sent from the air traffic controller) (1 float * 4bytes = 4 bytes)                  04
05. Global time: The time in seconds since the user code started (globally) (1 float * 4 bytes = 4 bytes)        04
06. Battery power (1 uint16 * 2 bytes = 2 bytes)                                                                 02
07. Voltage (1 uint16 * 2 bytes = 2 bytes)                                                                       02
08. ARMED state (1 uint16 * 2 bytes = 2 bytes)                                                                   02
09. Flight MODE (1 uint16 * 2 bytes = 2 bytes)                                                                   02
10. State lock: 0= unclocked, 1 = c has the lock, 2 = python has lock (1 chr * 1 bytes = 1 bytes)                01
11. Desired state lock: 0= unclocked, 1 = c has the lock, 2 = python has lock (1 chr * 1 bytes = 1 bytes)        01    
12. Counter that the desired state information is fresh (1char * 1bytes = 1bytes)                                01
        (c will only pull a desired state info if this flag is HIGHER than the value on the c side)
13. Safety code (1chr * 1 byte = 1 bytes)                                                                        01
14. User code running flag (1 chr * 1 byte = 1 bytes)                                                            01
15. Controller type: 0 = lqr, 1 = ?? (1 chr * 1 byte = 1 bytes)                                                  01


TOTAL = 134 bytes.
Call it 150 in case we add more later.
'''
class Locaroller():
    def __init__(self, id, conn, standalone) -> None:
        self.id = id
        self.bootloader_pipe_connection = conn
        self.standalone = standalone


        #Pre-calculate the K matrix (For the LQR controller)
        self.Ks = LQR_precalc()
        print(self.Ks)
        self.bootloader_pipe_connection.send(self.Ks)

        #define the indeces and lengths of the c shared memory.
        self.state_start_index = 0
        self.state_length = 48 #bytes
        self.desired_state_start_index = self.state_start_index + self.state_length
        self.desired_state_length = 16 #bytes
        self.KS_start_index = self.desired_state_start_index + self.desired_state_length
        self.KS_length = 48 #4 bytes / KS item * 12 items.
        self.user_code_start_time_index = self.KS_start_index + self.KS_length
        self.user_code_start_time_length = 4 #bytes
        self.global_time_index = self.user_code_start_time_index + self.user_code_start_time_length
        self.global_time_length = 4 #bytes
        self.battery_power_index = self.global_time_index + self.global_time_length
        self.battery_power_length = 2 #bytes
        self.battery_voltage_index = self.battery_power_index + self.battery_power_length
        self.battery_voltage_length = 2 #bytes
        self.armed_state_index = self.battery_voltage_index + self.battery_voltage_length
        self.armed_state_length = 2 #bytes
        self.flight_mode_index = self.armed_state_index + self.armed_state_length
        self.flight_mode_length = 2 #bytes
        self.state_lock_index = self.flight_mode_index + self.flight_mode_length
        self.state_lock_length = 1 # bytes
        self.lock_ds_index = self.state_lock_index + self.state_lock_length
        self.lock_ds_length = 1 #bytes
        self.ds_counter_index = self.lock_ds_index + self.lock_ds_length
        self.ds_counter_length = 1 #bytes
        self.safety_code_index = self.ds_counter_index + self.ds_counter_length
        self.safety_code_length = 1 #bytes
        self.user_code_running_index = self.safety_code_index + self.safety_code_length
        self.user_code_running_length = 1 #bytes
        self.controller_type_index = self.user_code_running_index + self.user_code_running_length
        self.controller_type_length = 1 #bytes
        self.controller_status_index = self.controller_type_index + self.controller_type_length
        self.controller_status_length = 1 #bytes

        #initialize the flag counter and the local representation fo the desired state.
        self.desired_state_flag_counter = 0
        self.desired_state = np.zeros(12, dtype=float)


        #Variables used to help us in the event of an emergency safe landing
        self.safe_descend_velocity = np.array([0,0,0.075]) #m/s                
        self.landed_threshold = 0.09 #m - the height of a drone on the ground.
        self.dt = 0.01

        self.low_voltage_threshold = 6.5#Volts
        self.low_voltage_threshold_timer = 3 #s (if the battery voltage is below the threshold for this many seconds, we will trigger an auto-land)
        self.last_time_above_threshold = time.time()








    def run(self, shared_data):
        '''
        The behavior that is executed when bootloader kicks off the Locaroller.
        shared_data = data shared between python processes. It is NOT the data shared between C processes.
        '''
        
        try:
            if not self.standalone:
                self.bootloader_pipe_connection.send('Locaroller up and running')
                print("Locaroller up and running.")
            else:
                print("Locaroller up and running.")

            #get a data manger for the shared data between python processes.
            self.data_manager = SharedDataManager(shared_data)

            #initialize a variable for holding state information
            #so we can tell when the state is new/fresh
            state = np.zeros(12)
            last_state = np.zeros(12)

            #set up some shared data with c code.
            #DEVELOPERS NOTE: mmap has to be made HERE (can't be done anywhere else or multiprocessing has an issue pickling it)
            #create a memory-mapped file - LINUX
            # Ensure file exists and has the right size
            with open(SHARED_MEMORY_NAME, "wb") as f:
                f.write(b"\x00" * DATA_SIZE)

            # Open the file and map it into memory
            with open(SHARED_MEMORY_NAME, "r+b") as f:
                mm = mmap.mmap(f.fileno(), DATA_SIZE, access=mmap.ACCESS_WRITE)


                #Make sure c code shared data is intialized correctly.
                # DEVELOPERS NOTE: Everything will be zero by default, but if we needed to specify, it would look like this:
                # # #make sure the desired state is zeros
                # # mm[self.desired_state_start_index : self.clock_info_start_index] = struct.pack("ffff", 0.0, 0.0, 0.0, 0.0)
                # # #make sure the desired state counter is zero
                # # mm[self.flag_info_start_index+3] = self.desired_state_flag_counter
                
                #share the KS matrix with the c code one piece at a time.
                Ksx1 = self.Ks[0][0,0]
                Ksx2 = self.Ks[0][0,1]
                Ksx3 = self.Ks[0][0,2]
                Ksx4 = self.Ks[0][0,3]

                Ksy1 = self.Ks[1][0,0]
                Ksy2 = self.Ks[1][0,1]
                Ksy3 = self.Ks[1][0,2]
                Ksy4 = self.Ks[1][0,3]

                Ksz1 = self.Ks[2][0,0]
                Ksz2 = self.Ks[2][0,1]

                Ksw1 = self.Ks[3][0,0]
                Ksw2 = self.Ks[3][0,1]
                mm[self.KS_start_index : self.KS_start_index + self.KS_length] = struct.pack('12f', Ksx1, Ksx2, Ksx3, Ksx4, Ksy1, Ksy2, Ksy3, Ksy4, Ksz1, Ksz2, Ksw1, Ksw2)



                if self.standalone:
                    print('Locaroller is waiting for a user code to run')
                    print(Ksx1, Ksx2, Ksx3, Ksx4)
                else:
                    self.bootloader_pipe_connection.send('Locaroller is waiting for a user code to run')

                user_code_running = False
                execute_safe_land = False

                time_of_last_print = time.time()

                ####################################
                #### Behavior PRIOR to kick off of user code.
                ####################################
                while not user_code_running:
                    #Check to see if any usercode is running.
                    user_code_running = self.data_manager.get_user_code_running()
                    time.sleep(0.001) #I don't care if it is a precise wait, I just don't want it to be a busy wait.

                    #get the state from the c code
                    temp_state, success = self.get_state_from_c(mm, last_state)
                    if success:
                        state = np.copy(temp_state)
                        last_state = np.copy(state)


                    #DEVELOPERS NOTE: the "set_safety" function will make sure we don't overwrite higher priority safety values.
                    #so, if c stil thinks everything is fine, but python threw a code 1, set_safety won't overwrite 1 with 0 before 1 is sent to c.
                    #Get the safety code from the flight controller c code
                    safety_code = self.get_safety_code_from_c(mm)

                    #set the safety to the rest of the python code (in the event that the c code threw an error)
                    if safety_code not in [6,7]: #DEVELOPERS NOTE: The c code can't generate a 6 or 7 code
                        self.data_manager.set_safety(safety_code)

                    #check safety from the rest of the python code
                    safety = self.data_manager.get_safety()

                    #write it to the c shared memory (in the event that the python code threw an error or received reset/stop)
                    self.write_safety_code_to_c(mm, safety)



                    #get the voltage from the c code
                    volts = self.get_voltage(mm)

                    #set the voltage in python shared memory.                    
                    self.data_manager.set_battery_voltage(volts)
                    
                    #get the power from the c code
                    watts = self.get_power(mm)

                    #set the power in python shared memory.
                    self.data_manager.set_battery_power(watts)
                    

                ####################################
                #### Transition from PRIOR to DURING
                ####################################
                #If the code reaches this point, then user code has been kicked off (Air Traffic Control has sent "start")
                #so, we need to transition the c code to running the controller IAW (in accordance with) the user code.

                #First, we get the start time of the user code so "global time" can be calculated correctly.
                user_code_global_start_time = self.data_manager.get_start_time()

                #set it in the c code shared memory so the c code can access it.
                mm[self.user_code_start_time_index : self.user_code_start_time_index + self.user_code_start_time_length] = struct.pack('f', user_code_global_start_time)

                #Let the c code know that the user code is running
                mm[self.user_code_running_index : self.user_code_running_index + self.user_code_running_length] = struct.pack('B', 1)
                #DEVELOPERS NOTE: This will cause the c code to begin logging the data in a control log.
                
                if self.standalone:
                    print('user code started...')
                else:
                    self.bootloader_pipe_connection.send('Locaroller has told c code the user code started')
                    print('user code started...')
                    # print(mm[self.user_code_running_index : self.user_code_running_index + self.user_code_running_length])
                    # print(self.user_code_running_index - self.state_start_index)


                ####################################
                #### Behavior DURING the execution of the user code.
                ####################################
                time_of_last_print = time.time()
                landed = False
                while not execute_safe_land:
                    
                    self.get_global_time_from_c(mm)

                    time.sleep(0.001) #I don't care if it is a precise wait, I just don't want it to be a busy wait.
            
                    #check safety
                    safety = self.data_manager.get_safety()

                    #write it to the c shared memory
                    self.write_safety_code_to_c(mm, safety)

                    #IF it's code 3 or 4, we need to execute a safe landing.
                    if safety in [3,4]:
                        execute_safe_land = True #This effectively breaks the while loop upon next loop (that's too late, so we just break below)
                        break
                    elif safety in [1, 2]:
                        landed = True
                        if not self.standalone:
                            self.bootloader_pipe_connection.send('an error code %d was initited from elsewhere. Flyer is stopping.' % safety)
                            print('an error code %d was initited from elsewhere. Flyer is stopping.' % safety)
                        else:
                            print('an error code %d was initited from elsewhere. Flyer is stopping.' % safety)
                        self.data_manager.set_ARM(0, blocking=True) #Disarm the controller on the python side
                        break



                    # #IF we decide to do this, it will be done here.
                    # #Get controller type
                    # desired_controller_type = self.data_manager.get_controller_type()
                    
                    self.send_desired_state_to_c(mm)

                    #Determine if the onboard controller is active 
                    controller_is_active = self.data_manager.get_controller_active()

                    if controller_is_active == True:
                        controller_is_active = 1
                    elif controller_is_active == False:
                        controller_is_active = 0

                    #write to the c shared memory if the controller is active or not.
                    if self.data_manager.get_ARM():
                        #write 1800 to the c shared memory armed state
                        mm[self.armed_state_index : self.armed_state_index + self.armed_state_length] =\
                              struct.pack('H', 1800) # goes to shared_armed_status in the c file
                        mm[self.controller_status_index : self.controller_status_index + self.controller_status_length] = \
                            struct.pack('B', controller_is_active)
                    else:
                        #write 1000 to the c shared memory
                        mm[self.armed_state_index : self.armed_state_index + self.armed_state_length] =\
                              struct.pack('H', 1000)
                        mm[self.controller_status_index : self.controller_status_index + self.controller_status_length] = \
                            struct.pack('B', controller_is_active)
                        

                    #Determine the flight mode of the controller
                    mode = self.data_manager.get_MODE(blocking=False)
                    if mode:
                        #write mode to the c shared memory armed state
                        mm[self.flight_mode_index : self.flight_mode_index + self.flight_mode_length] =\
                              struct.pack('H', mode)

                    #Get the safety code from the flight controller c code
                    safety_code = self.get_safety_code_from_c(mm)
                    
                    #if the safety flag is a 1, then initialte estop.
                    if safety_code == 1:
                        if not self.standalone:
                            self.bootloader_pipe_connection.send('the c code detected a localization error, roll over, or bounding box error.')
                            print('the c code detected a localization error, roll over, or bounding box error.')
                        else:
                            print('the c code detected a localization error, roll over, or bounding box error.')
                        self.data_manager.set_ARM(0, blocking=True) #Disarm the controller on the python side
                        self.data_manager.set_safety(1) #initiate an e-stop on the python side
                        landed = True #This will prevent us from trying to safe land.
                        break 

                    #read state from c shared memory, and share it with other python processes
                    temp_state, success = self.get_state_from_c(mm, last_state)
                    if success:
                        state = np.copy(temp_state)
                        last_state = np.copy(state)

                    #get the voltage from the c code
                    volts = self.get_voltage(mm)

                    #set the voltage in python shared memory.                    
                    self.data_manager.set_battery_voltage(volts)
                    
                    #get the power from the c code
                    watts = self.get_power(mm)

                    #set the power in python shared memory.
                    self.data_manager.set_battery_power(watts)

                    #check that the voltage isn't below threshold.
                    #check the battery voltage
                    if volts < self.low_voltage_threshold:
                        #If more than the low_voltage_threshold_timer time as passed since we were last above the threshold,
                        #then we will initialize a safety code 30
                        if time.time() - self.last_time_above_threshold >= self.low_voltage_threshold_timer:
                            self.data_manager.set_safety(4)
                            self.bootloader_pipe_connection.send('LOW VOLTAGE WARNING!! ' + str(volts))
                            print("LOW VOLTAGE WARNING")
                            self.data_manager.set_low_voltage_warning()
                    else:
                        self.last_time_above_threshold = time.time()
                    

                ####################################
                #### Transition from DURING to POST
                ####################################   
                #If we have broken out of the above loop, then it is time to safe land.
                #Unless landed is already True (meaning the c code already initiated the killing of the robot due to some error)
                        
                if not landed:
                    if not self.standalone:
                        self.bootloader_pipe_connection.send('An error %d occurred. Control manger is initiating safe landing execution' % safety)
                        print('transitioning to a safe land')
                        print('An error %d occurred. Control manger is initiating safe landing execution' % safety)
                    else:
                        print('transitioning to a safe land')
                        print('An error %d occurred. Control manger is initiating safe landing execution' % safety)

                    #Read the latest state and make that the desired state
                    temp_state, success = self.get_state_from_c(mm, last_state)
                    if success:
                        state = np.copy(temp_state)
                        self.desired_state = np.copy(state)
                        last_state = np.copy(state)
                    else:
                        self.desired_state = np.copy(last_state)

                #Let the c code know that the user code is no longer running, so the c code can stop logging data
                mm[self.user_code_running_index : self.user_code_running_index + self.user_code_running_length] =\
                      struct.pack('B', 0)


                ####################################
                #### Behavior POST the execution of the user code.
                ####################################

                #Execute the safe landing loop (slowly lower the setpoint)
                while not landed:

                    time.sleep(0.001) #I don't care if it is a precise wait, I just don't want it to be a busy wait.

                    #Check safety
                    safety = self.data_manager.get_safety()

                    #write it to the c shared memory
                    self.write_safety_code_to_c(mm, safety)

                    #IF it's code 1 or 2, we need to disarm and stop immediately
                    if safety in [1,2]:
                        if not self.standalone:
                            self.bootloader_pipe_connection.send('The safe landing sequence has been interrupted by higher priority Estop.')
                            print('The safe landing sequence has been interrupted by higher priority Estop.')
                        else:
                            print('The safe landing sequence has been interrupted by higher priority Estop.')
                        self.data_manager.set_ARM(0, blocking=True) #Disarm the controller on the python side                        
                        landed=True
                        break

                    #Get setpoint info
                    #if we are executing a safe land, we handle the setpoint ourselves
                    #lower our setpoint by the prescribed velocity
                    self.desired_state[0:3] = self.desired_state[0:3] - self.safe_descend_velocity*(self.dt)
                    #if our setpoint is below the threshold, set it to the threshold
                    # if self.desired_state[2] <= self.landed_threshold:
                    #     self.desired_state[2] = self.landed_threshold

                    #increment the desired state flag counter
                    self.desired_state_flag_counter += 1
                    if self.desired_state_flag_counter > 255:
                        self.desired_state_flag_counter = 0

                    #write the new setpoint to the c shared memory.
                    self.write_ds_to_c(mm)
                    
                    #determing if the controller is active.
                    controller_is_active = self.data_manager.get_controller_active()

                    #If the controller is not active, then just assume we've landed.
                    if not controller_is_active:
                        if not self.standalone:
                            self.bootloader_pipe_connection.send('The safe landing sequence has been successfully executed. - controller was never active.')
                            print('The safe landing sequence has been successfully executed. - controller was never active.')
                        else:
                            print('The safe landing sequence has been successfully executed. - controller was never active.')
                        self.data_manager.set_ARM(0, blocking=True) #Disarm the controller on the python side
                        self.data_manager.set_safety(2) #initiate a normal stop on the python side
                        #let the c code know that the user code is no longer running and the controller is disarmed.
                        #write to the c shared memory if the controller is active or not.
                        #write 1000 to the c shared memory
                        mm[self.armed_state_index : self.armed_state_index + self.armed_state_length] =\
                              struct.pack('H', 1000)
                        self.write_safety_code_to_c(mm, 2)
                        landed = True
                        break
                    
                    #get the state from the c code
                    temp_state, success = self.get_state_from_c(mm, last_state)
                    if success:
                        state = np.copy(temp_state)
                        last_state = np.copy(state)

                        #Check if we've landed (for safe land) and then initiate the board reset.
                        if state[2] <= self.landed_threshold:
                            if not self.standalone:
                                self.bootloader_pipe_connection.send('The safe landing sequence has been successfully executed.')
                            else:
                                print('the safe landing sequence has been executed successfully.')
                            self.data_manager.set_ARM(0, blocking=True) #Disarm the controller on the python side
                            self.data_manager.set_safety(2) #initiate a normal stop on the python side
                            self.write_safety_code_to_c(mm, 2)
                            landed = True
                        elif landed:
                            continue
                        
                    #get the voltage from the c code
                    volts = self.get_voltage(mm)

                    #set the voltage in python shared memory.                    
                    self.data_manager.set_battery_voltage(volts)
                    
                    #get the power from the c code
                    watts = self.get_power(mm)

                    #set the power in python shared memory.
                    self.data_manager.set_battery_power(watts)


                
                if not self.standalone:
                    self.bootloader_pipe_connection.send('The flyer has safely landed (or at least the code is done.)')
                    print('safe land over')
                else:
                    print('Safe land over')
          
        
        except Exception as error:          

            #log the error and continue
            logMessage = ['an error occurred']
            trace = traceback.extract_tb(error.__traceback__)
            for t in trace:
                logMessage.append(str(t))
            logMessage.append([str(type(error).__name__)])
            logMessage.append([str(error)])
            if not self.standalone:
                self.bootloader_pipe_connection.send(logMessage) #send the error to the bootloader to be logged
            print(logMessage)

            if not self.standalone:
                self.bootloader_pipe_connection.send('Safety code 1 as control manager failed.')
            self.data_manager.set_safety(1) #we have no control over the robot, so we need to kill it.

    def get_global_time_from_c(self, mm):
        gt = struct.unpack('f', mm[self.global_time_index : self.global_time_index + self.global_time_length])[0]
        self.data_manager.set_global_time(gt)
        return 
    
    def get_state_from_c(self, mm, last_state):
        '''
        Attempts to pull state information from c shared memory.
        Writes to python shared memory if successful.
        Returns the read state or []. and a success variable (TRUE = success, False = failed.)
        '''
        success = False
                    
        #read state from c shared memory, and share it with other python processes
        #check if lock is available.
        if struct.unpack('B', mm[self.state_lock_index : self.state_lock_index + self.state_lock_length])[0] == 0:
            #lock it
            mm[self.state_lock_index : self.state_lock_index + self.state_lock_length] = struct.pack('B', 2) #2 means python code has it locked

            #read the data
            state_bytes = mm[self.state_start_index:self.state_start_index + self.state_length]

            #flag that we should write the data to the python shared memory.
            #and that we successfully pulled data from c
            success = True

            #unlock it
            mm[self.state_lock_index : self.state_lock_index + self.state_lock_length] = struct.pack('B', 0) #0 indicates unlocked

        else:
            success = False
            state = np.zeros(12)

        #if we successfully pulled data from the c code, write it to the shared data IFF if it fresh.
        if success:
            state = np.array(struct.unpack('12f', state_bytes),dtype=float) #format the state into an array
            if np.all(state == last_state):
                success = False #The state is not fresh, so don't set it.
            else:
                #set the state in shared memory, don't bother blocking. 
                #if the space in memory is locked, we'll just write the next one.
                self.data_manager.set_state(state, blocking=False) #TODO get rid of timing info?
            
        return state, success

    def send_desired_state_to_c(self, mm):
        '''
        Attempts to pull the desired state information from PYTHON shared memory.
        Writes the desired state to the C shared memory IFF the desired state is new and non-zero.
        Returns True if written, False if not
        '''     

        #Get the setpoint from the data manager.
        #DEVELOPERS NOTE: Since we are NOT blocking here, we have to handle the false case. (see next line) 
        # In this case, the desired state is initalized earlier and only overwritten if success is True
        ret_val, success = self.data_manager.get_desired_state(blocking=False)
        if success:
            #don't set the desired state if they are all zeros x,y, and z anyway) (that's the default...)
            #Also, don't set the desired state if it is exactly the same as the desired state we currently have (i.e., it is not fresh)
            if not np.all(ret_val[0:3] == 0) and not np.all(ret_val == self.desired_state):

                
                self.desired_state = np.copy(ret_val)
                self.desired_state_flag_counter += 1
                if self.desired_state_flag_counter > 255:
                    self.desired_state_flag_counter = 0

                success = self.write_ds_to_c(mm)

            else:
                success = False

            #success will be TRUE IFF the desired state is read from python memory, is not all zeros, is NEW, and is written to the c code.
            #if any of those fail, success will be False.
            return success 

        else: #here we return False because the desired state was NOT written to c (due to the inability to pull it from python shared memory)
            return False

    def write_ds_to_c(self, mm):
        '''
        Writes desired state to c shared memory (handles locks, etc.)
        Returns True if written, False if not.
        '''

        #check if the desired state is unclocked
        if struct.unpack('B', mm[self.lock_ds_index : self.lock_ds_index + self.lock_ds_length])[0] == 0:
            
            #lock it
            mm[self.lock_ds_index : self.lock_ds_index + self.lock_ds_length] = struct.pack('B', 2) #2 indicates that python has it locked

            #write the new setpoint to the c shared memory.
            mm[self.ds_counter_index : self.ds_counter_index + self.ds_counter_length] = struct.pack('B', self.desired_state_flag_counter)
            mm[self.desired_state_start_index : self.desired_state_start_index + self.desired_state_length] = \
                struct.pack('ffff', self.desired_state[0], self.desired_state[1], self.desired_state[2], self.desired_state[8])
            
            #unlock it
            mm[self.lock_ds_index : self.lock_ds_index + self.lock_ds_length] = struct.pack('B', 0) #0 indicates unlocked


            return True
        else:
            return False
    
    def get_safety_code_from_c(self, mm):
        '''
        Reads the safety code from the c shared memory. Returns the unpacked value
        '''
        flag_bytes = mm[self.safety_code_index : self.safety_code_index + self.safety_code_length]
        flags = struct.unpack('B', flag_bytes)
        return flags[0]
    
    def write_safety_code_to_c(self, mm, code):
        mm[self.safety_code_index : self.safety_code_index + self.safety_code_length] = struct.pack('B', code)
        return
    
    def get_voltage(self, mm):
        '''
        Reads the voltage from the c code, divides by 10 to get volts as a float.
        Returns volts
        '''
        vbytes = mm[self.battery_voltage_index : self.battery_voltage_index + self.battery_voltage_length]
        volts = struct.unpack('H',vbytes)[0]
        return float(volts)/10.0
    
    def get_power(self, mm):
        '''
        Reads the power from the c code, divides by 100 to get watts (Volts * Amps) as a float.
        Returns watts
        '''
        wbytes = mm[self.battery_power_index : self.battery_power_index + self.battery_power_length]
        watts = float(struct.unpack('H',wbytes)[0])/100.0
        return watts
    
    
    
    def exit(self):
        pass

def LQR_precalc():
    '''
    Function to calculate the LQR K matrix (which is constant) so we don't have to do it
    each time we initiate the LQR controller (which may happen mid-flight!!)
    '''
    #Drone constants/parameters
    m = MASS #kg
    g = 9.81 #m/s/s
    l = 0.12 #m
    h = 0.04 #m
    Ix = 1.04 *1e-4
    Iy = 1.04 *1e-4
    Iz = 7.99*1e-5

    # A matrix
    # 12x12 matrix -> number of states x number of states matrix
    # Expresses how the state of the system [x,y,z,u,v,w,phi,theta,psi,p,q,r] changes 
    # from t-1 to t when no control command is executed.
    #broken into 4 subsystesms: X, Y, Z, and Yaw
    #x, xdot, pitch pitchdot
    Ax = np.array(
            [[0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, g, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, 0.0, 0.0]])
    
    #y, ydot, roll, rolldot
    Ay = np.array(
            [[0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, -g, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, 0.0, 0.0]])
    
    #z, zdot
    Az = np.array(
            [[0.0, 1.0],
            [0.0, 0.0]])
    
    #yaw, yawdot
    Ayaw = np.array(
            [[0.0, 1.0],
            [0.0, 0.0]])
    


    # B matrix
    # Expresses how the state of the system changes
    # from t-1 to t due to the control commands (i.e. control inputs).
    # again broken into subsystems.

    #x
    Bx = np.array(
        [[0.0],
        [0.0],
        [0.0],
        [1 / Ix]])
    
    #y
    By = np.array(
        [[0.0],
        [0.0],
        [0.0],
        [1 / Iy]])
    
    #z
    Bz = np.array(
        [[0.0],
        [1 / m]])
    
    #yaw
    Byaw = np.array(
        [[0.0],
        [1 / Iz]]) 
    
    # R matrix
    # The control input cost matrix
    # Experiment with different R matrices
    # This matrix penalizes actuator effort (i.e. rotation of the rotors) 
    # specifically in thrust, roll, pitch, and yaw commands.
    # The R matrix has the same number of rows as the number of control
    # inputs and same number of columns as the number of control inputs.
    # This matrix has positive values along the diagonal and 0s elsewhere.
    # We can target control inputs where we want low actuator effort 
    # by making the corresponding value of R large. 
    R = np.array([[1.4,    0,    0,    0], # Penalty for thrust effort
                    [   0, 10.5,    0,    0], # Penalty for roll effort
                    [   0,    0, 10.5,    0], # Penalty for pitch effort
                    [   0,    0,    0, 1.0]])# Penalty for yaw effort
    

    # Q matrix
    # The state cost matrix.
    # Experiment with different Q matrices.
    # Q helps us weigh the relative importance of each state in the 
    # state vector [x,y,z,u,v,w,phi,theta,psi,p,q,r]. 
    # Q is a square matrix that has the same number of rows as 
    # there are states.
    # Q penalizes bad performance.
    # Q has positive values along the diagonal and zeros elsewhere.
    # Q enables us to target states where we want low error by making the 
    # corresponding value of Q large.
    Q = np.diag((1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0))
    Q[0,0] = 36.0 #extra cost on x #1.5 
    Q[1,1] = 36.0 #extra cost on y #1.5
    Q[2,2] = 48.0 #extra cost on z #96
    Q[3,3] = 15.0 #extra cost on x velo #0.75
    Q[4,4] = 15.0 #extra cost on y velo #0.75
    Q[5,5] = 15.0 #extra cost on z velocity
    # self.Q[8,8] = 10.0 #extra cost on yaw
    Q[6,6] = 0.0 #extra cost on roll
    Q[9,9] = 0.0 #extra cost on roll velocity
    Q[7,7] = 0.0   #extra cost on pitch
    Q[10,10] = 0.0 #extra cost on pitch velocity
    Q[8,8] = 2.0 #extra cost on yaw
    Q[11,11] = 0.0 #extra cost on yaw velocity

    Ks = []  # feedback gain matrices K for each subsystem
    for A, B, subsystem in ((Ax, Bx, 'x'), (Ay, By, 'y'), (Az, Bz, 'z'), (Ayaw, Byaw, 'yaw')):
        n = A.shape[0]
        m = B.shape[1]
        if subsystem == 'x':
            q = np.diag([Q[0,0], Q[3,3], Q[7,7], Q[10,10]])
            r = np.array([[R[1,1]]])
        elif subsystem == 'y':
            q = np.diag([Q[1,1], Q[4,4], Q[6,6], Q[9,9]])
            r = np.array([[R[2,2]]])
        elif subsystem == 'z':
            q = np.diag([Q[2,2], Q[5,5]])
            r = np.array([[R[0,0]]])
        elif subsystem == 'yaw':
            q = np.diag([Q[8,8], Q[11,11]])
            r = np.array([[R[3,3]]])
        K, _, _ = lqr(A, B, q, r)
        Ks.append(K)

    return Ks

def lqr(A, B, Q, R):
    """Solve the continuous time lqr controller.
    dx/dt = A x + B u
    cost = integral x.T*Q*x + u.T*R*u
    """
    # http://www.mwm.im/lqr-controllers-with-python/
    # ref Bertsekas, p.151

    # first, try to solve the ricatti equation
    X = np.matrix(scipy.linalg.solve_continuous_are(A, B, Q, R))

    # compute the LQR gain
    K = np.matrix(scipy.linalg.inv(R) * (B.T * X))

    eigVals, eigVecs = scipy.linalg.eig(A - B * K)

    return np.asarray(K), np.asarray(X), np.asarray(eigVals)


        



class C_Handler():
    def __init__(self, id, conn, ip, port, timeout_s, timeout_nsec, standalone) -> None:
        self.id = id
        self.bootloader_pipe_connection = conn
        #This is set in localizer_timeout in params.json
        self.timeout_s = timeout_s #seconds #set in params.json (typically 1second.) 
        self.timeout_nsec = timeout_nsec
        self.port = port #typically 54321 #set in params.json. Called 'localizer_port'
        self.ip = ip #IP address of multicast group. set in params.json called "localizer_ip"
        self.standalone = standalone


    def run(self, shared_data):
        
        if not self.standalone:
            self.bootloader_pipe_connection.send('C handler up and running')

            #get a data manger for the shared data between python processes.
            self.data_manager = SharedDataManager(shared_data)

        arguments = [str(self.id), self.ip, str(self.port), str(self.timeout_s), str(self.timeout_nsec)]
        if self.standalone:
            arguments.append("1")
        else:
            arguments.append("0")
        process = subprocess.Popen(['./execute'] + arguments)
        stdout, stderr = process.communicate()

        if process.returncode == 0:
            print("C executable ran successfully.")
            print("Output:")
            print(stdout)
            if not self.standalone:
                self.bootloader_pipe_connection.send('C code ran successfully. Stdout:' + str(stdout))
        else:
            print(f"C executable failed with error code: {process.returncode}")
            if not self.standalone:
                self.bootloader_pipe_connection.send(f"C executable failed with error code: {process.returncode}")
            if stderr:
                print("Error output:")
                print(stderr)
                self.bootloader_pipe_connection.send(f"Error output: {stderr}")

    def exit(self):
        pass


if __name__ == "__main__":
    print('Testing the interface between python and the c code on the robot.')
    print('This is a standalone test.')

    shared_data = SharedData(10)


    locaroller = Locaroller(10, False, standalone = True)
    chanlder = C_Handler(10, False, "224.1.1.1", 54321, 5, standalone=True)

    locaroller_process = mp.Process(target=locaroller.run, args=(shared_data,))
    locaroller_process.start()

    time.sleep(0.5)

    c_process = mp.Process(target=chanlder.run, args=(shared_data,))
    c_process.start()

    print('Both processes should be running.')

    while True:
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            locaroller_process.join()
            c_process.join()
            c_process.terminate()
            locaroller_process.terminate()
            break









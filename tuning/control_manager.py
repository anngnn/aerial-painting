'''
This code was developed by Andrew Curtis in the year 2024.
It was developed at Northwestern University, Evanston, IL, USA.

The underlying control for each quadrotor in the QuadSwarm is housed here.
The user can select a controller type in the user code and that control will execute here
Input: State, desired state
Output: Control Commands

Pavi was here
so was Preston
hellooooo
'''

#import 3rd party libraries
#NONE

#import os files (files NU created)
from lib.shared_data_management import SharedDataManager
from logbook import get_time_stamp

#import native libraries
import time
import numpy as np
import scipy
import traceback
import csv

MASS = 0.120


class ControlManager():
    '''
    Class to manage which controller is running and execute said controller
    '''
    def __init__(self, id, conn) -> None:

        self.id = id
        self.bootloader_pipe_connection = conn
        
        #Here is a dictionary of different controllers that the robot "knows"
        #and can execute during an experimental run or flight.
        #DEVELOPERS NOTE: If you are going to add a new controller, you have to add it to this dictionary too.
        #The dictonary consists of a unique controller name as the KEY and the corresponding controller CLASS
        #DEVELOPERS NOTE: If you are going to add a new controller, you also need a new controller code
        #represented in the shared data management class
        self.control_options = {'none': Controller, 'pid': PID_Controller, 'lqr': LQR_Controller, 'safe land' : LQR_Controller}

        #Holds the dictonary KEY for the controller that is actively running.
        self.active_controller_key = 'none'


        #The OBJECT of the active controller. What is actually running. Defaults to none
        #DEVELOPERS NOTE: We are creating an OBJECT from the CLASS that is stored in the DICTONARY entry 'none'
        #That's why we need the () after the dictionary call, so the CLASS is instantiated as an object.
        self.active_controller = self.control_options[self.active_controller_key]()

        #Pre-calculate the K matrix (For the LQR controller)
        self.Ks = LQR_precalc()

        #SET UP VARIABLES WE WILL NEED 
        # | | | | |
        # | | | | |
        # V V V V V

        #Local state info
        #x, y, z, xdot, ydot, zdot, roll, pitch, yaw, rolldot, pitchdot, yawdot
        self.state = np.zeros(12, dtype=float)
        self.assumed_state = np.zeros(12, dtype=float)
        self.desired_state = np.zeros(12, dtype=float)
        self._desired_state_set = False
        self.accel = np.zeros(3) #x,y,z acceleration stored here.

        #Logic for the safety of the robot and its operators
        self.roll_over_error = False
        self.bounding_box_error = False

        #Variables used to help us in the event of an emergency safe landing
        self.safe_descend_velocity = np.array([0,0,0.075]) #m/s                
        self.landed_threshold = 0.12 #m - the height of a drone on the ground.
        self.landed = False

        #Initialize a Kalman Filter
        self.dt = 0.04
        self.kf = KalmanFilter(dt = self.dt, mass = MASS)

        #Initialize variables for maintaining a set control loop frequency
        self.loop_start = 0.0 #in seconds *10^9
        #20.000.000 = 20ms = 50Hz
        #40.000.000 = 40ms = 25Hz
        self.loop_end = 40000000.0 #in seconds *10^9
        self.controller_frequency = 1000000000/(self.loop_end - self.loop_start)
        self.controller_frequency = 42
        self.ideal_control_duration = 40000000.0 #nanoseconds #seconds *10^9
        

    def run(self, shared_data):
        '''
        The function that is kicked off by the process manager in bootloader.
        Responsible for making sure the right controller is running and running that controller.
        Also, interfaces with the shared data.
        '''
        self.bootloader_pipe_connection.send('Controll Manager up and running')

        self.data_manager = SharedDataManager(shared_data)

        try:           

            #Initialize some parameters prior to the while loop
            # robot_start_time = self.data_manager.get_robot_start_time() #TODO FIX THIS!
            robot_start_time = time.time()
            user_code_running = False
            execute_safe_land = False
            self.user_code_global_start_time = 0

            print('waiting for a user code to run')

            while not user_code_running:
                #Check to see if any usercode is running.
                user_code_running = self.data_manager.get_user_code_running()
                time.sleep(0.005) #I don't care if it is a precise wait, I just don't want it to be a busy wait.
            
            
            #create a cooresponding controller log for this user code run.
            #also get the user code global start time sent from the 'start' message.
            self.user_code_kickoff()

            #Get the state from shared memory
            #DEVELOPERS NOTE: Since we are BLOCKING here, we don't have to worry about it returning FALSE
            self.state = self.data_manager.get_state(clear_fresh_flag=True, blocking=True)

            #initialize the kalman filter.
            self.kf.x = np.array([self.state]).T

            #if we have never set a setpoint before, we'll make it our current pos, no velocity.
            if not self._desired_state_set:
                self._desired_state_set = True
                self.desired_state[0:3] = self.state[0:3] #only need x,y,z. rest are initialized zero already.

            print('running controller...', time.time()-robot_start_time, robot_start_time)

            #Enter normal controller operation
            while not execute_safe_land:
                self.loop_start = time.perf_counter_ns() #record the time at which this control loop started. in seconds * 10^9

                #Check safety
                safety = self.data_manager.get_safety()

                #IF it's code 3 or 4, we need to execute a safe landing.
                if safety in [3,4]:
                    execute_safe_land = True #This effectively breaks the while loop upon next loop through (that's too late, so we just break below)
                    break

                #Get controller type
                desired_controller_type = self.data_manager.get_controller_type()
                
                #Handle the controller change (if necessary)
                self.change_controllers(desired_controller_type)


                #GET THE STATE INFO
                logerror = 0 #for logging. assume we have no errors 
                use_estimate = False
                use_estimate, timing_info = self.get_state_info()
                
                #Get the setpoint from the data manager.
                #DEVELOPERS NOTE: Since we are NOT blocking here, we have to handle the false case. (see next line) 
                # In this case, the desired state is initalized earlier and only overwritten if success is True
                ret_val, success = self.data_manager.get_desired_state(blocking=False)
                if success:
                    if np.any(ret_val) == True: #don't set the desired state if they are all zeros (that's the default...)
                        self.desired_state = ret_val

                #Determine if the onboard controller is active (ARMED)
                controller_is_active = self.data_manager.get_controller_active()

            
                #if the controller is active, 'run it' (do error checks and feed the controller output to the fc board.)
                if controller_is_active:
                    written = self.run_error_checks_and_feed_controller(use_estimate)
                    if not written and not use_estimate:
                        logerror = 1
                    elif written and use_estimate:
                        logerror = 2
                    elif not written and use_estimate:
                        logerror = 3

                #Write data to the flight control log.
                raw_unrounded_local_time = time.time()
                raw_local_time = np.round(raw_unrounded_local_time,4) #raw time since January 1, 1970, 00:00:00 (UTC)
                robot_local_time = np.round(raw_unrounded_local_time - robot_start_time,4) #time since bootloader turned on.
                power = np.round(self.data_manager.get_battery_power(),2)
                voltage = np.round(self.data_manager.get_battery_voltage(),2)
                if not use_estimate: #here we actually have all of the data we want to print.
                    #raw local time = time.time() on the robot right now
                    #robot local time = time.time() - time the robot turned on
                    #received global time = timing_info[0]       [self.received_time, self.received_sequence_number, count_delta, time_between_messages]
                    received_global_time = np.round(timing_info[0],4)
                    global_time_since_start = np.round(received_global_time - self.user_code_global_start_time,4)
                    try:
                        ix = self.active_controller.KI_x * self.active_controller.integral_x
                        iy = self.active_controller.KI_y * self.active_controller.integral_y
                        iz = self.active_controller.KI_z * self.active_controller.integral_z
                    except:
                        ix = 0
                        iy = 0
                        iz = 0
                    #Raw local time, robot local time, received time, received time since user code start, rec seq no, cnt diff, time btwn msgs, ctrl freq, 
                    line_start = [raw_local_time, robot_local_time, received_global_time, global_time_since_start] + timing_info[1:4] + [np.round(self.controller_frequency,2)]
                    #x, y, z, dx, dy, dz, 'phi', 'theta', 'psi', 'r', 'p', 'y','setx', 'sety', 'setz', 'kfx', 'kfy', 'kfz', 'u_roll', 'u_pitch', 'u_z', 'u_yaw', 'int_x', 'int_y', 'integral z', 'power', 'voltage', 'error'  
                    line_end = np.round(self.state,3).tolist() + np.round(self.desired_state[0:3],3).tolist() + np.round(self.assumed_state[0:3],3).tolist() + self.active_controller.commands.tolist() + [ix, iy, iz, power, voltage, logerror]
                    line = line_start + line_end
                    self.writer.writerow(line)
                    self.csvfile.flush()
                else:
                    #Raw local time, robot local time, received time, received time since user code start, rec seq no, cnt diff, time btwn msgs, ctrl freq, 
                    line_start = [raw_local_time, robot_local_time, 0, 0] + [0, 0, 0] + [np.round(self.controller_frequency,2)]
                    #x, y, z, dx, dy, dz, 'phi', 'theta', 'psi', 'r', 'p', 'y','setx', 'sety', 'setz', 'kfx', 'kfy', 'kfz', 'u_roll', 'u_pitch', 'u_z', 'u_yaw', 'int_x', 'int_y', 'integral z', 'power', 'voltage', 'error'  
                    line_end = np.round(self.state,3).tolist() + [0, 0, 0] + np.round(self.assumed_state[0:3],3).tolist() + self.active_controller.commands.tolist() + [0, 0, 0, power, voltage, logerror]
                    line = line_start + line_end
                    self.writer.writerow(line)
                    self.csvfile.flush()


                #Calculate loop duration to see if we need to wait or not
                self.loop_end = time.perf_counter_ns() #record the time the control loop finished in seconds * 10^9
                control_duration = self.loop_end - self.loop_start
                

                #If we have extra time (we finished the control loop before the ideal duration)
                #Wait here to keep the duration at the desired frequency.
                #Otherwise, loop again.
                # print(control_duration, self.controller_frequency)
                if control_duration < self.ideal_control_duration:
                    wait_time = self.ideal_control_duration - control_duration
                    t0 = time.perf_counter_ns()
                    # print('waiting')
                    time.sleep(0.5*wait_time/1000000000)
                    j = 0
                    while time.perf_counter_ns() - t0 < wait_time:
                        # print('too fast wait... %d' %j)
                        # print(time.perf_counter(), t0, wait_time, control_duration, self.ideal_control_duration)
                        j += 1
                        pass

                else:
                    # print('too slow!')
                    pass

                #Calculte the loop frequency
                actual_duration = time.perf_counter_ns() - self.loop_start
                self.controller_frequency = 1000000000/(actual_duration) #calculate the frequency of the control loop (convert to seconds).

            print('transitioning to a safe land')
            #Transition to a safe landing.
            #The LQR controller is used to safely land. This effectively switches it to LQR.
            active_controller_cmds = self.active_controller.commands #get the current commands from the active controller
            x, y, z, yaw = self.active_controller.get_integral() #get the current integral terms (in x, y, z and yaw)
            self.active_controller_key = 'safe land'
            self.active_controller = self.control_options[self.active_controller_key](self.Ks)
            self.bootloader_pipe_connection.send('An error %d occurred. Control manger is initiating safe landing execution' % safety)
            self.active_controller.hardcode_cmds(active_controller_cmds) #initialize the new controller with the active controller commands
            self.active_controller.set_integral(x,y,z,yaw) #set the integral terms
            #set the desired state to our last known position.
            self.desired_state = np.copy(self.state)

            #finally, if we have a log file going, we need to stop it.
            if self.csvfile:
                self.csvfile.flush()
                self.csvfile.close()
                del self.writer
                del self.csvfile
                    

            #Handle the safe landing of the flyer (controller is now unresponsive to user code.)
            while not self.landed:

                self.loop_start = time.perf_counter_ns()

                #Check safety
                safety = self.data_manager.get_safety()

                #IF it's code 1 or 2, we need to disarm and stop immediately
                if safety in [1,2]:
                    self.landed=True
                    self.bootloader_pipe_connection.send('The safe landing sequence has been interrupted by higher priority Estop.')
                    self.data_manager.set_ARM(0, blocking=True) #Disarm the controllerexecute_safe_land = True #This effectively breaks the while loop upon next loop through (that's too late, so we just break below)
                    break

                #Get the state information
                use_estimate = self.get_state_info()

                #Get setpoint info
                #if we are executing a safe land, we handle the setpoint ourselves
                #lower our setpoint by the prescribed velocity
                self.desired_state[0:3] = self.desired_state[0:3] - self.safe_descend_velocity*(self.dt)
                #if our setpoint is below the threshold, set it to the threshold
                if self.desired_state[2] <= self.landed_threshold:
                    self.desired_state[2] = self.landed_threshold
                controller_is_active = self.data_manager.get_controller_active()

                #If the controller is not active, then just assume we've landed.
                if not controller_is_active:
                    self.landed = True
                    self.bootloader_pipe_connection.send('The safe landing sequence has been successfully executed. - controller was never active.')
                    self.data_manager.set_ARM(0, blocking=True) #Disarm the controller
                    self.data_manager.set_safety(2) #initiate a normal stop
                
                #Check if we've landed (for safe land) and then initiate the board reset.
                if self.active_controller_key == 'safe land' and not self.landed:
                    if self.state[2] <= self.landed_threshold:
                        self.landed = True
                        self.bootloader_pipe_connection.send('The safe landing sequence has been successfully executed.')
                        self.data_manager.set_ARM(0, blocking=True) #Disarm the controller
                        self.data_manager.set_safety(2) #initiate a normal stop
                elif self.landed:
                    continue

                #Run the controller
                if controller_is_active and not self.landed:
                    written = self.run_error_checks_and_feed_controller(use_estimate)

                #Calculate loop duration to see if we need to wait or not
                self.loop_end = time.perf_counter_ns() #record the time the control loop finished in seconds * 10^9
                control_duration = self.loop_end - self.loop_start
                

                #If we have extra time (we finished the control loop before the ideal duration)
                #Wait here to keep the duration at the desired frequency.
                #Otherwise, loop again.
                # print(control_duration, self.controller_frequency)
                if control_duration < self.ideal_control_duration:
                    wait_time = self.ideal_control_duration - control_duration
                    t0 = time.perf_counter_ns()
                    # print('waiting')
                    time.sleep(0.5*wait_time/1000000000)
                    j = 0
                    while time.perf_counter_ns() - t0 < wait_time:
                        # print('too fast wait... %d' %j)
                        # print(time.perf_counter(), t0, wait_time, control_duration, self.ideal_control_duration)
                        j += 1
                        pass
            print('safe land over')
                

                



        except Exception as error:          

            #log the error and continue
            logMessage = ['an error occurred']
            trace = traceback.extract_tb(error.__traceback__)
            for t in trace:
                logMessage.append(str(t))
            logMessage.append([str(type(error).__name__)])
            logMessage.append([str(error)])
            self.bootloader_pipe_connection.send(logMessage) #send the error to the bootloader to be logged
            print(logMessage)

            self.bootloader_pipe_connection.send('Safety code 1 as control manager failed.')
            self.data_manager.set_safety(1) #we have no control over the robot, so we need to kill it.


        # while True:
        #     time.sleep(5) #wait here for someone to tell use to end or reset.


                


                

         


    def change_controllers(self, desired_controller_type): 
        #If it's a change (different from previous controller type), go to the new controller (and only if it's not type 'safe land')
        #DEVELOPERS NOTE: If the 'safe land' safety triggers, then we don't want to switch controllers 
            #In that case, we will just use the lqr controller and put the drone down safely.
        if desired_controller_type != self.active_controller_key and self.active_controller_key != 'safe land':
            active_controller_cmds = self.active_controller.commands #get the current commands from the active controller
            del self.active_controller
            self.active_controller_key = desired_controller_type
            if self.active_controller_key == 'lqr':
                self.active_controller = self.control_options[self.active_controller_key](self.Ks)
                self.active_controller.hardcode_cmds(active_controller_cmds) #initialize the new controller with the active controller commands
                #DEVELOPERS NOTE: If your robot is temporarily falling out of the sky when conotrollers are switched, its likely the commands (and integral terms) aren't transfered correctly.
            else:
                self.active_controller = self.control_options[self.active_controller_key]()
                self.active_controller.hardcode_cmds(active_controller_cmds) #initialize the new controller with the active controller commands
            self.bootloader_pipe_connection.send('switched to controller type: ' + self.active_controller_key)

        return
    
    def get_state_info(self):
        '''
        Attempts to get the state information from shared data.
        If successful, it saves it in self.state and updates the Kalman Filter.
        If not successful, it gets an estimate from the Kalman Filter.
        Returns use_estimate as True of Kalman filter used to get state, False if state is from optitrack.
        Returns a timing info list (if from optitrack) or an empty list if not
        '''
        #Get state information
        #If there is fresh state data available, we go get it and use that for control. 
        #If not, we use the kalman filter to estimate the state
        if self.data_manager.is_state_fresh():

            #Get the state from shared memory
            #DEVELOPERS NOTE: Since we are BLOCKING here, we don't have to worry about it returning FALSE
            self.state, timing_info = self.data_manager.get_state(clear_fresh_flag=True, blocking=True, get_timing_info=True)
                
            #update the kalman filter:
            self.kf.dt = self.dt
            measurement = np.array([self.state[0], self.state[1], self.state[2], self.state[6], self.state[7], self.state[8]])
            self.kf.update(measurement)
            self.kf.predict(self.active_controller.get_raw_cmds())
            self.assumed_state = self.kf.get_kf_state()
            # self.assumed_state = np.copy(self.state)
            use_estimate = False

        else:
            self.kf.predict(self.active_controller.get_raw_cmds())
            self.assumed_state = self.kf.get_kf_state()
            # self.assumed_state = np.copy(self.state)
            use_estimate = True
            timing_info = []

        return use_estimate, timing_info
    

    def run_error_checks_and_feed_controller(self, use_estimate):
        '''
        Sould be run only if a controller is active (ARMED)
        It checks for things like bounding box and roll over error, then it runs the active controller with either the actual state or assumed state based on 'use_estimate'

        Returns True if commands were written,
        FALSE otherwise.
        '''
        
        #Check bounding box. roll over and bounding box errors set there
        self._check_bounding_box()

        #Feed info into controller IFF no errors
        if not self.roll_over_error and not self.bounding_box_error:
            if not use_estimate: #FEED THE ACTUAL POSITION IF WE HAVE IT
                self.active_controller.run_controller(self.state, self.desired_state)
            else: #FEED THE ASSUMED POSITION OTHERWISE
                # self.active_controller.run_controller(self.assumed_state, self.desired_state)
                self.active_controller.run_controller(self.state, self.desired_state)
                #TODO make it so we actually use the estimate.

        elif self.roll_over_error:
            self.data_manager.set_ARM(0, blocking=True) #Disarm the controller
            self.data_manager.set_safety(1) #initiate an e-stop
            self.bootloader_pipe_connection.send('roll over error detected')
        elif self.bounding_box_error:
            self.data_manager.set_ARM(0, blocking=True) #Disarm the controller
            self.data_manager.set_safety(1) #initiate an e-stop
            self.bootloader_pipe_connection.send('bounding box error detected')

        #Write commands to the fc board
        written = self.data_manager.set_AETR_commands(self.active_controller.commands, blocking=False)

        return written
                 
        
        
            


    def exit(self):
        '''
        The code that runs immediately before this process is killed.
        '''
        pass

    def _check_bounding_box(self):
        '''
        Safety check to make sure robot is within bounds of bounding box.
        Returns nothing, but writes to error booleans
        '''
        box_dim = 1.0 #meters #TODO make this a parameter we read in from somewhere.

        #Buid the box around the current setpoint
        x_min = self.desired_state[0] - box_dim
        x_max = self.desired_state[0] + box_dim
        y_min = self.desired_state[1] - box_dim
        y_max = self.desired_state[1] + box_dim
        z_min = 0.0
        z_max = self.desired_state[2] + box_dim

        #Parse position and orientation data
        x = self.state[0]
        y = self.state[1]
        z = self.state[2]
        r = self.state[6]
        p = self.state[7]

        #check if roll or pitch are way out of wack.
        if abs(r) > 1.0 or abs(p) > 1.0: #in radians
            self.roll_over_error = True

        #check if OUT of the box
        if z < z_min or z > z_max or y < y_min or y > y_max or x < x_min and x > x_max:
            self.bounding_box_error = True

        return
    
    def user_code_kickoff(self):
        '''
        Executes when a user code is kicked off/started.
        Creates a csv file and a writer to the csv file.
        Also retrieves the global start time send in the 'start' message from the air traffic controller.
        '''
        timestamp = get_time_stamp()
        filename = 'ctrl_logs/control_' + timestamp + '.csv'
        self.bootloader_pipe_connection.send('starting a control log: ' + filename)
        self.csvfile = open(filename, 'w', newline='')
        self.writer = csv.writer(self.csvfile)

        #write first line
        self.writer.writerow(['New controller log.'])
        #Raw local time, robot local time, received time, received time since user code start, rec seq no, rec counter, time btwn msgs, ctrl freq, 
        #x, y, z, dx, dy, dz, 'phi', 'theta', 'psi', 'r', 'p', 'y','setx', 'sety', 'setz', 'u_roll', 'u_pitch', 'u_z', 'u_yaw', 'int_x', 'int_y', 'integral z', 'power', 'voltage', 'error'  
        self.writer.writerow(['raw local time', 'local time', 'recieved time', 'adj rec time', 'recieved seq. no.', 'cnt diff', 'time btwn msgs', 'ctrl freq',\
                               'x', 'y', 'z', 'dx', 'dy', 'dz', 'phi', 'theta', 'psi', 'r', 'p', 'y', 'setx', 'sety', 'setz','kfx', 'kfy', 'kfz', 'u_roll', 'u_pitch', 'u_z', 'u_yaw', 'int_x', 'int_y', 'integral z', 'power', 'voltage', 'error'])

        #get the start time of the user code
        self.user_code_global_start_time = np.round(self.data_manager.get_start_time(),3)
            





class Controller():
    '''
    Controller base class
    It is NOT abstract because the 'none' controller is of this type.
    Other controllers inherit from here.
    '''
    def __init__(self) -> None:
        '''
        Basically sets up variables for the controller
        '''
        
        #controls (regardless of how we get them)
        self.roll_u = 1500
        self.pitch_u = 1500
        self.z_u = 900
        self.yaw_u = 1500      


        #integral terms (some controllers may ignore or not use these, but all controllers have them so they can be passed from one controller to the other)
        #(Worst case, a zero is passed and the new controller is essentially starting over (i.e., re-initialized integral term))
        self.integral_x = 0
        self.integral_y = 0
        self.integral_z = 0
        self.integral_yaw = 0 

        self.raw_cmds = np.array([0, 0, 9.8, 0])



    def run_controller(self, state, desired_state):
        '''
        Input state: numpy arrray(x, y, z, xdot, ydot, zdot, roll, pitch, yaw, rolldot, pitchdot, yawdot)
        Input desired state: numpy arrray(x, y, z, xdot, ydot, zdot, roll, pitch, yaw, rolldot, pitchdot, yawdot)

        Output: AETR (roll, pitch, throttle, yaw)
        '''

        #THE ACTUAL CALCULATING OF roll_u, etc. IS DONE BY EACH SUBCLASS!!
        #The only thing that happens here is that the controls are made into integers to protect the FC board.
        #DEVELOPERS NOTE: This should be called AFTER the subclass run_controller code is done.

        self.roll_u = int(self.roll_u)
        self.pitch_u = int(self.pitch_u)
        self.z_u = int(self.z_u)
        self.yaw_u = int(self.yaw_u)

    def hardcode_cmds(self, cmds):
        '''
        cmds = np.array of commands in roll, pitch, z, yaw order
        sets roll, pitch, z and yaw to cmds values
        returns nothing
        '''
        self.roll_u = cmds[0]
        self.pitch_u = cmds[1]
        self.z_u = cmds[2]
        self.yaw_u = cmds[3]
        

    @property
    def commands(self):
        '''
        returns the commands as an array when called.
        '''
        return np.array([self.roll_u, self.pitch_u, self.z_u, self.yaw_u])
    
    #getters and setters for integral terms. Used when changing controllers
    def get_integral(self):
        return self.integral_x, self.integral_y, self.integral_z, self.integral_yaw

    def set_integral(self, x, y, z, yaw):
        self.integral_x = x
        self.integral_y = y
        self.integral_z = z
        self.integral_yaw = yaw

    def set_raw_cmds(self, a, e, t, r):
        self.raw_cmds[0] = a #roll
        self.raw_cmds[1] = e #pitch
        self.raw_cmds[2] = t #trottle
        self.raw_cmds[3] = r #yaw

    def get_raw_cmds(self):
        return self.raw_cmds
    

    
        
#TODO need integral term getters and setters for in-air controller switching.
class PID_Controller(Controller):
    '''
    Simple PID Controller Class (executes simple PID control in x, y, z, and yaw)
    '''
    def __init__(self):
        super().__init__()

        #GAINS
        self.KP_yaw = -140.0
        self.KI_yaw = -1.0
        self.KD_yaw = 0.001

        self.KP_roll = -135.0
        self.KI_roll = -0.50
        self.KD_roll = 135.0

        self.KP_pitch = -1*self.KP_roll
        self.KI_pitch = -1*self.KI_roll
        self.KD_pitch = -1*self.KD_roll

        self.KP_z = 250.0
        self.KI_z = 10.0
        self.KD_z = -250.0

        #for position control
        self.prev_error = np.array([0,0,0],dtype='d')
        self.xyz_integral = np.array([self.integral_x, self.integral_y, self.integral_z],dtype='d')
        self.integral_windup_limit = 500/self.KI_z #because baseline is 1500 and max is 2000 and 2000-1500 = 500
        self.z_baseline = 1500
        self.pitch_baseline = 1500
        self.roll_baseline = 1500
        self.filtered_derivative = np.zeros(3)
        self.alpha = 1.0 #1 is all new, 0 is all old. For exponential filter

        #for yaw control
        self.yaw_setpoint = 0.0
        self.yaw_integral = self.integral_yaw
        self.yaw_baseline = 1500

    #getters and setters for integral terms. Used when changing controllers
    def get_integral(self):
        return super().get_integral()
    
    def set_integral(self, x, y, z, yaw):
        super().set_integral(x, y, z, yaw)
        #turns the integral terms from the super class into local elements used in the PID controller.
        self.xyz_integral = np.array([self.integral_x, self.integral_y, self.integral_z],dtype='d')
        self.yaw_integral = self.integral_yaw



    def run_controller(self, state, desired_state):

        #break down the state into components
        setpoint = desired_state[0:3] #x, y, z
        local_xyz = state[0:3] #x, y, z
        derivative = desired_state[3:6] #xdot, ydot, zdot
        yaw_setpoint = desired_state[8]
        yaw = state[8]
        yaw_derivative = desired_state[11]




        #get the error
        position_error = setpoint - local_xyz

        #update the integral error
        self.xyz_integral += self.position_error

        if self.xyz_integral[2] > self.integral_windup_limit:
            self.xyz_integral[2] = self.integral_windup_limit

        elif self.xyz_integral[2] < -1 * self.integral_windup_limit:
            self.xyz_integral[2] = -1*self.integral_windup_limit

        #get derivative
        self.filtered_derivative = (self.alpha * derivative) + ((1-self.alpha)* self.filtered_derivative)
        derivative = np.copy(self.filtered_derivative)

        #find roll (y) control
        index = 1
        self.roll_u = self.KP_roll*position_error[index] + self.KI_roll*self.xyz_integral[index] + self.KD_roll*derivative[index] + self.roll_baseline

        #find pitch (x) control
        index = 0
        self.pitch_u = self.KP_pitch*position_error[index] + self.KI_pitch*self.xyz_integral[index] + self.KD_pitch*derivative[index] + self.pitch_baseline

        #find thrust (z) control
        index = 2
        self.z_u = self.KP_z*position_error[index] + self.KI_z*self.xyz_integral[index] + self.KD_z*derivative[index] + self.z_baseline

        #NOW DO YAW
        yaw_error = yaw_setpoint - yaw

        #update the integral error
        self.yaw_integral += yaw_error

        #get derivative
        derivative = yaw_derivative

        #find yaw 
        self.yaw_u = self.KP_yaw*yaw_error + self.KI_yaw*self.yaw_integral + self.KD_yaw*derivative + self.yaw_baseline

        #Run the superclass's function (which will protect the flight controller)
        super().run_controller(state, desired_state)

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
                    [   0, 0.5,    0,    0], # Penalty for roll effort
                    [   0,    0, 0.5,    0], # Penalty for pitch effort
                    [   0,    0,    0, 1.0]])# Penalty for yaw effort
    

    # Q matrix
    # The state cost matrix.
    # Experiment with different Q matrices.
    # Q helps us weigh the relative importance of each state in the 
    # state vector [x,y,z,u,v,w,phi,theta,psi,p,q,r]. 
    # Q is a square matrix that has the same number of rows as 
    # there are states.
    # Q penalizes bad performance.
    # Q has positive values al ong the diagonal and zeros elsewhere.
    # Q enables us to target states where we want low error by making the 
    # corresponding value of Q large.
    Q = np.diag((1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0))
    Q[0,0] = 2.0 #extra cost on x #1.5 
    Q[1,1] = 2.0 #extra cost on y #1.5
    Q[2,2] = 6.0 #extra cost on z #96
    Q[3,3] = 2.0 #extra cost on x velo #0.75
    Q[4,4] = 2.0 #extra cost on y velo #0.75
    Q[5,5] = 6.0 #extra cost on z velocity
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


class LQR_Controller(Controller):
    '''
    Class to implement LQR control of the robot (full system state feedback)
    '''
    def __init__(self, Ks):
        super().__init__()

        self.Ks = Ks
        # self.Ks_x = Ks[0]
        # self.Ks_y = Ks[1]
        # self.Ks_z = Ks[2]
        # self.Ks_yaw = Ks[3]

        self.Ksx1 = Ks[0][0,0]
        self.Ksx2 = Ks[0][0,1]
        self.Ksx3 = Ks[0][0,2]
        self.Ksx4 = Ks[0][0,3]

        self.Ksy1 = Ks[1][0,0]
        self.Ksy2 = Ks[1][0,1]
        self.Ksy3 = Ks[1][0,2]
        self.Ksy4 = Ks[1][0,3]

        self.Ksz1 = Ks[2][0,0]
        self.Ksz2 = Ks[2][0,1]

        self.Ksw1 = Ks[3][0,0]
        self.Ksw2 = Ks[3][0,1]



        #THINGS TO TUNE (OTHER THAN Q + R)
        self.planning_horizon = 1 #seconds
        self.max_velo_horizontal = 0.25 #m/s #gives max angle of about 60 deg. 0.25 gives max angle of about 30 deg.
        self.max_velo_vertical = 0.15 #m/s

        #HYBRID
        self.KI_z = 1
        self.KI_x = 0.75 #2.0 was 1.25
        self.KI_y = -0.75 #-2.0 was -1.25
        self.u_hover = 295

        self.lqr_alpha = 1 #0 = all old, 1 = all new
        self.filtered_pitch = 0.0
        self.filtered_roll = 0.0

    #getters and setters for integral terms. Used when changing controllers
    def get_integral(self):
        return super().get_integral()

    def set_integral(self, x, y, z, yaw):
        return super().set_integral(x, y, z, yaw)


    def run_controller(self, state, desired_state):

        # #adding in a maximum velocity in x,y, and z
        # dest = np.copy(desired_state[0:3]) #DESIRED X, Y, Z position
        # dist_to_dest = np.linalg.norm(state[0:3] - desired_state[0:3])
        # if dist_to_dest > 0:
        #     horizontal_dist_to_dest = np.linalg.norm(state[0:2] - desired_state[0:2])
        #     vertical_dsit_to_dest = abs(state[2] - desired_state[2])
        #     unit_vect_to_dest = (desired_state[0:3] - state[0:3])/dist_to_dest
        #     if self.planning_horizon*self.max_velo_horizontal < horizontal_dist_to_dest:
        #         dest[0:2] = state[0:2] + (self.planning_horizon*self.max_velo_horizontal)*unit_vect_to_dest[0:2]
        #     if self.planning_horizon*self.max_velo_vertical < vertical_dsit_to_dest:
        #         dest[2] = state[2] + (self.planning_horizon*self.max_velo_vertical)*unit_vect_to_dest[2]


        dest_yaw = desired_state[8] #DESIRED YAW ORIENTATION (almost always zero)

        error_x = desired_state[0] - state[0]
        error_y = desired_state[1] - state[1]
        error_z = desired_state[2] - state[2]


        #HYBRID - set up the integral commands (to compensate wind, dying batteries)
        self.integral_z += (error_z)
        # if abs(desired_state[0] - state[0]) <= 0.25:
        self.integral_x += (error_x)
        # if abs(desired_state[1] - state[1]) <= 0.25:
        self.integral_y += (error_y)
        u_z_integral = self.KI_z*self.integral_z
        u_x_integral = self.KI_x*self.integral_x
        u_y_integral = self.KI_y*self.integral_y

        
        # def find_commands(x):
        #     '''
        #     Get commands in x, y, z, and yaw given the state (x)
        #     AND destination in x,y,z and yaw. Roll, pitch, and ALL velocities assumed ZERO.
        #     '''
        #     UX = self.Ks[0].dot(np.array([dest[0], 0, 0, 0]) - x[[0, 3, 7, 10]])[0]
        #     UY = self.Ks[1].dot(np.array([dest[1], 0, 0, 0]) - x[[1, 4, 6, 9]])[0]
        #     UZ = self.Ks[2].dot(np.array([dest[2], 0]) - x[[2, 5]])[0]
        #     UYaw = self.Ks[3].dot(np.array([dest_yaw, 0]) - x[[8, 11]])[0]
        #     return [UZ, UY, UX, UYaw]
        
        
        # #Find the commands for the current state, using the LQR calculated Ks
        # lqr_commands = find_commands(state)

        '''
        Does the same thing as the function above but FASTER by not doing a dot product or multiplying things we know will be zero.
        '''
        UX = self.Ksx1*(error_x) - self.Ksx2*(state[3]) - self.Ksx3*(state[7]) - self.Ksx4*(state[10])
        UY = self.Ksy1*(error_y) - self.Ksy2*(state[4]) - self.Ksy3*(state[6]) - self.Ksy4*(state[9])
        UZ = self.Ksz1*(error_z) - self.Ksz2*(state[5])
        UYaw = self.Ksw1*(dest_yaw - state[8]) - self.Ksw2*(state[11])
        lqr_commands = [UZ, UY, UX, UYaw]

        #Convert commands into 1000-2000 range, centered at 1500
        converted_lqr_commands = (np.array([100,100,100,-100]) * np.array(lqr_commands)) + np.array([1500,1500,1500,1500])

        #CONTROL LIMITS (NOT applied to integral terms, so we can combat wind.)
        # delta = 25
        # roll_u = converted_lqr_commands[1]
        # pitch_u = converted_lqr_commands[2]
        # upper_limit = 1500 + delta
        # lower_limit = 1500 - delta
        # if roll_u > upper_limit:
        #     roll_u = upper_limit
        # elif roll_u < lower_limit:
        #     roll_u = lower_limit

        # if pitch_u > upper_limit:
        #     pitch_u = upper_limit
        # elif pitch_u < lower_limit:
        #     pitch_u = lower_limit

        #HYBRID - add integral terms to the commands
        # converted_lqr_commands[2] = pitch_u + u_x_integral
        # converted_lqr_commands[1] = roll_u + u_y_integral

        converted_lqr_commands[2] = converted_lqr_commands[2] + u_x_integral
        converted_lqr_commands[1] = converted_lqr_commands[1] + u_y_integral

        # #Filter the pitch and roll commands (May not be necessary any longer)
        # if self.filtered_pitch == 0.0:
        #     self.filtered_pitch = converted_lqr_commands[2]
        #     self.filtered_roll = converted_lqr_commands[1]
        # else:
        #     self.filtered_pitch = self.lqr_alpha*converted_lqr_commands[2] + (1-self.lqr_alpha)*self.filtered_pitch
        #     self.filtered_roll = self.lqr_alpha*converted_lqr_commands[1] + (1-self.lqr_alpha)*self.filtered_roll

        #Set zu, rollu, etc.
        self.z_u = converted_lqr_commands[0] + u_z_integral  + self.u_hover#T
        self.yaw_u = converted_lqr_commands[3] #R
        self.roll_u = converted_lqr_commands[1]
        self.pitch_u = converted_lqr_commands[2]

        #CONTROL LIMITS (NOT applied to integral terms, so we can combat wind.)
        delta = 100
        upper_limit = 1500 + delta
        lower_limit = 1500 - delta
        if self.roll_u > upper_limit:
            self.roll_u = upper_limit
        elif self.roll_u < lower_limit:
            self.roll_u = lower_limit

        if self.pitch_u > upper_limit:
            self.pitch_u = upper_limit
        elif self.pitch_u < lower_limit:
            self.pitch_u = lower_limit

        # self.roll_u = 1500
        # self.pitch_u = 1500
        # self.z_u = 900
        # self.yaw_u = 1500

        #set the raw commands (for the kalman filter)
        raw_commands = (np.array([self.roll_u, self.pitch_u, self.z_u, self.yaw_u]) - np.array([1500,1500,1500,1500])) / np.array([100,100,100,-100])
        self.set_raw_cmds(*raw_commands)

        super().run_controller(state, desired_state)

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


class KalmanFilter:
    def __init__(self, dt, mass):
        self.dt = dt  # Time step
        self.m = mass  # Drone mass
        
        # State vector: [x, y, z, xdot, ydot, zdot, roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate]
        # The first three rows update position based on velocity (xdot, ydot, zdot)
        # The next three rows keep velocity constant (assuming 0 acceleration for now)
        # The next 6 rows update angles based on angular velocites, keeping angular velocities constant.
        self.A = np.eye(12) + dt * np.array([
            [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        ])
        
        self.B = dt * np.array([
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [1/self.m, 0, 0, 0],  # Thrust influences z acceleration
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, dt, 0, 0], #roll, pitch, and yaw commands impact roll pitch and yaw velocities.
            [0, 0, dt, 0],
            [0, 0, 0, dt]
        ])
        
        self.H = np.zeros((6, 12)) #maps measured values (x,y,z, roll,pitch,yaw) to their corresponding state variables.
        self.H[0, 0] = 1  # x
        self.H[1, 1] = 1  # y
        self.H[2, 2] = 1  # z
        self.H[3, 6] = 1  # roll
        self.H[4, 7] = 1  # pitch
        self.H[5, 8] = 1  # yaw
        
        #noise covatiance (Q)
        #If Q is too small, the filter trusts the model too much and doesn't correct itself with measurements enough.
        #If Q is too big, the filter will be noisy and unstanble
        self.Q = np.eye(12) * 0.005 # Process noise

        #Measurement Covariance (R)
        #If R is too small, the filter assumes localization data is perfect, which might cause oscillations
        #IF R is too big, the filter ignores measurements too much, leading to drift
        self.R = np.eye(6) * 0.0005    # Measurement noise

        #Estimte Covariance (uncertainty in the real-world initial state. Higher = more uncertain)
        self.P = np.eye(12) * 0.1     # Estimate covariance
        self.x = np.zeros((12, 1))  # Initial state
    
    def predict(self, u):
        # roll, pitch = self.x[6, 0], self.x[7, 0]  # Extract current roll and pitch
        # print(roll, pitch)
        # self.B[3, 0] = np.sin(pitch) / self.m  # Update x acceleration due to pitch
        # self.B[4, 0] = -np.sin(roll) / self.m  # Update y acceleration due to roll
        # self.B[9,1] = self.dt
        # self.B[10,2] = self.dt
        # self.B[11,3] = self.dt

        self.A = np.eye(12) + self.dt * np.array([
            [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        ])
        
        self.x = self.A @ self.x + self.B @ u.reshape(-1, 1)  # Predict next state
        self.P = self.A @ self.P @ self.A.T + self.Q  # Update covariance matrix
    
    def update(self, z):
        y = z.reshape(-1, 1) - self.H @ self.x  # Compute measurement residual
        # print(y)
        S = self.H @ self.P @ self.H.T + self.R  # Compute residual covariance
        # print(S)
        K = self.P @ self.H.T @ np.linalg.inv(S)  # Compute Kalman gain
        # print(K)
        self.x = self.x + K @ y  # Update state estimate
        # print(self.x)
        self.P = (np.eye(12) - K @ self.H) @ self.P  # Update covariance estimate
        # print(self.P)
    
    def get_kf_state(self):
        """Returns the current estimated position, velocity, and attitude."""
        return self.x.flatten()
    
if __name__ == "__main__":
    print('control manager only')

    Ks = LQR_precalc()
    Ksx1 = Ks[0][0,0]
    Ksx2 = Ks[0][0,1]
    Ksx3 = Ks[0][0,2]
    Ksx4 = Ks[0][0,3]

    Ksy1 = Ks[1][0,0]
    Ksy2 = Ks[1][0,1]
    Ksy3 = Ks[1][0,2]
    Ksy4 = Ks[1][0,3]

    Ksz1 = Ks[2][0,0]
    Ksz2 = Ks[2][0,1]

    Ksw1 = Ks[3][0,0]
    Ksw2 = Ks[3][0,1]

    print("float Ksx1 = %f;" % Ksx1)
    print("float Ksx2 = %f;" % Ksx2)
    print("float Ksx3 = %f;" % Ksx3)
    print("float Ksx4 = %f;" % Ksx4)

    print("float Ksy1 = %f;" % Ksy1)
    print("float Ksy2 = %f;" % Ksy2)
    print("float Ksy3 = %f;" % Ksy3)
    print("float Ksy4 = %f;" % Ksy4)

    print("float Ksz1 = %f;" % Ksz1)
    print("float Ksz2 = %f;" % Ksz2)

    print("float Ksw1 = %f;" % Ksw1)
    print("float Ksw2 = %f;" % Ksw2)

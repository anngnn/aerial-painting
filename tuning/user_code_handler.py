'''
This code was developed by Andrew Curtis in the year 2024.
It was developed at Northwestern University, Evanston, IL, USA.

QuadSwarm flyers kick off their user code from here.
The interface between the user code and the rest of the os is also defined here.
'''

#import 3rd party libraries
#NONE

#import os files (files NU created)
from lib.shared_data_management import SharedDataManager, SharedData


#import native libraries
import time
import numpy as np
import struct
import signal
import os
import traceback
from logbook import get_time_stamp
import csv
import importlib

USER_CODE_FOLDER = 'codes'

class Flyer():
    '''
    Class that defines the flyer
    Serves as the interface between the user code and the rest of the operating system

    Any external accessible function can be called by the user code!
    '''
    def __init__(self, id, data_manager:SharedDataManager, writer, file) -> None:
        self.id = id
        self._writer = writer
        self._file = file

        self._data_manager = data_manager

        self._time_of_last_send = time.time() #in seconds
        self._time_between_sends = 0.1 #0.001s

        self.time_zero = self._data_manager.get_start_time()

    def state(self):
        '''
        Provides the state as a numpy array
        State is x, y, z, xdot, ydot, zdot, roll, pitch, yaw, rolldot, pitchdot, yawdot
        '''
        return self._data_manager.get_state(clear_fresh_flag=False,blocking=False)

    def waypoint(self, s):
        '''
        Sets the robots desired state
        State is x, y, z, xdot, ydot, zdot, roll, pitch, yaw, rolldot, pitchdot, yawdot
        '''
        self._data_manager.set_desired_state(s, blocking=True)
        return

    def select_controller(self, selection):
        '''
        Sets the robot to run with the selected controller
        '''
        self._data_manager.set_controller(selection)
        return

    def log(self, data):
        '''
        Allows a user to add a string message to the flight log
        '''
        try:
            self._writer.writerow(data)
            self._file.flush()
        except:
            self._writer.writerow('Error in data logging. Make sure data is a string or a list.')
            self._file.flush()
        return

    def delay(self, delay_time=0.2):
        '''
        Accepts a delay time in seconds. Flyer user code process will block for that time.
        '''
        time.sleep(delay_time)
        return
    
    def send_msg(self, msg):
        '''
        Attemps to transmit the given message returning whether it was successful

        Parameters:
        msg (str): The message to attempt to transmit. 
        '''
        #Check that it has been long enough to send again. If not, return False
        if time.time() - self._time_of_last_send < self._time_between_sends:
            return False
        else:
        
            #reset clock so we have an accurate time check next time
            self._time_of_last_send = time.time()

            #pack the message into the send buffer 
            self._data_manager.write_send_buffer(msg)

            #send the signal by triggering signal.SIGUSR1 at the send process's pid (process id)
            os.kill(self._data_manager.get_send_pid(), signal.SIGUSR1)
            return True

    def recv_msg(self):
        '''
        Reads the shared data read buffer and returns a list of messages or False
        '''
        return self._data_manager.get_read_buffer()
    
    def arm(self):
        '''
        Arms the flight controller. Props will begin spinning
        '''
        #self._data_manager.set_controller_active(0) #not activating the controller
        self._data_manager.set_ARM(1, blocking=True) #arm the controller
        self.delay()

    def run_controller(self):
        '''
        Runs the controller. Flyer moves
        '''
        self._data_manager.set_controller_active(1) #activating the controller
        #self._data_manager.set_ARM(1, blocking=True) #arm the controller
        self.delay()

    def disarm(self):
        '''
        disarms the flight controller. Props will stop spinning
        '''
        self._data_manager.set_controller_active(0) #let's the control manager know that the user initiated a disarm
        self._data_manager.set_ARM(0, blocking = True)
        self.delay()

    def mode(self, mode):
        '''
        Sets the flight controller to the user specified mode
        '''
        ret_val = self._data_manager.set_MODE(mode, blocking = True)
        self.delay()
        return ret_val
    
    def get_volts(self):
        '''
        returns the voltage of the battery as a floating point value
        '''
        ret_val = self._data_manager.get_battery_voltage()
        return ret_val
    
    def get_watts(self):
        '''
        returns the power (in watts) of the battery as a floating point value
        '''
        ret_val = self._data_manager.get_battery_power()
        return ret_val
    
    def get_lander(self):
        '''
        Returns the numpy array of the lander location in the same form as a state.
        '''
        xyz_yaw = self._data_manager.get_lander_pos(blocking=True) #this should be FAST (nothing else should be blocking this when we are running the user code.)
        lander_setpoint = np.zeros(12)
        lander_setpoint[0:3] = xyz_yaw[0:3]
        lander_setpoint[8] = xyz_yaw[3]
        return lander_setpoint
    
    # def time(self):
    #     '''
    #     returns the GLOBAL time (as estimated from received optitrack data) Since the user code started
    #     '''
    #     diff = self._data_manager.get_optitime_diff()
    #     global_time = time.time() + diff
    #     global_time = global_time - self.time_zero
    #     return 
    
    def global_time(self):
        '''
        returns the GLOBAL time (as estimated from received optitrack data in ccodeLINUX)
        '''
        g_time = self._data_manager.get_global_time()
        return g_time


class UserCodeHandler():
    '''
    Class to kick off the user code and manage its running
    '''
    def __init__(self, id, conn, user_code) -> None:
        self.id = id
        self.bootloader_pipe_connection = conn

        self.error = False
        
        #default user code is kept in the params.json file
        self.import_user_code(user_code)

    def import_user_code(self, user_code_file):

        try:

            #import the module sent to us by the user code file
            self.bootloader_pipe_connection.send('user code module about to be imported')
            user_code_file = USER_CODE_FOLDER + '.' + user_code_file
            self.bootloader_pipe_connection.send(user_code_file)
            self.module = importlib.import_module(user_code_file)
            self.bootloader_pipe_connection.send(str(self.module))
            self.bootloader_pipe_connection.send('user code module imported')
            
            #Reload it just to be safe (in fact, we NEED to do this if the board was just reset,
            #But it doesn't hurt to do it if we didn't just reset, so we do it all the time anyway)
            self.module = importlib.reload(self.module)
            self.bootloader_pipe_connection.send('module reloaded.')


            self.bootloader_pipe_connection.send('imported the user code as %s' %user_code_file)
            self.error = False
        except Exception as error:
            print(error)
            self.bootloader_pipe_connection.send('User code import error')
            self.bootloader_pipe_connection.send(error)
            self.error = True


    

    def run(self, shared_data:SharedData):
        '''
        User code is kicked off here
        '''

        if not self.error:
            self.data_manager = SharedDataManager(shared_data)

            try:
                self.bootloader_pipe_connection.send('Kicking off user code now.')

                #Let's get a new log set up for this user code run.
                timestamp = get_time_stamp()
                filename = 'usr_logs/' + timestamp + '-user_log.csv'
                with open(filename, 'w', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerow(['This is the start of a new user code run.', str(self.module)])

                    self.data_manager.set_user_code_running(1)
                    
                    #create a flyer object (it will be passed to the user code)
                    flyer = Flyer(self.id, self.data_manager, writer, csvfile)
                    
                    #run the user code
                    self.module.usr(flyer)
                    
            except Exception as error:
                
                #Log the error in the flight log
                logMessage = ['an error occurred']
                trace = traceback.extract_tb(error.__traceback__)
                for t in trace:
                    logMessage.append(str(t))
                logMessage.append([str(type(error).__name__)])
                logMessage.append([str(error)])
                self.bootloader_pipe_connection.send(logMessage) #send the error to the bootloader to be logged
                print(logMessage)

                # tell the system to auto-land because there was a bug in the user code
                self.data_manager.set_safety(3)
        
        #This is to handle the case where someone tried to start a user code file that didn't update correctly.
        else:
            self.bootloader_pipe_connection.send('User code was not properly loaded and will not kick off. Make sure the desired user code file exists in codes or that it is spelled correctly in params.json')

            
        #sleep until someone tells us to end or reset
        while True:
            self.bootloader_pipe_connection.send('user code handler sleeping')
            time.sleep(5)

    def exit(self):
        '''
        Code that runs when this process is killed
        '''
        if hasattr(self, 'module'):
            del self.module

        if hasattr(self, 'data_manager'):
            self.data_manager.set_user_code_running(0)

        

        


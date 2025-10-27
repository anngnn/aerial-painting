'''
This code was developed by Andrew Curtis in the year 2024.
It was developed at Northwestern University, Evanston, IL, USA.

This serves as the "home page" for each quadrotor in the QuadSwarm.
From here, all other quadrotor functionality is initiated, called, and terminated.

This module also maintains connection with the base station controller (i.e., air traffic control)
'''

#import 3rd party libraries
from statemachine import exceptions as stmex

#import os files (files NU created)
import c_interface
import fc_handler
import user_code_handler
import logbook
import comms
import lib.shared_data_management
import lib.bootloader_stm


#import native libraries
import time
import socket
import struct
import multiprocessing as mp
import traceback
import numpy as np
import selectors
import types
import gc
import json
from subprocess import check_output
import importlib
import csv
import subprocess
import os

MAX_TRYS = 1 #the number of times we will let main() try to run before we kill the bootloader completely.

BOT_MENU = ['start', 'stop', 'update', 'reset', 'shutdown', 'ping', 'logs', 'rm_logs', 'sleep', 'wake', 'estop'] #all of the commands that the user can send to the robots
BOT_HEX_MENU = [0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB]
IDLE = 0x00

LOGS_DEBOUNCE_THRESHOLD = 10 #seconds

def main(voltage_status):
    '''
    The main function of the bootloader.
    It runs automatically when the robot turns on, because of the service that was created.
    The service is described in bootloader.service.
    See google doc notes for details on how to configure the service to kick off automatically
    '''

    #start the logger. This is where we will log information about everything that runs in python (p2p comms, user code, c_interface, etc.)
    csvfile, writer = logbook.open_file()

    #indicate that a new instance of main has been run/started.
    print('new main')
    writer.writerow(['new main'])

    #DEVELOPERS NOTE: This is a return string. If anything other than 'normal' is returned from main(), then 
    #the operating system files will be reloaded before main() is run again.
    #This allows us to update other code files when bootloader is re-started.
    ret_val = 'normal' 
    


    #Set a time zero reference for all processes on the drone (not 'global' to the swarm, but 'global' to the drone only.)
    ROBOT_START_TIME = time.time()
    def now():
        '''
        Function to get the current time w.r.t robot's start time
        '''
        return format(np.round(time.time() - ROBOT_START_TIME,3), '0.3f')
    
    #get the ip address from ifconfig
    self_ip = ''
    ip_flag = False
    id = ''
    while not ip_flag:
        time.sleep(0.5)
        try:
            byte_ip = check_output(['hostname', '-I'])
            raw_ip = byte_ip.decode('utf-8')
            self_ip = raw_ip[:-2] #this should give us 192.168.18.### 
            if '192.168.18' in self_ip:
            # if '10.106.' in self_ip:
                ip_flag = True
        except:
            ip_flag = False
    id = int(self_ip[-2:]) #this should give us the last 2 digits of 192.168.18.102

    #log the ip address and ID of the robot. If these are incorrect, the rest of this is probably not going to go well.
    writer.writerow([self_ip, id])
    print(self_ip, id)
    writer.writerow(['If this far, then the following were successful:'])
    writer.writerow(['deleted MySharedMemory'])
    writer.writerow(['deleted execute'])
    writer.writerow(['GCC Compiled successfully'])
    csvfile.flush()

    

    #Initialize Shared Memory
    #Create an object to hold all of the MultiProcessing (MP) Data Items/Objects
    #The MP objects will be passed to each of the processes when the processes are kicked off.
    shared_memory_itmes = lib.shared_data_management.SharedData(id)
    data_manager = lib.shared_data_management.SharedDataManager(shared_memory_itmes)


    #start the logger.
    csvfile, writer = logbook.open_file()
    
   
    #start the logger.
    csvfile, writer = logbook.open_file()
    

    #DEVELOPERS NOTE: Try, except implemented here to make sure all processes get killed 
    try:

        json_data = get_json_data()

        #GET INPUT/CONFIG DATA from JSON DATA
        port = json_data['server_port']
        ip = json_data['server_ip']
        direct_port = json_data['atc_direct_port']
        direct_ip = json_data['atc_direct_ip']
        

        #Connect to Base Station via UDP
        writer.writerow([now(), 'bootloader', 'Starting connection to ' + str(ip)])
        baseComms = BaseStationCommunicator(ip, port, id, json_data['update_time'])
        #Log it
        writer.writerow([now(), 'bootloader', 'Connected to base station.'])
        csvfile.flush()

        #Set up a process manager that will manage the processes
        process_manager = ProcessManager(id, self_ip, json_data['localizer_ip'], json_data['localizer_port'], json_data['localizer_timeout'])
        process_manager.set_user_code(json_data['user_code']) #initialize the user code to that defined in the json file
        writer.writerow([now(), 'bootloader', 'user code initialized to: ' + str(process_manager.ucf)])
        writer.writerow([now(), 'bootloader', 'localizer is planning to connect to optitrack at: ' + str(json_data['localizer_ip'])])
        csvfile.flush()

        # #processes are created here when the state machine is established.
        stm = lib.bootloader_stm.BootloaderStateMachine(json_data['fc_watchdog'], direct_ip)
        
        # #execute the startup sequence of transitions thru states until we're at the idle state
        startup_sequence(stm, process_manager, shared_memory_itmes, writer, now())
        csvfile.flush()
        ROBOT_START_TIME = time.time() #align t=0 with start of all processes.
        data_manager.set_robot_start_time(ROBOT_START_TIME)
        print("SETTING ROBOT START TIME = ", ROBOT_START_TIME)
        print(data_manager.get_robot_start_time())
        writer.writerow([now(), 'bootloader', 'the following start times should match: %0.3f = %0.3f' % (ROBOT_START_TIME, data_manager.get_robot_start_time())])

        time_of_last_send = time.time()
        def calculate_spam():
            return 1*np.random.rand()+0.5
        no_spam = calculate_spam() #this means the drone will only reply to the air traffic controller fastest every 0.5s, max 1.5s, with some randomness

        send_to_atc = False
        last_received = IDLE

        #record the position of the landing station
        try:
            #pull the landing station string from the json data and convert it to an array of floats
            landerString = json_data['lander_position']
            lander_data = convert_landing_station_data(landerString)

            #write that array to shared memroy.
            data_manager.set_lander_pos(lander_data, blocking=True)

        except Exception as error:
            writer.writerow([now(), 'bootloader', 'there was an error setting up the landing station location.'])
            writer.writerow([str(error)])
            csvfile.flush()
            print(str(error))

        last_checksum_received = 0


        #Begin infinite loop
        while True:

            execute_state_machine = False

            #first, see if we should be sending something to the air traffic controller based on time
            if time.time() - time_of_last_send > no_spam:
                send_to_atc = True
            
            
            #now, look for any incoming messages from the air traaffic controller, and handle them.
            try:
           
                #Get a packet of info from the socket
                info = baseComms._recv_socket.recvfrom(1024)

                #attempt to unpack it
                message = comms.ATCMessage(info[0], last_checksum_received)

                #if it's valid message, process it and prep flags for future actions (replying to atc, passing comamnd to state machine, etc.)
                if message.is_valid():
                    print('received valid message type: %02x' % (message.message_type))
                    print('sending a reply...')
                    send_to_atc = True
                    last_received = message.message_type
                    last_checksum_received = message.checksum
                    rec_command = BOT_MENU[BOT_HEX_MENU.index(message.message_type)] #convert the Hex message type to a string.
                    #Log the received message
                    writer.writerow([now(), 'bootloader', 'Base station message received: ' + rec_command + ' == %02x' % (message.message_type)])
                    csvfile.flush()

                    #set safety if we are going to be busy doing other things.
                    if rec_command in ['logs', 'rm_logs']:
                        if rec_command == 'logs':
                            data_manager.set_safety(6) #lets the user know we're fetching logs
                        if rec_command == 'rm_logs':
                            data_manager.set_safety(7) #lets the user know we're clearing logs

                    execute_state_machine = True

                    #record the start time if we got a start message
                    if rec_command == 'start':
                        print('setting start time to: ' + str(message.payload))
                        writer.writerow([now(), 'bootloader', 'setting start time to: ' + str(message.payload)])
                        data_manager.set_start_time(message.payload)

                    #if we got an update message, we update the user code!
                    if rec_command == 'update':
                        process_manager.set_user_code(message.payload)
                    del message
                    csvfile.flush()
                    
            except socket.timeout:
                pass
            
            


            #send messages to the air traffic controller if send_to_atc flag set (either by receipt or time lapse)
            if send_to_atc:

                #check if we need to declare a low voltage!
                lv = False
                low_voltage_warning = data_manager.get_low_voltage_warning()
                if low_voltage_warning or voltage_status == 'low': #append LV for 'LOW VOLTAGE' to the string of the state id IFF the FC handler had detected a low voltage and set the warning.
                    lv = True
                    voltage_status = 'low'
                state_string = stm.current_state.id


                #prep our payload here. If we got an 'update' or IDLE, our playload is the user code file saved in the process_manager
                if last_received == BOT_HEX_MENU[BOT_MENU.index('update')] or last_received == IDLE:
                    payload = [process_manager.ucf.encode('utf-8')]
                else:
                    payload = []


                #pack up the reply to the atc
                reply = comms.pack_atc_reply(state_string, last_received, data_manager, payload=payload, low_voltage=lv)

                #send it out
                baseComms._send_socket.sendto(reply, (direct_ip, direct_port))

                #mark the time of last send, identify how long until we send again (no_spam) and set flag so we don't send next time thru.
                time_of_last_send = time.time()
                no_spam = calculate_spam()
                send_to_atc = False

            #if we are currently sleeping and the message received is != wake, then we need to just loop until we get a 'wake' message
            #DEVELOPERS NOTE: This is done AFTER the send to atc so we still send messages while we are sleeping. That way we know who is sleeping and who is lost.
            if stm.current_state.name == 'sleep' and rec_command != 'wake':
                ret_val = 'reload'
                continue

            #TODO we need a COMMS UP check with the base station. This is the old one that won't work now that we've gone to UDP.      
            # if abort: #This is not an automatic abort. We will try to safely land here if we can.
            #     baseComms.comms_up = False #we know comms are down at this point, so we need to be explicit so fire_socket_event won't error next time thru loop.
            #     if data_manager.set_safety(3):
            #         #if this returns true, then we wrote 3 and auto-land should commence.
            #         writer.writerow([now(),'bootloader', 'set safety to 3 since the base station comms are down.'])
            #         csvfile.flush()
                    
            #         #DEVELOPERS NOTE:   We write safety to 3 first, so if everything was already up and running, the safety land will take over as soon as possible.
            #         #                   However, there are some cases where we will just want to restart the bootloader main() (e.g., the fc board is not connected)
            #         #                   These cases are handled here.

            #         #Case where the FC board is not connected
            #         if not data_manager.get_board_connected():
            #             try: #try to shutdown via the state machine. If that fails, just break the loop here.
            #                 abort = send_message_to_state_machine('shutdown', stm, process_manager, shared_memory_itmes, data_manager, writer, csvfile, now())
            #                 break
            #             except:
            #                 break
            #         # continue
                    
            #     else:
            #         #If this returns False, then there is a higher magnitude safety code and we should abort
            #         writer.writerow([now(),'bootloader', 'Base station comms are down. Could not set safety to 3 since another process set a higher priority safety error.'])
            #         csvfile.flush()
            #         break
            #END OF TODO
     

            #Execute the state machine
            #if we command something that isn't allowed per the state machine, we just ignore it and move on.
            if execute_state_machine:
                abort = False
                try:
                    abort = send_message_to_state_machine(rec_command, stm, process_manager, shared_memory_itmes, data_manager, writer, csvfile, now())
                    #we need to create a new log if we just removed the logs.
                    if rec_command == 'rm_logs':
                        csvfile, writer = logbook.open_file()
                        writer.writerow([now(),'bootloader', 'new log because old logs were cleared.'])
                        csvfile.flush()

                    #DEVELOPERS NOTE: It doesn't really hurt to reload (I think), so we reload on reset too (prevents having to send sleep and wake)
                    if rec_command == 'reset':
                        ret_val = 'reload'

                    if abort:
                        break
                except stmex.TransitionNotAllowed:
                    writer.writerow([now(),'bootloader', 'invalid statemachine transition ignored.'])
                    continue #Case of a transition/event that is sent to the state machine even though the state machine won't accept it
                            #This typically occurs if a transition from state B to state C is called, but the system is in state A.
            
            abort = False
            #Do the following if processes are running.
            if stm.current_state.id in ['idle', 'running']:
                abort = stm.check(process_manager, csvfile, writer, now())
                # writer.writerow([t, 'bootloader', 'state machine "check" complete.'])
            
            if abort:
                try: #try to shutdown via the state machine. If that fails, just break the loop here.
                    abort = send_message_to_state_machine('shutdown', stm, process_manager, shared_memory_itmes, data_manager, writer, csvfile, now())
                    break
                except:
                    break
                
            

            time.sleep(0.05) #there is no reason this needs to operate quickly. 20Hz is plenty.

        #if we aborted for any reason, check to make sure we return the coorect voltage status
        try:
            low_voltage_warning = data_manager.get_low_voltage_warning()
            if low_voltage_warning:
                voltage_status = 'low'
        except:
            pass

                
    

    except KeyboardInterrupt:
        writer.writerow([now(), 'bootloader', 'Keyboard interrupt'])
        csvfile.flush()
    except Exception as error:
        writer.writerow([now(), 'bootloader', 'An error occurred.'])
        csvfile.flush()
        trace = traceback.extract_tb(error.__traceback__)
        for t in trace:
            writer.writerow([str(t)])
            print(t)
        csvfile.flush()
        writer.writerow([str(type(error).__name__)])
        print(str(type(error).__name__))
        writer.writerow([str(error)])
        print(error)
        csvfile.flush()
    finally:
        writer.writerow([now(), 'bootloader', 'Initiating final shutdown sequence.'])
        # process_manager.kill_all()
        baseComms.kill()
        writer.writerow([now(), 'bootloader', 'All processes killed. All comms killed. Ending bootloader main.'])
        csvfile.flush()
        csvfile.close()
    return ret_val, voltage_status

def startup_sequence(stm, process_manager, shared_memory_itmes, writer, t):
    '''
    Moves the state machine thru the first few states (from intitialstate to idle state)
    '''
    process_manager.startup()


    writer.writerow([t, 'bootloader', 'Attempting to make processes. Localizer will attempt to connect to optitrack'])
    stm.create(process_manager, shared_memory_itmes)
    writer.writerow([t, 'bootloader', 'all processes created.'])

    #Do the preflight checklist
    #in other words, get the processes up and running.
    stm.do_preflight_checks(process_manager)
    #Log it
    writer.writerow([t, 'bootloader', 'all processes started.'])

    #move to the idle state
    stm.checks_complete()
    print('new startup')
    writer.writerow([t, 'bootloader', 'new startup complete.'])

def get_json_data():
    '''
    Returns all data in the 'params.json' file
    '''
    with open('params.json', 'r') as file:
        data = json.load(file)

    return data

def write_user_code_to_json(user_code):
    '''
    accepts the string name of the user code
    writes it to the json file
    '''
    json_data = get_json_data()
    json_data['user_code'] = user_code
    json.dump(json_data,(open('params.json','w')))



def send_message_to_state_machine(message, stm:lib.bootloader_stm.BootloaderStateMachine,\
                                   process_manager, shared_data, data_manager:lib.shared_data_management.SharedDataManager,\
                                     writer, csvfile, t):
    '''
    Converts a text message from the base station computer to an event call that controls the state machine
    Returns 'True' if the main() code should be aborted.
    '''
    safety = data_manager.get_safety()

    if message:

        #if the message is 'start', then we kick off the user code. and log it.
        #DEVELOPERS NOTE: by logging AFTER we send the event to the state machine, we guarantee that 
        #we will ONLY log it if its a success. If it's a fail, the TransitionNotAllowed exception will catch it
        #this way, we are only logging the 1st instance of the message/event that triggers the transition.
        if message == 'start' and safety == 0:
            writer.writerow([t, 'bootloader', 'calling "start" as there are no safety issues.'])
            stm.start(process_manager)
            if stm.user_code_running:
                writer.writerow([t, 'bootloader', 'user code started'])
            else:
                writer.writerow([t, 'bootloader', 'user code failed to start. Reasons unknown.'])
            csvfile.flush()
        elif message == 'start' and safety != 0:
            writer.writerow([t, 'bootloader', 'not starting because safety is ' , str(safety)])
            csvfile.flush()
        
        elif message == 'stop':
            data_manager.set_safety(3) #turn on auto-land when commanding user code to stop
            stm.stop(process_manager)
            writer.writerow([t, 'bootloader', 'stopping the user code'])
            data_manager.set_user_code_running(0)
            csvfile.flush()
        
        elif message == 'update':
            writer.writerow([t, 'bootloader', 'updating user code'])
            csvfile.flush()
            stm.update_user_code(process_manager)
            if stm.user_code_updated:
                writer.writerow([t, 'bootloader', 'user code update successful.'])
            else: 
                writer.writerow([t, 'bootloader', 'Trying to update user code but the file is "none" OR system needs to be reset'])
            csvfile.flush()

        elif message == 'reset' and safety != 5:  
            #DEVELOPERS NOTE: we can't try to reset if the board is in the Xs waiting period where it is trying to send disarm only. That's why we need the safety value to NOT be 5.

            data_manager.set_safety(2) #Set the safety value to 2 to reset the board
            #DEVELOPERS NOTE: the board MAY NOT BE ON. If it had been flying and it landed due to a 'stop',
            #The board may have reset itself. In that case, nothing will react to the safety and we will basically
            #just wait here for an unecessary 1s. I figured this wasn't a problem.
            writer.writerow([t, 'bootloader', 'resetting the state machine'])
            csvfile.flush()
            time.sleep(3) #give the board a second to shut down
            stm.reset(process_manager) #this kills all processes

            #Reset the safety value
            if data_manager.get_safety() == 5: #if safety is 5, then the board must be connected and it was set to 5 when the reset occurred on the board's end.
                pass
            else:
                data_manager.set_safety(0, override=True) 
            #DEVELOPERS NOTE: This is really important. If we don't do this, then we run the board will likely shutdown as soon as its kicked off again
            data_manager.set_ARM(0) #disarm so when we reset we don't automatically arm again
            data_manager.set_MODE(1500) #reset mode to default
            
            
            #collect garbage
            gc.collect()

            #Start up all the processes again.
            startup_sequence(stm, process_manager, shared_data, writer, t)

            process_manager.reset_user_code()

        elif message == 'reset' and safety == 5:
            writer.writerow([t, 'bootloader', 'ignoring reset command. FC is not done initializing.'])

        elif message == 'estop':
            data_manager.set_safety(1) #Set the safety value to 1 to reset the board
            writer.writerow([t, 'bootloader', 'emergency stop has been called. waiting 1s before stop/reset'])
            csvfile.flush()
            time.sleep(1)
            writer.writerow([t, 'bootloader', 'shutting down the state machine to reset after estop'])
            stm.estop(process_manager)
            writer.writerow([t, 'bootloader', 'all processes should be killed'])
            data_manager.set_user_code_running(0)
            csvfile.flush()
            


            #Reset the safety value
            if data_manager.get_safety() == 5: #if safety is 5, then the board must be connected and it was set to 5 when the reset occurred on the board's end.
                pass
            else:
                data_manager.set_safety(0, override=True) 
            #DEVELOPERS NOTE: This is really important. If we don't do this, then we run the board will likely shutdown as soon as its kicked off again
            data_manager.set_ARM(0) #disarm so when we reset we don't automatically arm again
            data_manager.set_MODE(1500) #reset mode to default
            
            
            #collect garbage
            gc.collect()

            return True
            

        elif message == 'shutdown':
            writer.writerow([t, 'bootloader', 'shutting down the state machine'])
            stm.shutdown(process_manager)
            writer.writerow([t, 'bootloader', 'all processes should be killed'])
            csvfile.flush()
            time.sleep(2) #wait a bit for everything to shut down correctly.
            return True
        
        elif message == 'logs':
            writer.writerow([t, 'bootloader', 'fetching the logs'])
            stm.fetch(process_manager.robot_id, writer, csvfile, t) #call the transition from idle to fetch ()
            #The logs should be sent once the transition completes and the entry behavior is performed.
            #so, we can now go back to the idle state
            stm.done()
            writer.writerow([t, 'bootloader', 'log fetch successful'])
            csvfile.flush()
            data_manager.set_safety(0, override=True) #let the user know we're back to ready

        elif message == 'rm_logs':
            writer.writerow([t, 'bootloader', 'removing the logs and killing this log.'])
            csvfile.flush()
            csvfile.close()

            stm.remove_logs()
            #The logs should be removed once the transition completes and the entry behavior is performed.
            #so, we can now go back to the idle state
            stm.done()
            data_manager.set_safety(0, override=True) #let the user know we're back to ready

        elif message == 'sleep':
            data_manager.set_safety(2) #Set the safety value to 2 to reset the board
            #DEVELOPERS NOTE: the board MAY NOT BE ON. If it had been flying and it landed due to a 'stop',
            #The board may have reset itself. In that case, nothing will react to the safety and we will basically
            #just wait here for an unecessary 1s. I figured this wasn't a problem.
            writer.writerow([t, 'bootloader', 'resetting the fc board in anticipation of a sleep'])
            csvfile.flush()
            time.sleep(3) #give the board a second to shut down
            stm.go_to_sleep(process_manager) #this kills all processes
            writer.writerow([t, 'bootloader', 'going to sleep.'])
            csvfile.flush()

        elif message == 'wake':
            stm.wake(process_manager)
            writer.writerow([t, 'bootloader', 'all processes should be killed'])
            writer.writerow([t, 'bootloader', 'I am going to kill main() and wake back up shortly.'])
            csvfile.flush()
            return True


    


    return False




    



class ProcessManager():
    '''
    Class to manage the processes (keep track of them, start them, kill them, etc.)
    '''
    def __init__(self, id, ip, localizer_ip, localizer_port, localizer_timeout) -> None:

        #unique id of this robot
        self.robot_id = id

        self.ucf = 'none' #default to not having a user code file

        self.robot_ip_addr = ip

        self.localizer_ip = localizer_ip
        self.localizer_port = localizer_port
        self.localizer_timeout = localizer_timeout


    def set_user_code(self, user_code):
        '''
        accepts the name of the user code file and stores it for future use
        '''
        self.ucf = user_code
    
    def startup(self):
        #Processes will be stored in this list
        self._processes = []

        #Objects of the classes running the processes will be stored in this dictionary
        self.objects = {} #DEVELOPERS NOTE: This dict allows us to access the object running each process by keying the process name

        self.pipes = {} #dictonary of pipe connections to listen through during logging.

    def make_processes(self, shared_data):
        '''
        Create an object and then a process for each class in the name list below.
        DEVELOPERS NOTE: If you want to kick of MORE processes (or fewer), then you need to edit his list.
        Name list is just the names we use for each process (unique ID)
        '''
        name_list = ['locaroller', 'c_handler', 'user code handler', 'sender', 'receiver']

        for i,name in enumerate(name_list):

            #Create the LOCAROLLER object and corresponding process.
            if name == 'locaroller':
                #Pipe connection
                temp, locaroller_conn = mp.Pipe() #create a pipe connection
                #DEVELOPERS NOTE: parent, child = Pipe() from here: https://docs.python.org/3/library/multiprocessing.html
                obj = c_interface.Locaroller(self.robot_id, locaroller_conn, standalone = False) #create the object associated w. the process
                self._make_process(obj.run, name, shared_data) #create the process w. the shared data (process will be stored)
                self.objects[name] = obj #store the object 
                self.pipes[name] = temp #store the pipe connection

            #THIS HAS BEEN MADE OBSOLETE BY THE C CODE
            # #Create the FLIGHT CONTROL HANDLER object and corresponding process.
            # elif name == 'fc handler':
            #     #Pipe connection
            #     temp, fc_conn = mp.Pipe() #create a pipe connection
            #     obj = fc_handler.FlightControllerInterface(self.robot_id, fc_conn) #create the object associated w. the process
            #     self._make_process(obj.run, name, shared_data) #create the process w. the shared data (process will be stored)
            #     self.objects[name] = obj #store the object
            #     self.pipes[name] = temp #store the pipe connection

            #Create the CONTROL MANAGER object and corresponding process.
            elif name == 'c_handler':
                time.sleep(0.25) #we want to sleep a little here so that the locaroller has had time to initialize, solve the LQR equations, and share the KS matrix with the c code.
                #Pipe connection
                temp, cm_conn = mp.Pipe() #create a pipe connection
                localizer_timeout_sec = int(self.localizer_timeout)
                localizer_timeout_nsec = int((self.localizer_timeout - int(self.localizer_timeout))*1000000000)
                obj = c_interface.C_Handler(self.robot_id, cm_conn, self.localizer_ip, self.localizer_port, localizer_timeout_sec, localizer_timeout_nsec, standalone=False) #create the object associated w. the process
                self._make_process(obj.run, name, shared_data) #create the process w. the shared data (process will be stored)
                self.objects[name] = obj #store the object
                self.pipes[name] = temp #store the pipe connection

            #Create the USER CODE HANDLER object and corresponding process.
            elif name == 'user code handler':
                #Pipe connection
                temp, uc_conn = mp.Pipe() #create a pipe connection
                obj = user_code_handler.UserCodeHandler(self.robot_id, uc_conn, self.ucf) #create the object associated w. the process
                proc = self._make_process(obj.run, name, shared_data) #create the process w. the shared data (process will be stored)
                self.objects[name] = obj #store the object
                self.pipes[name] = temp #store the pipe connection
                self.user_code_process = proc #store the user code process so it can be accessed directly

            #Create the MESSENGER objects and corresponding processes.
            elif name == 'sender':
                temp_send, send_conn = mp.Pipe() #create a pipe connection
                obj = comms.Sender(self.robot_id, send_conn, self.robot_ip_addr) #create the object associated w. the process
                self._make_process(obj.run, name, shared_data) #create the process w. the shared data (process will be stored)
                self.objects[name] = obj #store the object
                self.pipes[name] = temp_send #store the pipe connection
            elif name == 'receiver':
                temp_rec, rec_conn = mp.Pipe() #create a pipe connection
                obj = comms.Receiver(self.robot_id, rec_conn, self.robot_ip_addr) #create the object associated w. the process
                self._make_process(obj.run, name, shared_data) #create the process w. the shared data (process will be stored)
                self.objects[name] = obj #store the object
                self.pipes[name] = temp_rec #store the pipe connection
        return


    def start_all(self):
        '''
        kick off ALL processes EXCEPT user code
        '''
        for p in self._processes:
            if p.name == 'user code handler':
                continue
            p.start()
        return
    
    def kick_off_user_code(self):
        '''
        Starts the user code process IFF the process exists
        '''
        if hasattr(self, 'user_code_process'):
            self.user_code_process.start()
            return True
        else:
            return False

    def kill_all(self):
        '''
        kill ALL active processes
        '''

        self._exit_all() #Run the exit behavior for each process

        #Close any open pipes to the processes
        for key in self.pipes:
            p = self.pipes[key]
            p.close()       

        #kill the active processes
        for p in self._processes:
            if p.is_alive():
                p.terminate()
                p.join()
    
        self._processes = []
        self.pipes = {}
        return
    
    def kill_process(self, name):
        '''
        Kill a specific process (given by 'name')
        '''
        for p in self._processes:
            if p.name == name:
                if p.is_alive():
                    p.terminate()

        #'trying to remove the user code handler object')
        obj = self.objects[name]
        obj.exit()
        del self.objects[name]

        #Trying to remove the associated pipe')
        del self.pipes[name]
    

        

    def _exit_all(self):
        '''
        Run any exit behavior for those objects
        '''
        for key in self.objects:
            obj = self.objects[key]
            obj.exit()

        self.objects = {}

    def poll_processes(self):
        '''
        Listen to each pipe and pull any data together, then return it as a list

        DEVELOPERS NOTE: If you receive a EOFError when polling or at p.recv(), its probably because the
        pipe connection has been destroyed. MAKE SURE YOUR PROCESSES DO NOT PREMATURELY TERMINATE.
        I.e., you have to have them at least sleep in a while loop until the bootloader can kill them and
        terminate its pipe connection.
        ALSO, be sure the correct pipe is being passed to each process. If a pipe connection isn't passed
        to an object when the pipes and processes are created, it will be killed.
        ''' 
        data = {}   
        for key in self.pipes:
            p = self.pipes[key]
            data_there = p.poll(0.1) #built in pipe function. blocks for 0.1s or if there is something to receive
            if data_there:
                msg = p.recv()
                data[key] = msg
            else:
                continue
        if data == {}:
            return False
        else:
            return data

    #Make the process and return it to the caller (whoever called this function)
    #also add it to the _processes list
    def _make_process(self, target_function, process_name, shared_data):
        #ONLY make a process if it doesn't already exist in the list.
        #If id DOES exist already, kill it and then return False
        p = mp.Process(target=target_function, name=process_name, args=(shared_data,))
        if p not in self._processes:
            self._processes.append(p)
            return p
        else:
            del p
            return False
        
    def check_processes(self):
        '''
        Return True if any of the processes in the list below are dead.
        Return True, 'name of process'
        Otherwise, return False, ''
        '''
        check_list = ['c_handler', 'locaroller']
        for p in self._processes:
            if p.name in check_list:
                if not p.is_alive():
                    return True, p.name   
        return False, ''
    
    def update_user_code(self):
        '''
        Tells the user code process to update the user code
        '''
        if self.ucf != 'none' and 'user code handler' in self.objects.keys():
            user_code_object = self.objects['user code handler']
            user_code_object.import_user_code(self.ucf)
            
            #write to the json file so this is the user code we kick off from now on.
            write_user_code_to_json(self.ucf)
            return True
        else:
            return False
        
    def reset_user_code(self):
        '''
        re-imports the user code from the json file (called during 'reset')
        '''
        data = get_json_data()
        self.set_user_code(data['user_code'])
        if self.ucf != 'none' and 'user code handler' in self.objects.keys():
            user_code_object = self.objects['user code handler']
            user_code_object.import_user_code(self.ucf)
                






class BaseStationCommunicator():
    '''
    The entity responsible for handling communication with the base station computer
    '''
    def __init__(self, ip, port, id, update_time) -> None:
        '''
        Set up all of the details of the socket, so we can both talk and recv from the base station puter
        '''
        self.id = id #store the id locally

        #stores the last command we received 
        self.last_command = 'idle'

        self.response = 'required'
        self.last_command_received = ''
        self.last_command_sent = ''

        #Constants - hard coded for now, but should be made parameters later #TODO
        self.event_timeout = 1 #time in s 

        #Port on the pi/robot. IP address of the base station computer
        self.ip = ip
        self.port = port
        
        #This creates a UDP socket 
        self._recv_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        #This allows the socket address to be reused (which I think helps with the sender socket above)
        self._recv_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        #We're going to listen to anything coming over the port, regardless of IP address. So we bind just to the port, empty address
        self._recv_socket.bind(('', self.port))
        #We need to join the multicast group as a listener.
        self.mreq = struct.pack('4sl', socket.inet_aton(self.ip), socket.INADDR_ANY)
        self._recv_socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, self.mreq)
        #slightly increase the buffer to hanlde burst traffic more smoothly
        self._recv_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4096)
        #make this blocking for 0.1s (so receiver runs at 10Hz.)
        self._recv_socket.settimeout(0.1)
        #make it so we can't hear our own messages
        self._recv_socket.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 0)
        

        self.comms_up = True

        self.first_send = False

        self.time_last_sent = time.time()
        self.update_time = update_time #how frequently we send a heartbeat message to the server.

        self.response_required = True


        #Create a UDP socket for sending data BACK to the ATC
        self._send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

        #Allow the socket address to be reused
        self._send_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        self.time_of_last_log = 0
        self.time_of_last_rm_log = 0
       
    
    def kill(self):
        '''
        kill the client socket so that the resouce can be freed up for next time.
        DEVELOPERS NOTE: If you get a BlockingIOError [Errno 11] Resource temporarily unavilable, then the socket
        probably didn't close the last time. OR Theres a blocking error and data wasn't available, so it timed out.
        '''
        try:
            self._recv_socket.set
            self._recv_socket.setsockopt(socket.IPPROTO_IP, socket.IP_DROP_MEMBERSHIP, self.mreq)
            self._recv_socket.close()
            self._send_socket.close()
        except:
            pass

        return
    
    def check_log_debounce(self):
        ''' 
        check to see  if enough time has passed that we can react to a log message again.
        Returns TRUE if enough time has passed, FALSE otherwise.
        if TRUE, it sets the time of last log receipt to current time (in seconds)
        '''
        if time.time() - self.time_of_last_log > LOGS_DEBOUNCE_THRESHOLD*6:
            self.time_of_last_log = time.time()
            return True
        else:
            return False
        
    def check_rmlog_debounce(self):
        ''' 
        Same as above, but for rm_log message
        '''
        if time.time() - self.time_of_last_rm_log > LOGS_DEBOUNCE_THRESHOLD:
            self.time_of_last_rm_log = time.time()
            return True
        else:
            return False
    

def convert_landing_station_data(inputString):
    '''
    Takes a string of input data from the params.json file and returns a numpy array of landing position data
    '''
    ret_val = np.zeros(6, dtype=float)

    split_string = inputString.split(",")
    for i,item in enumerate(split_string):
        ret_val[i] = float(item)

    return ret_val



if __name__ == "__main__":

    run_while = False

    try:
        #start with a clean log
        # if len(os.listdir('logs')) > 0:
        #     os.system('sudo rm logs/*')

        #make sure we have all the right folders for logs
        os.makedirs("ctrl_logs", exist_ok=True)
        os.makedirs("clogs", exist_ok=True)
        os.makedirs("logs",exist_ok=True)
        os.makedirs("usr_logs",exist_ok=True)

        #start with clean shared memory.
        file_path = "MySharedMemory" # Replace with the actual file path
        if os.path.exists(file_path):
            os.system('sudo rm ' + file_path)
            print(f"File '{file_path}' deleted successfully.")

        #start with clean c code
        file_path = "execute" # Replace with the actual file path
        if os.path.exists(file_path):
            os.system('sudo rm ' + file_path)
            print(f"File '{file_path}' deleted successfully.")

        #compile the c code
        c_file = "ccodeLINUX.c"
        executable_name = "execute"
        compile_command = ["gcc", c_file, "control_helper.c",  "myserial.c", "-lm", "-o", executable_name]
        result = subprocess.run(compile_command, capture_output=True, text=True)

        if result.returncode != 0:
            print('THERE WAS AN ERROR DOING THE GCC COMPILE')
            print(result.stderr)
            with open('logs/emergency.csv', 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['an error occurred when trying to compile the c code.'])
                writer.writerow([str(result.stderr)])
                csvfile.flush()
            run_while = False
        else:
            print('gcc compiled successfully.')

            run_while = True
    
    except Exception as error:
        with open('logs/emergency.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['an error occurred when trying to reload modules.'])
            writer.flush()
            trace = traceback.extract_tb(error.__traceback__)
            logMessage = []
            for t in trace:
                logMessage.append(str(t))
                logMessage.append([str(type(error).__name__)])
                logMessage.append([str(error)])
                writer.writerow(logMessage)
                writer.flush()

            writer.writerow(['Bootloader will now end.'])

        time.sleep(600)

        run_while = False

        


    voltage_status = 'normal' 
    #DEVELOPERS NOTE: This is how we keep track of when a low voltage error occurred. It is sent and returned to main().
    #if a low voltage occurs, it will be set to 'low' until bootloader is completely rerun again (drone is unplugged and plugged back in.)
    while run_while:       
        status, voltage_status = main(voltage_status)

        if status != 'normal':
            try:
                importlib.reload(c_interface)
                importlib.reload(fc_handler)
                importlib.reload(user_code_handler)
                importlib.reload(logbook)
                importlib.reload(comms)
                importlib.reload(lib.shared_data_management)
                importlib.reload(lib.bootloader_stm)

                #start with a clean log
                # if len(os.listdir('logs')) > 0:
                #     os.system('sudo rm logs/*')

                #start with clean shared memory.
                file_path = "MySharedMemory" # Replace with the actual file path

                if os.path.exists(file_path):
                    os.system('sudo rm ' + file_path)
                    print(f"File '{file_path}' deleted successfully.")



                #compile the c code
                c_file = "ccodeLINUX.c"
                executable_name = "execute"
                compile_command = ["gcc", c_file, "control_helper.c",  "myserial.c", "-lm", "-o", executable_name]
                result = subprocess.run(compile_command, capture_output=True, text=True)

                if result.returncode != 0:
                    print('THERE WAS AN ERROR DOING THE GCC COMPILE')
                    print(result.stderr)
                    with open('logs/emergency.csv', 'w', newline='') as csvfile:
                        writer = csv.writer(csvfile)
                        writer.writerow(['an error occurred when trying to compile the c code.'])
                        writer.writerow([str(result.stderr)])
                        csvfile.flush()
                    break
                else:
                    print('gcc compiled successfully.')



            except Exception as error:
                with open('logs/emergency.csv', 'w', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerow(['an error occurred when trying to reload modules.'])
                    writer.flush()
                    trace = traceback.extract_tb(error.__traceback__)
                    logMessage = []
                    for t in trace:
                        logMessage.append(str(t))
                        logMessage.append([str(type(error).__name__)])
                        logMessage.append([str(error)])
                        writer.writerow(logMessage)
                        writer.flush()

                    writer.writerow(['Bootloader will now end after 600s wait.'])

                time.sleep(600)

                break



        



        time.sleep(2)
    
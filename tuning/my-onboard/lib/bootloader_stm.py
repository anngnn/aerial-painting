'''
This code was developed by Andrew Curtis in the year 2024.
It was developed at Northwestern University, Evanston, IL, USA.

This serves as the state machine for the bootloader for each quadrotor in the QuadSwarm.
From here, the robot can change states (i.e., idle -> running processes -> running user code)

This module follows SysML state machine protocols
Each state has 3 possible behaviors: entry, do, exit. 
Entry and exit happen only 1x each (when entering the state and when exiting the state, respectively)
The do behavior is where any 'while True' loops might be (so it can be run continuously)
The entry and exit behaviors are NOT interruptable. The do behavior is.

Each transition also has en effect behavior that is also NOT interruptable.
'''

#import 3rd party libraries
from typing import Any, List
from statemachine import StateMachine, State

#import os files (files NU created)
#NONE

#import native libraries
import os
from paramiko import SSHClient, AutoAddPolicy
from scp import SCPClient, SCPException
import time


class BootloaderStateMachine(StateMachine):
    '''
    The state machine for the bootloader
    Houses all states and transitions and behavior when states transition
    python docs on statemachine: https://python-statemachine.readthedocs.io/en/latest/readme.html
    '''

    #Here are all the states in the state machine.
    startup = State('startup', initial=True)
    preflight = State('preflight', enter="start_processes")
    idle = State('idle')
    running = State('running', enter="start_user_code", exit="kill_user_code")
    shutting_down = State('shutting down', final=True, enter="kill_all_processes")
    fetching_logs = State('logFetch', enter="fetch_logs")
    removing_logs = State('rmLogs', enter="do_remove_logs")
    sleep = State('sleep', enter="kill_all_processes")

    #Here are all the events/transitions in the state machine
    do_preflight_checks = startup.to(preflight)
    checks_complete = preflight.to(idle)
    shutdown = (idle.to(shutting_down) | running.to(shutting_down)) #the event 'shutdown' will cause a transition to shutting down from idle OR running
    start = idle.to(running)
    stop = running.to(idle)
    update_user_code = idle.to.itself(on="user_code_update")
    reset = idle.to(startup, on="kill_all_processes")
    estop = (running.to(startup, on="kill_all_processes") | idle.to(startup, on="kill_all_processes") | fetching_logs.to(startup, on="kill_all_processes") | removing_logs.to(startup, on="kill_all_processes"))
    check = (idle.to.itself(internal = True, on="check_processes") | running.to.itself(internal=True, on="check_processes"))
    create = startup.to.itself(internal = True, on="create_processes")
    fetch = idle.to(fetching_logs) #transitions to fetching_logs from idle .
    done = (fetching_logs.to(idle) | removing_logs.to(idle)) #general transition to go back to idle (from various other states)
    remove_logs = idle.to(removing_logs)
    go_to_sleep = idle.to(sleep)
    wake = sleep.to(shutting_down)
    #DEVELOPERS NOTE: if you want a function to run more than once, it needs to be in an internal transition to self
    #and then you need to call that event within each loop (from the bootloader)



    def __init__(self, watchdog, host_ip, model: Any = None, state_field: str = "state", start_value: Any = None, rtc: bool = True, allow_event_without_transition: bool = False, listeners: List[object] | None = None):
        super().__init__(model, state_field, start_value, rtc, allow_event_without_transition, listeners)
        self.count = 0
        self.watchdog_timer_limit = watchdog #s
        self.user_code_running = False
        self.user_code_updated = False
        self.host_ip = host_ip

    def create_processes(self, process_manager, shared_data):
        '''
        Function that creates Multiprocessing Process objects for each process that is going to run on the pi
        '''
        process_manager.make_processes(shared_data) #this function is defined in bootloader.py in the ProcessManager class
 
    def start_processes(self, process_manager):
        '''
        Starts all of the processes that have been created (except for the user code process)
        '''
        process_manager.start_all()

    def check_processes(self, process_manager, csvfile, writer, t):
        abort = False

        #log data from the processes (if anything)
        message = process_manager.poll_processes()
        if message:
            for key in message:
                writer.writerow([t,key,message[key]])
                csvfile.flush()

        
        #Check if processes are still running. Some processes will end when safety codes are triggered.
        #when that happens, we want to shut everything down gracefully.
        abort, dead_process = process_manager.check_processes()
        if abort:
            writer.writerow([t, 'bootloader', dead_process + ' process has ended, so all processes have been ended.'])
            return abort
            
        return abort

    def start_user_code(self, process_manager):
        #Start the user code handler process
        self.user_code_running = process_manager.kick_off_user_code()

    def kill_user_code(self, process_manager):
        #Kill the user code handler process
        process_manager.kill_process('user code handler')

    def user_code_update(self, process_manager):
        self.user_code_updated = False
        self.user_code_updated = process_manager.update_user_code()


    def kill_all_processes(self, process_manager):
        process_manager.kill_all()

    def fetch_logs(self, id, writer, csvfile, t):
        print('going to fetch logs')
        #destination is IP of air traffic controller:/home/pi/atc/logs/id
        dest = '/home/pi/atc/logs/' + str(id).zfill(2)

        #Create a connection to the base station computer
        ssh = SSHClient()
        ssh.set_missing_host_key_policy(AutoAddPolicy())
        ssh.load_system_host_keys()
        ssh.connect(hostname=self.host_ip, username='pi', password='1')

        #Information on getting logs in this way: https://stackoverflow.com/questions/43577248/scp-in-python-by-using-password

        try:
            #Send the entire 'clogs' folder
            if len(os.listdir('clogs')) > 0:
                print('sending clogs')
                with SCPClient(ssh.get_transport()) as scp:
                    scp.put('clogs', recursive=True, remote_path=dest)
                writer.writerow([t,'bootloader','clogs sent.'])
        except SCPException:
            print('exception on clogs')
            writer.writerow([t,'bootloader','exception on clogs'])
        csvfile.flush()
        time.sleep(2)

        try:
            #Send the entier 'logs' folder
            if len(os.listdir('logs')) > 0:
                print('sending logs')
                with SCPClient(ssh.get_transport()) as scp:
                    scp.put('logs', recursive=True, remote_path=dest)
                writer.writerow([t,'bootloader','logs sent.'])
        except SCPException:
            writer.writerow([t,'bootloader','exception on logs'])
            pass
        csvfile.flush()
        time.sleep(2)

        try:
            #Send the entire 'ctrl_logs' folder
            if len(os.listdir('ctrl_logs')) > 0:
                print('sending control logs')
                with SCPClient(ssh.get_transport()) as scp:
                    scp.put('ctrl_logs', recursive=True, remote_path=dest)
                writer.writerow([t,'bootloader','ctrl_logs sent.'])
        except SCPException:
            writer.writerow([t,'bootloader','exception on ctrl_logs'])
            pass
        csvfile.flush()
        time.sleep(2)

        try:
            #Send the entire 'usr_logs' folder
            if len(os.listdir('usr_logs')) > 0:
                print('sending user logs')
                with SCPClient(ssh.get_transport()) as scp:
                    scp.put('usr_logs', recursive=True, remote_path=dest)
                writer.writerow([t,'bootloader','usr_logs sent.'])
        except SCPException:
            writer.writerow([t,'bootloader','exception on usr_logs'])
            pass
        csvfile.flush()

        del ssh

    def do_remove_logs(self):
        #remove the logs locally on this platform (only if there are logs to remove)
        if len(os.listdir('logs')) > 0:
            os.system('sudo rm logs/*')
        if len(os.listdir('usr_logs')) > 0:
            os.system('sudo rm usr_logs/*')
        if len(os.listdir('clogs')) > 0:
            os.system('sudo rm clogs/*')
        if len(os.listdir('ctrl_logs')) > 0:
            os.system('sudo rm ctrl_logs/*')
        print('logs removed.')


        




def flight_controller_board_watchdog_check(data_manager,t, limit):
    '''
    checks to see if the the FC board is connected.
    If not, checks if the watchdog timer has expired.
    Returns abort = True if the timer has expired. False otherwise.
    '''
    if not data_manager.get_board_connected():
        if t > limit:
            return True
    return False


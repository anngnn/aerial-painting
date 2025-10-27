'''
This code was developed by Andrew Curtis in the year 2024.
It was developed at Northwestern University, Evanston, IL, USA.

This serves as the "Localizer" for each quadrotor in the QuadSwarm.
From here, each robot communicates with the optitrack system to receive its state information.
'''

#import 3rd party libraries
#NONE

#import os files (files NU created)
from lib.shared_data_management import SharedDataManager
from logbook import get_time_stamp

#import native libraries
import time
import socket
import numpy as np
import traceback
import selectors
import types
import csv
import struct



class LocalizerUDP():
    '''
    A class to handle gathering and calculating state information using UDP.
    It is intialized by bootloader before it's 'run' task is kicked off as a process in the multiprocess setup
    '''
    def __init__(self, id, conn, ip, port, timeout, standalone = False) -> None:
        '''
        gets a unique robot id (from bootloader when the localizer object is created) and stores it

        Also gets a pipe connection to bootloader so it can pass info for logging.

        Creates a udp client socket to listen for information from the optitrack computer. 
        DEVELOPERS NOTE: Port and ip address defined in params.json

        If standalone, the localizer assumes it is running on its own (without the rest of the operating system)
        '''

        self.id = id #store the id locally
        self.standalone = standalone

        self.__watchdog_timer_limit = timeout #seconds #set in params.json (typically 1second.)
        self.__time_of_last_receipt = time.time() #initailize to now in seconds

        self.port = port #typically 54321
        self.ip = ip #IP address of MOTIVE machine

        #set up a socket to receive messages from the server.
        #AF_INET is the internet address family for IPv4
        #SOCK_DGRAM is the socket type for the UDP protocol.
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        #This allows the socket address to be reused (which I think helps with the sender socket above)
        self.client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        #this increases the buffer size (for data incoming too fast)
        self.client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 131072)

        #We're going to listen to anything coming over the port from the ip address given
        self.client_socket.bind(('', self.port))

        #make the socket blocking
        self.client_socket.setblocking(False)

        #save the bootloader connection so we can access it later.
        if not self.standalone:
            self.bootloader_pipe_connection = conn

        #variables for keeping track of the optitrack time difference 
        self.optitime_diff = 0
        self.optitime_diff_alpha = 0.05 #alpha*NEW + (1-alpha)*OLD

        print('everything is set for udp init')
        

    def run(self, shared_data):
        '''
        listen for incoming information from the optitrack system
        calculate and update shared memory data with new state information.
        '''
        
        if not self.standalone:
            #object for managing the shared data
            data_manager = SharedDataManager(shared_data)

            #let the logger know we're up and running.
            self.bootloader_pipe_connection.send('localizer up and running')
        
        self.__time_of_last_receipt = time.time() #initailize to now in seconds
        self.time_of_last_successful_parse = time.time()
        first_data_point = True #used so we only begin calculating velocities when we have our 2nd data point.
        
        timestamp = get_time_stamp()
        filename = 'alogs/localize_' + timestamp + '.csv'
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['new log ' + str(self.id)])
            writer.writerow(['local time', 'opti time recvd', 'filtered diff', 'raw diff',  'opti counter', 'cnts missed', 'len(raw msg)'])
            csvfile.flush()

            local_start_time = 0
            local_receipt_time = time.time()
            local_last_receipt_time = local_receipt_time-0.01
            last_sequence_number = 0
        
            while True:

                try: #using this to catch any blocking errors, too many blocking errors initiates an e-stop

                    #Fire a socket event (either send or receive data w. motive computer)
                    recv_data, addr = self.client_socket.recvfrom(1024) #data will return already decoded, but not split
                    data_length = len(recv_data)
                    # print(recv_data)
                    recv_data = recv_data.decode('utf-8')

                    log_no_data = False #set this to True if you want to log that data was missed for this particular loop 
                    error_msg = 'normal' #set this to identify the reason data was lost.

                    #if we got data to parse
                    if recv_data:
                        split_data = recv_data.split(',')
                    

                        self.__time_of_last_receipt = time.time()
                        # print(split_data)
                        
                


                        #messages are in the format: 'opti', timestamp, sequence number, id, x, y, z, q1, q2, q3, q4, id, x, y, z, q1, q2, q3, q4, timestamp, sequence number...
                        #                                                                 0, 1, 2, 3,  4,  5,  6,  7,
                        # First, we need to identify where each message starts (we may have more than one message in our 1024 bytes of data tha we pulled)
                        #
                        message_start = [] #will be used to hold the indexes of the beginning of the message
                        for i,item in enumerate(split_data):
                            #first, look for the start of the message (may not be at the beginning of the received data stream.)
                            if item == 'opti': #we are looking for 'opti' because that is the identifier at the beginning of each optitrack message
                                message_start.append(i)

                        #Here we check if we even received anything that could be interpreted as a message in this stream of data
                        if len(message_start) > 0: #this is the case where we at least logged 1 'opti'

                            #The robot msgs start 3 data position in from the 'opti'
                            offset = 3
                            modulus = 8 # there are 8 pieces of data for each robot, so we only need to check every 8th piece of data for a robot id

                            data_found = False #used to break out of the loop once we found our most up-to-date data
                            
                            #Now, we need to find our data, starting with the most recent message first (opposite order in which they were received)
                            for start_index in reversed(message_start):

                                #create a temporary message data item with just the message from the start index to the end.
                                msg_data = split_data[start_index+offset:]

                                try:
                                    #look at just the robot information portion of the message
                                    for i,item in enumerate(msg_data):
                                        if i % modulus: #skip anything that isn't a robot id
                                            continue
                                        else:
                                            rec_id = int(item) #get the integer of the received id
                                            if rec_id == self.id: #check if it's intended for us, and store the data
                                                mi = i
                                                received_xyz = np.array(msg_data[mi+1:mi+4], dtype = float)
                                                received_quaternion = np.array(msg_data[mi+4:mi+8], dtype=float)
                                                roll, pitch, yaw = quaternion_to_euler(received_quaternion)
                                                received_rpy = np.array([roll, pitch, yaw])
                                                received_time = float(split_data[start_index+1])
                                                received_sequence_number = int(split_data[start_index+2])
                                                if received_sequence_number <= last_sequence_number:
                                                    data_found = False
                                                    break
                                
                                                diff = received_time - time.time()
                                                data_found = True
                                                break 

                                except IndexError:
                                    continue #try the next older message in message_start

                                if data_found: #we found our data, so we don't care about any previous messages we may have to look through
                                    break
                           

                            if data_found: #if we found data, process and store it!                              

                                #if this is the first data point we've seen,
                                #then we should make our velocities zero 
                                if first_data_point:
                                    first_data_point = False
                                    xyz_velo = np.zeros(3, dtype=float)
                                    rpy_velo = np.zeros(3, dtype=float)
                                    xyz_accel = np.zeros(3, dtype=float)
                                    self.optitime_diff = diff 
                                    last_sequence_number = received_sequence_number

                                #if this is NOT the first data point, calculate velocities and accelerations
                                else:
                                    if received_time - last_rec_time == 0:
                                        continue #avoid the divide by zero case
                                    xyz_velo = (received_xyz-last_xyz)/(received_time-last_rec_time)
                                    rpy_velo = (received_rpy-last_rpy)/(received_time-last_rec_time)

                                    xyz_accel = (xyz_velo - last_velo)/(received_time-last_rec_time)
                                    self.optitime_diff = self.optitime_diff_alpha*diff + (1-self.optitime_diff_alpha)*self.optitime_diff

                                #get the state
                                #x, y, z, xdot, ydot, zdot, roll, pitch, yaw, rolldot, pitchdot, yawdot
                                state = np.concatenate([received_xyz, xyz_velo, received_rpy, rpy_velo])

                                if not self.standalone:
                                    #write the state to shared data
                                    #DO NOT BLOCK - if we don't have the lock, just circle back to get new data
                                    set = data_manager.set_state(state, blocking=False)

                                    data_manager.set_optitime_diff(self.optitime_diff)

                                local_time = time.time() - local_start_time
                                delta = received_sequence_number - last_sequence_number
                                writer.writerow([local_time, received_time, self.optitime_diff, diff, received_sequence_number, delta, data_length])
                                csvfile.flush()
                                # print([received_time, *state[0:3]])

                                #before looping back, store data so we can calculate velocities next time
                                last_xyz = np.copy(received_xyz)
                                last_rpy = np.copy(received_rpy)
                                last_velo = np.copy(xyz_velo)
                                last_rec_time = received_time
                                self.time_of_last_successful_parse = time.time()
                                last_sequence_number = received_sequence_number

                                #TODO add a data limits check!!

                            else:
                                log_no_data = True #log that we didn't get any data because none of the messages were for us.
                                error_msg = 'no data for this robot'

                        #if len(message_start) > 0:
                        else:
                            log_no_data = True
                            error_msg = 'no opti msg detected'

                    #if recv_data  
                    else:
                        log_no_data = True
                        error_msg = 'no data received'
                    

                    if log_no_data:
                        rec_time = (time.time() - self.time_of_last_successful_parse) + self.__time_of_last_receipt
                        writer.writerow([rec_time, 1, 1, error_msg])
                        csvfile.flush()

                except BlockingIOError:
                    
                    #check how long it has been since we successfully parsed data.
                    #if its longer than the watchdog timer limit, we throw an error.
                    if time.time() - self.time_of_last_successful_parse > self.__watchdog_timer_limit:

                        if not self.standalone:
                            # log the timout
                            self.bootloader_pipe_connection.send('the localization code timed out. There is likely no optitrack running, optitrack connection was lost, or we missed too many packets.')
                            print('data loss issue.')
                            # initialize a complete e-stop
                            data_manager.set_safety(1)
                            
                        while True:
                            try:
                                recv_data, addr = self.client_socket.recvfrom(1024) #data will return already decoded, but not split
                                recv_data = recv_data.decode('utf-8')

                                log_no_data = False #set this to True if you want to log that data was missed for this particular loop 
                                error_msg = 'normal' #set this to identify the reason data was lost.

                                #if we got data to parse
                                if recv_data:
                                    split_data = recv_data.split(',')
                                

                                    print(split_data)
                                    print(last_sequence_number, time.time() - self.time_of_last_successful_parse)
                                    break
                                    
                            


                                    

                            except BlockingIOError:
                                continue

                        
                        break
                    continue
                
                except KeyboardInterrupt:
                    self.client_socket.close()

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
                    else:
                        print(logMessage)
                    break
        
        while True:
            if not self.standalone:
                self.bootloader_pipe_connection.send('localizer sleeps')
                time.sleep(30) #sleep until the bootloader stops us 

    def exit(self):
        '''
        The code that runs immediately before this process is killed
        '''
        self.client_socket.close()













#Information on multicast: https://stackoverflow.com/questions/603852/how-do-you-udp-multicast-in-python
class LocalizerMulticast():
    '''
    A class to handle gathering and calculating state information using UDP Multicast.
    It is intialized by bootloader before it's 'run' task is kicked off as a process in the multiprocess setup
    '''
    def __init__(self, id, conn, ip, port, timeout, standalone = False, writer=False, csvfile=False) -> None:
        '''
        gets a unique robot id (from bootloader when the localizer object is created) and stores it

        Also gets a pipe connection to bootloader so it can pass info for logging.

        Creates a udp client socket to listen for information from the optitrack computer. 
        via a multicast network
        DEVELOPERS NOTE: Port and ip address defined in params.json
        The ip address is likely 224.1.1.1, which is the multicast group that this robot will join.

        If standalone, the localizer assumes it is running on its own (without the rest of the operating system)
        '''

        self.id = id #store the id locally

        #In some cases, (debugging), we may run the localize code on its own. 
        #In that case, all tasks associated with shared memory or other processes will be skipped.
        self.standalone = standalone

        #This is set in localizer_timeout in params.json
        self.__watchdog_timer_limit = timeout #seconds #set in params.json (typically 1second.) 

        self.port = port #typically 54321 #set in params.json. Called 'localizer_port'
        self.ip = ip #IP address of multicast group. set in params.json called "localizer_ip"

        #set up a socket to receive messages from the server.
        #AF_INET is the internet address family for IPv4
        #SOCK_DGRAM is the socket type for the UDP protocol.
        #socket.IPPROTO_UDP is also for the UDP protocol (to remove any ambiguity.)
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

        #This allows the socket address to be reused (which I think helps with the sender socket above)
        self.client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        #REUSE THE PORT TOO (ONLY ON LINUX)
        self.client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)

        #this increases the buffer size (for data incoming too fast)
        self.client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 131072)

        #We're going to listen to anything coming over the port from the ip address given
        self.client_socket.bind(('', self.port))

        # Join the multicast group
        mreq = struct.pack("4sL", socket.inet_aton(self.ip), socket.INADDR_ANY)
        self.client_socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

        #make the socket non blocking
        self.client_socket.setblocking(False)

        #save the bootloader connection so we can access it later within "run".
        if not self.standalone:
            self.bootloader_pipe_connection = conn
        #otherwise, if we are running standalone, save the writer and the file so we can write data
        else:
            self.writer = writer
            self.csvfile = csvfile

        #variables for keeping track of the optitrack time difference 
        self.optitime_diff = 0 #this is where we store the difference between our clock and the optitrack clock.
        self.optitime_diff_alpha = 0.05 #alpha*NEW + (1-alpha)*OLD #factor for our exponential filter of the optirack data time difference.

        self.filtered_velocity = np.zeros(3,dtype=float)
        self.velo_filter_alpha = 0.35 #0 for all old, 1 for all new

        print('everything is set for udp init, multicast!')
        print(f"Listening for multicast messages on {self.ip}:{self.port}...")

        self.frequency = 42
        

    def run(self, shared_data):
        '''
        listen for incoming information from the optitrack system
        calculate and update shared memory data with new state information.
        '''
        
        if not self.standalone:
            #object for managing the shared data
            self.data_manager = SharedDataManager(shared_data)

            #let the logger know we're up and running.
            self.bootloader_pipe_connection.send('localizer up and running')
        
        #TIMING INITIALIZATION
        self.initialize_timers()

        first_data_point = True #used so we only begin calculating velocities when we have our 2nd data point.
        
    
        while True:

            loop_start = time.perf_counter()

            try: #using this to catch any blocking errors, too many blocking errors initiates an e-stop

                #Attempt to receive data
                recv_data, addr = self.client_socket.recvfrom(1024) #data will be in raw bytes
                data_length = len(recv_data)

                #if we got data to parse. If we don't (there was nothing there), then the blockingIO exception would have triggered, so we don't need an 'else' statement here.
                if recv_data:                  
                  
                    #We assume we only have 1 message per 1024 bytes pulled from the receive buffer.
                    #We quickly check this by looking at the length of the data. We expect each message to be less than 800 bytes long (29bytes/drone*25drones/message=725bytes/message)
                    #if we only have 1 message, we know 'optiX' should be at the beginning, 
                    #and we are looking for 'optiX' because that is the identifier at the beginning of each optitrack message
                    ##where 'optiX' could be 'opti1' for IDs 5-29 and 'opti2' for IDs 30-54
                    #if we have more than 1 message, we loop through until we find 'optiX'

                    #messages are in the format: 'opti', timestamp, sequence number, id, x, y, z, q1, q2, q3, q4, id, x, y, z, q1, q2, q3, q4,
                    #                                                                 0, 1, 2, 3,  4,  5,  6,  7,

                    message_start = [] #will be used to hold the indexes of the beginning of the message

                    #this is the case where (we assume) we only have 1 message in the 1024 bytes
                    if data_length < 800:
                        #unpacked the first 5 bytes (should be 'optiX')
                        segment = struct.unpack('5s', recv_data[0:5])[0]
                        #try to turn the segment of the message into a string. If you can't do that (UnicodeDecodeError, just continue to recv the next message (restart while loop))
                        try:
                            segment = segment.decode('utf-8')
                        except UnicodeDecodeError:
                            continue

                        #check if it is a message for us or not
                        #'opti1' for IDs 5-29 and 'opti2' for IDs 30-54
                        if self.id < 30 and segment == 'opti1': 
                            message_start.append(0)
                        elif self.id >= 30 and segment == 'opti2':
                            message_start.append(0)
                        else:
                            #if we didn't meet either criterion above, then the message was not for us, so we ignore it and restart while loop.
                            continue

                    #this is the case where the message is big enough that it is likely there are multiple messages (or parts of multiple messages) in the 1024 bytes
                    #this RARELY happens. (I've never seen this message printed since I wrote the code - Drew)
                    else:                    
                    
                        print('MORE THAN ONE MESSAGE IN OUR 1024 BYTES')
                        # First, we need to identify where each message starts (we likely have more than one message in our 1024 bytes of data tha we pulled, and its possible the first byte is not the start of a message.)
                        for i,item in enumerate(recv_data):
                            #first, look for the start of the message (may not be at the beginning of the received data stream.)
                            segment = struct.unpack('5s',recv_data[i:i+5])[0]
                            #try to turn the segment of the message into a string. If you can't do that (UnicodeDecodeError, just continue through the message)
                            try:
                                segment = segment.decode('utf-8')
                            except UnicodeDecodeError:
                                print('unicode decode error')
                                continue
                            
                            #Ids under 30 are looking for 'opti1' messages
                            #'opti1' for IDs 5-29 and 'opti2' for IDs 30-54
                            if self.id < 30 and segment == 'opti1': #we are looking for 'optiX' because that is the identifier at the beginning of each optitrack message
                                message_start.append(i)
                            elif self.id >= 30 and segment == 'opti2':
                                message_start.append(i)

                            #if we have reached the end of the bytes and we don't have enough bytes left to parse and get an 'optix', then we just break (no point in looking at the last few bytes)
                            if i+5 >= data_length:
                                break

                    #Here we try to parse a message if we even received anything that could be interpreted as a message in this stream of data
                    if len(message_start) > 0: #this is the case where we at least logged 1 'optiX' for our corresponding ID
                                                #if we didn't get at least 1 optiX for our ID range, then we will go back to the start of the while loop 
                                                #the only logic after this IF statement are exception catchers

                        #The robot msgs start 14 bytes in from the start of 'opti' (I.e., it takes 14 bytes to capture the preamble: optiX, timestamp, sequence number,)
                        offset = 14
                        modulus = 29 # there are 29 bytes for each robot (7 floats * 4bytes/float + 1byte for ID), so we only need to check every 29th byte for a robot id

                        data_found = False #used to break out of the loop once we found our most up-to-date data
                        
                        #Now, we need to find our data, starting with the most recent message first (opposite order in which they were received)
                        for start_index in reversed(message_start):

                            #create a temporary message data item with just the message from the start index (after the preamble) to the end.
                            msg_data = recv_data[start_index+offset:]

                            try:
                                #look at just the robot information portion of the message (everything AFTER the preamble)
                                for i,item in enumerate(msg_data):
                                    if i % modulus: #skip anything that isn't a robot id (so we only read every 29th byte to check for an ID.)
                                        continue
                                    else:
                                        #check the first byte as the rec_id
                                        rec_id = struct.unpack('B',msg_data[i:i+1])[0]
                                        if rec_id == self.id: #check if it's intended for us (the ID is our ID), and store the data if it is.
                                            my_data = struct.unpack('7f',msg_data[i+1:i+modulus]) #unpack the next 28 bytes (after our id) as floating point data
                                            received_xyz = np.array(my_data[0:3], dtype = float) #convert the first 3 floats to our x,y,z position
                                            received_quaternion = np.array(my_data[3:], dtype=float) #convert the next 4 floats as the quaternion
                                            roll, pitch, yaw = quaternion_to_euler(received_quaternion) #convert that quaternion into roll, pitch and yaw
                                            received_rpy = np.array([roll, pitch, yaw]) #save the roll pitch and yaw as a attitude array.
                                            preamble = struct.unpack('5sfH', recv_data[start_index:start_index+offset]) #we also want timing and count number info, so unpack the preamble of the message.
                                            self.received_time = preamble[1] 
                                            self.received_sequence_number = preamble[2]
                                            #Make sure the sequence number is not an old one.
                                            #and check the roll over at 65535 
                                            #to do this, see if the number we received is between 0 and 100, and our previous number was between 65435 and 65535 
                                            #This is a range of 200 messages, incase we actually missed a message or 2 during the course of the roll over.
                                            #The 200 message rangge is arbitrary (if we miss more than 200 messages, we are screwed anyway)
                                            roll_over_flag = False
                                            if self.received_sequence_number < 0+100 and self.last_sequence_number >= 65535 - 100:
                                                roll_over_flag = True

                                            #if we didn't encounter a roll over and our received sequence number is less than (or equal to) the last one we received, then we are looking at an old message.
                                            if self.received_sequence_number <= self.last_sequence_number and not roll_over_flag:
                                                data_found = False
                                                break
                            
                                            #Finally, calculate the difference between the optitrack's clock and our robot's clock.
                                            robot_time_received = time.time() - self.robot_start_time
                                            clock_difference = self.received_time - robot_time_received

                                            #Flag found data so we run the next segment of code, and break out of the loop
                                            data_found = True
                                            break 

                            #if comms get messy, its possible that a message is cut off (since we can only pull 1024 bytes, if a message starts at 1000 byts, then it will be cut off (29 bytes long))
                            #in that case, an index error will throw at some point during the processing above.
                            #since we don't have complete data, we just move on the the next message
                            except IndexError:
                                continue #try the next older message in message_start

                            if data_found: #we found our data, so we don't care about any other messages we may have to look through
                                break
                        

                        if data_found: #if we found data, process and store it!                              

                            #if this is the first data point we've seen,
                            #then we should make our velocities zero 
                            if first_data_point:
                                first_data_point = False
                                xyz_velo = np.zeros(3, dtype=float)
                                rpy_velo = np.zeros(3, dtype=float)

                                #We should also initialize the optitrack clock difference with our most recent value (so the filter is seeded with the first known difference.)
                                self.optitime_diff = clock_difference

                                #intitialzie our last sequence number (since the optitrack may not start at sequence no 1)
                                self.last_sequence_number = self.received_sequence_number -1

                            #if this is NOT the first data point, calculate velocities and accelerations
                            else:
                                if self.received_time - self.last_received_time == 0:
                                    continue #avoid the divide by zero case

                                #velocities are calculated using the time sent with the data from the optitrack machine, 
                                #so transmission latency and processing latency are not incorporated in the denomenator of the velocity calculations
                                xyz_velo = (received_xyz-last_xyz)/(self.received_time-self.last_received_time)
                                rpy_velo = (received_rpy-last_rpy)/(self.received_time-self.last_received_time)

                                self.filtered_velocity = (1-self.velo_filter_alpha)*self.filtered_velocity + self.velo_filter_alpha*xyz_velo

                                #update the optitrack clock time difference via an exponential filter
                                self.optitime_diff = self.optitime_diff_alpha*clock_difference + (1-self.optitime_diff_alpha)*self.optitime_diff

                            #get the state
                            #x, y, z, xdot, ydot, zdot, roll, pitch, yaw, rolldot, pitchdot, yawdot
                            state = np.concatenate([received_xyz, self.filtered_velocity, received_rpy, rpy_velo])
                            # state = np.concatenate([received_xyz, xyz_velo, received_rpy, rpy_velo])

                            #Prep timing data for writing (to shared data if NOT standalone, or to csv file IF standalone)
                            #DATA WILL PRINT: #'raw local time', 'local time', 'raw global time', 'filtered diff', 'raw diff',  'opti counter', 'cnts missed', 'time_between', 'error'])
                            raw_local_time = time.time() #raw time since January 1, 1970, 00:00:00 (UTC)
                            robot_local_time = raw_local_time - self.robot_start_time #time since bootloader turned on.
                            count_delta = (self.received_sequence_number - self.last_sequence_number)%65536
                            self.time_of_current_parse = time.perf_counter()
                            time_between_messages = np.round(self.time_of_current_parse - self.time_of_last_successful_parse,5)

                            if not self.standalone:
                                #write the state to shared data
                                #DO NOT BLOCK - if we don't have the lock, just circle back to get new data
                                #TODO - set the associated timing and counter values with that TOO!
                                associated_messaging_info = [self.received_time, self.received_sequence_number, count_delta, np.round(time_between_messages,4)]
                                set = self.data_manager.set_state(state, blocking=False, timing_info = associated_messaging_info)

                                #write the clock information
                                self.data_manager.set_optitime_diff(self.optitime_diff)


                            else:
                                self.writer.writerow([raw_local_time, robot_local_time, self.received_time, self.frequency, clock_difference, self.received_sequence_number, count_delta, time_between_messages, 0])
                                self.csvfile.flush()

                        

                            #before looping back, store data so we can calculate velocities next time
                            last_xyz = np.copy(received_xyz)
                            last_rpy = np.copy(received_rpy)

                            #store timer and counter data
                            self.update_timers()
                            
                            #TODO add a data limits check!!

                            loop_end = time.perf_counter()
                            self.frequency = 1/(loop_end-loop_start)
             

            except BlockingIOError:
                
                #check how long it has been since we successfully parsed data.
                #if its longer than the watchdog timer limit, we throw an error.
                if time.perf_counter() - self.time_of_last_successful_parse > self.__watchdog_timer_limit:

                    if not self.standalone:
                        # log the timout
                        self.bootloader_pipe_connection.send('the localization code timed out. There is likely no optitrack running, optitrack connection was lost, or we missed too many packets.')
                        print('data loss issue.')
                        # initialize a complete e-stop
                        self.data_manager.set_safety(1)
                        break
                        
                    else:
                        print('data loss issue. missed more than %fs of data. ' % self.__watchdog_timer_limit)
                        raw_local_time = time.time() #raw time since January 1, 1970, 00:00:00 (UTC)
                        robot_local_time = raw_local_time - self.robot_start_time #time since bootloader turned on.
                    
                        writer.writerow([raw_local_time, robot_local_time, 0, 0, 0, 0, 0, 0, 42])
                        csvfile.flush()
                        continue
                       
                continue
            
            except KeyboardInterrupt:
                self.client_socket.close()

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
                else:
                    print(logMessage)
                break
        
        while True:
            if not self.standalone:
                self.bootloader_pipe_connection.send('localizer sleeps')
                time.sleep(30) #sleep until the bootloader stops us 


    def initialize_timers(self):
        '''
        Initializes all of the timers and counters used when receiving and processing localization data
        In general, we are going to track the following timing information:
            time_of_current_parse           = the time when the current info was parsed (right before sent to shared data)
            time_of_last_successful_parse   = the time when the last location information was successfully parsed
            robot_start_time                = the time when the bootloader kicked off (t=0 for the drone)
            received_sequence_number        = the sequence number received from the most recent optitrack message.
            last_sequence_number            = the sequence number previously received from the optirack system
            received_time                   = the time that is sent to the robot in an optitrack message. Used to calculate clock differences and velocities
            last_received_time              = the time that was stamped on the previous message from optitrack. Used to calculate velocities.
            

        '''

        # initialize to now in fractional seconds so we don't have to call it multiple times in this initializer
        now = time.perf_counter()             

        self.time_of_current_parse = now
        self.time_of_last_successful_parse = now - 0.01
        if not self.standalone:
            self.robot_start_time = self.data_manager.get_robot_start_time()
        else:
            self.robot_start_time = time.time()
        self.received_sequence_number = 0
        self.last_sequence_number = 0
        self.received_time = 0 
        self.last_received_time = 0 #this will get overwritten with the 1st received time befoe it is used in calculating velocities

        return
    
    def update_timers(self):
        '''
        Updates all of the timers and counters used when receiving and processing localization data
        '''
        self.time_of_last_successful_parse = self.time_of_current_parse
        self.last_sequence_number = self.received_sequence_number
        self.last_received_time = self.received_time
        return


        

    def exit(self):
        '''
        The code that runs immediately before this process is killed
        '''
        self.client_socket.close()


def quaternion_to_euler(q):
    # Extract quaternion components
    w, x, y, z = q

    yaw = np.arctan2(2.0*(y*z + w*x), w*w - x*x - y*y + z*z)
    pitch = np.arcsin(-2.0*(x*z - w*y))
    roll = np.arctan2(2.0*(x*y + w*z), w*w + x*x - y*y - z*z)

    #pi/2 phase shift, mod pi, phase shift back
    roll = roll + np.pi/2
    roll = roll % np.pi
    roll = roll - np.pi/2

    return roll, pitch, yaw


if __name__ == "__main__":
    print('running localizer only!')

    # obj = Localizer(10, False, "192.168.18.100", 54321, 5, standalone=True)
    # obj.run(False)

    timestamp = get_time_stamp()
    filename = 'alogs/localize_' + timestamp + '.csv'
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['new log standalone'])
        writer.writerow(['raw local time', 'local time', 'raw global time', 'filtered diff', 'raw diff',  'opti counter', 'cnts missed', 'time_between', 'error'])
        csvfile.flush()

        obj = LocalizerMulticast(10, False, "224.1.1.1", 54321, 5, standalone=True, writer=writer, csvfile=csvfile)
        obj.run(False)

    # obj = LocalizerHybrid(10, False, "192.168.18.100", 54321, 5, standalone=True)
    # obj.run(False)
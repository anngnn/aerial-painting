'''
This code was developed by Andrew Curtis in the year 2024.
It was developed at Northwestern University, Evanston, IL, USA.

The underlying robot-to-robot communications and networking protocols are housed here.
Anything and everything to do with the details of communication is processed by the Messenger class
Interfaces with the user code api to provide messages and access messages when user calls recv() and send(), respectively
'''

#import 3rd party libraries
#NONE

#import os files (files NU created)
from lib.shared_data_management import SharedDataManager

#import native libraries
import time
import numpy as np
import socket
import signal
import struct
import os
import traceback

p2p_PORT = 50001
p2p_ADDRESS = "239.2.0.1"

class Sender():
    '''
    Class to manage sender side of robot-to-robot comms and networking
    '''
    def __init__(self, id, conn, ip) -> None:
        '''
        Initialize the messanger with robot id, logging pipe connection
        '''
        self.id = id
        self.bootloader_pipe_connection = conn

        self.port = p2p_PORT
        self.ip = ip
        
        pass

    def run(self, shared_data):
        '''
        The function that is kicked off by the process manager in bootloader.
        Responsible for handling communication with other robots.
        Also, interfaces with the shared data.
        '''
        #create a data manager responsible for interfacing with shared data
        data_manager = SharedDataManager(shared_data)
        
        try:            

            #store the process id of this process so the user code can access it for sending messages
            data_manager.set_send_pid(os.getpid())

            self.bootloader_pipe_connection.send('Sender process is up and running')

            #set up a socket over which we can send messages out to our peers
            self._set_up_socket()

            #create a function for sending the messages
            #when the user code hander executes send_msg, it calls os.kill (via the shared data management class)
            #this results in signal.SIGUSR1 getting 'triggered' for lack of a better term, which results in this funciton running.
            def send(*args):
                signal.signal(signal.SIGUSR1, signal.SIG_IGN)
                length = shared_data.send_msg_length.value #Get how long the message is
                pkt = shared_data.send_buffer[:length] #send only the message (not the padded zeros)
                data = self._send_socket.sendto(pkt, (p2p_ADDRESS, p2p_PORT))
                signal.signal(signal.SIGUSR1, send)
            signal.signal(signal.SIGUSR1, send)

            #Have the signal just pause. It will kick off and send any time signal.SIGUSR1 is fired (I think)
            while True:
                signal.pause()

        except Exception as error:
            data_manager.set_safety(3)

            #log the error and continue
            logMessage = ['an error occurred']
            trace = traceback.extract_tb(error.__traceback__)
            for t in trace:
                logMessage.append(str(t))
            logMessage.append([str(type(error).__name__)])
            logMessage.append([str(error)])
            self.bootloader_pipe_connection.send(logMessage) #send the error to the bootloader to be logged

            #sleep until someone tells us to end or reset
            while True:
                time.sleep(5)

    def _set_up_socket(self):
        '''
        Creates a socket to send information out of the send buffer
        '''
        #This creates a socket of UDP protocol
        self._send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

        #set TTL (time-to-live) low to keep traffic local and reduce routing delays.
        #The default value for IP_MULTICAST_TTL is typically 1, meaning multicast packets are only forwarded within the local subnet. 
        self._send_socket.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)

        #Setting ToS to 0x10 suggests that the application sending the data prefers packets with low latency, meaning they should be routed quickly and efficiently.
        # Set low latency priority (this part is key!)
        self._send_socket.setsockopt(socket.IPPROTO_IP, socket.IP_TOS, 0x10)  # Low delay ToS

        #DEVELOPERS NOTE: #The KEY is NOT to bind this socket to a port, so we can also use that port for listening in our other process (below)
        ### self._send_socket.bind((self.ip, self.port))

        #keep the buffer small for low latency
        self._send_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1024)

        #make it so we can't hear our own messages
        self._send_socket.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 0)
        

    def exit(self):
        '''
        The code that runs immediately before this process is killed.
        '''
        try:
            self._send_socket.close()
        except:
            pass
        pass









class Receiver():
    '''
    Class to manage  receiver side of robot-to-robot comms and networking
    '''
    def __init__(self, id, conn, ip) -> None:
        '''
        Initialize the messanger with robot id, logging pipe connection
        '''
        self.id = id
        self.bootloader_pipe_connection = conn

        self.port = p2p_PORT
        self.ip = ip
        
        pass

    def run(self, shared_data):
        '''
        The function that is kicked off by the process manager in bootloader.
        Responsible for handling communication with other robots.
        Also, interfaces with the shared data.
        '''

        #create a data manager responsible for interfacing with shared data
        data_manager = SharedDataManager(shared_data)

        #store the process id of this process so the user code can access it for sending messages
        data_manager.set_recv_pid(os.getpid())

        self.bootloader_pipe_connection.send('Receiver process is up and running')

        #set up a socket over which we can receive messages from our peers
        self._set_up_socket()

        consecutive_errors = 0
        consecutive_error_limit = 10

        while True:

            try: #try to parse the packet

            
                #Get a packet of info from the socket
                info = self._recv_socket.recvfrom(shared_data.EXPECTED_LEN)   

                pkt = info[0]
                sender_ip = info[1][0]

                #ignore anything we sent to ourselves.
                if sender_ip == self.ip:
                    # print('ignoring. its ours.')
                    continue

            

                #see if the first bit is the 'p2p'
                try:
                    header = pkt[0:3].decode('utf-8')
                    if header != 'p2p':
                        # print('not the right header p2p')
                        continue #skip the packet if the header is not correct.
                except:
                    continue

                #get the sender id and the length of the rest of message
                sender_id, msg_length =  int(pkt[3]), pkt[4:8]
                msg_length = struct.unpack('I', msg_length)[0]

                #Throw out the packet if the payload length exceeds the packet length - 8byte header
                if msg_length > len(pkt) - 8:
                    # print('payload too big')
                    continue

                # #skip storing messages that we sent ourselves.
                if sender_id == self.id:
                    continue

                msg = pkt[8:8+msg_length] #gets just the payload (cuts the header and any fluff.)
                
                #pad the message with zeros to get to the full msg length, so we can overwrite the entire buffer line
                n_msg_bytes = min(len(msg), shared_data.MSG_LEN)
                n_padding = max(0, shared_data.MSG_LEN - len(msg))
                padding = b'\x00' * n_padding #This way the send buffer gets completely overwritten

                msg = msg + padding
                
                #Pack the send buffer
                data_manager.pack_buffer(msg)
                consecutive_errors = 0

            except socket.timeout:
                pass



                
                

            except Exception as error:
                consecutive_errors += 1
                if consecutive_errors > consecutive_error_limit:
                    self.bootloader_pipe_connection.send('There were so many consecutive comm erros that we have to land the drone.')
                    data_manager.set_safety(3)

                #log the error and continue
                logMessage = ['an error occurred']
                trace = traceback.extract_tb(error.__traceback__)
                for t in trace:
                    logMessage.append(str(t))
                logMessage.append([str(type(error).__name__)])
                logMessage.append([str(error)])
                self.bootloader_pipe_connection.send(logMessage) #send the error to the bootloader to be logged

                if consecutive_errors > consecutive_error_limit:
                    #sleep until someone tells us to end or reset
                    while True:
                        time.sleep(5)

                continue
                



    def exit(self):
        '''
        The code that runs immediately before this process is killed.
        '''
        try:
            self._recv_socket.close()
        except:
            pass
        pass

    def _set_up_socket(self):
        #This creates a UDP socket 
        self._recv_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        #This allows the socket address to be reused (which I think helps with the sender socket above)
        self._recv_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        #We're going to listen to anything coming over the port, regardless of IP address. So we bind just to the port, empty address
        self._recv_socket.bind(('', self.port))
        #We need to join the multicast group as a listener.
        mreq = struct.pack('4sl', socket.inet_aton(p2p_ADDRESS), socket.INADDR_ANY)
        self._recv_socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        #slightly increase the buffer to hanlde burst traffic more smoothly
        self._recv_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4096)
        #make this blocking for 0.1s (so receiver runs at 10Hz.)
        self._recv_socket.settimeout(0.1)
        #make it so we can't hear our own messages
        self._recv_socket.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 0)

        return


class ATCMessage():
    '''
    Class to parse an inbound air traffic controller message
    And store the data as an object unitl no longer needed.
    '''
    def __init__(self, raw_msg, last_checksum):
        self.is_message_valid = False #assume it is NOT valid until we know it is.
        #unpack the pre-amble and see if it is correct.
        preamble = struct.unpack('BBB', raw_msg[0:3])
        if preamble[0] == 0xAC: #we expect the first byte to be "AC" for Airtraffic Controller
            if preamble[1] == 0xDD: #we expect the message to be "DD" for Destination Drone (i.e., a message meant to go TO the drones)
                self.payload_length = preamble[2]
                self.message_type = struct.unpack('B', raw_msg[3:4])[0]
                local_checksum = 0x00
                for item in raw_msg[0:-1]:
                    local_checksum ^= item
                checksum = struct.unpack('B', raw_msg[-1:])

                #The checksums need to match AND it has to be different from the previously received checksum to be valid.
                #Even the same message sent 2x by the user will have a different checksum due to the unique ID that is added to each message (after the payload, before the checksum)
                #The only way 2 back-to-back messages will have the exact same checksum is if the network sent the same message multiple times.
                if local_checksum == checksum[0] and checksum[0] != last_checksum:
                    self.is_message_valid = True

                    if self.payload_length > 0:
                        # print('PAYLOAD LENGTH: ' + str(self.payload_length))
                        self.unpack_payload(raw_msg[4:4+self.payload_length]) #because payload starts at 4th index

                    self.checksum = checksum[0]
                elif local_checksum == checksum[0] and checksum[0] == last_checksum:
                    print('duplilcate message ignored!!')

 
        return

    def is_valid(self):
        return self.is_message_valid
    
    def unpack_payload(self, info):
        # print('unpacking: ' + str(info))
        if self.message_type == 0xA1: #This is the hex value for start
            self.payload = struct.unpack('f', info)[0]
            # print(self.payload)
        elif self.message_type == 0xA3: #this is the hex value for update
            self.payload = info.decode('utf-8')
            # print(self.payload)

#map the states of the robot to hex values for easier sending.
state_string_map = {'startup':0x01, 'preflight': 0x02, 'idle':0x03, 'running': 0x04,
                    'shutting down':0x05, 'logFetch': 0x06, 'rmLogs': 0x07, 'sleep': 0x08}
    
def pack_atc_reply(state_string, last_rec, data_manager, payload, low_voltage = False):
    '''
    Pack up a reply to an atc message
    '''

    #message in the form: 0xAC, 0xDA, payload length, last command recieved, safety, voltage, state, low voltage warning, payload, checkksum

    payload_length = 0
    if len(payload)>0:
        for item in payload:
            payload_length += len(item)

    msg = struct.pack('BBB', 0xAC, 0xDA, payload_length)
    msg = msg + struct.pack('B', last_rec)

    #Get the battery voltage * 10 and turn into an int
    volts = int(data_manager.get_battery_voltage()*10)

    #get the robot's safety value
    safety = data_manager.get_safety()

    #pack the safety value and voltage
    msg = msg + struct.pack('Bb', safety, volts)

    #pack the state
    msg = msg + struct.pack('B', state_string_map[state_string])

    #pack the low voltage warning
    pack_val = 0x00 if not low_voltage else 0xFF
    msg = msg + struct.pack('B', pack_val)

    #pack the rest of the payload here
    if payload_length > 0:
        for item in payload:
            msg = msg + item
    

    checksum = 0x00
    for item in msg:
        checksum ^= item

    msg = msg + struct.pack('B', checksum)

    return msg




    
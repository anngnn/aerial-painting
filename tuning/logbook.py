'''
This code was developed by Andrew Curtis in the year 2024.
It was developed at Northwestern University, Evanston, IL, USA.

It has functions to help with the "Log Book" for each quadrotor in the QuadSwarm.
The bootloader writes all data into a flight log for each particular run.
EVERY TIME the drone is turned on, it logs data in the logs folder.

This module helps the bootloader do that by providing functions the bootloader can use.
'''

#import 3rd party libraries
#NONE

#import os files (files NU created)
#NONE

#import native libraries
import datetime
import csv

def open_file():
    '''
    Creates a csv file to log robot information
    Returns the csv file and a corresponding file writer
    '''
    timestamp = get_time_stamp()
    filename = 'logs/'+ timestamp + '_flight_log.csv'
    csvfile = open(filename, 'w', newline='')

    writer = csv.writer(csvfile)
    writer.writerow(['Welcome! This is the first entry in the log book at the onset of the bootloader main().'])
    writer.writerow(['Time stamp: ' + get_time_stamp()])
    csvfile.flush()

    return csvfile, writer


def get_time_stamp():
    '''
    Return a formatted string of 'year-month-day-time'
    '''
    #get the real date and time info for this very second.
    RDT = datetime.datetime.now()
    
    #format the hour to have a leading zero if its less than 10
    hour = str(RDT.hour)
    if RDT.hour < 10:
        hour = '0' + hour
    
    #format the minute to hae a leading zer of its less than 10
    minute = str(RDT.minute)
    if RDT.minute < 10:
        minute = '0' + minute

    #format the second to have a leading zero if its less than 10
    second = str(RDT.second)
    if RDT.second < 10:
        second = '0' + second

    #format the month to have a leading zero if less than 10
    month = str(RDT.month)
    if RDT.month < 10:
        month = '0' + month

    #format the day to have a leading zero if less than 10
    day = str(RDT.day)
    if RDT.day < 10:
        day = '0' + day
    
    time_string = hour + minute + second

    return str(RDT.year) + '-' + month + '-' + day + '-' + time_string
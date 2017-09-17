'''
MIT LICENSE

Copyright 2014 Inertial Sense, LLC - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
'''

import argparse
import sys

from time import sleep
from pySDK.inertialsensemodule import InertialSense, pyUpdateFlashConfig
from pySDK.display import cInertialSenseDisplay 
from pySDK.logger import cISLogger
import pySDK.isutilities as util 


# TODO: Make Defaults, Definitions and Member Variables available directly from C++ or put them in another python module
IS_COM_BAUDRATE_DEFAULT = 3000000
DEFAULT_LOGS_DIRECTORY = "IS_logs"
DMODE_PRETTY = 0
DMODE_SCROLL = 1
DMODE_STATS = 2
DMODE_QUITE = 3

def main():
    parser = argparse.ArgumentParser(description="Embeedded Test Analytics")
    parser.add_argument('-baud',          '--baudRate', type=int, help='Baud Rate', default=IS_COM_BAUDRATE_DEFAULT)
    parser.add_argument('-b',              '--bootloaderFileName', type=str, help='Path to Boot Loader File', default=None)
    parser.add_argument('-c',              '--comPort', type=str, help='COM Port Number (eg. com4)', default=None)
    parser.add_argument('-lms',          '--maxLogSpaceMB', type=int, help='Max Space Alloted to Logger', default=1024.0)
    parser.add_argument('-lmf',          '--maxLogFileSize', type=int, help='Max size of Log File', default= 1024 * 1024 * 5)
    parser.add_argument('-lmm',          '--maxLogMemory', type=int, help='Max Log Memory', default=131072)
    parser.add_argument('-lts',          '--useLogTimestampSubFolder', type=bool, help='Notification Email', default=True)
    parser.add_argument('-lp',          '--logPath', type=str, help='Log Path', default=DEFAULT_LOGS_DIRECTORY)
    parser.add_argument('-lt',          '--logType', type=str, help='Log Type (SDAT, DAT, CSV, KML)', default="DAT")
    parser.add_argument('-loff',          '--enableLogging', type=bool, help='Enable Logging', default=True)
    parser.add_argument('-q',              '--qlite', type=bool, help='Display: Qlite Mode', default=False)
    parser.add_argument('-rp',          '--replayDataLogPath', type=bool, help='Replay Data (True), Note: use logPath -lp for the directory location.', default=False)
    parser.add_argument('-rs',          '--replayDataSpeed', type=int, help='Data log replay speed', default=1.0)
    parser.add_argument('-r',            '--replayDataLog', type=bool, help='Config Json file', default=False)
    parser.add_argument('-sINS1',        '--sINS1', type=bool, help='Stream INL1 msg', default=False)
    parser.add_argument('-sINS2',        '--sINS2', type=bool, help='Stream INL2 msg', default=False)
    parser.add_argument('-sDualIMU',     '--sDualIMU', type=bool, help='Stream Dual IMU msg', default=False)
    parser.add_argument('-sIMU1',        '--sIMU1', type=bool, help='Stream IMU1 msg', default=False)
    parser.add_argument('-sIMU2',        '--sIMU2', type=bool, help='Stream IMU2 msg', default=False)
    parser.add_argument('-sGPS',         '--sGPS', type=bool, help='Stream GPS msg', default=False)
    parser.add_argument('-sRawGPS',      '--sRawGPS', type=bool, help='Stream GPS msg', default=False)
    parser.add_argument('-sMag1',        '--sMag1', type=bool, help='Stream Magnetometer1 msg', default=False)
    parser.add_argument('-sBaro',        '--sBaro', type=bool, help='Stream Barometer msg', default=False)
    parser.add_argument('-sSol',         '--sSol', type=bool, help='Stream Sol', default=False)
    parser.add_argument('-sSensors',     '--sSensors', type=bool, help='Stream Sensors', default=False)
    parser.add_argument('-sDThetaVel',     '--sDThetaVel', type=bool, help='Stream dThetaVel', default=False)
    parser.add_argument('-scroll',      '--scroll', type=bool, help='Display: Scroll Mode', default=False)
    parser.add_argument('-stats',          '--stats', type=bool, help='Display: Stats Mode', default=False)
    parser.add_argument('-svr',          '--serverConnection', type=str, help='Server Connection', default=None)
    parser.add_argument('-host',          '--host', type=str, help='Host', default=None)
    parser.add_argument('-flashConfig', '--flashConfig', type=str, help='Flash Config', default=None)

    # TODO: More Detailed Help (Same as C++ SDK)

    opts, extra = parser.parse_known_args()

    # TODO: Check for extra Args and throw error

    if opts.qlite is True: opts.displayMode = DMODE_QUITE
    elif opts.scroll is True: opts.displayMode = DMODE_SCROLL
    elif opts.stats is True: opts.displayMode = DMODE_STATS
    else: opts.displayMode = DMODE_PRETTY

    if opts.replayDataLogPath is True: opts.replayDataLog = True
    if opts.replayDataSpeed is not 1.0: opts.replayDataLog = True

    # Check for proper usage
    if opts.comPort is None and not opts.replayDataLog:
        print "ERROR - incompataible commnad line arguments"
        return False

    if opts.replayDataLog:
        if 'dat' in opts.logType:
            opts.logType = cISLogger.eLogType.LOGTYPE_DAT
        elif 'sdat' in opts.logType:
            opts.logType = cISLogger.eLogType.LOGTYPE_SDAT
        elif 'csv' in opts.logType:
            opts.logType = cISLogger.eLogType.LOGTYPE_CSV
        elif 'kml' in opts.logType:
            opts.logType = cISLogger.eLogType.LOGTYPE_KML
        else:
            print "ERROR: Invalid Log type"
            return

        print str(opts.logType)    

    opts.logSolution = 2 # INL2

    print "Inertial Sense - Python interface to CommandLine Tool"
    print "====================================================="

    CLTool(opts)

class CLTool(object):
    def __init__(self, opts):
        self.opts = opts
        self.display = cInertialSenseDisplay()
        self.cltool_main()

    def cltool_dataCallback(self,data):
        
        #print "Callback 2 - Received msg: %d" % data['header']['id']    

        # Example for data access
        DID_INS_2 = 5
        DID_DUAL_IMU = 58
        DID_DELTA_THETA_VEL = 3
        DID_GPS = 5
        DID_MAGNETOMETER_1 = 52
        DID_MAGNETOMETER_2 = 55
        DID_BAROMETER = 53
        DID_RAW_DATA = 60

        if data['header']['id'] == DID_INS_2:
            #print "qn2b Data Legth %d" % len(data['data']['qn2b'])     # quaternion attitude 
            #print "Quaternions: %f, %f, %f, %f" % (data['data']['qn2b'][0],data['data']['qn2b'][1],data['data']['qn2b'][2],data['data']['qn2b'][3])
            data['data']['qn2b']     # quaternion attitude 
            data['data']['uvw']      # body velocities
            data['data']['lla']      # latitude, longitude, altitude
         
        elif data['header']['id'] == DID_DUAL_IMU:    
            data['data']['time'];      
            data['data']['imu1acc']
            data['data']['imu1pqr']
            data['data']['imu2acc']
            data['data']['imu2pqr']  

        elif data['header']['id'] == DID_DELTA_THETA_VEL:   
            data['data']['time'] 
            data['data']['theta'] 
            data['data']['uvw'] 
            data['data']['dt']      

        elif data['header']['id'] == DID_GPS:                 
            data['data']['tOffset'] 
            data['data']['gpsPos']
            data['data']['gpsVel']
            data['data']['rxps']

        elif data['header']['id'] == DID_MAGNETOMETER_1:   
            data['data']['time']
            data['data']['mag']   

        #elif data['header']['id'] == DID_MAGNETOMETER_2:       # See Mag 1
        elif data['header']['id'] == DID_BAROMETER:              
            data['data']['time']
            data['data']['bar']
            data['data']['barTemp']
            data['data']['humidity']
            data['data']['mslBar']

        elif data['header']['id'] == DID_RAW_DATA:
            print "Received the Raw message in Python!!"
            data['data']['receiverIndex']
            data['data']['type']  # Indicates the message type
            data['data']['count']
            data['data']['reserved']
            data['data']['buf']   # 1020 byte buffer
            # Handle the different GPS Raw messages here 

        # elif ... (add other messages here)    
           

    def cltool_main(self):

        #clear display
        self.display.SetDisplayMode(self.opts.displayMode);
        self.display.Clear();

        # if replay data log specified on command line, do that now and return
        if self.opts.replayDataLog:
            return not self.__cltool_replayDataLog()
        
        # if bootloader was specified on the command line, do that now and return out
        elif self.opts.bootloaderFileName is not None:
            return self.__cltool_runBootloader()

        # if host was specified on the command line, create a tcp server
        elif self.opts.host is not None:
            return self.__cltool_createHost();
         
        # open the device, start streaming data and logging
        else:
            # [COMM INSTRUCTION] 1.) Create InertialSense object and open serial port. if reading/writing flash config, don't bother with data callback
            self.inertialSenseInterface = InertialSense()
            self.inertialSenseInterface.SetPyCallback(self.cltool_dataCallback)
            self.inertialSenseInterface.SetPyDisplay(self.display)
            if not self.inertialSenseInterface.Open(self.opts.comPort, self.opts.baudRate):          
                print "Failed to open serial port at %s - %d" % (self.opts.comPort, self.opts.baudRate)
                return -1;    # Failed to open serial port
            
            # [COMM INSTRUCTION] 2.) Enable data broadcasting from uINS
            if self.__cltool_setupCommunications():
                # [LOGGER INSTRUCTION] Setup and start data logger
                self.__cltool_setupLogger();

        try:
            # Main loop. Could be in separate thread if desired.
            while True:
                # [COMM INSTRUCTION] 3.) Process data and messages
                self.inertialSenseInterface.Update();
                if self.inertialSenseInterface.GetTcpByteCount() != 0:
                    self.display.GoToRow(1);
                    print "Tcp bytes read: %d"  % self.inertialSenseInterface.GetTcpByteCount()
                
                # Specify the minimum time between read/write updates.
                sleep(.001);
        
        except Exception as e: 
            print  "Unknown exception, %s" % str(e)
        
        except:
            # Catch System Exit or Keyboard Interupt
            pass 

        print "Shutting down..."

        # close the interface cleanly, this ensures serial port and any logging are shutdown properly
        self.inertialSenseInterface.Close();
        return 0;

    def __cltool_runBootloader(self):    
        # [BOOTLOADER INSTRUCTIONS] Update firmware
        print "Bootloading file at %s" % self.opts.bootloaderFileName
        uIns = InertialSense()
        success = uIns.PyBootloadFile(self.opts.comPort, self.opts.bootloaderFileName)
        if not success:
            print "Error bootloading file %s, error: %s" % (self.opts.bootloaderFileName)
        
        return success

    def __cltool_replayDataLog(self):
        if self.opts.logPath is None:    
            print "Please specify the replay log path!"
            return False
        
        print str(self.opts.logType)
        print type(self.opts.logType)

        logger = cISLogger();
        if not logger.LoadFromDirectory(self.opts.logPath,self.opts.logType):
            print "Failed to load log files: %s" % self.opts.logPath 
            return False;
        
        print "Replaying log files: %s" % self.opts.logPath

        # TODO: There are a lot of Debug statements stemming from SDK
        #   1. Consider creating a python specific Read method that calls the display (will need to set display)
        while True:
            data = logger.PyReadData(0)
            if 'header' not in data:
                break

            # Call Data Callback here to handle the data the same way as from the uINS    

        print "Done replaying log files: %s" % self.opts.logPath
        self.display.Goodbye();
        return True;

    def __cltool_setupCommunications(self):
        periodMs = 50;
        self.inertialSenseInterface.StopBroadcasts();    # Stop streaming any prior messages

        # ask for device info every 2 seconds
        DID_DEV_INFO = 1
        self.inertialSenseInterface.PyBroadcastBinaryData(DID_DEV_INFO, 2000);

        # depending on command line options. stream various data sets
        if self.opts.sSol:
            self.inertialSenseInterface.SetBroadcastSolutionEnabled(true);

        if self.opts.sINS1:
            DID_INS_1 = 4
            self.inertialSenseInterface.PyBroadcastBinaryData(DID_INS_1, periodMs);
        
        if self.opts.sINS2:
            DID_INS_2 = 5
            self.inertialSenseInterface.PyBroadcastBinaryData(DID_INS_2, periodMs);
        
        if self.opts.sSensors:
            DID_SYS_SENSORS = 11
            self.inertialSenseInterface.PyBroadcastBinaryData(DID_SYS_SENSORS, 100);
        
        if self.opts.sDualIMU:
            DID_DUAL_IMU = 58
            self.inertialSenseInterface.PyBroadcastBinaryData(DID_DUAL_IMU, periodMs);
        
        if self.opts.sIMU1:
            DID_IMU_1 = 2
            self.inertialSenseInterface.PyBroadcastBinaryData(DID_IMU_1, periodMs);
        
        if self.opts.sIMU2:
            DID_IMU_2 = 54
            self.inertialSenseInterface.PyBroadcastBinaryData(DID_IMU_2, periodMs);
        
        if self.opts.sDThetaVel:
            DID_DELTA_THETA_VEL = 3
            self.inertialSenseInterface.PyBroadcastBinaryData(DID_DELTA_THETA_VEL, periodMs);
        
        if self.opts.sGPS:
            DID_GPS = 6
            self.inertialSenseInterface.PyBroadcastBinaryData(DID_GPS, 200);
        
        if self.opts.sMag1:
            DID_MAGNETOMETER_1 = 52
            self.inertialSenseInterface.PyBroadcastBinaryData(DID_MAGNETOMETER_1, periodMs);
        
        if self.opts.sBaro:
            DID_BAROMETER = 53
            self.inertialSenseInterface.PyBroadcastBinaryData(DID_BAROMETER, periodMs);

        if self.opts.sRawGPS:
            DID_RAW_DATA = 60
            self.inertialSenseInterface.PyBroadcastBinaryData(DID_RAW_DATA, periodMs);    
        
        if self.opts.serverConnection is not None:
            if self.opts.serverConnection.find("RTCM3:") == 0:
                if not self.inertialSenseInterface.OpenServerConnectionRTCM3(self.opts.serverConnection.substr(6)):
                    print "Failed to connect to server." 
            elif self.opts.serverConnection.find("IS:") == 0:
                if not self.inertialSenseInterface.OpenServerConnectionInertialSense(self.opts.serverConnection.substr(3)):
                    print "Failed to connect to server." 
            elif self.opts.serverConnection.find("UBLOX:") == 0:
                if not self.inertialSenseInterface.OpenServerConnectionUblox(self.opts.serverConnection.substr(6)):
                    print "Failed to connect to server." 
            else:
                print "Invalid server connection, must prefix with RTCM3: or IS:, %s" % self.opts.serverConnection 
                return alse;
        if self.opts.flashConfig is not None:
            return cltool_updateFlashConfig(self.opts.flashConfig)
        return True

    def __cltool_setupLogger(self):
        # Enable logging in continuous background mode
        self.inertialSenseInterface.SetLoggerEnabled(
            self.opts.enableLogging, # enable logger
            self.opts.logPath, # path to log to, if empty defaults to DEFAULT_LOGS_DIRECTORY
            self.opts.logSolution, # solution logging options
            self.opts.maxLogSpaceMB, # max space in mb to use, 0 for unlimited - only MAX_PERCENT_OF_FREE_SPACE_TO_USE_FOR_IS_LOGS% of free space will ever be allocated
            self.opts.maxLogFileSize, # each log file will be no larger than this in bytes
            self.opts.maxLogMemory, # logger will try and keep under this amount of memory
            self.opts.useLogTimestampSubFolder # whether to place log files in a new sub-folder with the current timestamp as the folder name
        )

    def __cltool_updateFlashConfig(self):
       return pyUpdateFlashConfig(self.inertialSenseInterface,self.opts.flashConfig)
        

    def __cltool_createHost(self):
    
        self.inertialSenseInterface = InertialSense()
        if not self.inertialSenseInterface.Open(self.opts.comPort, self.opts.baudRate):
            print  "Failed to open serial port at %s" % self.opts.comPort
            return -1  # Failed to open serial port
        
        elif self.opts.flashConfig is not None and not self.__cltool_updateFlashConfig(self.inertialSenseInterface):
            return -1
        
        elif not self.inertialSenseInterface.CreateHost(self.opts.host):
            print "Failed to create host at %s" % self.opts.host
            return -1; # Failed to open host
        
        try: 
            while True:
                self.inertialSenseInterface.Update();
                self.display.Home();
                print "Tcp bytes sent: %d" % self.inertialSenseInterface.GetTcpByteCount()
                sleep(.001);

        except Exception as e: 
            print  "Unknown exception, %s" % str(e)
        
        except:
            # Catch System Exit or Keyboard Interupt
            pass 

        print "Shutting down..."

        # close the interface cleanly, this ensures serial port and any logging are shutdown properly
        self.inertialSenseInterface.Close();
        
        return 0;
    

if __name__ == "__main__":
    main()        
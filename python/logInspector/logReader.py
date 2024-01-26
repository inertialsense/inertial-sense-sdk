import numpy as np
import sys
import glob
import serial
import os
import subprocess
import yaml
import datetime

from os.path import expanduser, exists
from scipy.interpolate import interp1d

file_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(file_path + '/..'))
sys.path.append(os.path.normpath(file_path + '/../math/src'))

from log_reader import LogReader
# from ci_hdw.data_sets import *
from pylib.data_sets import *
from inertialsense_math.pose import *
from pylib.ISToolsDataSorted import refLla, getTimeFromTowMs, getTimeFromTow, setGpsWeek, getTimeFromGTime

RAD2DEG = 180.0 / np.pi
DEG2RAD = np.pi / 180.0

RED = '\u001b[31m'
RESET = '\u001b[0m'

INS_STATUS_NAV_MODE                         = 0x00001000
GPS_STATUS_FLAGS_GPS1_RTK_POSITION_ENABLED  = 0x00100000
GPS_STATUS_FLAGS_GPS2_RTK_COMPASS_ENABLED   = 0x00400000

class Log:
    def __init__(self):
        self.c_log = LogReader()
        self.init_vars()

    def init_vars(self):
        self.data = []
        self.serials = []
        self.passRMS = 0    # 1 = pass, -1 = fail, 0 = unknown
        self.refSerials = []
        self.refIdx = []
        self.devIdx = []
        self.refData = []
        self.truth = []
        self.refINS = False
        self.hardware = []
        self.numRef = 0
        self.refINS = False
        self.using_mounting_bias = False

    def load(self, directory, serials=['ALL']):
        self.init_vars()
        self.c_log.init(self, directory, serials)
        self.c_log.load()
        self.serials = self.c_log.getSerialNumbers()
        self.sanitize()
        self.data = np.array(self.data, dtype=object)
        self.directory = directory
        self.mount_bias_filepath = directory + '/angular_mount_bias.yml'
        self.numDev = self.data.shape[0]

        if self.numDev == 0:
            print("No devices found in log or no logs found!!!")
            return False
        if len(self.data[0, DID_DEV_INFO]):
            self.serials = [self.data[d, DID_DEV_INFO]['serialNumber'][0] for d in range(self.numDev)]

        for i in range(self.numDev):
            try:
                self.hardware.append(self.data[i, DID_DEV_INFO]['hardwareVer'][0][0])
            except:
                self.hardware.append(0)
            if any(self.data[i,DID_FLASH_CONFIG]['sysCfgBits'] & eSysConfigBits.SYS_CFG_USE_REFERENCE_IMU_IN_EKF.value):
                # Use this INS as reference
                self.refINS = True
                self.serials[i] = 'Reference IMU'
                self.refIdx.append(i)
                self.numRef = self.numRef + 1
                if len(self.data[i, DID_DEV_INFO]):
                    self.refSerials.append(self.data[i, DID_DEV_INFO]['serialNumber'][0])
            else:
                self.devIdx.append(i)
            if self.serials[i] == 10101 or self.serials[i] == 99999:
                # Use Novatel INS as reference, discard previously found references
                self.serials[i] = 'Ref INS'
                self.refINS = True
                self.refIdx.clear()
                self.refIdx.append(i)
                self.numRef = 1
                if len(self.data[i, DID_DEV_INFO]):
                    self.refSerials.clear()
                    self.refSerials.append(self.data[i, DID_DEV_INFO]['serialNumber'][0])
            if len(self.data[0, DID_INS_2]) == 0 and len(self.data[0, DID_INS_1]) != 0:
                self.ins1ToIns2(i)
            #If you want to view data of log with only refIns:
            if len(self.serials) == 1 and self.refINS == True:
                return True

        if len(self.serials) == len(self.refSerials):
            self.devIdx = self.refIdx

#        if self.refINS and len(self.serials) > len(self.refSerials):
#            self.serials = np.delete(self.serials, self.refIdx, 0)

        if len(self.refIdx):
            self.refData = self.data[self.refIdx].copy()

        self.numIns = self.numDev 
        if(len(self.serials) > len(self.refSerials)):
            self.numIns = self.numIns - self.numRef

        self.mount_bias_euler = np.zeros([self.numDev, 3], dtype=float)
        if exists(self.mount_bias_filepath):
            with open(self.mount_bias_filepath, 'r') as file:
                mount_bias = yaml.safe_load(file)
                for n, dev in enumerate(self.serials):
                    if (n < self.numIns) and (int(dev) in mount_bias):
                        self.mount_bias_euler[n, :] = np.array(mount_bias[int(dev)])
                        self.using_mounting_bias = True
        self.mount_bias_quat = euler2quat(self.mount_bias_euler)

        self.compassing = None  
        self.rtk = None  
        self.navMode = None  
        ins2 = self.data[0, DID_INS_2]
        if len(ins2):
            self.navMode    = (ins2['insStatus'][-1] & INS_STATUS_NAV_MODE) == INS_STATUS_NAV_MODE
        gps1Pos = self.data[0, DID_GPS1_POS]
        if len(gps1Pos):
            self.rtk        = (gps1Pos['status'][-1] & GPS_STATUS_FLAGS_GPS1_RTK_POSITION_ENABLED) == GPS_STATUS_FLAGS_GPS1_RTK_POSITION_ENABLED
            self.compassing = (gps1Pos['status'][-1] & GPS_STATUS_FLAGS_GPS2_RTK_COMPASS_ENABLED) == GPS_STATUS_FLAGS_GPS2_RTK_COMPASS_ENABLED

        # Reference INS like Novatel may not have all status fields. Find a first device that is not reference
        uINS_device_idx = [n for n in range(self.numDev) if n in self.devIdx and not (n in self.refIdx)]
        idx = uINS_device_idx[0]
        # except:
            # print(RED + "error loading log" + sys.exc_info()[0] + RESET)
        return True

    def getSerialNumbers(self):
        return self.c_log.getSerialNumbers()

    def protocolVersion(self):
        return self.c_log.protocolVersion()

    def ins1ToIns2(self, device_id=0):
        self.c_log.ins1ToIns2(device_id)

    def exitHack(self, exit_code=0):
        self.c_log.exitHack(exit_code)
        
    def did_callback(self, did, arr, dev_id):
        if did >= NUM_DIDS:
            return
        while dev_id >= len(self.data):
            self.data.append([[] for i in range(NUM_DIDS+1)])
        self.data[dev_id][did] = arr

    def gps_raw_data_callback(self, did, arr, dev_id, msg_type):
        if did >= NUM_DIDS:
            return
        if self.data[dev_id][did] == []:
            self.data[dev_id][did] = [[] for i in range(6)]
        if dev_id < len(self.data) and did < len(self.data[dev_id]) and 6 <= len(self.data[dev_id][did]):
            if msg_type == eRawDataType.raw_data_type_observation.value:
                self.data[dev_id][did][0].append(arr)
            elif msg_type == eRawDataType.raw_data_type_ephemeris.value:
                self.data[dev_id][did][1] = arr
            elif msg_type == eRawDataType.raw_data_type_glonass_ephemeris.value:
                self.data[dev_id][did][2] = arr
            elif msg_type == eRawDataType.raw_data_type_sbas.value:
                self.data[dev_id][did][3] = arr
            elif msg_type == eRawDataType.raw_data_type_ionosphere_model_utc_alm.value:
                self.data[dev_id][did][4] = arr
            elif msg_type == eRawDataType.raw_data_type_base_station_antenna_position.value:
                self.data[dev_id][did][5] = arr


    def sanitize(self):
        return
        GPS_start_Time = datetime.datetime.strptime('6/Jan/1980', "%d/%b/%Y")

        # Use DID_INS_1 if necessary
        if np.size(self.data[0][DID_INS_2])==0 and np.size(self.data[0][DID_INS_1])!=0:
            self.data[0][DID_INS_2].resize(np.size(self.data[0][DID_INS_1]))
            self.data[0][DID_INS_2]['timeOfWeek'] = self.data[0][DID_INS_1]['timeOfWeek']
            self.data[0][DID_INS_2]['week'] = self.data[0][DID_INS_1]['week']
            self.data[0][DID_INS_2]['insStatus'] = self.data[0][DID_INS_1]['insStatus']
            self.data[0][DID_INS_2]['hdwStatus'] = self.data[0][DID_INS_1]['hdwStatus']
            self.data[0][DID_INS_2]['qn2b'] = euler2quatArray(self.data[0][DID_INS_1]['theta'])
            self.data[0][DID_INS_2]['uvw'] = self.data[0][DID_INS_1]['uvw']
            self.data[0][DID_INS_2]['lla'] = self.data[0][DID_INS_1]['lla']

        week_time = GPS_start_Time + (datetime.timedelta(weeks=int(self.data[0][DID_INS_2]['week'][-1])))

        for d, dev in enumerate(self.data):
            if len(dev[DID_INS_2]) == 0:
                print("\033[93m" + "missing DID_INS_2 data: removing device\033[0m")
                del self.data[d]
                break

            if len(dev[DID_DEV_INFO]) == 0:
                print("\033[93m" + "missing DID_DEV_INFO data: making some up\033[0m")
                self.data[d][DID_DEV_INFO] = np.resize(self.data[d][DID_DEV_INFO], 1)
                self.data[d][DID_DEV_INFO]['serialNumber'][0] = d

        for d, dev in enumerate(self.data):
            for did in range(len(self.data[0])):
                if isinstance(dev[did], list):
                    continue
                for field in ['towMs', 'timeOfWeekMs', 'tow', 'timeOfWeek']:
                    if field in dev[did].dtype.names:
                        if (np.diff(dev[did][field].astype(np.int64)) < 0).any():
                            idx = np.argmin(np.diff(dev[did][field].astype(np.int64)))
                            if 'Ms' in field:
                                t1 = week_time + datetime.timedelta(milliseconds=int(dev[did][field][idx]))
                                t2 = week_time + datetime.timedelta(milliseconds=int(dev[did][field][idx+1]))
                            else:
                                t1 = week_time + datetime.timedelta(seconds=int(dev[did][field][idx]))
                                t2 = week_time + datetime.timedelta(seconds=int(dev[did][field][idx+1]))
                            print("\033[93m" + "Time went backwards in " + did_name_lookup[did] + r"!!!, " +
                                  " Time went from " + str(t1) + " to "  + str(t2) + ". removing all data "
                                  + ("before" if idx < len(dev[did][field])/2.0 else "after") + "\033[0m")
                            if idx < len(dev[did][field])/2.0:
                                self.data[d][did] = dev[did][idx+1:]
                            else:
                                self.data[d][did] = dev[did][:idx]
                        ms_multiplier = 1000.0 if 'Ms' in field else 1.0
                        if (np.diff(dev[did][field]) > 3600 * ms_multiplier).any():
                            print("\033[93m" + "greater than 1 minute gap in " + did_name_lookup[did]
                                  + " data, assuming GPS fix was acquired during data set, and chopping data"+ "\033[0m")
                            idx = np.argmax(np.diff(dev[did][field])) + did
                            self.data[d][did] = dev[did][idx:]

    def getRMSArray(self):
        if self.numDev > 1 or self.refINS:
            print("\nComputing RMS Accuracies: (%d devices)" % (self.numIns))

            # Build a 3D array of the data.  idx 0 = Device,    idx 1 = t,     idx 2 = [t, lla, uvw, log(q)]
            data = [np.hstack((self.data[i, DID_INS_2]['timeOfWeek'][:, None],
                               self.data[i, DID_INS_2]['lla'],
                               self.data[i, DID_INS_2]['uvw'],
                               self.data[i, DID_INS_2]['qn2b'])) for i in range(self.numDev)]

            # Make sure that the time stamps are realistic
            for dev in range(self.numDev):
                if (np.diff(data[dev][:, 0]) > 10.0).any():
                    print("\033[93m" + "large gaps in data for dev" + str(dev)
                          + "chopping off data before gap".format(dev) + "\033[0m")
                    idx = np.argmax(np.diff(data[dev][:, 0])) + 1
                    data[dev] = data[dev][idx:, :]

            self.min_time = max([np.min(data[i][:, 0]) for i in range(self.numDev)])
            self.max_time = min([np.max(data[i][:, 0]) for i in range(self.numDev)])

            # If we are in compassing mode, then only calculate RMS after all devices have fix
            if self.compassing:
                # time_of_fix_ms = [self.data[dev, DID_GPS1_RTK_CMP_REL]['timeOfWeekMs'][np.argmax(self.data[dev, DID_GPS1_RTK_CMP_REL]['arRatio'] > 3.0)] / 1000.0 for dev in range(self.numDev)]
                time_of_fix_ms = [self.data[dev, DID_GPS1_POS]['timeOfWeekMs'][np.argmax(self.data[dev, DID_GPS1_POS]['status'] & 0x08000000)] / 1000.0 for dev in range(self.numDev)]
                # print time_of_fix_ms
                self.min_time = max(time_of_fix_ms)

            # Use only partial data for RMS calculations
            self.min_time = self.max_time - (self.max_time - self.min_time)*(2.0/3.0)  # do not use the first 1/3 (alignment)
            # self.min_time = self.max_time - (self.max_time - self.min_time)*(1.0/2.0)  # do not use the first 1/2 (alignment)

            # Resample at a steady 10 Hz
            dt = 0.1
            # dt = np.average(data[0][1:,0] - data[0][:-1,0])
            t = np.arange(1.0, self.max_time - self.min_time - 1.0, dt)
            for i in range(self.numDev):
                # Chop off extra data at beginning and end
                data[i] = data[i][data[i][:, 0] > self.min_time]
                data[i] = data[i][data[i][:, 0] < self.max_time]

                # Chop off the min time so everything is wrt to start
                data[i][:, 0] -= self.min_time

                # Interpolate data so that it has all the same timestamps
                fi = interp1d(data[i][:, 0], data[i][:, 1:].T, kind='linear', fill_value='extrapolate',
                              bounds_error=False)
                data[i] = np.hstack((t[:, None], fi(t).T))

                # Normalize Quaternions
                data[i][:, 7:] /= norm(data[i][:, 7:], axis=1)[:, None]

            # Make a big 3D numpy array we can work with [dev, sample, data]
            data = np.array(data)

            # Convert lla to ned using first device lla at center of data as reference
            refLla = data[0, int(round(len(t) / 2.0)), 1:4].copy()
            for i in range(self.numDev):
                data[i, :, 1:4] = lla2ned(refLla, data[i, :, 1:4])
            self.stateArray = data

    def getRMSTruth(self):
        means = np.empty((len(self.stateArray[0]), 10))
        if not self.refINS:
            refData = self.stateArray
        else:
            refData = self.stateArray[self.refIdx, :, :]
            if len(self.serials) > len(self.refSerials):
                self.stateArray = np.delete(self.stateArray, self.refIdx, 0)
        # Find Mean Data
        means[:, :6] = np.mean(refData[:, :, 1:7], axis=0)  # calculate mean position and velocity across devices
        means[:, 6:] = meanOfQuatArray(refData[:, :, 7:].transpose((1, 0, 2)))  # Calculate mean attitude of all devices at each timestep
        self.truth = means

    def calcAttitudeError(self):
        att_error = np.array([qboxminus(self.stateArray[dev, :, 7:], self.truth[:, 6:]) for dev in range(len(self.stateArray))])
        self.att_error = att_error

    def calculateRMS(self):
        self.data = np.array(self.data)
        self.getRMSArray()
        self.getRMSTruth()
        self.calcAttitudeError()


        # Calculate the Mounting Bias for all devices (assume the mounting bias is the mean of the attitude error)
        uINS_device_idx = [n for n in range(self.numDev) if n in self.devIdx and not (n in self.refIdx)]
        self.uvw_error = np.empty_like(self.stateArray[:, :, 4:7])
        mount_bias_output = dict()
        for n, dev in enumerate(uINS_device_idx):
            mount_bias = np.mean(self.att_error[n, :, :], axis=0)
            #if self.compassing:
                # When in compassing, assume all units are sharing the same GPS antennas and should therefore have
                # no mounting bias in heading
                # NOTE 2: The units can have a common mounting bias relative to antenna offsets. So we should keep non-zero mount_bias[2]
            #    mount_bias[2] = 0
            self.mount_bias_euler[dev, :] = mount_bias  # quat2eulerArray(qexp(mount_bias))
            self.mount_bias_quat[dev,:] = euler2quat(self.mount_bias_euler[dev, :])
            self.using_mounting_bias = True
            self.att_error[n, :, :] = self.att_error[n, :, :] - mount_bias[None, :]
            self.uvw_error[n, :, :] = quatRot(self.mount_bias_quat[dev,:], self.stateArray[n, :, 4:7]) - self.truth[:,3:6]
            mount_bias_output[int(self.serials[dev])] = mount_bias.tolist()

        # Writing mounting bias to file 
        with open(self.mount_bias_filepath, 'w') as file:
            yaml.dump(mount_bias_output, file)

        # RMS = sqrt ( 1/N sum(e^2) )
        self.RMSNED = np.sqrt(np.mean(np.square(self.stateArray[:, :, 1:4] - self.truth[:,0:3]), axis=1)) # [ pos ]
        self.RMSUVW = np.sqrt(np.mean(np.square(self.uvw_error), axis=1)) # [ vel }
        self.RMSAtt = np.sqrt(np.mean(np.square(self.att_error), axis=1)) # [ att }

        # Average RMS across devices
        self.averageRMSNED = np.mean(self.RMSNED, axis=0)
        self.averageRMSUVW = np.mean(self.RMSUVW, axis=0)
        self.averageRMSAtt = np.mean(self.RMSAtt, axis=0)

    def pass_fail(self, ratio):
        if ratio > 1.0:
            self.tmpPassRMS = -1
            return 'FAIL'
        else:
            # self.tmpPassRMS = -1  # Debug
            return 'PASS'

    def printRMSReport(self):
        uINS_device_idx = [n for n in range(self.numDev) if n in self.devIdx and not (n in self.refIdx)]
        self.tmpPassRMS = 1
        filename = os.path.join(self.directory, 'RMS_report_new_logger.txt')
        # Make sure all devices have the same hardware
        hardware = self.hardware[uINS_device_idx[0]]
        for dev in uINS_device_idx:
            if self.hardware[dev] != hardware:
                # Use default value if not all devices use the same hardware
                hardware = 0

        # Thresholds for uINS-3
        # Nav
        thresholdNED = np.array([0.35,  0.35, 0.8])     # (m)   NED
        thresholdUVW = np.array([0.04,  0.04, 0.07])    # (m/s) UVW
        thresholdAtt = np.array([0.11,  0.11, 0.3])     # (deg) Att (roll, pitch, yaw)
        if not self.navMode:
            # AHRS
            thresholdAtt[2]  = 2.0  # (deg) Att (yaw)

        # Thresholds for IMX-5
        if hardware == 5:
            # Nav 
            thresholdNED = np.array([0.35,  0.35,  0.8])    # (m)   NED
            thresholdUVW = np.array([0.035, 0.035, 0.07])   # (m/s) UVW
            thresholdAtt = np.array([0.045, 0.045, 0.16])   # (deg) Att (roll, pitch, yaw)
            if not self.navMode: 
                # AHRS
                thresholdAtt[:2] = 0.1  # (deg) Att (roll, pitch)
                thresholdAtt[2]  = 1.0  # (deg) Att (yaw)

        if self.compassing:
            thresholdNED[:2] = 0.5

        if self.refINS:     # SPAN INS has position offset
            thresholdNED[:2] =  2.5
            thresholdNED[2]  =  6.0
        elif self.rtk:      # RTK positioning w/o ref INS
            thresholdNED[:2] = 0.04
            thresholdNED[2]  = 0.05

        if not (self.navMode or self.compassing):
            thresholdNED[:] = np.inf    # Disable NED
            thresholdUVW[:] = np.inf    # Disable UVW

        thresholdAtt[:] *= DEG2RAD  # convert degrees threshold to radians

        self.specRatioNED = self.averageRMSNED / thresholdNED
        self.specRatioUVW = self.averageRMSUVW / thresholdUVW
        self.specRatioAtt = self.averageRMSAtt / thresholdAtt

        f = open(filename, 'w')
        f.write('*****   Performance Analysis Report - %s   *****\n' % (self.directory))
        f.write('\n')
        mode = ('IMX-5' if hardware == 5 else 'uINS-3' )
        mode += (", NAV" if self.navMode else ", AHRS")
        if self.rtk:        mode += ", RTK"
        if self.compassing: mode += ", DUAL GNSS"
        if self.refINS:     mode += ", Ref INS"

        # Print Table of RMS accuracies
        line = 'Device      '
        if self.navMode:
            f.write(
                '------------------------------------------------- RMS Accuracy -------------------------------------------\n')
        else:  # AHRS mode
            f.write(
                '-------------- RMS Accuracy --------------\n')
        line += ' Att[  (deg)   (deg)   (deg) ]'
        if self.navMode:
            line += ',  UVW[  (m/s)   (m/s)   (m/s) ],  NED[    (m)     (m)     (m) ]'
        line += '\n'
        f.write(line)

        for n, dev in enumerate(uINS_device_idx):
            devInfo = self.data[dev,DID_DEV_INFO][0]
            line = '%2d SN%d      ' % (n, devInfo['serialNumber'])
            line += '[ %6.4f  %6.4f  %6.4f ]' % (
            self.RMSAtt[n, 0] * RAD2DEG, self.RMSAtt[n, 1] * RAD2DEG, self.RMSAtt[n, 2] * RAD2DEG)
            if self.navMode:
                line += ',     [ %6.4f  %6.4f  %6.4f ]' % (self.RMSUVW[n, 0], self.RMSUVW[n, 1], self.RMSUVW[n, 2])
                line += ',     [ %6.4f  %6.4f  %6.4f ]' % (self.RMSNED[n, 0], self.RMSNED[n, 1], self.RMSNED[n, 2])
            line += '\n'
            f.write(line)

        if self.navMode:
            f.write(
                '----------------------------------------------------------------------------------------------------------\n')
        else:  # AHRS mode
            f.write(
                '------------------------------------------\n')
        line = 'AVERAGE:        '
        line += '[%7.4f %7.4f %7.4f ]' % (
        self.averageRMSAtt[0] * RAD2DEG, self.averageRMSAtt[1] * RAD2DEG, self.averageRMSAtt[2] * RAD2DEG)
        if self.navMode:
            line += ',     [%7.4f %7.4f %7.4f ]' % (self.averageRMSUVW[0], self.averageRMSUVW[1], self.averageRMSUVW[2])
            line += ',     [%7.4f %7.4f %7.4f ]' % (self.averageRMSNED[0], self.averageRMSNED[1], self.averageRMSNED[2])
        line += '\n'
        f.write(line)

        line = 'THRESHOLD:      '
        line += '[%7.4f %7.4f %7.4f ]' % (
        thresholdAtt[0] * RAD2DEG, thresholdAtt[1] * RAD2DEG, thresholdAtt[2] * RAD2DEG)
        if self.navMode:
            line += ',     [%7.4f %7.4f %7.4f ]' % (thresholdUVW[0], thresholdUVW[1], thresholdUVW[2])
            line += ',     [%7.4f %7.4f %7.4f ]' % (thresholdNED[0], thresholdNED[1], thresholdNED[2])
        line += '\n'
        f.write(line)

        if self.navMode:
            f.write(
                '----------------------------------------------------------------------------------------------------------\n')
        else:  # AHRS mode
            f.write('------------------------------------------\n')
        line = 'RATIO:          '
        line += '[%7.4f %7.4f %7.4f ]' % (self.specRatioAtt[0], self.specRatioAtt[1], self.specRatioAtt[2])
        if self.navMode:
            line += ',     [%7.4f %7.4f %7.4f ]' % (self.specRatioUVW[0], self.specRatioUVW[1], self.specRatioUVW[2])
            line += ',     [%7.4f %7.4f %7.4f ]' % (self.specRatioNED[0], self.specRatioNED[1], self.specRatioNED[2])
        line += '\n'
        f.write(line)

        line = 'PASS/FAIL:      '
        line += '[   %s    %s    %s ]' % (
        self.pass_fail(self.specRatioAtt[0]), 
        self.pass_fail(self.specRatioAtt[1]), 
        self.pass_fail(self.specRatioAtt[2]))  # ATT
        if self.navMode:
            line += ',     [   %s    %s    %s ]' % (
            self.pass_fail(self.specRatioUVW[0]), 
            self.pass_fail(self.specRatioUVW[1]), 
            self.pass_fail(self.specRatioUVW[2]))  # UVW
            line += ',     [   %s    %s    %s ]' % (
            self.pass_fail(self.specRatioNED[0]), 
            self.pass_fail(self.specRatioNED[1]), 
            self.pass_fail(self.specRatioNED[2]))  # NED
        line += '\n'
        f.write(line)

        f.write('MODE:            (' + mode + ')\n\n')

        # Print Mounting Biases
        f.write('--------------- Angular Mounting Biases ----------------\n')
        f.write('Device       Euler Biases[   (deg)     (deg)     (deg) ]\n')
        for dev in uINS_device_idx:
            devInfo = self.data[dev, DID_DEV_INFO][0]
            f.write('%2d SN%d               [ %7.4f   %7.4f   %7.4f ]\n' % (
                n, devInfo['serialNumber'], 
                self.mount_bias_euler[dev, 0] * RAD2DEG, 
                self.mount_bias_euler[dev, 1] * RAD2DEG,
                self.mount_bias_euler[dev, 2] * RAD2DEG))
        f.write('\n')

        f.write("----------------- Average Attitude ---------------------\n")
        f.write("Dev:  \t[ Roll\t\tPitch\t\tYaw ]\n")
        for i in range(self.numIns):
            qavg = meanOfQuat(self.stateArray[i, :, 7:])[0]
            euler = quat2euler(qavg.T) * 180.0 / np.pi
            f.write("%d\t%f\t%f\t%f\n" % (self.serials[uINS_device_idx[i]], euler[0], euler[1], euler[2]))

        # Print Device Version Information
        f.write(
            '\n\n------------------------------------------- Device Info -------------------------------------------------\n')
        for n, dev in enumerate(uINS_device_idx):
            devInfo = self.data[dev, DID_DEV_INFO][0]
            hver = devInfo['hardwareVer']
            cver = devInfo['protocolVer']
            fver = devInfo['firmwareVer']
            buld = devInfo['buildNumber']
            repo = devInfo['repoRevision']
            date = devInfo['buildDate']
            time = devInfo['buildTime']
            addi = devInfo['addInfo']
            f.write(
                '%2d SN%d  HW: %d.%d.%d.%d   FW: %d.%d.%d.%d build %d repo %d   Proto: %d.%d.%d.%d  Date: %04d-%02d-%02d %02d:%02d:%02d  %s\n' % (
                    n, devInfo['serialNumber'],
                    hver[3], hver[2], hver[1], hver[0],
                    fver[3], fver[2], fver[1], fver[0], buld, repo,
                    cver[3], cver[2], cver[1], cver[0],
                    2000 + date[2], date[1], date[0],
                    time[3], time[2], time[1],
                    addi))
        f.write('\n')

        f.close()

        # Report if RMS passed all
        self.passRMS = self.tmpPassRMS
        return self.passRMS

    def debugPlot(self):
        import matplotlib.pyplot as plt
        colors = ['r', 'g', 'b', 'm']
        plt.figure()
        plt.subplot(3,1,1) # Position
        plt.title("position error")
        for m in range(3):
            for n in range(len(self.stateArray)):
                plt.plot(self.stateArray[n,:,0], self.stateArray[n, :, m+1], color = colors[m])
            plt.plot(self.stateArray[0,:,0], self.truth[:, m], linewidth=2, color = colors[m])
        plt.subplot(3,1,2)
        plt.title("velocity error")
        for m in range(3):
            for n in range(len(self.stateArray)):
                plt.plot(self.stateArray[n,:,0], self.stateArray[n, :, m+4], color = colors[m] )
            plt.plot(self.stateArray[0,:,0], self.truth[:, m+3], linewidth=2, color = colors[m])
        plt.subplot(3,1,3)
        plt.title("attitude")
        for m in range(4):
            for n in range(len(self.stateArray)):
                plt.plot(self.stateArray[n,:,0], self.stateArray[n, :, m+7], color = colors[m])
            plt.plot(self.stateArray[0,:,0], self.truth[:, m+6], linewidth=2, color = colors[m])

        plt.figure()
        for m in range(3):
            plt.subplot(3, 1, m +1)
            for n in range(len(self.stateArray)):
                plt.plot(self.att_error[n, :, m])
        plt.show()

    # This does not work when running in debug mode
    def openRMSReport(self):
        filename = os.path.join(self.directory, 'RMS_report_new_logger.txt')
        if 'win' in sys.platform:
            subprocess.Popen(["notepad.exe", filename])
        if 'linux' in sys.platform:
            subprocess.Popen(['gedit', filename])


if __name__ == '__main__':
    np.set_printoptions(linewidth=200)
    
    home = expanduser("~")

    # 2nd argument: Log Directory
    if len(sys.argv) >= 2:
        directory = sys.argv[1]
        serials = ["ALL"]

    if 'directory' not in locals():
        print("First parameter must be directory!")
        exit()

        # Load from config.yaml
        file = open(home + "/Documents/Inertial_Sense/config.yaml", 'r')
        config = yaml.load(file)
        directory = config["directory"]
        serials = ["ALL"]

    log = Log()
    if log.load(directory):
        # Compute and output RMS Report
        log.calculateRMS()
        # log.debugPlot()
        log.printRMSReport()
        log.openRMSReport()

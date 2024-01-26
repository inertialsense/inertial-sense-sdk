from ctypes import sizeof
import math, allantools, sys, yaml, os
from typing import List, Any, Union

import numpy as np
import matplotlib.pyplot as plt
from os.path import expanduser
from inertialsense_math.pose import *
from datetime import date

BLACK = r"\u001b[30m"
RED = r"\u001b[31m"
GREEN = r"\u001b[32m"
YELLOW = r"\u001b[33m"
BLUE = r"\u001b[34m"
MAGENTA = r"\u001b[35m"
CYAN = r"\u001b[36m"
WHITE = r"\u001b[37m"
RESET = r"\u001b[0m"

RAD2DEG = 180.0 / 3.14159
DEG2RAD = 3.14159 / 180.0

RTHR2RTS = 60 # sqrt(hr) to sqrt(sec)

SHOW_GPS2 = 0
SHOW_GPS_W_INS = 0

file_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(file_path + '/..'))
sys.path.append(os.path.normpath(file_path + '/../math/src'))

from logReader import Log
from pylib.ISToolsDataSorted import refLla, getTimeFromTowMs, getTimeFromTow, setGpsWeek, getTimeFromGTime
from pylib.data_sets import *
from inertialsense_math.pose import quat2euler, lla2ned, rotmat_ecef2ned, quatRot, quatConjRot, quat_ecef2ned
import datetime

class logPlot:
    def __init__(self, show, save, format, log):
        self.show = show
        self.save = save
        self.format = format
        self.d = 1
        self.residual = False
        if log:
            self.setLog(log)
        else:
            self.log = None

    def setLog(self, log):
        self.log = log
        self.directory = log.directory
        self.setActiveSerials(self.log.serials)
        if len(self.log.data[0, DID_INS_2]):
            setGpsWeek(self.log.data[0, DID_INS_2]['week'][-1])

    def setDownSample(self, dwns):
        self.d = dwns

    def enableResidualPlot(self, enable):
        self.residual = enable

    def setActiveSerials(self, serials):
        self.active_devs = []
        self.active_devs_no_ref = []
        for d, ser in enumerate(self.log.serials):
            if ser in serials:
                self.active_devs.append(d)
                if ser != 'Ref INS':
                    self.active_devs_no_ref.append(d)

    def configureSubplot(self, ax, title, ylabel='', xlabel=''):
        ax.set_title(title)
        ax.set_ylabel(ylabel)
        ax.set_xlabel(xlabel)

    def saveFig(self, fig, name, sizeInches=[]):
        if self.save:
            restoreSize = fig.get_size_inches()
            if not sizeInches:
                if self.format == 'png':     # Increase size/resolution for saved png
                    sizeInches = [16,11]
                    # sizeInches = [20,14]
                else: # svg or png
                    sizeInches = [11,8]
            fig.set_size_inches(sizeInches)
            directory = os.path.dirname(self.directory + '/figures/')
            if not os.path.exists(directory):
                os.makedirs(directory)
            fig.savefig(os.path.join(directory + "/" + name + '.' + self.format), bbox_inches='tight')
            fig.set_size_inches(restoreSize)

    def getData(self, dev, DID, field):
        try:
            return self.log.data[dev, DID][field][::self.d]
        except:
            return []

    def setPlotYSpanMin(self, ax, limit):
        ylim = ax.get_ylim()
        yspn = np.max( [ylim[1] - ylim[0], limit] )
        ylim = (np.mean(ylim)-yspn/2, np.mean(ylim)+yspn/2)
        ax.set_ylim(ylim)

    def posNED(self, fig=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(3, (2 if self.residual else 1), sharex=True, squeeze=False)
        self.configureSubplot(ax[0,0], 'North', 'm')
        self.configureSubplot(ax[1,0], 'East', 'm')
        self.configureSubplot(ax[2,0], 'Down', 'm')
        fig.suptitle('INS NED - ' + os.path.basename(os.path.normpath(self.log.directory)))
        refLla = None
        refTime = None
        refNed = None
        sumDelta = None
        sumCount = 1
        if self.residual:
            self.configureSubplot(ax[0,1], 'North Residual', 'm')
            self.configureSubplot(ax[1,1], 'East Residual',  'm')
            self.configureSubplot(ax[2,1], 'Down Residual',  'm')
            # Use 'Ref INS' if available
            for d in self.active_devs:
               if self.log.serials[d] == 'Ref INS':
                    refTime = getTimeFromTow(self.getData(d, DID_INS_2, 'timeOfWeek'))
                    refLla = self.getData(d, DID_INS_2, 'lla')[0]
                    continue
            # If 'Ref INS' is not available, use GPS as reference
            if refTime is None:
                for d in self.active_devs:
                    refTime = getTimeFromTowMs(self.getData(d, DID_GPS1_POS, 'timeOfWeekMs'))
                    refLla = self.getData(d, DID_GPS1_POS, 'lla')[0]
                    refNed = lla2ned(refLla, self.getData(d, DID_GPS1_POS, 'lla'))
                    continue

        for d in self.active_devs:
            if refLla is None:
                refLla = self.getData(d, DID_INS_2, 'lla')[0]
            ned = lla2ned(refLla, self.getData(d, DID_INS_2, 'lla'))
            time = getTimeFromTow(self.getData(d, DID_INS_2, 'timeOfWeek'))
            ax[0,0].plot(time, ned[:,0], label=self.log.serials[d])
            ax[1,0].plot(time, ned[:,1])
            ax[2,0].plot(time, ned[:,2])

            if(np.shape(self.active_devs)[0]==1 or SHOW_GPS_W_INS):
                timeGPS = getTimeFromTowMs(self.getData(d, DID_GPS1_POS, 'timeOfWeekMs'))
                nedGps = lla2ned(refLla, self.getData(d, DID_GPS1_POS, 'lla'))
                ax[0,0].plot(timeGPS, nedGps[:, 0], label=("%s GPS1" % (self.log.serials[d])))
                ax[1,0].plot(timeGPS, nedGps[:, 1])
                ax[2,0].plot(timeGPS, nedGps[:, 2])

            if(np.shape(self.active_devs)[0]==1 or (SHOW_GPS_W_INS and SHOW_GPS2)):
                timeGPS = getTimeFromTowMs(self.getData(d, DID_GPS2_POS, 'timeOfWeekMs'))
                nedGps = lla2ned(refLla, self.getData(d, DID_GPS2_POS, 'lla'))
                ax[0,0].plot(timeGPS, nedGps[:, 0], label=("%s GPS2" % (self.log.serials[d])))
                ax[1,0].plot(timeGPS, nedGps[:, 1])
                ax[2,0].plot(timeGPS, nedGps[:, 2])

            if self.residual and not (refTime is None) and self.log.serials[d] != 'Ref INS': 
                intNed = np.empty_like(refNed)
                for i in range(3):
                    intNed[:,i] = np.interp(refTime, time, ned[:,i], right=np.nan, left=np.nan)
                resNed = intNed - refNed
                ax[0,1].plot(refTime, resNed[:,0], label=self.log.serials[d])
                ax[1,1].plot(refTime, resNed[:,1])
                ax[2,1].plot(refTime, resNed[:,2])

        ax[0,0].legend(ncol=2)
        if self.residual: 
            ax[0,1].legend(ncol=2)
            for i in range(3):
                self.setPlotYSpanMin(ax[i,1], 1.0)
        for a in ax:
            for b in a:
                b.grid(True)

        self.saveFig(fig, 'posNED')

    def drawNEDMapArrow(self, ax, ned, heading):
        # arrowLen = 0.2
        # arrowWid = arrowLen/2
        # arrows = np.array([arrowLen * 0.7 * np.cos(heading), arrowLen * 0.7 * np.sin(heading)]).T

        markersize = 10
        downsample = 6
        len = np.shape(heading)[0]
        for i in range(1, len, downsample):
            # ax.arrow(ned[i,1], ned[i,0], arrows[i,1], arrows[i,0], head_width=arrowWid, head_length=arrowLen, length_includes_head=True, fc='k', ec='k')
            ax.plot(ned[i,1], ned[i,0], marker=(3, 0, -heading[i]*(180.0/np.pi)), color='g', markersize=markersize, linestyle='None')
            ax.plot(ned[i,1], ned[i,0], marker=(2, 0, -heading[i]*(180.0/np.pi)), color='k', markersize=markersize, linestyle='None')

    def posNEDMap(self, fig=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(1,1)
        ax.set_xlabel('East (m)')
        ax.set_ylabel('North (m)')
        fig.suptitle('NED Map - ' + os.path.basename(os.path.normpath(self.log.directory)))
        refLla = None
        for d in self.active_devs:
            lla = self.getData(d, DID_INS_2, 'lla')
            if len(lla) == 0:
                continue
            if refLla is None:
                refLla = lla[0]
            ned = lla2ned(refLla, self.getData(d, DID_INS_2, 'lla'))
            euler = quat2euler(self.getData(d, DID_INS_2, 'qn2b'))
            ax.plot(ned[:,1], ned[:,0], label=self.log.serials[d])

            if(np.shape(self.active_devs)[0]==1 or SHOW_GPS_W_INS):
                if (np.shape(self.active_devs)[0]==1):
                    self.drawNEDMapArrow(ax, ned, euler[:, 2])

                nedGps = lla2ned(refLla, self.getData(d, DID_GPS1_POS, 'lla'))
                ax.plot(nedGps[:, 1], nedGps[:, 0], label=("%s GPS1" % (self.log.serials[d])))

                if SHOW_GPS2:
                    nedGps = lla2ned(refLla, self.getData(d, DID_GPS2_POS, 'lla'))
                    ax.plot(nedGps[:, 1], nedGps[:, 0], label=("%s GPS2" % (self.log.serials[d])))

        ax.set_aspect('equal', 'datalim')
        ax.legend(ncol=2)
        ax.grid(True)
        self.saveFig(fig, 'posNEDMap')

    def gpsPosNEDMap(self, fig=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(1,1)
        ax.set_xlabel('East (m)')
        ax.set_ylabel('North (m)')
        fig.suptitle('GPS NED Map - ' + os.path.basename(os.path.normpath(self.log.directory)))
        refLla = None
        for d in self.active_devs:
            lla = self.getData(d, DID_GPS1_POS, 'lla')
            if len(lla) == 0:
                continue
            if refLla is None:
                refLla = lla[-1]

            nedGps = lla2ned(refLla, self.getData(d, DID_GPS1_POS, 'lla'))
            ax.plot(nedGps[:, 1], nedGps[:, 0], label=("%s" % (self.log.serials[d])))

            if SHOW_GPS2:
                nedGps = lla2ned(refLla, self.getData(d, DID_GPS2_POS, 'lla'))
                ax.plot(nedGps[:, 1], nedGps[:, 0], label=("%s GPS2" % (self.log.serials[d])))

        ax.set_aspect('equal', 'datalim')
        ax.legend(ncol=2)
        ax.grid(True)
        self.saveFig(fig, 'gpsPosNEDMap')

    def posLLA(self, fig=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(3,1, sharex=True)
        self.configureSubplot(ax[0], 'Latitude', 'deg')
        self.configureSubplot(ax[1], 'Longitude', 'deg')
        self.configureSubplot(ax[2], 'Altitude', 'm')
        fig.suptitle('INS LLA - ' + os.path.basename(os.path.normpath(self.log.directory)))
        for d in self.active_devs:
            time = getTimeFromTow(self.getData(d, DID_INS_2, 'timeOfWeek'))
            ax[0].plot(time, self.getData(d, DID_INS_2, 'lla')[:,0], label=self.log.serials[d])
            ax[1].plot(time, self.getData(d, DID_INS_2, 'lla')[:,1])
            ax[2].plot(time, self.getData(d, DID_INS_2, 'lla')[:,2])

            if(np.shape(self.active_devs)[0]==1):
                timeGPS = getTimeFromTowMs(self.getData(d, DID_GPS1_POS, 'timeOfWeekMs'))
                ax[0].plot(timeGPS, self.getData(d, DID_GPS1_POS, 'lla')[:, 0], label='GPS1')
                ax[1].plot(timeGPS, self.getData(d, DID_GPS1_POS, 'lla')[:, 1])
                ax[2].plot(timeGPS, self.getData(d, DID_GPS1_POS, 'lla')[:, 2], label='GPS1')

                timeGPS = getTimeFromTowMs(self.getData(d, DID_GPS2_POS, 'timeOfWeekMs'))
                ax[0].plot(timeGPS, self.getData(d, DID_GPS2_POS, 'lla')[:, 0], label='GPS2')
                ax[1].plot(timeGPS, self.getData(d, DID_GPS2_POS, 'lla')[:, 1])
                ax[2].plot(timeGPS, self.getData(d, DID_GPS2_POS, 'lla')[:, 2], label='GPS2')

                timeBaro = getTimeFromTow(self.getData(d, DID_BAROMETER, 'time')+ self.getData(d, DID_GPS1_POS, 'towOffset')[-1])
                ax[2].plot(timeBaro, self.getData(d, DID_BAROMETER, 'mslBar'), label='Baro')

        ax[0].legend(ncol=2)
        ax[2].legend(ncol=2)
        for a in ax:
            a.grid(True)
        self.saveFig(fig, 'insLLA')

    def gpsLLA(self, fig=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(3,1, sharex=True)
        self.configureSubplot(ax[0], 'Latitude', 'deg')
        self.configureSubplot(ax[1], 'Longitude', 'deg')
        self.configureSubplot(ax[2], 'Altitude', 'm')
        fig.suptitle('GPS LLA - ' + os.path.basename(os.path.normpath(self.log.directory)))
        for d in self.active_devs:
            time = getTimeFromTowMs(self.getData(d, DID_GPS1_POS, 'timeOfWeekMs'))
            ax[0].plot(time, self.getData(d, DID_GPS1_POS, 'lla')[:,0], label=('%s' % self.log.serials[d]))
            ax[1].plot(time, self.getData(d, DID_GPS1_POS, 'lla')[:,1])
            ax[2].plot(time, self.getData(d, DID_GPS1_POS, 'lla')[:,2])

            time = getTimeFromTowMs(self.getData(d, DID_GPS2_POS, 'timeOfWeekMs'))
            if (time.size and SHOW_GPS2):
                gpslla = self.getData(d, DID_GPS2_POS, 'lla')
                if (gpslla.size):
                    if (np.any(gpslla)):
                        ax[0].plot(time, self.getData(d, DID_GPS2_POS, 'lla')[:,0], label=('%s GPS2' % self.log.serials[d]))
                        ax[1].plot(time, self.getData(d, DID_GPS2_POS, 'lla')[:,1])
                        ax[2].plot(time, self.getData(d, DID_GPS2_POS, 'lla')[:,2])

        ax[0].legend(ncol=2)
        for a in ax:
            a.grid(True)
        self.saveFig(fig, 'gpsLLA')

    def getGpsPosNED(self, device, did, refLla):
        gpsTime = getTimeFromTowMs(self.getData(device, did, 'timeOfWeekMs'))
        gpsNed = lla2ned(refLla, self.getData(device, did, 'lla'))
        return [gpsTime, gpsNed]

    def gpsPosNED(self, fig=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(4,1, sharex=True)
        self.configureSubplot(ax[0], 'GPS North', 'm')
        self.configureSubplot(ax[1], 'GPS East', 'm')
        self.configureSubplot(ax[2], 'GPS Down', 'm')
        self.configureSubplot(ax[3], 'GPS NED Magnitude', 'm')
        fig.suptitle('GPS NED - ' + os.path.basename(os.path.normpath(self.log.directory)))
        refLla = None
        for d in self.active_devs:
            if refLla is None:
                refLla = self.getData(d, DID_GPS1_POS, 'lla')[-1]

            [gpsTime, gpsNed] = self.getGpsPosNED(d, DID_GPS1_POS, refLla)
            gpsNedNorm = np.linalg.norm(gpsNed, axis=1)
            ax[0].plot(gpsTime, gpsNed[:, 0], label=self.log.serials[d])
            ax[1].plot(gpsTime, gpsNed[:, 1])
            ax[2].plot(gpsTime, gpsNed[:, 2])
            ax[3].plot(gpsTime, gpsNedNorm)

            if (np.shape(self.active_devs)[0]==1) or SHOW_GPS2:
                [gps2Time, gps2Ned] = self.getGpsPosNED(d, DID_GPS2_POS, refLla)
                gps2NedNorm = np.linalg.norm(gps2Ned, axis=1)
                ax[0].plot(gps2Time, gps2Ned[:, 0], label=("%s GPS2" % (self.log.serials[d])))
                ax[1].plot(gps2Time, gps2Ned[:, 1])
                ax[2].plot(gps2Time, gps2Ned[:, 2])
                ax[3].plot(gps2Time, gps2NedNorm)

        ax[0].legend(ncol=2)
        for a in ax:
            a.grid(True)
        self.saveFig(fig, 'gpsPosNED')

    def getGpsVelNed(self, device, did, refLla):
        gpsTime = getTimeFromTowMs(self.getData(device, did, 'timeOfWeekMs'))
        status = self.getData(device, did, 'status')[0]
        gpsVelNed = None
        if (status & 0x00008000):
            gpsVelNed = self.getData(device, did, 'vel')    # NED velocity
        else:
            gpsVelEcef = self.getData(device, did, 'vel')   # ECEF velocity
            if len(gpsVelEcef) > 0:
                qe2n = quat_ecef2ned(refLla[0:2]*np.pi/180.0)
                gpsVelNed = quatConjRot(qe2n, gpsVelEcef)
        return [gpsTime, gpsVelNed]

    def gpsVelNED(self, fig=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(4,1, sharex=True)
        self.configureSubplot(ax[0], 'GPS Velocity North', 'm/s')
        self.configureSubplot(ax[1], 'GPS Velocity East', 'm/s')
        self.configureSubplot(ax[2], 'GPS Velocity Down', 'm/s')
        self.configureSubplot(ax[3], 'GPS Velocity Magnitude', 'm/s')
        fig.suptitle('GPS Velocity NED - ' + os.path.basename(os.path.normpath(self.log.directory)))
        refLla = None
        for d in self.active_devs:
            if refLla is None:
                refLla = self.getData(d, DID_GPS1_POS, 'lla')[-1]
            [gpsTime, gpsVelNed] = self.getGpsVelNed(d, DID_GPS1_VEL, refLla)
            gpsVelNorm = np.linalg.norm(gpsVelNed, axis=1)
            ax[0].plot(gpsTime, gpsVelNed[:, 0], label=self.log.serials[d])
            ax[1].plot(gpsTime, gpsVelNed[:, 1])
            ax[2].plot(gpsTime, gpsVelNed[:, 2])
            ax[3].plot(gpsTime, gpsVelNorm)

            if SHOW_GPS2:
                [gps2Time, gps2VelNed] = self.getGpsVelNed(d, DID_GPS2_VEL, refLla)
                gps2VelNorm = np.linalg.norm(gps2VelNed, axis=1)
                ax[0].plot(gps2Time, gps2VelNed[:, 0], label=("%s GPS2" % (self.log.serials[d])))
                ax[1].plot(gps2Time, gps2VelNed[:, 1])
                ax[2].plot(gps2Time, gps2VelNed[:, 2])
                ax[3].plot(gps2Time, gps2VelNorm)

        ax[0].legend(ncol=2)
        for a in ax:
            a.grid(True)
        self.saveFig(fig, 'gpsVelNED')
        
    def getGpsNedVel(self, d):
        velNed = None
        status = self.getData(d, DID_GPS1_VEL, 'status')[0]
        if (status & 0x00008000):
            velNed = self.getData(d, DID_GPS1_VEL, 'vel')    # NED velocity
        else:
            velEcef = self.getData(d, DID_GPS1_VEL, 'vel')   # ECEF velocity
            qe2n = quat_ecef2ned(refLla[0:2]*np.pi/180.0)
            if len(velEcef) > 0:
                velNed = quatConjRot(qe2n, velEcef)
            #R = rotmat_ecef2ned(self.getData(d, DID_GPS1_POS, 'lla')[0,0:2]*np.pi/180.0)
            #velNed = R.dot(velEcef.T).T
        return velNed

    def velNED(self, fig=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(3, (2 if self.residual else 1), sharex=True, squeeze=False)
        self.configureSubplot(ax[0,0], 'Vel North', 'm/s')
        self.configureSubplot(ax[1,0], 'Vel East',  'm/s')
        self.configureSubplot(ax[2,0], 'Vel Down',  'm/s')
        fig.suptitle('NED Vel - ' + os.path.basename(os.path.normpath(self.log.directory)))
        refLla = None
        refTime = None
        refVelNed = None
        if self.residual:
            self.configureSubplot(ax[0,1], 'Residual Vel North', 'm/s')
            self.configureSubplot(ax[1,1], 'Residual Vel East',  'm/s')
            self.configureSubplot(ax[2,1], 'Residual Vel Down',  'm/s')
            # Use 'Ref INS' if available
            for d in self.active_devs:
               if self.log.serials[d] == 'Ref INS':
                    refTime = getTimeFromTow(self.getData(d, DID_INS_2, 'timeOfWeek'))
                    refVelNed = self.getData(d, DID_INS_2, 'lla')[0]
                    continue
            # If 'Ref INS' is not available, use GPS as reference
            if refTime is None:
                for d in self.active_devs:
                    refTime = getTimeFromTowMs(self.getData(d, DID_GPS1_VEL, 'timeOfWeekMs'))
                    refLla = self.getData(d, DID_GPS1_POS, 'lla')[-1]
                    refVelNed = self.getGpsNedVel(d)                    
                    continue

        for d in self.active_devs:
            if refLla is None:
                refLla = self.getData(d, DID_INS_2, 'lla')[-1]
            time = getTimeFromTow(self.getData(d, DID_INS_2, 'timeOfWeek'))
            insVelNed = quatRot(self.getData(d, DID_INS_2, 'qn2b'), self.getData(d, DID_INS_2, 'uvw'))
            ax[0,0].plot(time, insVelNed[:,0], label=self.log.serials[d])
            ax[1,0].plot(time, insVelNed[:,1])
            ax[2,0].plot(time, insVelNed[:,2])

            if np.shape(self.active_devs)[0] == 1 or SHOW_GPS_W_INS:  # Show GPS if #devs is 1
                timeGPS = getTimeFromTowMs(self.getData(d, DID_GPS1_VEL, 'timeOfWeekMs'))
                gpsVelNed = self.getGpsNedVel(d)
                ax[0,0].plot(timeGPS, gpsVelNed[:, 0], label=('%s GPS' % self.log.serials[d]))
                ax[1,0].plot(timeGPS, gpsVelNed[:, 1])
                ax[2,0].plot(timeGPS, gpsVelNed[:, 2])

            if self.residual and not (refTime is None) and self.log.serials[d] != 'Ref INS': 
                intVelNed = np.empty_like(refVelNed)
                for i in range(3):
                    intVelNed[:,i] = np.interp(refTime, time, insVelNed[:,i], right=np.nan, left=np.nan)
                resNed = intVelNed - refVelNed
                ax[0,1].plot(refTime, resNed[:,0], label=self.log.serials[d])
                ax[1,1].plot(refTime, resNed[:,1])
                ax[2,1].plot(refTime, resNed[:,2])

        ax[0,0].legend(ncol=2)
        if self.residual: 
            ax[0,1].legend(ncol=2)
            for i in range(3):
                self.setPlotYSpanMin(ax[i,1], 1.0)
        for a in ax:
            for b in a:
                b.grid(True)
        self.saveFig(fig, 'velNED')

    def angle_wrap(self, angle):
        result = np.copy(angle)
        for i in range(np.shape(result)[0]):
            while result[i] > np.pi: 
                result[i] -= 2*np.pi
            while result[i] < -np.pi: 
                result[i] += 2*np.pi
        return result

    def angle_unwrap(self, angle):
        unwrap = 0.0
        result = np.empty_like(angle)
        anglePrev = angle[0]
        for i in range(np.shape(angle)[0]):
            result[i] = angle[i] + unwrap

            deltaAngle = result[i]-anglePrev
            if deltaAngle > np.pi:                
                unwrap -= 2*np.pi
                result[i] = angle[i] + unwrap
            elif deltaAngle < -np.pi: 
                unwrap += 2*np.pi
                result[i] = angle[i] + unwrap

            anglePrev = result[i]
        return result

    def vec3_wrap(self, vec):
        result = np.empty_like(vec)
        for i in range(3):
            result[:,i] = self.angle_wrap(vec[:,i])
        return result

    def vec3_unwrap(self, vec):
        result = np.empty_like(vec)
        for i in range(3):
            result[:,i] = self.angle_unwrap(vec[:,i])
        return result

    def velUVW(self, fig=None):
        if fig is None:
            fig = plt.figure()

        ax = fig.subplots(3, (2 if self.residual else 1), sharex=True, squeeze=False)
        fig.suptitle('INS uvw - ' + os.path.basename(os.path.normpath(self.log.directory)))
        self.configureSubplot(ax[0,0], 'Vel U', 'm/s')
        self.configureSubplot(ax[1,0], 'Vel V', 'm/s')
        self.configureSubplot(ax[2,0], 'Vel W', 'm/s')
        refTime = None
        refUvw = None
        sumDelta = None
        sumCount = 1
        if self.residual:
            self.configureSubplot(ax[0,1], 'Vel U Residual', 'm/s')
            self.configureSubplot(ax[1,1], 'Vel V Residual', 'm/s')
            self.configureSubplot(ax[2,1], 'Vel W Residual', 'm/s')
            for d in self.active_devs:
               if self.log.serials[d] == 'Ref INS':
                    refTime = getTimeFromTow(self.getData(d, DID_INS_2, 'timeOfWeek'))
                    refUvw = self.getData(d, DID_INS_2, 'uvw')
                    continue

            # Reference INS does not exist.  Compute reference from average INS.
            if refTime is None:
                for d in self.active_devs:
                    time = getTimeFromTow(self.getData(d, DID_INS_2, 'timeOfWeek'))

                    # Adjust data for attitude bias
                    uvw = quatRot(self.log.mount_bias_quat[d,:], self.getData(d, DID_INS_2, 'uvw'))

                    if refTime is None:
                        refTime = time
                        refUvw = np.copy(uvw)
                        sumDelta = np.zeros_like(uvw)
                    else:
                        intUvw = np.empty_like(refUvw)
                        for i in range(3):
                            intUvw[:,i] = np.interp(refTime, time, uvw[:,i])
                        delta = intUvw - refUvw
                        sumDelta += delta
                        sumCount += 1
                refUvw += sumDelta / sumCount

        for d in self.active_devs:
            time = getTimeFromTow(self.getData(d, DID_INS_2, 'timeOfWeek'))
            # Adjust data for attitude bias
            uvw = quatRot(self.log.mount_bias_quat[d,:], self.getData(d, DID_INS_2, 'uvw'))
            ax[0,0].plot(time, uvw[:,0], label=self.log.serials[d])
            ax[1,0].plot(time, uvw[:,1])
            ax[2,0].plot(time, uvw[:,2])

            if self.residual and not (refTime is None) and self.log.serials[d] != 'Ref INS': 
                intUvw = np.empty_like(refUvw)
                for i in range(3):
                    intUvw[:,i] = np.interp(refTime, time, uvw[:,i], right=np.nan, left=np.nan)
                resUvw = intUvw - refUvw
                ax[0,1].plot(refTime, resUvw[:,0], label=self.log.serials[d])
                ax[1,1].plot(refTime, resUvw[:,1])
                ax[2,1].plot(refTime, resUvw[:,2])

        ax[0,0].legend(ncol=2)
        if self.residual: 
            ax[0,1].legend(ncol=2)
            for i in range(3):
                self.setPlotYSpanMin(ax[i,1], 1.0)
        for a in ax:
            for b in a:
                b.grid(True)
        self.saveFig(fig, 'velUVW')

    def attitude(self, fig=None):
        if fig is None:
            fig = plt.figure()

        ax = fig.subplots(3, (2 if self.residual else 1), sharex=True, squeeze=False)
        fig.suptitle('INS Attitude - ' + os.path.basename(os.path.normpath(self.log.directory)))
        self.configureSubplot(ax[0,0], 'Roll', 'deg')
        self.configureSubplot(ax[1,0], 'Pitch', 'deg')
        self.configureSubplot(ax[2,0], 'Yaw', 'deg')
        refTime = None
        sumDelta = None
        sumCount = 1
        if self.residual:
            self.configureSubplot(ax[0,1], 'Roll Residual', 'deg')
            self.configureSubplot(ax[1,1], 'Pitch Residual', 'deg')
            self.configureSubplot(ax[2,1], 'Yaw Residual', 'deg')
            for d in self.active_devs:
                if self.log.serials[d] == 'Ref INS':
                    quat = self.getData(d, DID_INS_2, 'qn2b')
                    refEuler = self.vec3_wrap(quat2euler(quat))
                    refTime = getTimeFromTow(self.getData(d, DID_INS_2, 'timeOfWeek'))

            # Reference INS does not exist.  Compute reference from average INS.
            if refTime is None:
                for d in self.active_devs:
                    time = getTimeFromTow(self.getData(d, DID_INS_2, 'timeOfWeek'))
                    # Adjust data for attitude bias
                    quat = mul_ConjQuat_Quat(self.log.mount_bias_quat[d,:], self.getData(d, DID_INS_2, 'qn2b'))
                    euler = quat2euler(quat)

                    if refTime is None:
                        refTime = time
                        refEuler = np.copy(euler)
                        sumDelta = np.zeros_like(euler)
                    else:
                        unwrapEuler = self.vec3_unwrap(euler)
                        intEuler = np.empty_like(refEuler)
                        for i in range(3):
                            intEuler[:,i] = np.interp(refTime, time, unwrapEuler[:,i])
                        delta = self.vec3_wrap(intEuler - refEuler)
                        sumDelta += delta
                        sumCount += 1
                refEuler += sumDelta / sumCount

        for d in self.active_devs:
            # Adjust data for attitude bias
            quat = mul_ConjQuat_Quat(self.log.mount_bias_quat[d,:], self.getData(d, DID_INS_2, 'qn2b'))
            euler = quat2euler(quat)
            time = getTimeFromTow(self.getData(d, DID_INS_2, 'timeOfWeek'))
            ax[0,0].plot(time, euler[:,0]*RAD2DEG, label=self.log.serials[d])
            ax[1,0].plot(time, euler[:,1]*RAD2DEG)
            ax[2,0].plot(time, euler[:,2]*RAD2DEG)

            if self.residual and not (refTime is None) and self.log.serials[d] != 'Ref INS': 
                unwrapEuler = self.vec3_unwrap(euler)
                intEuler = np.empty_like(refEuler)
                for i in range(3):
                    intEuler[:,i] = np.interp(refTime, time, unwrapEuler[:,i], right=np.nan, left=np.nan)
                resEuler = self.vec3_wrap(intEuler - refEuler)
                ax[0,1].plot(refTime, resEuler[:,0]*RAD2DEG, label=self.log.serials[d])
                ax[1,1].plot(refTime, resEuler[:,1]*RAD2DEG)
                ax[2,1].plot(refTime, resEuler[:,2]*RAD2DEG)

        ax[0,0].legend(ncol=2)
        if self.residual: 
            ax[0,1].legend(ncol=2)
            for i in range(3):
                self.setPlotYSpanMin(ax[i,1], 3.0)
        for a in ax:
            for b in a:
                b.grid(True)
        self.saveFig(fig, 'attINS')

    def heading(self, fig=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(3, 1, sharex=True)
        fig.suptitle('Heading - ' + os.path.basename(os.path.normpath(self.log.directory)))
        self.configureSubplot(ax[0], 'Magnetic Heading', 'deg')
        self.configureSubplot(ax[1], 'RTK Compassing', 'deg')
        self.configureSubplot(ax[2], 'INS Heading', 'deg')
        for d in self.active_devs:
            magTime = getTimeFromTowMs(self.getData(d, DID_INL2_MAG_OBS_INFO, 'timeOfWeekMs'))
            gpsTime = getTimeFromTowMs(self.getData(d, DID_GPS1_RTK_CMP_REL, 'timeOfWeekMs'))
            insTime = getTimeFromTow(self.getData(d, DID_INS_2, 'timeOfWeek'))
            magHdg = self.getData(d, DID_INL2_MAG_OBS_INFO, 'magHdg')
            gpsHdg = self.getData(d, DID_GPS1_RTK_CMP_REL, 'baseToRoverHeading')
            euler = quat2euler(self.getData(d, DID_INS_2, 'qn2b'))
            if magTime.any():
                ax[0].plot(magTime, magHdg * RAD2DEG)
            if gpsTime.any():
                ax[1].plot(gpsTime, gpsHdg*RAD2DEG)
            ax[2].plot(insTime, euler[:,2]*RAD2DEG, label=self.log.serials[d])
        ax[2].legend(ncol=2)
        for a in ax:
            a.grid(True)
        self.saveFig(fig, 'heading')

    def insStatus(self, fig=None):
        try:
            if fig is None:
                fig = plt.figure()
            ax = fig.subplots(1, 1, sharex=True)
            fig.suptitle('INS Status - ' + os.path.basename(os.path.normpath(self.log.directory)))

            for d in self.active_devs:
                r = d == self.active_devs[0]    # plot text w/ first device
                cnt = 0
                instime = getTimeFromTow(self.getData(d, DID_INS_2, 'timeOfWeek'))
                iStatus = self.getData(d, DID_INS_2, 'insStatus')

                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x00000001) != 0))
                p1 = ax.get_xlim()[0] + 0.02 * (ax.get_xlim()[1] - ax.get_xlim()[0])
                if r: ax.text(p1, -cnt * 1.5, 'Att Coarse')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x00000010) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Att Fine')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x00000002) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Vel Coarse')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x00000020) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Vel Fine')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x00000004) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Pos Coarse')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x00000040) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Pos Fine')
                cnt += 1
                cnt += 1

                # ax.plot(instime, -cnt * 1.5 + ((iStatus >> 9) & 1))
                # ax.text(p1, -cnt * 1.5, 'GPS Update')
                # cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x00000100) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'GPS aiding Pos')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x00004000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'GPS aiding Vel')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x00000080) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'GPS aiding Hdg')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x00000800) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'MAG aiding Hdg')
                cnt += 1
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x00001000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Nav Mode')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x000F0000) >> 16) / 4.0)
                if r: ax.text(p1, -cnt * 1.5, 'Solution Status')
                cnt += 1
                cnt += 1

                ax.plot(instime, -cnt * 1.5 + (((iStatus & 0x03000000) >> 24) == 3))
                if r: ax.text(p1, -cnt * 1.5, 'RTK: Precision Position Valid')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x04000000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'RTK: Compassing Valid (fix & hold)')
                cnt += 1
                cnt += 1

                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x00100000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'RTK: Compassing Baseline UNSET')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x00200000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'RTK: Compassing Baseline BAD')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x08000000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'RTK: No Observ.')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x10000000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'RTK: Base No Pos.')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x20000000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'RTK: Base Pos. Moving')
                cnt += 1
                cnt += 1

                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x00400000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Mag: Recalibrating')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x00800000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Mag: Inter. or Bad Cal')
                cnt += 1
                cnt += 1

                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x40000000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'RTOS Task Period Overrun')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x80000000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'General Fault')
                cnt += 1

            ax.grid(True)
            self.saveFig(fig, 'iStatus')
        except:
            print(RED + "problem plotting insStatus: " + sys.exc_info()[0] + RESET)

    def hdwStatus(self, fig=None):
        try:
            if fig is None:
                fig = plt.figure()
            ax = fig.subplots(1, 1, sharex=True)
            fig.suptitle('Hardware Status - ' + os.path.basename(os.path.normpath(self.log.directory)))

            for d in self.active_devs:
                r = d == self.active_devs[0]    # plot text w/ first device
                cnt = 0
                instime = getTimeFromTow(self.getData(d, DID_INS_2, 'timeOfWeek'))
                hStatus = self.getData(d, DID_INS_2, 'hdwStatus')

                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x00000001) != 0))
                p1 = ax.get_xlim()[0] + 0.02 * (ax.get_xlim()[1] - ax.get_xlim()[0])
                if r: ax.text(p1, -cnt * 1.5, 'Motion Gyr Sig')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x00000002) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Motion Acc Sig')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x00000004) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Motion Gyr Dev')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x00000005) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Motion Acc Dev')
                cnt += 1
                cnt += 1

                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x00000010) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Satellite Rx')
                cnt += 1
                cnt += 1

                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x00000100) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Saturation Gyr')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x00000200) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Saturation Acc')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x00000400) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Saturation Mag')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x00000800) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Saturation Baro')
                cnt += 1
                cnt += 1

                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x00002000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'EKF using ref. IMU')
                cnt += 1
                cnt += 1

                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x00010000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Err Com Tx Limited')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x00020000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Err Com Rx Overrun')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x00040000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Err GPS Tx Limited')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x00080000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Err GPS Rx Overrun')
                cnt += 1
                cnt += 1

                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x00F00000) >> 20) / 4)
                if r: ax.text(p1, -cnt * 1.5, 'Com Parse Error Count')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x01000000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'BIT Self Test Fault')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x02000000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Temperature error')
                cnt += 1
                cnt += 1

                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x10000000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Reset Backup Mode')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x20000000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Watchdog Reset')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x30000000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Software Reset')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x40000000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Hardware Reset')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x80000000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Critical Sys Fault')
                cnt += 1
                cnt += 1
                
            ax.grid(True)
            self.saveFig(fig, 'Hardware Status')
        except:
            print(RED + "problem plotting hdwStatus: " + sys.exc_info()[0] + RESET)

    def gpsStats(self, fig=None, did_gps_pos=DID_GPS1_POS):
        # try:
        if fig is None:
            fig = plt.figure()

        ax = fig.subplots(5, 1, sharex=True, gridspec_kw={'height_ratios': [1, 2, 2, 2, 1]})
        did_gps_vel = did_gps_pos+(DID_GPS1_VEL-DID_GPS1_POS)
        if did_gps_pos==DID_GPS1_POS:
            gps_num = 1
        else:
            gps_num = 2
        fig.suptitle('GPS ' + str(gps_num) + ' Stats - ' + os.path.basename(os.path.normpath(self.log.directory)))
        self.configureSubplot(ax[0], 'Satellites Used in Solution', '')
        self.configureSubplot(ax[1], 'CNO (dBHz)', 'dBHz')
        self.configureSubplot(ax[2], 'Position Accuracy (m)', 'm')
        self.configureSubplot(ax[3], 'Speed Accuracy: sAcc (m/s)', 'm/s')
        self.configureSubplot(ax[4], 'Status', '')

        plot_legend = 1
        for d in self.active_devs:
            r = d == self.active_devs[0]  # plot text w/ first device
            time = getTimeFromTowMs(self.getData(d, did_gps_pos, 'timeOfWeekMs'))
            velTime = getTimeFromTowMs(self.getData(d, did_gps_vel, 'timeOfWeekMs'))
            gStatus = self.getData(d, did_gps_pos, 'status')

            ax[0].plot(time, gStatus & 0xFF, label=self.log.serials[d])
            ax[1].plot(time, self.getData(d, did_gps_pos, 'cnoMean'), label=self.log.serials[d])
            ax[2].plot(time, self.getData(d, did_gps_pos, 'hAcc'), 'r', label="hAcc")
            ax[2].plot(time, self.getData(d, did_gps_pos, 'vAcc'), 'b', label="vAcc")
            ax[2].plot(time, self.getData(d, did_gps_pos, 'pDop'), 'm', label="pDop")
            if self.log.data[d, DID_GPS1_RTK_POS] is not []:
                rtktime = getTimeFromTowMs(self.getData(d, DID_GPS1_RTK_POS, 'timeOfWeekMs'))
                ax[2].plot(rtktime, self.getData(d, DID_GPS1_RTK_POS, 'vAcc'), 'g', label="rtkHor")
            ax[3].plot(velTime, self.getData(d, did_gps_vel, 'sAcc'), label="sAcc")

            if plot_legend:
                plot_legend = 0
                ax[2].legend(ncol=2)

            cnt = 0
            ax[4].plot(time, -cnt * 1.5 + ((gStatus & 0x04000000) != 0))
            p1 = ax[4].get_xlim()[0] + 0.02 * (ax[4].get_xlim()[1] - ax[4].get_xlim()[0])
            if r: ax[4].text(p1, -cnt * 1.5, 'RTK Positioning Valid')
            cnt += 1
            ax[4].plot(time, -cnt * 1.5 + ((gStatus & 0x08000000) != 0))
            if r: ax[4].text(p1, -cnt * 1.5, 'RTK Compassing Valid (fix & hold)')
            cnt += 1
            ax[4].plot(time, -cnt * 1.5 + ((gStatus & 0x00002000) != 0))
            if r: ax[4].text(p1, -cnt * 1.5, 'GPS Compass Baseline BAD')
            cnt += 1
            ax[4].plot(time, -cnt * 1.5 + ((gStatus & 0x00004000) != 0))
            if r: ax[4].text(p1, -cnt * 1.5, 'GPS Compass Baseline UNSET')
            cnt += 1

        self.setPlotYSpanMin(ax[1], 5)

        ax[1].legend(ncol=2)
        for a in ax:
            a.grid(True)
        self.saveFig(fig, 'Gps Stats')
        # except:
        #     print(RED + "problem plotting gpsStats: " + sys.exc_info()[0] + RESET)

    def gps2Stats(self, fig=None):
        self.gpsStats(fig=fig, did_gps_pos=DID_GPS2_POS)

    def rtkPosStats(self, fig=None):
        self.rtkStats("Position", DID_GPS1_RTK_POS_REL, fig=fig)

    def rtkCmpStats(self, fig=None):
        self.rtkStats("Compassing", DID_GPS1_RTK_CMP_REL, fig=fig)

    def rtkStats(self, name, relDid, fig=None):
        # try:
        n_plots = 6
        if fig is None:
            fig = plt.figure()

        ax = fig.subplots(n_plots, 1, sharex=True)
        fig.suptitle('RTK ' + name + ' Stats - ' + os.path.basename(os.path.normpath(self.log.directory)))
        self.configureSubplot(ax[0], 'GPS Fix Type: 2=2D, 3=3D, 10=Single, 11=Float, 12=Fix', '')
        self.configureSubplot(ax[1], 'Age of Differential', 's')
        self.configureSubplot(ax[2], 'AR Ratio', '')
        self.configureSubplot(ax[3], 'Base to Rover Distance', 'm')
        self.configureSubplot(ax[4], 'Base to Rover Heading', 'deg')
        self.configureSubplot(ax[5], 'Base to Rover Heading Accuracy', 'deg')

        for i, d in enumerate(self.active_devs):
            rtkRelTime = getTimeFromTowMs(self.getData(d, relDid, 'timeOfWeekMs'))
            # rtkMiscTime = getTimeFromTowMs(self.getData(d, DID_GPS1_RTK_CMP_MISC, 'timeOfWeekMs'))
            if not self.log.compassing:
                gps1PosTime = getTimeFromTowMs(self.getData(d, DID_GPS1_POS, 'timeOfWeekMs'))
                fixType = self.getData(d, DID_GPS1_POS, 'status') >> 8 & 0x1F
                ax[0].plot(gps1PosTime, fixType, label=self.log.serials[d])
            else:
                fixType = self.getData(d, relDid, 'arRatio').copy()
                fixType[(fixType > 3)] = 12
                fixType[(fixType > 0) & (fixType < 3)] = 11
                fixType[fixType == 0] = 10
                ax[0].plot(rtkRelTime, fixType, label=self.log.serials[d])
            ax[1].plot(rtkRelTime, self.getData(d, relDid, 'differentialAge'))
            if i == 0:
                ax[2].semilogy(rtkRelTime, np.ones_like(rtkRelTime)*3.0, 'k--')
            ax[2].semilogy(rtkRelTime, self.getData(d, relDid, 'arRatio'))
            dist2base = self.getData(d, relDid, 'baseToRoverDistance')
            dist2base[dist2base > 1e5] = np.nan
            ax[3].plot(rtkRelTime, dist2base)
            ax[4].plot(rtkRelTime, self.getData(d, relDid, 'baseToRoverHeading')*180.0/np.pi)
            ax[5].plot(rtkRelTime, self.getData(d, relDid, 'baseToRoverHeadingAcc')*180.0/np.pi)
            ax[0].legend(ncol=2)

        self.setPlotYSpanMin(ax[1], 0.5)    # Differential age
        self.setPlotYSpanMin(ax[3], 1.0)    # Distance to base

        for a in ax:
            a.grid(True)
        self.saveFig(fig, 'rtk'+name+'Stats')
        # except:
            # print(RED + "problem plotting rtkStats: " + sys.exc_info()[0] + RESET)

    def rtkPosMisc(self, fig=None):
        self.rtkMisc("Position", DID_GPS1_RTK_POS_MISC, fig=fig)

    def rtkCmpMisc(self, fig=None):
        self.rtkMisc("Position", DID_GPS1_RTK_CMP_MISC, fig=fig)

    def rtkMisc(self, name, miscDid, fig=None):
        # try:
        n_plots = 10
        if fig is None:
            fig = plt.figure()

        ax = fig.subplots(5, 2, sharex=True)
        fig.suptitle('RTK ' + name + ' Misc - ' + os.path.basename(os.path.normpath(self.log.directory)))
        self.configureSubplot(ax[0,0], 'Correction checksum failure count', '')
        self.configureSubplot(ax[1,0], 'Time to First Fix', 's')
        self.configureSubplot(ax[2,0], 'GPS Observation Count - Rover', '')
        self.configureSubplot(ax[3,0], 'GPS Observation Count - Base', '')
        self.configureSubplot(ax[4,0], 'Glonass Observation Count - Rover', '')
        self.configureSubplot(ax[0,1], 'Glonass Observation Count - Base', '')
        self.configureSubplot(ax[1,1], 'Galileo Observation Count - Rover', '')
        self.configureSubplot(ax[2,1], 'Galileo Observation Count - Base', '')
        self.configureSubplot(ax[3,1], 'SBAS Count Base', '')
        self.configureSubplot(ax[4,1], 'Base Antenna Position Count', '')

        for i, d in enumerate(self.active_devs):
            # rtkRelTime = getTimeFromTowMs(self.getData(d, DID_GPS1_RTK_POS_REL, 'timeOfWeekMs'))
            rtkMiscTime = getTimeFromTowMs(self.getData(d, miscDid, 'timeOfWeekMs'))
            ax[0,0].plot(rtkMiscTime, self.getData(d, miscDid, 'correctionChecksumFailures'))
            ax[1,0].plot(rtkMiscTime, self.getData(d, miscDid, 'timeToFirstFixMs')*0.001)
            ax[2,0].plot(rtkMiscTime, self.getData(d, miscDid, 'roverGpsObservationCount'))
            ax[3,0].plot(rtkMiscTime, self.getData(d, miscDid, 'baseGpsObservationCount'))
            ax[4,0].plot(rtkMiscTime, self.getData(d, miscDid, 'roverGlonassObservationCount'))
            ax[0,1].plot(rtkMiscTime, self.getData(d, miscDid, 'baseGlonassObservationCount'))
            ax[1,1].plot(rtkMiscTime, self.getData(d, miscDid, 'roverGalileoObservationCount'))
            ax[2,1].plot(rtkMiscTime, self.getData(d, miscDid, 'baseGalileoObservationCount'))
            ax[3,1].plot(rtkMiscTime, self.getData(d, miscDid, 'baseSbasCount'))
            ax[4,1].plot(rtkMiscTime, self.getData(d, miscDid, 'baseAntennaCount'))

            # # ax[0].plot(rtkRelTime, self.getData(d, DID_GPS1_RTK_POS_REL, 'differentialAge'))
            # if i == 0:
            #     ax[2].semilogy(rtkRelTime, np.ones_like(rtkRelTime)*3.0, 'k--')
            # ax[2].semilogy(rtkRelTime, self.getData(d, DID_GPS1_RTK_POS_REL, 'arRatio'))
            # dist2base = self.getData(d, DID_GPS1_RTK_POS_REL, 'distanceToBase')
            # dist2base[dist2base > 1e5] = np.nan
            # ax[3].plot(rtkRelTime, dist2base)
            # ax[4].plot(rtkMiscTime, self.getData(d, miscDid, 'cycleSlipCount'))
            # ax[0].legend(ncol=2)
            for a in ax:
                for b in a:
                    b.grid(True)

        self.saveFig(fig, 'rtk'+name+'Misc')
        # except:
            # print(RED + "problem plotting rtkStats: " + sys.exc_info()[0] + RESET)

    def rtkRel(self, fig=None):
        # try:
        n_plots = 3
        if fig is None:
            fig = plt.figure()

        ax = fig.subplots(3, 1, sharex=True)
        fig.suptitle('RTK Rel - ' + os.path.basename(os.path.normpath(self.log.directory)))
        self.configureSubplot(ax[0], 'GPS Base to Rover Heading', '')
        self.configureSubplot(ax[1], 'GPS Base to Rover Distance', '')

        for i, d in enumerate(self.active_devs):
            rtkRelTime = getTimeFromTowMs(self.getData(d, DID_GPS1_RTK_POS_REL, 'timeOfWeekMs'))
            ax[0].plot(rtkRelTime, self.getData(d, DID_GPS1_RTK_POS_REL, 'baseToRoverHeading')*RAD2DEG)
            ax[1].plot(rtkRelTime, self.getData(d, DID_GPS1_RTK_POS_REL, 'baseToRoverDistance'))

            for a in ax:
                a.grid(True)

        self.saveFig(fig, 'rtkRel')

    def loadGyros(self, device):
        return self.loadIMU(device, 0)

    def loadAccels(self, device):
        return self.loadIMU(device, 1)

    def loadIMU(self, device, accelSensor):   # 0 = gyro, 1 = accelerometer
        imu1 = None
        imu2 = None
        imu3 = None
        imuCount = 0
        time = None
        dt = None

        if accelSensor==0:
            # I = np.copy(self.getData(device, DID_IMU_RAW, 'I'))  # to plot raw gyro data
            # imu1 = I['pqr']                                      # to plot raw gyro data
            imu1 = np.copy(self.getData(device, DID_PIMU, 'theta'))
        else:
            imu1 = np.copy(self.getData(device, DID_PIMU, 'vel'))

        if np.shape(imu1)[0] != 0:  # DID_PIMU
            # time = self.getData(device, DID_IMU_RAW, 'time')     # to plot raw gyro data
            time = self.getData(device, DID_PIMU, 'time')
            dt = self.getData(device, DID_PIMU, 'dt') 
            # dt = time[1:] - time[:-1]
            # dt = np.append(dt, dt[-1])
            # Convert from preintegrated IMU to IMU.
            for i in range(3):
                imu1[:, i] /= dt
            imuCount = 1

        else:
            time = self.getData(device, DID_REFERENCE_PIMU, 'time')

            if time.size: # DID_REFERENCE_PIMU
                dt = self.getData(device, DID_REFERENCE_PIMU, 'dt')
                if accelSensor == 0:
                    # Gyro
                    refTheta = self.getData(device, DID_REFERENCE_PIMU, 'theta')
                    ref = refTheta / dt[:,None]
                else:
                    # Accel
                    refVel = self.getData(device, DID_REFERENCE_PIMU, 'vel')
                    ref = refVel / dt[:,None]
                imu1 = []
                for sample in range(0, len(I)):
                    imu1.append(ref)
                imu1 = np.array(imu1)
                imuCount = 1

            else:  
                time = self.getData(device, DID_IMU, 'time')

                if len(time) != 0:  # DID_IMU
                    I = self.getData(device, DID_IMU, 'I')
                    dt = time[1:] - time[:-1]
                    dt = np.append(dt, dt[-1])
                    imu1 = []
                    for sample in range(0, len(I)):
                        imu1.append(I[sample][accelSensor])
                    imu1 = np.array(imu1)
                    imuCount = 1

                else:   
                    time = self.getData(device, DID_IMU3_RAW, 'time')

                    if len(time) != 0: # DID_IMU3_RAW 
                        I = self.getData(device, DID_IMU3_RAW, 'I')
                        imuStatus = self.getData(device, DID_IMU3_RAW, 'status')
                        dt = time[1:] - time[:-1]
                        dt = np.append(dt, dt[-1])
                        imu1 = None
                        imu2 = None
                        imu3 = None
                        if (imuStatus[0] & (0x00010000<<(accelSensor*3))):     # Gyro or accel 1
                            for sample in range(0, len(I)):
                                imu1.append(I[sample][0][accelSensor])
                        if (imuStatus[0] & (0x00020000<<(accelSensor*3))):     # Gyro or accel 2
                            for sample in range(0, len(I)):
                                imu2.append(I[sample][1][accelSensor])
                        if (imuStatus[0] & (0x00040000<<(accelSensor*3))):     # Gyro or accel 3
                            for sample in range(0, len(I)):
                                imu3.append(I[sample][2][accelSensor])
                        imu1 = np.array(imu1)
                        imu2 = np.array(imu2)
                        imu3 = np.array(imu3)
                        imuCount = 3

        if self.log.serials[device] != 'Ref INS':
            towOffset = self.getData(device, DID_GPS1_POS, 'towOffset')
            if towOffset.size:
                time = time + np.mean(towOffset)
        # else: # HACK: to correct for improper SPAN INS direction and gyro scalar
        #     tmp = np.copy(imu1)   
        #     tmp *= 125.0 
        #     imu1[:,0] =  tmp[:,1]
        #     imu1[:,1] =  tmp[:,0]
        #     imu1[:,2] = -tmp[:,2]

        return (time, dt, imu1, imu2, imu3, imuCount)

    def imuPQR(self, fig=None):
        if fig is None:
            fig = plt.figure()

        refTime = []
        refPqr = []
        for d in self.active_devs:
            refTime_ = self.getData(d, DID_REFERENCE_PIMU, 'time')
            if np.any(refTime_):
                refTheta = self.getData(d, DID_REFERENCE_PIMU, 'theta')
                refDt = self.getData(d, DID_REFERENCE_PIMU, 'dt')
                refPqr.append(refTheta / refDt[:,None])
                refTime.append(refTime_)

        fig.suptitle('PQR - ' + os.path.basename(os.path.normpath(self.log.directory)))
        (time, dt, acc0, acc1, acc2, pqrCount) = self.loadGyros(0)

        plotResidual = pqrCount==1 and self.residual 
        if pqrCount:
            ax = fig.subplots(3, (2 if plotResidual else pqrCount), sharex=True, squeeze=False)
        if plotResidual:
            for d in self.active_devs:
                if self.log.serials[d] == 'Ref INS':
                    (time, dt, pqr0, pqr1, pqr2, pqrCount) = self.loadGyros(d)
                    refTime = time
                    refPqr = pqr0
                    continue

        for dev_idx, d in enumerate(self.active_devs):
            (time, dt, pqr0, pqr1, pqr2, pqrCount) = self.loadGyros(d)
            if pqrCount:
                for i in range(3):
                    axislable = 'P' if (i == 0) else 'Q' if (i==1) else 'R'
                    for n, pqr in enumerate([ pqr0, pqr1, pqr2 ]):
                        if n<pqrCount:
                            if np.all(pqr) is not None:
                                pqr = quatRot(self.log.mount_bias_quat[d,:], pqr)
                                mean = np.mean(pqr[:, i])
                                std = np.std(pqr[:, i])
                                alable = 'Gyro'
                                if pqrCount > 1:
                                    alable += '%d ' % n
                                else:
                                    alable += ' '
                                self.configureSubplot(ax[i, n], alable + axislable + ' (deg/s), mean: %.4g, std: %.3g' % (mean, std), 'deg/s')
                                ax[i, n].plot(time, pqr[:, i] * 180.0/np.pi, label=self.log.serials[d])
                                if plotResidual and not (refTime is None) and self.log.serials[d] != 'Ref INS':
                                    self.configureSubplot(ax[i,1], 'Residual', 'deg/2')
                                    intPqr = np.empty_like(refPqr)
                                    intPqr[:,i] = np.interp(refTime, time, pqr[:,i], right=np.nan, left=np.nan)
                                    resPqr = intPqr - refPqr
                                    ax[i,1].plot(refTime, resPqr[:,i]*RAD2DEG, label=(self.log.serials[d] if dev_idx==0 else None))

        if not plotResidual:
            for dev_idx, d in enumerate(self.active_devs):
                if len(refTime) > 0 and len(refTime[d]) > 0: # and dev_idx == 0:    # Only plot reference IMU for first device
                    for i in range(3):
                        if dev_idx == 0:
                            plabel = 'reference'
                        else:
                            plabel = ''
                        ax[i, 0].plot(refTime[d], refPqr[d][:, i] * 180.0/np.pi, color='black', linestyle = 'dashed', label = plabel)

        for i in range(pqrCount):
            ax[0][i].legend(ncol=2)
            if plotResidual: 
                ax[0,1].legend(ncol=2)
                for i in range(3):
                    self.setPlotYSpanMin(ax[i,1], 1.0)
        for a in ax:
            for b in a:
                b.grid(True)
        self.saveFig(fig, 'pqrIMU')

    def imuAcc(self, fig=None):
        if fig is None:
            fig = plt.figure()

        refTime = []
        refAcc = []
        for d in self.active_devs:
            refTime_ = self.getData(d, DID_REFERENCE_PIMU, 'time')
            if np.any(refTime_):
                refVel = self.getData(d, DID_REFERENCE_PIMU, 'vel')
                refDt = self.getData(d, DID_REFERENCE_PIMU, 'dt')
                refAcc.append(refVel / refDt[:,None])
                refTime.append(refTime_)

        fig.suptitle('Accelerometer - ' + os.path.basename(os.path.normpath(self.log.directory)))
        (time, dt, acc0, acc1, acc2, accCount) = self.loadAccels(0)

        plotResidual = accCount==1 and self.residual 
        if accCount:
            ax = fig.subplots(3, (2 if plotResidual else accCount), sharex=True, squeeze=False)
        if plotResidual:
            for d in self.active_devs:
                if self.log.serials[d] == 'Ref INS':
                    (time, dt, acc0, acc1, acc2, accCount) = self.loadAccels(d)
                    refTime = time
                    refAcc = acc0
                    continue

        for dev_idx, d in enumerate(self.active_devs):
            (time, dt, acc0, acc1, acc2, accCount) = self.loadAccels(d)
            if accCount:
                for i in range(3):
                    axislable = 'X' if (i == 0) else 'Y' if (i==1) else 'Z'
                    for n, acc in enumerate([ acc0, acc1, acc2 ]):
                        if n<accCount:
                            if np.all(acc) is not None:
                                mean = np.mean(acc[:, i])
                                std = np.std(acc[:, i])
                                alable = 'Accel'
                                if accCount > 1:
                                    alable += '%d ' % n
                                else:
                                    alable += ' '
                                self.configureSubplot(ax[i, n], alable + axislable + ' (m/s^2), mean: %.4g, std: %.3g' % (mean, std), 'm/s^2')
                                ax[i, n].plot(time, acc[:, i], label=self.log.serials[d])
                                if plotResidual and not (refTime is None) and self.log.serials[d] != 'Ref INS':
                                    self.configureSubplot(ax[i,1], 'Residual', 'm/s^2')
                                    intAcc = np.empty_like(refAcc)
                                    intAcc[:,i] = np.interp(refTime, time, acc[:,i], right=np.nan, left=np.nan)
                                    resAcc = intAcc - refAcc
                                    ax[i,1].plot(refTime, resAcc[:,i], label=(self.log.serials[d] if dev_idx==0 else None))

        if not plotResidual:
            for dev_idx, d in enumerate(self.active_devs):
                if len(refTime) > 0 and len(refTime[d]) > 0: # and dev_idx == 0:    # Only plot reference IMU for first device
                    for i in range(3):
                        if dev_idx == 0:
                            plabel = 'reference'
                        else:
                            plabel = ''
                        ax[i, 0].plot(refTime[d], refAcc[d][:, i], color='black', linestyle = 'dashed', label = plabel)

        for i in range(accCount):
            ax[0][i].legend(ncol=2)
            if plotResidual: 
                ax[0,1].legend(ncol=2)
                for i in range(3):
                    self.setPlotYSpanMin(ax[i,1], 1.0)
        for a in ax:
            for b in a:
                b.grid(True)
        self.saveFig(fig, 'accIMU')

    def allanVariancePQR(self, fig=None):
        if fig is None:
            fig = plt.figure()

        (time, dt, pqr0, pqr1, pqr2, pqrCount) = self.loadGyros(0)
        ax = fig.subplots(3, pqrCount, sharex=True, squeeze=False)
        fig.suptitle('Allan Variance: PQR - ' + os.path.basename(os.path.normpath(self.log.directory)))

        sumARW = []
        sumBI = []

        # Set up the statistics lists
        for i in range(3):
            sumARW.append([])
            sumBI.append([])
            for n, pqr in enumerate([ pqr0, pqr1, pqr2 ]):
                sumARW[i].append([])
                sumBI[i].append([])

        for d in self.active_devs:
            (time, dt, pqr0, pqr1, pqr2, pqrCount) = self.loadGyros(d)

            if pqrCount:
                dtMean = np.mean(dt)
                for i in range(3):
                    for n, pqr in enumerate([ pqr0, pqr1, pqr2 ]):
                        if np.all(pqr) != None and n<pqrCount:
                            # Averaging window tau values from dt to dt*Nsamples/10
                            t = np.logspace(np.log10(dtMean), np.log10(0.1*np.sum(dt)), 200)
                            # Compute the overlapping ADEV
                            (t2, ad, ade, adn) = allantools.oadev(pqr[:,i], rate=1/(dtMean/self.d), data_type="freq", taus=t)
                            # Compute random walk and bias instability
                            t_bi_max = 1000
                            idx_max = (np.abs(t2 - t_bi_max)).argmin()
                            bi = np.amin(ad[0:idx_max])
                            rw_idx = (np.abs(t2 - 1.0)).argmin()
                            rw = ad[rw_idx] * np.sqrt(t2[rw_idx])
                            
                            ax[i, n].loglog(t2, ad * RAD2DEG * 3600, label='%s: %.2g, %.2g' % (self.log.serials[d], rw * RAD2DEG * 3600/RTHR2RTS, bi * RAD2DEG * 3600))

                            # (t2, ad, ade, adn) = allantools.ohdev(pqr[:,i], rate=1/(dtMean/self.d), data_type="freq", taus=t)
                            # # Compute random walk and bias instability
                            # t_bi_max = 1000
                            # idx_max = (np.abs(t2 - t_bi_max)).argmin()
                            # bi = np.amin(ad[0:idx_max])
                            # rw_idx = (np.abs(t2 - 1.0)).argmin()
                            # rw = ad[rw_idx] * np.sqrt(t2[rw_idx])

                            # ax[i, n].loglog(t2, ad * RAD2DEG * 3600, '--', label='%s: %.2f, %.3g' % (self.log.serials[d], rw * RAD2DEG * 3600/RTHR2RTS, bi * RAD2DEG * 3600))

                            sumARW[i][n].append(rw * RAD2DEG * 3600/RTHR2RTS)
                            sumBI[i][n].append(bi * RAD2DEG * 3600)

                            # Error bounds debug plots
                            # ax[i, n].loglog(t2, (ad + ade) * RAD2DEG*3600, '--')
                            # ax[i, n].loglog(t2, (ad - ade) * RAD2DEG*3600, '--')

        # Calculate the stats for ARW and bias instability over all units
        # The plots show the mean + 1 std deviation in accordance with IEEE spec (Analog Devices website)
        for i in range(3):
            axislable = 'P' if (i == 0) else 'Q' if (i==1) else 'R'
            for n, pqr in enumerate([ pqr0, pqr1, pqr2 ]):
                if np.all(pqr) != None and n<pqrCount:
                    alable = 'Gyro'
                    if pqrCount > 1:
                        alable += '%d ' % n
                    else:
                        alable += ' '
                    self.configureSubplot(ax[i, n], alable + axislable + ' ($deg/hr$), ARW: %.3g $deg/\sqrt{hr}$,  BI: %.3g $deg/hr$' % (np.mean(sumARW[i][n]) + np.std(sumARW[i][n]), np.mean(sumBI[i][n]) + np.std(sumBI[i][n])), 'deg/hr')

        for i in range(pqrCount):
            for d in range(3):
                ax[d][i].grid(True, which='both')
                ax[d][i].legend(ncol=2)
        self.saveFig(fig, 'pqrIMU')

        with open(self.log.directory + '/allan_variance_pqr.csv', 'w') as f:
            f.write('Hardware,Date,SN,BI-P,BI-Q,BI-R,ARW-P,ARW-Q,ARW-R,BI-X\n')
            f.write(',,,(deg/hr),(deg/hr),(deg/hr),(deg / rt hr),(deg / rt hr),(deg / rt hr)\n')
            today = date.today()
            for d in self.active_devs:
                hdwVer = self.getData(d, DID_DEV_INFO, 'hardwareVer')[d]
                f.write('%d.%d.%d,%s,%d,' % (hdwVer[0], hdwVer[1], hdwVer[2], str(today), self.log.serials[d]))
                for n, pqr in enumerate([ pqr0, pqr1, pqr2 ]):
                    if np.all(pqr) != None and n<pqrCount:
                        for i in range(3):
                            f.write('%f,' % (sumBI[i][n][d]))
                        for i in range(3):
                            f.write('%f,' % (sumARW[i][n][d]))
                f.write('\n')

    def allanVarianceAcc(self, fig=None):
        if fig is None:
            fig = plt.figure()

        (time, dt, acc0, acc1, acc2, accCount) = self.loadAccels(0)
        ax = fig.subplots(3, accCount, sharex=True, squeeze=False)
        fig.suptitle('Allan Variance: Accelerometer - ' + os.path.basename(os.path.normpath(self.log.directory)))

        sumRW = []
        sumBI = []

        # Set up the statistics lists
        for i in range(3):
            sumRW.append([])
            sumBI.append([])
            for n, pqr in enumerate([ acc0, acc1, acc2 ]):
                sumRW[i].append([])
                sumBI[i].append([])


        for d in self.active_devs:
            (time, dt, acc0, acc1, acc2, accCount) = self.loadAccels(d)

            dtMean = np.mean(dt)
            for i in range(3):
                for n, acc in enumerate([ acc0, acc1, acc2 ]):
                    if np.all(acc) != None and n<accCount:
                        if acc.any(None):
                            # Averaging window tau values from dt to dt*Nsamples/10
                            t = np.logspace(np.log10(dtMean), np.log10(0.1*np.sum(dt)), 200)
                            # Compute the overlapping ADEV
                            (t2, ad, ade, adn) = allantools.oadev(acc[:,i], rate=1/(dtMean/self.d), data_type="freq", taus=t)
                            # Compute random walk and bias instability
                            t_bi_max = 1000
                            idx_max = (np.abs(t2 - t_bi_max)).argmin()
                            bi = np.amin(ad[0:idx_max])
                            rw_idx = (np.abs(t2 - 0.1)).argmin()
                            rw = ad[rw_idx] * np.sqrt(t2[rw_idx])

                            ax[i, n].loglog(t2, ad, label='%s: %.2g, %.2g' % (self.log.serials[d], rw * RTHR2RTS, bi))

                            sumRW[i][n].append(rw * RTHR2RTS) 
                            sumBI[i][n].append(bi)

        # Calculate the stats for ARW and bias instability over all units
        # The plots show the mean + 1 std deviation in accordance with IEEE spec (Analog Devices website)
        for i in range(3):
            axislable = 'X' if (i == 0) else 'Y' if (i==1) else 'Z'
            for n, pqr in enumerate([ acc0, acc1, acc2 ]):
                if np.all(pqr) != None and n<accCount:
                    alable = 'Accel'
                    if accCount > 1:
                        alable += '%d ' % n
                    else:
                        alable += ' '
                    self.configureSubplot(ax[i, n], alable + axislable + ' ($m/s^2$), RW: %.3g $m/s/\sqrt{hr}$, BI: %.3g $m/s^2$' % (np.mean(sumRW[i][n]) + np.std(sumRW[i][n]), np.mean(sumBI[i][n]) + np.std(sumBI[i][n])), 'm/s^2')

        for i in range(accCount):
            for d in range(3):
                ax[d][i].grid(True, which='both')
                ax[d][i].legend(ncol=2)
        self.saveFig(fig, 'accIMU')        

        with open(self.log.directory + '/allan_variance_acc.csv', 'w') as f:
            f.write('Hardware,Date,SN,BI-X,BI-Y,BI-Z,ARW-X,ARW-Y,ARW-Z\n')
            f.write(',,,(m/s^2 / hr),(m/s^2 / hr),(m/s^2 / hr),(m/s / rt hr),(m/s / rt hr),(m/s / rt hr)\n')
            today = date.today()
            for d in self.active_devs:
                hdwVer = self.getData(d, DID_DEV_INFO, 'hardwareVer')[d]
                f.write('%d.%d.%d,%s,%d,' % (hdwVer[0], hdwVer[1], hdwVer[2], str(today), self.log.serials[d]))
                for n, acc in enumerate([ acc0, acc1, acc2 ]):
                    if np.all(acc) != None and n<accCount:
                        for i in range(3):
                            f.write('%f,' % (sumBI[i][n][d]))
                        for i in range(3):
                            f.write('%f,' % (sumRW[i][n][d]))
                f.write('\n')

    def accelPSD(self, fig=None):
        if fig is None:
            fig = plt.figure()

        (time, dt, acc0, acc1, acc2, accCount) = self.loadAccels(0)
        ax = fig.subplots(3, accCount, sharex=True, squeeze=False)
        fig.suptitle('Power Spectral Density - ' + os.path.basename(os.path.normpath(self.log.directory)))
        
        for d in self.active_devs:
            (time, dt, acc0, acc1, acc2, accCount) = self.loadAccels(d)
            refTime = self.getData(d, DID_REFERENCE_PIMU, 'time')
            if np.any(refTime):
                refVel = self.getData(d, DID_REFERENCE_PIMU, 'vel')
                refDt = self.getData(d, DID_REFERENCE_PIMU, 'dt')
                refAcc = refVel / refDt[:,None]

            N = time.size
            psd = np.zeros((N//2, 3))
            # 1/T = frequency
            Fs = 1 / np.mean(dt)
            f = np.linspace(0, 0.5*Fs, N // 2)

            for n, acc in enumerate([ acc0, acc1, acc2 ]):
                if np.all(acc) != None and n<accCount:
                    for i in range(3):
                        sp0 = np.fft.fft(acc[:,i] / 9.8)
                        sp0 = sp0[:N // 2]
                        # psd = abssp*abssp
                        # freq = np.fft.fftfreq(time.shape[-1])
        #                    np.append(psd, [1/N/Fs * np.abs(sp0)**2], axis=1)
                        psd[:,i] = 1/N/Fs * np.abs(sp0)**2
                        psd[1:-1,i] = 2 * psd[1:-1,i]

                    for i in range(3):
                        axislable = 'X' if (i == 0) else 'Y' if (i==1) else 'Z'
                        # ax[i].loglog(f, psd[:, i])
                        alable = 'Accel'
                        if accCount > 1:
                            alable += '%d ' % n
                        else:
                            alable += ' '
                        self.configureSubplot(ax[i, n], alable + axislable + ' PSD (dB (m/s^2)^2/Hz)', 'Hz')
                        ax[i][n].plot(f, 10*np.log10(psd[:, i]), label=self.log.serials[d])

        for i in range(accCount):
            ax[0][i].legend(ncol=2)
            for d in range(3):
                ax[d][i].grid(True)
        self.saveFig(fig, 'accelPSD')

    def gyroPSD(self, fig=None):
        if fig is None:
            fig = plt.figure()

        (time, dt, pqr0, pqr1, pqr2, pqrCount) = self.loadGyros(0)
        ax = fig.subplots(3, pqrCount, sharex=True, squeeze=False)
        fig.suptitle('Power Spectral Density - ' + os.path.basename(os.path.normpath(self.log.directory)))
        
        for d in self.active_devs:
            (time, dt, pqr0, pqr1, pqr2, pqrCount) = self.loadGyros(d)
            refTime = self.getData(d, DID_REFERENCE_PIMU, 'time')
            if np.any(refTime):
                refImu = self.getData(d, DID_REFERENCE_PIMU, 'I')
                refImu = refImu
                refAcc = refImu['acc']

            N = time.size
            Nhalf = N // 2 + 1
            psd = np.zeros((Nhalf, 3))
            # 1/T = frequency
            Fs = 1 / np.mean(dt)
            f = np.linspace(0, 0.5*Fs, Nhalf)

            for n, pqr in enumerate([ pqr0, pqr1, pqr2 ]):
                if np.all(pqr) != None and n<pqrCount:
                    for i in range(3):
                        sp0 = np.fft.fft(pqr[:,i] * 180.0/np.pi)
                        sp0 = sp0[:Nhalf]
                        # psd = abssp*abssp
                        # freq = np.fft.fftfreq(time.shape[-1])
            #                    np.append(psd, [1/N/Fs * np.abs(sp0)**2], axis=1)
                        psd[:,i] = 1/N/Fs * np.abs(sp0)**2
                        psd[1:-1,i] = 2 * psd[1:-1,i]

                    for i in range(3):
                        axislable = 'P' if (i == 0) else 'Q' if (i==1) else 'R'
                        # ax[i].loglog(f, psd[:, i])
                        alable = 'Gyro'
                        if pqrCount > 1:
                            alable += '%d ' % n
                        else:
                            alable += ' '
                        self.configureSubplot(ax[i, n], alable + axislable + ' PSD (dB dps^2/Hz)', 'Hz')
                        ax[i][n].plot(f, 10*np.log10(psd[:, i]), label=self.log.serials[d])

        for i in range(pqrCount):
            ax[0][i].legend(ncol=2)
            for d in range(3):
                ax[d][i].grid(True)
        self.saveFig(fig, 'gyroPSD')

    def altitude(self, fig=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(3, 1, sharex=True)

        self.configureSubplot(ax[0], 'Altitude - Barometer', ',m')
        self.configureSubplot(ax[1], 'Altitude - GPS', ',m')
        self.configureSubplot(ax[2], 'Altitude - Barometer & GPS', ',m')
        fig.suptitle('Altitude - ' + os.path.basename(os.path.normpath(self.log.directory)))
        
        for d in self.active_devs:
            timeBar = self.getData(d, DID_BAROMETER, 'time')
            towOffset = self.getData(d, DID_GPS1_POS, 'towOffset')
            timeGps = getTimeFromTowMs(self.getData(d, DID_GPS1_POS, 'timeOfWeekMs'))
            altGps = self.getData(d, DID_GPS1_POS, 'lla')[:, 2]
            if np.shape(towOffset)[0] != 0:
                timeBar = timeBar + towOffset[-1]
            mslBar = self.getData(d, DID_BAROMETER, 'mslBar')
            ax[0].plot(timeBar, mslBar, label=self.log.serials[d])
            ax[1].plot(timeGps, altGps)
            ax[2].plot(timeBar, mslBar - (mslBar[0] - altGps[0]), label=("Bar %s" % self.log.serials[d]))
            ax[2].plot(timeGps, altGps, label=("GPS %s" % self.log.serials[d]))

        ax[0].legend(ncol=2)
        ax[2].legend(ncol=2)
        for a in ax:
            a.grid(True)
        self.saveFig(fig, 'altitude')

    def magnetometer(self, fig=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(3, 1, sharex=True)

        self.configureSubplot(ax[0], 'Mag X', 'gauss')
        self.configureSubplot(ax[1], 'Mag Y', 'gauss')
        self.configureSubplot(ax[2], 'Mag Z', 'gauss')
        fig.suptitle('Magnetometer - ' + os.path.basename(os.path.normpath(self.log.directory)))
        for d in self.active_devs:
            if 1:
                time = self.getData(d, DID_MAGNETOMETER, 'time')
                towOffset = self.getData(d, DID_GPS1_POS, 'towOffset')
                if np.shape(towOffset)[0] != 0:
                    time = time + towOffset[-1]
                mag = self.getData(d, DID_MAGNETOMETER, 'mag')
                magX = mag[:,0]
                magY = mag[:,1]
                magZ = mag[:,2]
            else:
                mag = self.getData(d, DID_SENSORS_UCAL, 'mag')
                magX = mag[:,0]['xyz'][:,0]
                magY = mag[:,0]['xyz'][:,1]
                magZ = mag[:,0]['xyz'][:,2]
                time = range(np.shape(magX)[0])
            ax[0].plot(time, magX, label=self.log.serials[d])
            ax[1].plot(time, magY)
            ax[2].plot(time, magZ)

            refTime = self.getData(d, DID_REFERENCE_MAGNETOMETER, 'time')
            if len(refTime)!=0:
                refMag = self.getData(d, DID_REFERENCE_MAGNETOMETER, 'mag')
                refMag = refMag
                for i in range(3):
                    ax[2*i].plot(refTime, refMag[:, i], color='red', label="reference")

        ax[0].legend(ncol=2)
        for a in ax:
            a.grid(True)
        self.saveFig(fig, 'magnetometer')


    def temp(self, fig=None):
        try:
            if fig is None:
                fig = plt.figure()
            ax = fig.subplots(3, 1, sharex=True)
            fig.suptitle('Temperature - ' + os.path.basename(os.path.normpath(self.log.directory)))

            self.configureSubplot(ax[0], 'IMU Temperature (C)')
            self.configureSubplot(ax[1], 'Barometer Temperature (C)')
            self.configureSubplot(ax[2], 'MCU Temperature (C)')

            for d in self.active_devs:
                time = getTimeFromTowMs(self.getData(d, DID_SYS_PARAMS, 'timeOfWeekMs'))
                tempImu = self.getData(d, DID_SYS_PARAMS, 'imuTemp')
                tempBar = self.getData(d, DID_SYS_PARAMS, 'baroTemp')
                tempMcu = self.getData(d, DID_SYS_PARAMS, 'mcuTemp')
                
                ax[0].plot(time, tempImu, label=self.log.serials[d])
                ax[1].plot(time, tempBar)
                ax[1].plot(time, tempMcu)
            for a in ax:
                a.grid(True)
            self.saveFig(fig, 'Temp')
        except:
            print(RED + "problem plotting temp: " + sys.exc_info()[0] + RESET)

    def debugfArr(self, fig=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(5,2, sharex=True)
        fig.suptitle('Debug float Array - ' + os.path.basename(os.path.normpath(self.log.directory)))
        for d in self.active_devs:
            debug_f = self.getData(d, DID_DEBUG_ARRAY, 'f')
            for i in range(9):
                ax[i%5, i//5].set_ylabel('f[' + str(i) +']')
                ax[i%5, i//5].plot(debug_f[:,i], label=self.log.serials[d])
        ax[0,0].legend(ncol=2)
        for b in ax:
            for a in b:
                a.grid(True)

    def debugiArr(self, fig=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(5,2, sharex=True)
        fig.suptitle('Debug int array - ' + os.path.basename(os.path.normpath(self.log.directory)))
        for d in self.active_devs:
            debug_i = self.getData(d, DID_DEBUG_ARRAY, 'i')
            for i in range(9):
                ax[i%5, i//5].set_ylabel('i[' + str(i) +']')
                ax[i%5, i//5].plot(debug_i[:,i], label=self.log.serials[d])
        ax[0,0].legend(ncol=2)
        for b in ax:
            for a in b:
                a.grid(True)

    def debuglfArr(self, fig=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(3,1, sharex=True)
        fig.suptitle('Debug double Array - ' + os.path.basename(os.path.normpath(self.log.directory)))
        for d in self.active_devs:
            debug_lf = self.getData(d, DID_DEBUG_ARRAY, 'lf')
            for i in range(3):
                ax[i].set_ylabel('lf[' + str(i) +']')
                ax[i].plot(debug_lf[:,i], label=self.log.serials[d])
        ax[0].legend(ncol=2)
        for a in ax:
            a.grid(True)

    def gpxDebugfArray(self, fig=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(5,2, sharex=True)
        fig.suptitle('GPX Debug float Array - ' + os.path.basename(os.path.normpath(self.log.directory)))
        for d in self.active_devs:
            debug_f = self.getData(d, DID_GPX_DEBUG_ARRAY, 'f')
            for i in range(9):
                ax[i%5, i//5].set_ylabel('f[' + str(i) +']')
                ax[i%5, i//5].plot(debug_f[:,i], label=self.log.serials[d])
        ax[0,0].legend(ncol=2)
        for b in ax:
            for a in b:
                a.grid(True)

    def magDec(self, fig=None):
        if fig is None:
            fig = plt.figure()

        ax = fig.subplots(2, 1, sharex=True)
        fig.suptitle('Magnetometer Declination - ' + os.path.basename(os.path.normpath(self.log.directory)))
        self.configureSubplot(ax[0], 'Declination', 'deg')
        self.configureSubplot(ax[1], 'Inclination', 'deg')

        for d in self.active_devs:
            time = getTimeFromTow(self.getData(d, DID_INL2_STATES, 'timeOfWeek'))
            mag_declination = 180.0/np.pi * self.getData(d, DID_INL2_STATES, 'magDec')
            mag_inclination = 180.0/np.pi * self.getData(d, DID_INL2_STATES, 'magInc')
            ax[0].plot(time, mag_declination, label=self.log.serials[d])
            ax[1].plot(time, mag_inclination)
        ax[0].legend(ncol=2)
        self.saveFig(fig, 'magDec')
        for a in ax:
            a.grid(True)

    def deltatime(self, fig=None):
        if fig is None:
            fig = plt.figure()

        refImuPresent = False
        for d in self.active_devs:
            timeRef = self.getData(d, DID_REFERENCE_PIMU, 'time')
            if np.any(timeRef):
                refImuPresent = True

        N = 4
        if refImuPresent:
            N = N + 2
        ax = fig.subplots(N, 1, sharex=True)

        fig.suptitle('Timestamps - ' + os.path.basename(os.path.normpath(self.log.directory)))
        self.configureSubplot(ax[0], 'INS dt', 's')
        self.configureSubplot(ax[1], 'GPS dt', 's')
        self.configureSubplot(ax[2], 'IMU Integration Period', 's')
        self.configureSubplot(ax[3], 'IMU Delta Timestamp', 's')

        for d in self.active_devs_no_ref:
            dtIns = self.getData(d, DID_INS_2, 'timeOfWeek')[1:] - self.getData(d, DID_INS_2, 'timeOfWeek')[0:-1]
            dtIns = dtIns / self.d
            timeIns = getTimeFromTow(self.getData(d, DID_INS_2, 'timeOfWeek')[1:])

            dtGps = 0.001*(self.getData(d, DID_GPS1_POS, 'timeOfWeekMs')[1:] - self.getData(d, DID_GPS1_POS, 'timeOfWeekMs')[0:-1])
            dtGps = dtGps / self.d
            timeGps = getTimeFromTowMs(self.getData(d, DID_GPS1_POS, 'timeOfWeekMs')[1:])

            dtPimu = self.getData(d, DID_PIMU, 'dt')
            if dtPimu.size:
                integrationPeriod = dtPimu[1:]
            else:
                integrationPeriod = np.empty_like(timeIns)

            towOffset = self.getData(d, DID_GPS1_POS, 'towOffset')
            if np.size(towOffset) > 0:
                towOffset = towOffset[-1]
            else:
                towOffset = 0

            deltaTimestamp = 0
            timeImu = 0
            timePimu = self.getData(d, DID_PIMU, 'time')
            timeIMU = self.getData(d, DID_IMU, 'time')
            timeImu3 = self.getData(d, DID_IMU3_RAW, 'time')
            if timePimu.size:
                deltaTimestamp = timePimu[1:] - timePimu[0:-1]
                deltaTimestamp = deltaTimestamp / self.d
                timeImu = getTimeFromTow(timePimu[1:] + towOffset)            
            elif timeIMU.size:
                deltaTimestamp = timeIMU[1:] - timeIMU[0:-1]
                deltaTimestamp = deltaTimestamp / self.d
                timeImu = getTimeFromTow(timeIMU[1:] + towOffset)
            elif timeImu3.size:
                deltaTimestamp = timeImu3[1:] - timeImu3[0:-1]
                deltaTimestamp = deltaTimestamp / self.d
                timeImu = getTimeFromTow(timeImu3[1:] + towOffset)

            ax[0].plot(timeIns, dtIns, label=self.log.serials[d])
            ax[1].plot(timeGps, dtGps)
            if integrationPeriod.size:
                ax[2].plot(timeImu, integrationPeriod)
            ax[3].plot(timeImu, deltaTimestamp)

        self.setPlotYSpanMin(ax[0], 0.005)
        self.setPlotYSpanMin(ax[1], 0.005)
        self.setPlotYSpanMin(ax[2], 0.005)
        self.setPlotYSpanMin(ax[3], 0.005)

        if refImuPresent:
            self.configureSubplot(ax[4], 'Reference IMU Integration Period', 's')
            self.configureSubplot(ax[5], 'Reference IMU Delta Timestamp', 's')
            for d in self.active_devs:
                deltaTimestampRef = 0
                timeImuRef = 0
                timeRef = self.getData(d, DID_REFERENCE_PIMU, 'time')
                if np.any(timeRef):
                    integrationPeriodRef = self.getData(d, DID_REFERENCE_PIMU, 'dt')[1:]
                    deltaTimestampRef = timeRef[1:] - timeRef[0:-1]
                    deltaTimestampRef = deltaTimestampRef / self.d
                    timeImuRef = getTimeFromTow(timeRef[1:] + towOffset)
                    ax[4].plot(timeImuRef, integrationPeriodRef)
                    ax[5].plot(timeImuRef, deltaTimestampRef)
            self.setPlotYSpanMin(ax[4], 0.005)
            self.setPlotYSpanMin(ax[5], 0.005)

        ax[0].legend(ncol=2)
        for a in ax:
            a.grid(True)
        self.saveFig(fig, 'deltatime')

    def gpsRawTime(self, fig=None):
        if fig is None:
            fig = plt.figure()

        ax = fig.subplots(6, 1, sharex=True)
        fig.suptitle('Timestamps - ' + os.path.basename(os.path.normpath(self.log.directory)))
        self.configureSubplot(ax[0], 'GPS1 Raw dt', 's')
        self.configureSubplot(ax[1], 'GPS2 Raw dt', 's')
        self.configureSubplot(ax[2], 'GPS Base Raw dt', 's')
        self.configureSubplot(ax[3], 'GPS1 Raw Number of Satellites Observed', 's')
        self.configureSubplot(ax[4], 'GPS2 Raw Number of Satellites Observed', 's')
        self.configureSubplot(ax[5], 'GPS Base Raw Number of Satellites Observed', 's')

        for d in self.active_devs:
            N1 = len(self.log.data[d, DID_GPS1_RAW][0])
            N2 = len(self.log.data[d, DID_GPS2_RAW][0])
            NB = len(self.log.data[d, DID_GPS_BASE_RAW][0])
            tgps1 = np.zeros(N1)
            nsat1 = np.zeros(N1)
            tgps2 = np.zeros(N2)
            nsat2 = np.zeros(N2)
            tgpsB = np.zeros(NB)
            nsatB = np.zeros(NB)
            cnt = 0
            for iobs in range(N1):
                ns = round(len(self.log.data[d, DID_GPS1_RAW][0][iobs]) * 0.5) # 0.5 because there is a bug that pads half of the data with zeros
                t0 = self.log.data[d, DID_GPS1_RAW][0][iobs]['time']['time'][-1] + \
                     self.log.data[d, DID_GPS1_RAW][0][iobs]['time']['sec'][-1]
                nsat1[cnt] = nsat1[cnt] + ns
                tgps1[cnt] = t0
                if iobs < N1 - 1:
                    t1 = self.log.data[d, DID_GPS1_RAW][0][iobs + 1]['time']['time'][-1] + \
                         self.log.data[d, DID_GPS1_RAW][0][iobs + 1]['time']['sec'][-1]
                    if t1 > t0 + 0.01:
                        cnt = cnt + 1
            tgps1 = tgps1[0: cnt + 1]
            nsat1 = nsat1[0: cnt + 1]
            cnt = 0
            for iobs in range(N2):
                ns = round(len(self.log.data[d, DID_GPS2_RAW][0][iobs]) * 0.5) # 0.5 because there is a bug that pads half of the data with zeros
                t0 = self.log.data[d, DID_GPS2_RAW][0][iobs]['time']['time'][-1] + \
                     self.log.data[d, DID_GPS2_RAW][0][iobs]['time']['sec'][-1]
                nsat2[cnt] = nsat2[cnt] + ns
                tgps2[cnt] = t0
                if iobs < N2 - 1:
                    t1 = self.log.data[d, DID_GPS2_RAW][0][iobs + 1]['time']['time'][-1] + \
                         self.log.data[d, DID_GPS2_RAW][0][iobs + 1]['time']['sec'][-1]
                    if t1 > t0 + 0.01:
                        cnt = cnt + 1
            tgps2 = tgps2[0: cnt + 1]
            nsat2 = nsat2[0: cnt + 1]
            cnt = 0
            for iobs in range(NB):
                ns = round(len(self.log.data[d, DID_GPS_BASE_RAW][0][iobs]) * 0.5) # 0.5 because there is a bug that pads half of the data with zeros
                t0 = self.log.data[d, DID_GPS_BASE_RAW][0][iobs]['time']['time'][-1] + \
                     self.log.data[d, DID_GPS_BASE_RAW][0][iobs]['time']['sec'][-1]
                nsatB[cnt] = nsatB[cnt] + ns
                tgpsB[cnt] = t0
                if iobs < NB - 1:
                    t1 = self.log.data[d, DID_GPS_BASE_RAW][0][iobs + 1]['time']['time'][-1] + \
                         self.log.data[d, DID_GPS_BASE_RAW][0][iobs + 1]['time']['sec'][-1]
                    if t1 > t0 + 0.01:
                        cnt = cnt + 1
            tgpsB = tgpsB[0: cnt + 1]
            nsatB = nsatB[0: cnt + 1]
            dtGps1 = tgps1[1:] - tgps1[0:-1]
#            dtGps1 = dtGps1 / self.d
            dtGps2 = tgps2[1:] - tgps2[0:-1]
#            dtGps2 = dtGps2 / self.d
            dtGpsB = tgpsB[1:] - tgpsB[0:-1]

            ax[0].plot(tgps1[1:], dtGps1, label=self.log.serials[d])
            ax[1].plot(tgps2[1:], dtGps2)
            ax[2].plot(tgpsB[1:], dtGpsB)
            ax[3].plot(tgps1, nsat1, label=self.log.serials[d])
            ax[4].plot(tgps2, nsat2)
            ax[5].plot(tgpsB, nsatB)

        self.setPlotYSpanMin(ax[0], 2.0)
        self.setPlotYSpanMin(ax[1], 2.0)
        self.setPlotYSpanMin(ax[2], 2.0)
        self.setPlotYSpanMin(ax[3], 25)
        self.setPlotYSpanMin(ax[4], 25)
        self.setPlotYSpanMin(ax[5], 25)

        ax[0].legend(ncol=2)
        for a in ax:
            a.grid(True)
        self.saveFig(fig, 'gpsRawTime')

    def ekfBiases(self, fig=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(4, 2, sharex=True)
        self.configureSubplot(ax[0,0], 'bias P', 'deg/s')
        self.configureSubplot(ax[1,0], 'bias Q', 'deg/s')
        self.configureSubplot(ax[2,0], 'bias R', 'deg/s')
        self.configureSubplot(ax[3,0], 'bias Barometer', 'm')

        self.configureSubplot(ax[0,1], 'bias acc X', 'm/s^2')
        self.configureSubplot(ax[1,1], 'bias acc Y', 'm/s^2')
        self.configureSubplot(ax[2,1], 'bias acc Z', 'm/s^2')
        fig.suptitle('EKF Biases - ' + os.path.basename(os.path.normpath(self.log.directory)))
        for d in self.active_devs:
            time = getTimeFromTow(self.getData(d, DID_INL2_STATES, 'timeOfWeek'))
            ax[0,0].plot(time, self.getData(d, DID_INL2_STATES, 'biasPqr')[:, 0]*180.0/np.pi, label=self.log.serials[d])
            ax[1,0].plot(time, self.getData(d, DID_INL2_STATES, 'biasPqr')[:, 1]*180.0/np.pi)
            ax[2,0].plot(time, self.getData(d, DID_INL2_STATES, 'biasPqr')[:, 2]*180.0/np.pi)
            ax[3,0].plot(time, self.getData(d, DID_INL2_STATES, 'biasBaro'), label=self.log.serials[d])

            ax[0,1].plot(time, self.getData(d, DID_INL2_STATES, 'biasAcc')[:, 0], label=self.log.serials[d])
            ax[1,1].plot(time, self.getData(d, DID_INL2_STATES, 'biasAcc')[:, 1])
            ax[2,1].plot(time, self.getData(d, DID_INL2_STATES, 'biasAcc')[:, 2])

        ax[0,0].legend(ncol=2)
        for a in ax:
            for b in a:
                b.grid(True)
        self.saveFig(fig, 'ekfBiases')

    def rtkResiduals(self, type, page, fig=None):
        if fig is None:
            fig = plt.figure()

        if type == 'phase':
            did = DID_RTK_PHASE_RESIDUAL
        elif type == 'code':
            did = DID_RTK_CODE_RESIDUAL

        sat_ids = np.unique(self.log.data[0, did]['sat_id_j'])
        sat_ids = sat_ids[sat_ids != 0][page*6:(page+1)*6]

        ax = fig.subplots(6, 1, sharex=True)
        fig.suptitle(type + ' Residuals Page ' + str(page+1) + ' - ' + os.path.basename(os.path.normpath(self.log.directory)))

        for i, id in enumerate(sat_ids):
            if id == 0: continue
            ax[i].set_ylabel(str(id))
            for d in self.active_devs:
                idx = np.where(self.getData(d, did, 'sat_id_j') == id)
                time_idx = idx[0]
                sat_state_idx = idx[1]
                time = np.array(getTimeFromGTime(self.getData(d, did, 'time')))[time_idx]
                residuals = self.getData(d, did, 'v')[time_idx, sat_state_idx]
                residuals[np.abs(residuals) > 1e6] = np.nan
                ax[i].plot(time, residuals, label=self.log.serials[d])
        ax[0].legend(ncol=2)

    def rtkDebug(self, fig=None):
        if fig is None:
            fig = plt.figure()

        rtkData = self.log.data[0, DID_RTK_DEBUG]
        if rtkData.size == 0:
            return 
        fields = list(rtkData.dtype.names)
        fields.remove('time')
        num_plots = 0
        for field in fields:
            dat = rtkData[field][0]
            if isinstance(dat, np.ndarray):
                num_plots += len(dat)
            else:
                num_plots += 1

        cols = 4
        rows = math.ceil(num_plots/float(cols))
        ax = fig.subplots(rows, cols, sharex=True)
        fig.suptitle('RTK Debug Counters - ' + os.path.basename(os.path.normpath(self.log.directory)))

        for d in self.active_devs:
            time = np.array(getTimeFromGTime(self.getData(d, DID_RTK_DEBUG, 'time')))
            valid = time > datetime.datetime.strptime('2017', "%Y")

            # Use index rather than time
            if 0:
                time = np.arange(0, len(time))

            i = 0
            for field in fields:
                data = self.getData(d, DID_RTK_DEBUG, field)[valid]
                if (len(data) == 0):
                    continue
                if isinstance(data[0], np.ndarray):
                    for j in range(len(data[0])):
                        ax[ i%rows, i//rows].set_title(field + "_" + str(j))
                        ax[ i % rows, i // rows ].plot(time[valid], data[:,j], label=self.log.serials[d])
                        i += 1
                else:
                    ax[i % rows, i // rows].set_title(field)
                    ax[i % rows, i // rows].title.set_fontsize(8)
                    # for item in ax[i % rows, i // rows].get_yticklabels():
                        # item.set_fontsize(8)
                    ax[i % rows, i // rows].plot(time[valid], data, label=self.log.serials[d])
                    i += 1
        ax[0,0].legend(ncol=2)

    def rtkDebug2(self, fig=None):
        if fig is None:
            fig = plt.figure()

        ax = fig.subplots(6, 4, sharex=True)

        rtkDebug2 = self.getData(0, DID_RTK_DEBUG_2, 'num_biases')
        if not np.any(rtkDebug2):
            return
        max_num_biases = rtkDebug2[-1]
        for r in range(0,6):
            for c in range(0,4):
                self.configureSubplot(ax[r,c])

        fig.suptitle('RTK Debug2 - ' + os.path.basename(os.path.normpath(self.log.directory)))
        for d in self.active_devs:
            time = np.array(getTimeFromGTime(self.getData(d, DID_RTK_DEBUG_2, 'time')))
            ib = 0
            for r in range(0, 6):
                for c in range(0, 4):
                    if ib < max_num_biases:
                        ax[r,c].plot(time, self.getData(d, DID_RTK_DEBUG_2, 'satBiasFloat')[:, c + r * 4], label=self.log.serials[d])
                        r1 = r
                        c1 = c
                    ib = ib + 1

        # Show serial numbers
        ax[r1,c1].legend(ncol=2)

        for a in ax:
            for b in a:
                b.grid(True)

    def rtkDebug2Sat(self, fig=None):
        if fig is None:
            fig = plt.figure()

        ax = fig.subplots(6, 4, sharex=True)

        rtkDebug2 = self.getData(0, DID_RTK_DEBUG_2, 'num_biases')
        if not np.any(rtkDebug2):
            return
        max_num_biases = rtkDebug2[-1]
        for r in range(0,6):
            for c in range(0,4):
                self.configureSubplot(ax[r,c])

        fig.suptitle('RTK Debug2 - Sat# - ' + os.path.basename(os.path.normpath(self.log.directory)))
        for d in self.active_devs:
            time = np.array(getTimeFromGTime(self.getData(d, DID_RTK_DEBUG_2, 'time')))
            ib = 0
            for r in range(0, 6):
                for c in range(0, 4):
                    if ib < max_num_biases:
                        ax[r,c].plot(time, self.getData(d, DID_RTK_DEBUG_2, 'sat')[:, c + r * 4], label=self.log.serials[d])
                        r1 = r
                        c1 = c
                    ib = ib + 1

        # Show serial numbers
        ax[r1,c1].legend(ncol=2)

        for a in ax:
            for b in a:
                b.grid(True)

    def rtkDebug2Std(self, fig=None):
        if fig is None:
            fig = plt.figure()

        ax = fig.subplots(6, 4, sharex=True)

        rtkDebug2 = self.getData(0, DID_RTK_DEBUG_2, 'num_biases')
        if not np.any(rtkDebug2):
            return
        max_num_biases = rtkDebug2[-1]
        for r in range(0,6):
            for c in range(0,4):
                self.configureSubplot(ax[r,c])

        fig.suptitle('RTK Debug 2 - Sat Bias Std - ' + os.path.basename(os.path.normpath(self.log.directory)))
        for d in self.active_devs:
            time = np.array(getTimeFromGTime(self.getData(d, DID_RTK_DEBUG_2, 'time')))
            ib = 0
            for r in range(0, 6):
                for c in range(0, 4):
                    if ib < max_num_biases:
                        ax[r,c].plot(time, self.getData(d, DID_RTK_DEBUG_2, 'satBiasStd')[:, c + r * 4], label=self.log.serials[d])
                        r1 = r
                        c1 = c
                    ib = ib + 1

        # Show serial numbers
        ax[r1,c1].legend(ncol=2)

        for a in ax:
            for b in a:
                b.grid(True)

    def rtkDebug2Lock(self, fig=None):
        if fig is None:
            fig = plt.figure()

        ax = fig.subplots(6, 4, sharex=True)

        rtkDebug2 = self.getData(0, DID_RTK_DEBUG_2, 'num_biases')
        if not np.any(rtkDebug2):
            return
        max_num_biases = rtkDebug2[-1]
        for r in range(0,6):
            for c in range(0,4):
                self.configureSubplot(ax[r,c])

        fig.suptitle('RTK Debug 2 - Lock Count - ' + os.path.basename(os.path.normpath(self.log.directory)))
        for d in self.active_devs:
            time = np.array(getTimeFromGTime(self.getData(d, DID_RTK_DEBUG_2, 'time')))
            ib = 0
            for r in range(0, 6):
                for c in range(0, 4):
                    if ib < max_num_biases:
                        ax[r,c].plot(time, self.getData(d, DID_RTK_DEBUG_2, 'satLockCnt')[:, c + r * 4], label=self.log.serials[d])
                        r1 = r
                        c1 = c
                    ib = ib + 1

        # Show serial numbers
        ax[r1,c1].legend(ncol=2)

        for a in ax:
            for b in a:
                b.grid(True)

    def wheelEncoder(self, fig=None):
        if fig is None:
            fig = plt.figure()

        fig.suptitle('Wheel Encoder - ' + os.path.basename(os.path.normpath(self.log.directory)))
        ax = fig.subplots(4, 1, sharex=True)
        titles = ['Left Wheel Angle', 'Right Wheel Angle', 'Left Wheel Velocity', 'Right Wheel Velocity']
        fields = ['theta_l', 'theta_r', 'omega_l', 'omega_r']

        for d in self.active_devs:
            time = np.array(getTimeFromTow(self.getData(d, DID_WHEEL_ENCODER, 'timeOfWeek')))
            for i, a in enumerate(ax):
                a.plot(time, self.getData(d, DID_WHEEL_ENCODER, fields[i]), label=self.log.serials[d])
                if i == 0:
                    a.legend(ncol=2)

        for i, a in enumerate(ax):
            a.set_ylabel(fields[i])
            a.set_title(titles[i])
            a.grid(True)

    def groundVehicle(self, fig=None):
        if fig is None:
            fig = plt.figure()

        fig.suptitle('Ground Vehicle - ' + os.path.basename(os.path.normpath(self.log.directory)))
        ax = fig.subplots(8, 2, sharex=True)

        ax[0,0].set_title('Status')
        ax[0,1].set_title('Mode')
        ax[1,0].set_title('e_b2w')
        ax[1,1].set_title('e_b2w_sigma')
        ax[4,0].set_title('t_b2w')
        ax[4,1].set_title('t_b2w_sigma')
        ax[7,0].set_title('Radius')
        ax[7,1].set_title('Track Width')

        for d in self.active_devs:
            time = getTimeFromTowMs(self.getData(d, DID_GROUND_VEHICLE, 'timeOfWeekMs'))
            wheelConfig = self.getData(d, DID_GROUND_VEHICLE, 'wheelConfig')
            ax[0,0].plot(time, self.getData(d, DID_GROUND_VEHICLE, 'status'))
            ax[0,1].plot(time, self.getData(d, DID_GROUND_VEHICLE, 'mode'))

            ax[1,0].plot(time, wheelConfig['transform']['e_b2w'][:, 0], label=self.log.serials[d])
            ax[2,0].plot(time, wheelConfig['transform']['e_b2w'][:, 1])
            ax[3,0].plot(time, wheelConfig['transform']['e_b2w'][:, 2])
            ax[1,1].plot(time, wheelConfig['transform']['e_b2w_sigma'][:, 0], label=self.log.serials[d])
            ax[2,1].plot(time, wheelConfig['transform']['e_b2w_sigma'][:, 1])
            ax[3,1].plot(time, wheelConfig['transform']['e_b2w_sigma'][:, 2])

            ax[4,0].plot(time, wheelConfig['transform']['t_b2w'][:, 0], label=self.log.serials[d])
            ax[5,0].plot(time, wheelConfig['transform']['t_b2w'][:, 1])
            ax[6,0].plot(time, wheelConfig['transform']['t_b2w'][:, 2])
            ax[4,1].plot(time, wheelConfig['transform']['t_b2w_sigma'][:, 0], label=self.log.serials[d])
            ax[5,1].plot(time, wheelConfig['transform']['t_b2w_sigma'][:, 1])
            ax[6,1].plot(time, wheelConfig['transform']['t_b2w_sigma'][:, 2])

            ax[7,0].plot(time, wheelConfig['radius'])
            ax[7,1].plot(time, wheelConfig['track_width'])

        # Show serial numbers
        ax[0,0].legend(ncol=2)

        for a in ax:
            for b in a:
                b.grid(True)

    def wheelControllerTime(self, fig=None):
        if fig is None:
            fig = plt.figure()

        fig.suptitle('Wheel Controller Time - ' + os.path.basename(os.path.normpath(self.log.directory)))
        ax = fig.subplots(4, 1, sharex=True)

        ax[0].set_title('effOut - Left')
        ax[1].set_title('Wheel Velocity - Left')
        ax[2].set_title('effOut - Right')
        ax[3].set_title('Wheel Velocity - Right')

        for d in self.active_devs:
            time = self.getData(d, DID_EVB_LUNA_VELOCITY_CONTROL, 'timeMs') * 0.001
            effAct_l = self.getData(d, DID_EVB_LUNA_VELOCITY_CONTROL, 'effDuty_l')
            effAct_r = self.getData(d, DID_EVB_LUNA_VELOCITY_CONTROL, 'effDuty_r')
            vel_l = self.getData(d, DID_EVB_LUNA_VELOCITY_CONTROL, 'vel_l')
            vel_r = self.getData(d, DID_EVB_LUNA_VELOCITY_CONTROL, 'vel_r')

            ax[0].plot(time, effAct_l)
            ax[1].plot(time, vel_l)
            ax[2].plot(time, effAct_r)
            ax[3].plot(time, vel_r)

        for a in ax:
            a.grid(True)

    def wheelControllerVel(self, fig=None):
        if fig is None:
            fig = plt.figure()

        fig.suptitle('Wheel Controller Velocity - ' + os.path.basename(os.path.normpath(self.log.directory)))
        ax = fig.subplots(2, 1, sharex=True)

        ax[0].set_title('Velocity vs effOut - Left')
        ax[1].set_title('Velocity vs effOut - Right')

        for a in ax:
            a.set_xlabel('Velocity (rad/s)')
            a.set_ylabel('effOut')

        for d in self.active_devs:
            time = self.getData(d, DID_EVB_LUNA_VELOCITY_CONTROL, 'timeMs') * 0.001
            if np.any(time):
                eff_l = self.getData(d, DID_EVB_LUNA_VELOCITY_CONTROL, 'effDuty_l')
                eff_r = self.getData(d, DID_EVB_LUNA_VELOCITY_CONTROL, 'effDuty_r')
                vel_l = self.getData(d, DID_EVB_LUNA_VELOCITY_CONTROL, 'vel_l')
                vel_r = self.getData(d, DID_EVB_LUNA_VELOCITY_CONTROL, 'vel_r')

                actuatorTrim_l = 0.545               # (duty) Angle that sets left actuator zero velocity (center) position relative to home point  
                actuatorTrim_r = 0.625               # (duty) Angle that sets right actuator zero velocity (center) position relative to home point

                eff_l -= actuatorTrim_l
                eff_r -= actuatorTrim_r

                # deadbandDuty_l = 0.045
                deadbandDuty_r = 0.0335
                deadbandDuty_l = deadbandDuty_r # match left and right
                deadbandVel = 0.05

                c_l = self.solveInversePlant(ax[0], vel_l, eff_l, deadbandVel, deadbandDuty_l, "left ")
                c_r = self.solveInversePlant(ax[1], vel_r, eff_r, deadbandVel, deadbandDuty_r, "right")

                # string = []
                # for element in c_l:
                #     string.append("{:.9f}".format(element))
                # string = "[" + ", ".join(string) + "]"
                # # print(label, "inverse plant:" , string, " deadband:", deadbandDuty)

                print("\nADD TO MODEL FILE:")
                print("  InversePlant_l: [%.9f, %.9f, %.9f, %.9f, %.9f]" % (c_l[4], c_l[3], c_l[2], c_l[1], c_l[0]))
                print("  InversePlant_r: [%.9f, %.9f, %.9f, %.9f, %.9f]" % (c_r[4], c_r[3], c_r[2], c_r[1], c_r[0]))
                print("  actuatorDeadbandDuty_l: %.9f # (duty) Left  control effort angle from zero (trim) before wheels start spinning." % (deadbandDuty_l))
                print("  actuatorDeadbandDuty_r: %.9f # (duty) Right control effort angle from zero (trim) before wheels start spinning." % (deadbandDuty_r))
                print("  actuatorDeadbandVel: %.9f    # (rad/s) Commanded velocity" % (deadbandVel))

        for a in ax:
            a.grid(True)

    def solveInversePlant(self, ax, vel, eff, deadbandVel, deadbandDuty, label):
            effMod = eff.copy()

            for i in range(len(effMod)):
                if effMod[i] >= 0:
                    effMod[i] = effMod[i] - deadbandDuty
                else:
                    effMod[i] = effMod[i] + deadbandDuty

            c = np.polyfit(vel, effMod, 4)

            velLin = np.linspace(np.min(vel)-1, np.max(vel)+1, 1000)
            effEst = np.polyval(c, velLin)

            for i in range(len(velLin)):
                if velLin[i] > deadbandVel:
                    effEst[i] += deadbandDuty
                elif velLin[i] < -deadbandVel:
                    effEst[i] -= deadbandDuty
                else:
                    effEst[i] += deadbandDuty/deadbandVel * velLin[i]

            ax.plot(vel, eff, '.')
            # ax.plot(vel, effMod, 'g')
            ax.plot(velLin, effEst, 'r')

            return c
    def sensorCompGyrTemp(self, fig=None):
        if fig is None:
            fig = plt.figure()
        self.sensorCompGen(fig, 'pqr', useTemp=True)

    def sensorCompAccTemp(self, fig=None):
        if fig is None:
            fig = plt.figure()
        self.sensorCompGen(fig, 'acc', useTemp=True)

    def sensorCompMagTemp(self, fig=None):
        if fig is None:
            fig = plt.figure()
        self.sensorCompGen(fig, 'mag', useTemp=True)

    def sensorCompGyr(self, fig=None):
        if fig is None:
            fig = plt.figure()
        self.sensorCompGen(fig, 'pqr')

    def sensorCompAcc(self, fig=None):
        if fig is None:
            fig = plt.figure()
        self.sensorCompGen(fig, 'acc')

    def sensorCompMag(self, fig=None):
        if fig is None:
            fig = plt.figure()
        self.sensorCompGen(fig, 'mag')

    def sensorCompGen(self, fig, name, useTemp=False):
        fig.suptitle('Sensor Comp ' + name + ' - ' + os.path.basename(os.path.normpath(self.log.directory)))
        ax = fig.subplots(4, 3, sharex=True)

        useSampleNumber = 1
        noData = True

        for i in range(3):
            ax[0, i].set_title('X %s %d' % (name, i))
            ax[1, i].set_title('Y %s %d' % (name, i))
            ax[2, i].set_title('Z %s %d' % (name, i))
            ax[3, i].set_title('Magnitude %s %d' % (name, i))
            for d in range(3):
                if useTemp:
                    ax[d,i].set_xlabel("Temperature (C)")
                else:
                    if useSampleNumber:
                        ax[d,i].set_xlabel("Sample")
                    else:
                        ax[d,i].set_xlabel("Time (s)")
                if name=='pqr':
                    ax[d,i].set_ylabel("Gyro (deg/s)")
                elif name=='acc':
                    ax[d,i].set_ylabel("Accel (m/s^2)")
                elif name=='mag':
                    ax[d,i].set_ylabel("Mag")

        for d in self.active_devs:
            time = 0.001 * self.getData(d, DID_SCOMP, 'timeMs')
            if np.any(time):
                noData = False
                imu = self.getData(d, DID_SCOMP, name)
                status = self.getData(d, DID_SCOMP, 'status')

                if name=='mag':
                    refVal = self.getData(d, DID_SCOMP, 'referenceMag')
                else:
                    refImu = self.getData(d, DID_SCOMP, 'referenceImu')
                    refImu = refImu
                    refVal = refImu[name]

                print("Serial#: SN" + str(self.log.serials[d]) + " " + name + ": ")
                for i in range(3):
                    temp = imu[:,i]['lpfTemp']
                    sensor = imu[:,i]['lpfLsb']

                    if name=='pqr':
                        scalar = RAD2DEG
                        yspan = 2.5 # deg/s
                    else:   # 'acc'
                        scalar = 1.0
                        yspan = 0.5 # m/s^2 

                    if useTemp:
                        x = temp
                    else:
                        if useSampleNumber:
                            x = np.arange(len(sensor[:,0]))
                        else:
                            x = time

                    # ax[0,i].plot(x, sensor[:,0], label=self.log.serials[d] if i==0 else None )
                    ax[0,i].plot(x, sensor[:,0]*scalar, label=self.log.serials[d] )
                    ax[1,i].plot(x, sensor[:,1]*scalar)
                    ax[2,i].plot(x, sensor[:,2]*scalar)
                    if name=='acc':
                        ax[3,i].plot(x, np.linalg.norm(sensor, axis=1)*scalar)

                    # Print mean and last value
                    xstr = "IMU" + str(i) + ", Mean: ["
                    for j in range(3):
                        xstr += format(np.mean(sensor[:,j])*scalar, '10.6f')
                        if j < 2:
                            xstr += ","
                    xstr += "],  "
                    xstr += "Last: ["
                    for j in range(3):
                        xstr += format(sensor[-1,j]*scalar, '10.6f')
                        if j < 2:
                            xstr += ","
                    xstr += "]"
                    print(xstr)

                    if 1:
                        # Show sensor valid status bit
                        if name=='acc':
                            valid = 0.0 + ((status & 0x00000200) != 0) * scalar * 0.25
                        else:
                            valid = 0.0 + ((status & 0x00000100) != 0) * scalar * 0.25
                        ax[0,i].plot(x, valid * np.max(sensor[:,0]), color='y', label="Sensor Valid")
                        ax[1,i].plot(x, valid * np.max(sensor[:,1]), color='y')
                        ax[2,i].plot(x, valid * np.max(sensor[:,2]), color='y')

                    if 1:
                        for j in range(3):
                            ax[j, i].plot(x, refVal[:, j] * scalar, color='red', label="reference")

        if noData:
            return

        # Show serial numbers
        ax[0,0].legend(ncol=2)

        for a in ax:
            for b in a:
                b.grid(True)
                lim = b.get_ylim()
                mid = 0.5 * (lim[0] + lim[1])
                span = lim[1] - lim[0]
                span = max(span, yspan)
                b.set_ylim([mid-span/2, mid+span/2])

    def linearityAcc(self, fig=None):
        fig.suptitle('Accelerometer Linearity - ' + os.path.basename(os.path.normpath(self.log.directory)))
        ax = fig.subplots(3, 3)

        for i in range(3):
            ax[0, i].set_title('Accel%d X' % (i))
            ax[1, i].set_title('Accel%d Y' % (i))
            ax[2, i].set_title('Accel%d Z ' % (i))
            for d in range(3):
                ax[d,i].set_xlabel("ref m/s^2")
                ax[d,i].set_ylabel("residual m/s^2")

        for d in self.active_devs:
            sampleCount = self.getData(d, DID_SCOMP, 'sampleCount')
            # imu = self.getData(d, DID_SCOMP, 'pqr')
            imu = self.getData(d, DID_SCOMP, 'acc')
            reference = self.getData(d, DID_SCOMP, 'referenceImu')

            for i in range(3): # range through axes
                data0 = imu[:,0]['lpfLsb'][:,i]
                data1 = imu[:,1]['lpfLsb'][:,i]
                data2 = imu[:,2]['lpfLsb'][:,i]

                refdata = reference['acc'][:,i]

                # refdata0 = reference['pqr'][sampleCount > 10000,i]
                # refdata1 = reference['pqr'][sampleCount > 10000,i]
                # refdata2 = reference['pqr'][sampleCount > 10000,i]

                # ax[i,0].plot(refdata0, data0)
                # ax[i,1].plot(refdata0, data1)
                # ax[i,2].plot(refdata0, data2)

                residual0 = [a - b for a, b in zip(refdata, data0)]
                residual1 = [a - b for a, b in zip(refdata, data1)]
                residual2 = [a - b for a, b in zip(refdata, data2)]

                # ax[i,0].plot(range(0,np.size(residual0)), residual0)
                # ax[i,1].plot(range(0,np.size(residual1)), residual1)
                # ax[i,2].plot(range(0,np.size(residual2)), residual2)

                if i == 0:
                    ax[i,0].plot(refdata, residual0, label=self.log.serials[d])
                else:
                    ax[i,0].plot(refdata, residual0)
                ax[i,1].plot(refdata, residual1)
                ax[i,2].plot(refdata, residual2)


        # Show serial numbers
        ax[0,0].legend(ncol=2)

        yspan = 0.5 # m/s^2 

        for a in ax:
            for b in a:
                b.grid(True)
                # lim = b.get_ylim()
                # mid = 0.5 * (lim[0] + lim[1])
                # span = lim[1] - lim[0]
                # span = max(span, yspan)
                # b.set_ylim([mid-span/2, mid+span/2])

    def linearityGyr(self, fig=None):
        fig.suptitle('Gyro Linearity - ' + os.path.basename(os.path.normpath(self.log.directory)))
        ax = fig.subplots(3, 3)

        for i in range(3):
            ax[0, i].set_title('Gyro%d P' % (i))
            ax[1, i].set_title('Gyro%d Q' % (i))
            ax[2, i].set_title('Gyro%d R ' % (i))
            for d in range(3):
                ax[d,i].set_xlabel("ref rad/s")
                ax[d,i].set_ylabel("residual rad/s")

        for d in self.active_devs:
            sampleCount = self.getData(d, DID_SCOMP, 'sampleCount')
            imu = self.getData(d, DID_SCOMP, 'pqr')
            # imu = self.getData(d, DID_SCOMP, 'acc')
            reference = self.getData(d, DID_SCOMP, 'referenceImu')

            for i in range(3): # range through axes
                data0 = imu[:,0]['lpfLsb'][:,i]
                data1 = imu[:,1]['lpfLsb'][:,i]
                data2 = imu[:,2]['lpfLsb'][:,i]

                refdata = reference['pqr'][:,i]

                # refdata0 = reference['pqr'][sampleCount > 10000,i]
                # refdata1 = reference['pqr'][sampleCount > 10000,i]
                # refdata2 = reference['pqr'][sampleCount > 10000,i]

                # ax[i,0].plot(refdata0, data0)
                # ax[i,1].plot(refdata0, data1)
                # ax[i,2].plot(refdata0, data2)

                residual0 = [a - b for a, b in zip(refdata, data0)]
                residual1 = [a - b for a, b in zip(refdata, data1)]
                residual2 = [a - b for a, b in zip(refdata, data2)]

                # ax[i,0].plot(range(0,np.size(residual0)), residual0)
                # ax[i,1].plot(range(0,np.size(residual1)), residual1)
                # ax[i,2].plot(range(0,np.size(residual2)), residual2)

                if i == 0:
                    ax[i,0].plot(refdata, residual0, label=self.log.serials[d])
                else:
                    ax[i,0].plot(refdata, residual0)
                ax[i,1].plot(refdata, residual1)
                ax[i,2].plot(refdata, residual2)


        # Show serial numbers
        ax[0,0].legend(ncol=2)

        yspan = 0.5 # m/s^2 

        for a in ax:
            for b in a:
                b.grid(True)
                # lim = b.get_ylim()
                # mid = 0.5 * (lim[0] + lim[1])
                # span = lim[1] - lim[0]
                # span = max(span, yspan)
                # b.set_ylim([mid-span/2, mid+span/2])

    def showFigs(self):
        if self.show:
            plt.show()


if __name__ == '__main__':
    np.set_printoptions(linewidth=200)
    home = expanduser("~")
    file = open(home + "/Documents/Inertial_Sense/config.yaml", 'r')
    config = yaml.load(file)
    directory = config["directory"]
    directory = "/home/superjax/Code/IS-src/cpp/SDK/cltool/build/IS_logs"
    directory = r"C:\Users\quaternion\Downloads\20181218_Compass_Drive\20181218 Compass Drive\20181218_101023"
    serials = config['serials']

    log2 = Log()
    log2.load(directory, serials)

    plotter = logPlot(True, True, 'svg', log2)
    plotter.setDownSample(5)
    # plotter.deltatime()
    # plotter.debugfArr()
    # plotter.debugiArr()
    # plotter.debuglfArr()
    # plotter.gpsStats()
    # plotter.rtkStats()
    # plotter.insStatus()
    # plotter.hdwStatus()
    # plotter.temp()
    # plotter.att()
    # plotter.lla()
    # plotter.uvw()
    # plotter.ned()
    # plotter.nedMap()
    # plotter.magDec()
    plotter.rtkDebug()
    # plotter.wheelEncoder()
    # plotter.groundVehicle()

    plotter.showFigs()

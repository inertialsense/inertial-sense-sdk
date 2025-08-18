from ctypes import sizeof
import math, allantools, sys, yaml, os
from typing import List, Any, Union

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
from os.path import expanduser
from inertialsense_math.pose import *
from datetime import date, datetime
import pandas as pd

from ui import InteractiveLegend

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
SHOW_GPS_W_INS = 1
SHOW_HEADING_ARROW = 0

file_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(file_path + '/..'))
sys.path.append(os.path.normpath(file_path + '/../math/src'))

from logReader import Log
from pylib.ISToolsGNSS import refLla, getTimeFromGpsTowMs, getTimeFromGpsTow, setGpsWeek, getTimeFromGTime, setShowUtcTime
from pylib.data_sets import *
from inertialsense_math.pose import quat2euler, lla2ned, rotmat_ecef2ned, quatRot, quatConjRot, quat_ecef2ned
import datetime

class logPlot: 
    def __init__(self, show=False, save=False, format='svg', log=None):
        self.show = show
        self.save = save
        self.format = format
        self.d = 1
        self.residual = False
        self.timestamp = False
        self.xAxisSample = False
        self.utcTime = False
        self.enableLegends = False  # Enable interactive legends
        if self.enableLegends:
            self.legends = InteractiveLegend()
        if log:
            self.setLog(log)
        else:
            self.log = None
            
    def setup_and_wire_legend(self):
        if self.enableLegends:
            self.legends.setup_and_wire()
        
    def legends_add(self, l):
        if self.enableLegends:
            self.legends.add(l)

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

    def enableTimestamp(self, enable):
        self.timestamp = enable

    def enableXAxisSample(self, enable):
        self.xAxisSample = enable

    def enableUtcTime(self, enable):
        self.utcTime = enable
        setShowUtcTime(enable)        

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

    def saveFigJoinAxes(self, ax, axs, fig, name, sizeInches=[]):
        self.saveFig(fig, name, sizeInches)
        self.joinFigXAxes(ax,axs)
        return ax

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

    def getData(self, dev, DID, field, removeLeadingZeros=0):
        try:
            data = self.log.data[dev, DID][field][::self.d]
            if removeLeadingZeros:
                # Copy the first nonzero data entry to leading zeros 
                # (e.g. first position initialized from GNSS to the initial default position in AHRS)
                startIdx = np.nonzero(data)[0][0]
                data[0:startIdx] = data[startIdx]
            return data
        except:
            return []

    def getGpsTowOffset(self, dev):
        towOffset = self.getData(dev, DID_GPS1_POS, 'towOffset')
        if len(towOffset) == 0:
            towOffset = self.getData(dev, DID_GPS2_POS, 'towOffset')
        return towOffset

    def setPlotYSpanMin(self, ax, limit):
        ylim = ax.get_ylim()
        yspn = np.max( [ylim[1] - ylim[0], limit] )
        ylim = (np.mean(ylim)-yspn/2, np.mean(ylim)+yspn/2)
        ax.set_ylim(ylim)

    def isEmpty(self, data):
        if isinstance(data, np.ndarray):
            return data.size == 0
        elif isinstance(data, list):
            return len(data) == 0
        else:
            return False  # Or raise an error if unexpected types are unacceptable

    def joinFigXAxes(self, ax1, ax2):
        if ax2 is None:
            return

        # Ensure ax1 and ax2 are at least 1-dimensional numpy arrays
        ax1 = np.atleast_1d(ax1)
        ax2 = np.atleast_1d(ax2)

        # Flatten the arrays to handle all axes in one loop
        ax1_flat = ax1.flatten()
        ax2_flat = ax2.flatten()

        # Access the shared x-axis GrouperView
        shared_x_axes = ax1_flat[0].get_shared_x_axes()

        # Explicitly share the x-axis across all axes
        for a in ax1_flat:
            for b in ax2_flat:
                if b not in shared_x_axes.get_siblings(a):
                    a.sharex(b)

    def posNED(self, fig=None, axs=None):
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
                    refLla = self.getData(d, DID_INS_2, 'lla', True)
                    if len(refLla) == 0:
                        # No position data: AHRS?
                        continue
                    refTime = getTimeFromGpsTow(self.getData(d, DID_INS_2, 'timeOfWeek', True), True)
                    refLla = refLla[0]
                    continue
            # If 'Ref INS' is not available, use GPS as reference
            if refTime is None or refLla is None:
                for d in self.active_devs:
                    lla = self.getData(d, DID_GPS1_POS, 'lla', True)
                    if len(refLla) == 0:
                        # No position data: AHRS?
                        continue
                    refLla = lla[0]
                    refTime = getTimeFromGpsTowMs(self.getData(d, DID_GPS1_POS, 'timeOfWeekMs', True))
                    refNed = lla2ned(refLla, lla)
                    continue

        for d in self.active_devs:
            lla = self.getData(d, DID_INS_2, 'lla', True)
            if len(lla) == 0:
                # No position data: AHRS?
                continue
            if refLla is None:
                refLla = lla[0]
            ned = lla2ned(refLla, lla)
            tow = self.getData(d, DID_INS_2, 'timeOfWeek', True)
            time = getTimeFromGpsTow(tow, True)
            ax[0,0].plot(time, ned[:,0], label=self.log.serials[d])
            ax[1,0].plot(time, ned[:,1])
            ax[2,0].plot(time, ned[:,2])

            if(np.shape(self.active_devs)[0]==1 or SHOW_GPS_W_INS):
                timeGPS = getTimeFromGpsTowMs(self.getData(d, DID_GPS1_POS, 'timeOfWeekMs', True))
                if not self.isEmpty(timeGPS):
                    nedGps1 = lla2ned(refLla, self.getData(d, DID_GPS1_POS, 'lla', True))
                    ax[0,0].plot(timeGPS, nedGps1[:, 0], label=("%s GPS1" % (self.log.serials[d])))
                    ax[1,0].plot(timeGPS, nedGps1[:, 1])
                    ax[2,0].plot(timeGPS, nedGps1[:, 2])

            if(np.shape(self.active_devs)[0]==1 or (SHOW_GPS_W_INS and SHOW_GPS2)):
                timeGPS = getTimeFromGpsTowMs(self.getData(d, DID_GPS2_POS, 'timeOfWeekMs', True))
                if not self.isEmpty(timeGPS):
                    nedGps2 = lla2ned(refLla, self.getData(d, DID_GPS2_POS, 'lla', True))
                    ax[0,0].plot(timeGPS, nedGps2[:, 0], label=("%s GPS2" % (self.log.serials[d])))
                    ax[1,0].plot(timeGPS, nedGps2[:, 1])
                    ax[2,0].plot(timeGPS, nedGps2[:, 2])

            if self.residual and not (refTime is None) and self.log.serials[d] != 'Ref INS': 
                intNed = np.empty_like(refNed)
                for i in range(3):
                    intNed[:,i] = np.interp(refTime, time, ned[:,i], right=np.nan, left=np.nan)
                resNed = intNed - refNed
                ax[0,1].plot(refTime, resNed[:,0], label=self.log.serials[d])
                ax[1,1].plot(refTime, resNed[:,1])
                ax[2,1].plot(refTime, resNed[:,2])

        self.legends_add(ax[0,0].legend(ncol=2))
        if self.residual: 
            self.legends_add(ax[0,1].legend(ncol=2))
            for i in range(3):
                self.setPlotYSpanMin(ax[i,1], 1.0)
        for a in ax:
            for b in a:
                b.grid(True)

        self.setup_and_wire_legend()
        return self.saveFigJoinAxes(ax, axs, fig, 'posNED')
        self.joinFigXAxes(ax,axs)
        return ax

    def nedAnnotateTimestamp(self, ax, time, east, north, textOffset=(0.0, 0.0)):
        ax.annotate('%.1f' % time, xy=(east, north), xycoords='data', xytext=textOffset, textcoords='offset points')

    def drawNEDMapArrow(self, ax, time, ned, heading):
        # arrowLen = 0.2
        # arrowWid = arrowLen/2
        # arrows = np.array([arrowLen * 0.7 * np.cos(heading), arrowLen * 0.7 * np.sin(heading)]).T

        markersize = 10
        downsample = 6
        len = np.shape(heading)[0]
        for i in range(1, len, downsample):
            # ax.arrow(ned[i,1], ned[i,0], arrows[i,1], arrows[i,0], head_width=arrowWid, head_length=arrowLen, length_includes_head=True, fc='k', ec='k')
            east = ned[i,1]
            north = ned[i,0]
            ax.plot(east, north, marker=(3, 0, -heading[i]*(180.0/np.pi)), color='g', markersize=markersize, linestyle='None')
            ax.plot(east, north, marker=(2, 0, -heading[i]*(180.0/np.pi)), color='k', markersize=markersize, linestyle='None')

            if self.timestamp:
                self.nedAnnotateTimestamp(ax, time[i], east, north, textOffset=(3.5, 3.5))

    def posNEDMap(self, fig=None, axs=None):
        if fig is None:
            fig = plt.figure()
        
        ax = fig.subplots(1,1)
        ax.set_xlabel('East (m)')
        ax.set_ylabel('North (m)')
        fig.suptitle('NED Map - ' + os.path.basename(os.path.normpath(self.log.directory)))
        refLla = None
        for d in self.active_devs:
            lla = self.getData(d, DID_INS_2, 'lla', True)
            if len(lla) == 0:
                continue
            if refLla is None:
                refLla = lla[0]
            time = getTimeFromGpsTow(self.getData(d, DID_INS_2, 'timeOfWeek', True), True)
            ned = lla2ned(refLla, self.getData(d, DID_INS_2, 'lla', True))
            euler = quat2euler(self.getData(d, DID_INS_2, 'qn2b', True))
            ax.plot(ned[:,1], ned[:,0], label=self.log.serials[d])

            if(np.shape(self.active_devs)[0]==1 or SHOW_GPS_W_INS):
                if (np.shape(self.active_devs)[0]==1):
                    self.drawNEDMapArrow(ax, time, ned, euler[:, 2])

                lla1 = self.getData(d, DID_GPS1_POS, 'lla', True)
                if len(lla1):
                    nedGps1 = lla2ned(refLla, lla1)
                    ax.plot(nedGps1[:, 1], nedGps1[:, 0], label=("%s GPS1" % (self.log.serials[d])))

                if SHOW_GPS2 or len(lla1) == 0:
                    lla2 = self.getData(d, DID_GPS2_POS, 'lla', True)
                    if len(lla2):
                        nedGps2 = lla2ned(refLla, lla2)
                        ax.plot(nedGps2[:, 1], nedGps2[:, 0], label=("%s GPS2" % (self.log.serials[d])))

        ax.set_aspect('equal', 'datalim')
        self.legends_add(ax.legend(ncol=2))
        ax.grid(True)

        self.setup_and_wire_legend()
        return self.saveFigJoinAxes(ax, None, fig, 'posNEDMap')

    def gpsPosNEDMap(self, fig=None, axs=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(1,1)
        ax.set_xlabel('East (m)')
        ax.set_ylabel('North (m)')
        fig.suptitle('GPS NED Map - ' + os.path.basename(os.path.normpath(self.log.directory)))
        refLla = None
        for d in self.active_devs:
            lla1 = self.getData(d, DID_GPS1_POS, 'lla')
            if len(lla1) == 0:
                continue
            if refLla is None:
                refLla = lla1[-1]

            time = getTimeFromGpsTowMs(self.getData(d, DID_GPS1_POS, 'timeOfWeekMs'))
            lla1 = self.getData(d, DID_GPS1_POS, 'lla')
            if len(lla1):
                nedGps1 = lla2ned(refLla, lla1)
                ax.plot(nedGps1[:, 1], nedGps1[:, 0], label=("%s" % (self.log.serials[d])))

            if self.timestamp:
                lasttime = 0
                for time, east, north in zip(time, nedGps1[:, 1], nedGps1[:, 0]):
                    if time - lasttime > 5:
                        lasttime = time
                        # ax.annotate('%.1f' % time, xy=(east, north), textcoords='data')
                        self.nedAnnotateTimestamp(ax, time, east, north)


            if SHOW_GPS2:
                nedGps2 = lla2ned(refLla, self.getData(d, DID_GPS2_POS, 'lla'))
                ax.plot(nedGps2[:, 1], nedGps2[:, 0], label=("%s GPS2" % (self.log.serials[d])))

        ax.set_aspect('equal', 'datalim')
        self.legends_add(ax.legend(ncol=2))
        ax.grid(True)

        self.setup_and_wire_legend()
        return self.saveFigJoinAxes(ax, None, fig, 'gpsPosNEDMap')

    def posLLA(self, fig=None, axs=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(3,1, sharex=True)
        self.configureSubplot(ax[0], 'Latitude', 'deg')
        self.configureSubplot(ax[1], 'Longitude', 'deg')
        self.configureSubplot(ax[2], 'Altitude', 'm')
        fig.suptitle('INS LLA - ' + os.path.basename(os.path.normpath(self.log.directory)))
        for d in self.active_devs:
            time = getTimeFromGpsTow(self.getData(d, DID_INS_2, 'timeOfWeek'), True)
            lla = self.getData(d, DID_INS_2, 'lla', True)
            if len(lla) != len(time):
                continue
            ax[0].plot(time, lla[:,0], label=self.log.serials[d])
            ax[1].plot(time, lla[:,1])
            ax[2].plot(time, lla[:,2])

            if(np.shape(self.active_devs)[0]==1):
                towOffset = 0
                timeGPS1 = getTimeFromGpsTowMs(self.getData(d, DID_GPS1_POS, 'timeOfWeekMs'))
                if len(timeGPS1):
                    towOffset = self.getGpsTowOffset(d)[-1]
                    ax[0].plot(timeGPS1, self.getData(d, DID_GPS1_POS, 'lla')[:, 0], label='GPS1')
                    ax[1].plot(timeGPS1, self.getData(d, DID_GPS1_POS, 'lla')[:, 1])
                    ax[2].plot(timeGPS1, self.getData(d, DID_GPS1_POS, 'lla')[:, 2], label='GPS1')

                timeGPS2 = getTimeFromGpsTowMs(self.getData(d, DID_GPS2_POS, 'timeOfWeekMs'))
                if len(timeGPS2):
                    if towOffset == 0:
                        towOffset = self.getData(d, DID_GPS2_POS, 'towOffset')[-1]
                    ax[0].plot(timeGPS2, self.getData(d, DID_GPS2_POS, 'lla')[:, 0], label='GPS2')
                    ax[1].plot(timeGPS2, self.getData(d, DID_GPS2_POS, 'lla')[:, 1])
                    ax[2].plot(timeGPS2, self.getData(d, DID_GPS2_POS, 'lla')[:, 2], label='GPS2')

                timeBaro = getTimeFromGpsTow(self.getData(d, DID_BAROMETER, 'time')+ towOffset)
                ax[2].plot(timeBaro, self.getData(d, DID_BAROMETER, 'mslBar'), label='Baro')

        self.legends_add(ax[0].legend(ncol=2))
        self.legends_add(ax[2].legend(ncol=2))
        for a in ax:
            a.grid(True)

        self.setup_and_wire_legend()
        return self.saveFigJoinAxes(ax, axs, fig, 'insLLA')

    def gpsLLA(self, fig=None, axs=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(3,1, sharex=True)
        self.configureSubplot(ax[0], 'Latitude', 'deg')
        self.configureSubplot(ax[1], 'Longitude', 'deg')
        self.configureSubplot(ax[2], 'Altitude', 'm')
        fig.suptitle('GPS LLA - ' + os.path.basename(os.path.normpath(self.log.directory)))
        for d in self.active_devs:
            time1 = getTimeFromGpsTowMs(self.getData(d, DID_GPS1_POS, 'timeOfWeekMs'))
            if len(time1):
                lla1 = self.getData(d, DID_GPS1_POS, 'lla')
                ax[0].plot(time1, lla1[:,0], label=('%s GPS1' % self.log.serials[d]))
                ax[1].plot(time1, lla1[:,1])
                ax[2].plot(time1, lla1[:,2])

            time2 = getTimeFromGpsTowMs(self.getData(d, DID_GPS2_POS, 'timeOfWeekMs'))
            if (len(time2) and (SHOW_GPS2 or len(time1) == 0)):
                lla2 = self.getData(d, DID_GPS2_POS, 'lla')
                ax[0].plot(time2, lla2[:,0], label=('%s GPS2' % self.log.serials[d]))
                ax[1].plot(time2, lla2[:,1])
                ax[2].plot(time2, lla2[:,2])

        self.legends_add(ax[0].legend(ncol=2))
        for a in ax:
            a.grid(True)

        self.setup_and_wire_legend()
        return self.saveFigJoinAxes(ax, axs, fig, 'gpsLLA')

    def getGpsPosNED(self, device, did, refLla):
        gpsTime = getTimeFromGpsTowMs(self.getData(device, did, 'timeOfWeekMs'))
        gpsNed = lla2ned(refLla, self.getData(device, did, 'lla'))
        return [gpsTime, gpsNed]

    def gpsPosNED(self, fig=None, axs=None):
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
                lla1 = self.getData(d, DID_GPS1_POS, 'lla')
                lla2 = self.getData(d, DID_GPS2_POS, 'lla')
                if len(lla1):
                    refLla = lla1[-1]
                elif len(lla2):
                    refLla = lla2[-1]

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

        self.legends_add(ax[0].legend(ncol=2))
        for a in ax:
            a.grid(True)

        self.setup_and_wire_legend()
        return self.saveFigJoinAxes(ax, axs, fig, 'gpsPosNED')

    def getGpsVelNed(self, device, did, refLla):
        gpsTime = getTimeFromGpsTowMs(self.getData(device, did, 'timeOfWeekMs'))
        if len(gpsTime) == 0:
            return [[], []]
        status = self.getData(device, did, 'status')[0]
        gpsVelNed = None
        if (status & 0x00008000):
            gpsVelNed = self.getData(device, did, 'vel')    # NED velocity
        else:
            gpsVelEcef = self.getData(device, did, 'vel')   # ECEF velocity
            if len(gpsVelEcef) > 0:
                qe2n = quat_ecef2ned(refLla[0:2]*np.pi/180.0)
                gpsVelNed = quatConjRot(qe2n, gpsVelEcef)
                # gpsVelNed = np.copy(gpsVelEcef)   # Override to ECEF
        return [gpsTime, gpsVelNed]

    def gpsVelNED(self, fig=None, axs=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(4, (2 if self.residual else 1), sharex=True, squeeze=False)
        self.configureSubplot(ax[0,0], 'GPS Velocity North', 'm/s')
        self.configureSubplot(ax[1,0], 'GPS Velocity East', 'm/s')
        self.configureSubplot(ax[2,0], 'GPS Velocity Down', 'm/s')
        self.configureSubplot(ax[3,0], 'GPS Speed', 'm/s')
        fig.suptitle('GPS Velocity NED - ' + os.path.basename(os.path.normpath(self.log.directory)))
        refLla = None
        refTime = None
        refVelNed = None
        sumDelta = None
        sumCount = 1

        if self.residual:
            self.configureSubplot(ax[0,1], 'Vel North Residual (GPS - Mean)', 'm/s')
            self.configureSubplot(ax[1,1], 'Vel East Residual (GPS - Mean)',  'm/s')
            self.configureSubplot(ax[2,1], 'Vel Down Residual (GPS - Mean)',  'm/s')
            self.configureSubplot(ax[3,1], 'Speed Residual (GPS - Mean)',  'm/s')
            # Use 'Ref INS' if available
            for d in self.active_devs:
               if self.log.serials[d] == 'Ref INS':
                    refTime = getTimeFromGpsTow(self.getData(d, DID_INS_2, 'timeOfWeek'))
                    refVelNed = self.getData(d, DID_INS_2, 'lla')[0]
                    continue
            # 'Ref INS' is not available. Compute reference from average GPS.
            if refTime is None:
                for d in self.active_devs:
                    lla1 = self.getData(d, DID_GPS1_POS, 'lla')
                    lla2 = self.getData(d, DID_GPS2_POS, 'lla')
                    if len(lla1):
                        refLla = lla1[-1]
                    elif len(lla2):
                        refLla = lla2[-1]

                for d in self.active_devs:
                    [gps1Time, gps1VelNed] = self.getGpsVelNed(d, DID_GPS1_VEL, refLla)
                    [gps2Time, gps2VelNed] = self.getGpsVelNed(d, DID_GPS2_VEL, refLla)
                    if refTime is None:
                        if len(gps1Time):
                            refTime = gps1Time
                            refVelNed = np.copy(gps1VelNed)
                            sumDelta = np.zeros_like(gps1VelNed)
                        elif len(gps2Time):
                            refTime = gps2Time
                            refVelNed = np.copy(gps2VelNed)
                            sumDelta = np.zeros_like(gps2VelNed)
                    else:
                        intVelNed = np.empty_like(refVelNed)
                        for i in range(3):
                            intVelNed[:,i] = np.interp(refTime, gps1Time, gps1VelNed[:,i])
                        delta = intVelNed - refVelNed
                        sumDelta += delta
                        sumCount += 1
                refVelNed += sumDelta / sumCount
        
        for d in self.active_devs:
            if refLla is None:
                lla1 = self.getData(d, DID_GPS1_POS, 'lla')
                lla2 = self.getData(d, DID_GPS2_POS, 'lla')
                if len(lla1):
                    refLla = lla1[-1]
                elif len(lla2):
                    refLla = lla2[-1]
            [gps1Time, gps1VelNed] = self.getGpsVelNed(d, DID_GPS1_VEL, refLla)
            [gps2Time, gps2VelNed] = self.getGpsVelNed(d, DID_GPS2_VEL, refLla)
            if len(gps1Time):
                gps1VelNorm = np.linalg.norm(gps1VelNed, axis=1)
                ax[0,0].plot(gps1Time, gps1VelNed[:, 0])
                ax[1,0].plot(gps1Time, gps1VelNed[:, 1])
                ax[2,0].plot(gps1Time, gps1VelNed[:, 2])
                ax[3,0].plot(gps1Time, gps1VelNorm, label=("%s GPS1" % (self.log.serials[d])))
            if len(gps2Time) and (SHOW_GPS2 or len(gps1Time) == 0):
                gps2VelNorm = np.linalg.norm(gps2VelNed, axis=1)
                ax[0,0].plot(gps2Time, gps2VelNed[:, 0])
                ax[1,0].plot(gps2Time, gps2VelNed[:, 1])
                ax[2,0].plot(gps2Time, gps2VelNed[:, 2])
                ax[3,0].plot(gps2Time, gps2VelNorm, label=("%s GPS2" % (self.log.serials[d])))

            if self.residual and not (refTime is None) and self.log.serials[d] != 'Ref INS': 
                intVelNed = np.empty_like(refVelNed)
                for i in range(3):
                    intVelNed[:,i] = np.interp(refTime, gps1Time, gps1VelNed[:,i], right=np.nan, left=np.nan)
                resNed = intVelNed - refVelNed
                resVelNorm = np.linalg.norm(resNed, axis=1)
                ax[0,1].plot(refTime, resNed[:,0])
                ax[1,1].plot(refTime, resNed[:,1])
                ax[2,1].plot(refTime, resNed[:,2])
                ax[3,1].plot(refTime, resVelNorm, label=self.log.serials[d])

        self.legends_add(ax[3,0].legend(ncol=2))
        if self.residual:
            self.legends_add(ax[0,1].legend(ncol=2))
            for i in range(3):
                self.setPlotYSpanMin(ax[i,1], 1.0)
        for a in ax:
            for b in a:
                b.grid(True)

        self.setup_and_wire_legend()
        return self.saveFigJoinAxes(ax, axs, fig, 'gpsVelNED')
        
    def getGpsNedVel(self, d):
        velNed = None
        velDid = DID_GPS2_VEL if self.getData(d, DID_GPS1_VEL, 'status').size == 0 else DID_GPS1_VEL
        status = self.getData(d, velDid, 'status')
        if len(status) == 0:
            return []
        status = status[0]
        if (status & 0x00008000):
            velNed = self.getData(d, velDid, 'vel')    # NED velocity
        else:
            velEcef = self.getData(d, velDid, 'vel')   # ECEF velocity
            qe2n = quat_ecef2ned(refLla[0:2]*np.pi/180.0)
            if len(velEcef) > 0:
                velNed = quatConjRot(qe2n, velEcef)
            # velNed = np.copy(velEcef)     # Override to ECEF velocity

            #R = rotmat_ecef2ned(self.getData(d, DID_GPS1_POS, 'lla')[0,0:2]*np.pi/180.0)
            #velNed = R.dot(velEcef.T).T
        return velNed

    def velNED(self, fig=None, axs=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(4, (2 if self.residual else 1), sharex=True, squeeze=False)
        self.configureSubplot(ax[0,0], 'Vel North', 'm/s')
        self.configureSubplot(ax[1,0], 'Vel East',  'm/s')
        self.configureSubplot(ax[2,0], 'Vel Down',  'm/s')
        self.configureSubplot(ax[3,0], 'Speed',  'm/s')
        fig.suptitle('NED Vel - ' + os.path.basename(os.path.normpath(self.log.directory)))
        refLla = None
        refTime = None
        refVelNed = None
        if self.residual:
            self.configureSubplot(ax[0,1], 'Residual Vel North', 'm/s')
            self.configureSubplot(ax[1,1], 'Residual Vel East',  'm/s')
            self.configureSubplot(ax[2,1], 'Residual Vel Down',  'm/s')
            self.configureSubplot(ax[3,1], 'Residual Speed',  'm/s')
            # Use 'Ref INS' if available
            for d in self.active_devs:
               if self.log.serials[d] == 'Ref INS':
                    refTime = getTimeFromGpsTow(self.getData(d, DID_INS_2, 'timeOfWeek'), True)
                    # TODO: does the ref INS store velNED in lla or is it a copy/paste bug?
                    refVelNed = self.getData(d, DID_INS_2, 'lla')[0]
                    continue
            # If 'Ref INS' is not available, use GPS as reference
            if refTime is None:
                for d in self.active_devs:
                    refTime = getTimeFromGpsTowMs(self.getData(d, DID_GPS1_VEL, 'timeOfWeekMs'))
                    lla1 = self.getData(d, DID_GPS1_POS, 'lla')
                    if len(refTime) and len(lla1): 
                        refLla = lla1[-1]
                        refVelNed = self.getGpsNedVel(d)                    
                    continue

        for d in self.active_devs:
            if refLla is None:
                refLla = self.getData(d, DID_INS_2, 'lla')[-1]
            time = getTimeFromGpsTow(self.getData(d, DID_INS_2, 'timeOfWeek'), True)
            uvw = self.getData(d, DID_INS_2, 'uvw')
            qn2b = self.getData(d, DID_INS_2, 'qn2b')
            if len(uvw) == 0 or len(qn2b) == 0:
                continue
            insVelNed = quatRot(qn2b, uvw)
            insVelNorm = np.linalg.norm(insVelNed, axis=1)
            ax[0,0].plot(time, insVelNed[:,0])
            ax[1,0].plot(time, insVelNed[:,1])
            ax[2,0].plot(time, insVelNed[:,2])
            ax[3,0].plot(time, insVelNorm, label=self.log.serials[d])

            if np.shape(self.active_devs)[0] == 1 or SHOW_GPS_W_INS:  # Show GPS if #devs is 1
                timeGPS = getTimeFromGpsTowMs(self.getData(d, DID_GPS1_VEL, 'timeOfWeekMs'))
                if len(timeGPS) == 0:
                    continue
                gpsVelNed = self.getGpsNedVel(d)
                gpsVelNorm = np.linalg.norm(gpsVelNed, axis=1)
                ax[0,0].plot(timeGPS, gpsVelNed[:, 0])
                ax[1,0].plot(timeGPS, gpsVelNed[:, 1])
                ax[2,0].plot(timeGPS, gpsVelNed[:, 2])
                ax[3,0].plot(timeGPS, gpsVelNorm, label=('%s GPS' % self.log.serials[d]))

            if self.residual and not (refTime is None) and self.log.serials[d] != 'Ref INS': 
                intVelNed = np.empty_like(refVelNed)
                for i in range(3):
                    intVelNed[:,i] = np.interp(refTime, time, insVelNed[:,i], right=np.nan, left=np.nan)
                resNed = intVelNed - refVelNed
                resVelNorm = np.linalg.norm(resNed, axis=1)
                ax[0,1].plot(refTime, resNed[:,0])
                ax[1,1].plot(refTime, resNed[:,1])
                ax[2,1].plot(refTime, resNed[:,2])
                ax[3,1].plot(refTime, resVelNorm, label=self.log.serials[d])

        self.legends_add(ax[3,0].legend(ncol=2))
        if self.residual:
            self.legends_add(ax[0,1].legend(ncol=2))
            for i in range(3):
                self.setPlotYSpanMin(ax[i,1], 1.0)
        for a in ax:
            for b in a:
                b.grid(True)

        self.setup_and_wire_legend()
        return self.saveFigJoinAxes(ax, axs, fig, 'velNED')

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

    def velUVW(self, fig=None, axs=None):
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
                    refTime = getTimeFromGpsTow(self.getData(d, DID_INS_2, 'timeOfWeek'), True)
                    refUvw = self.getData(d, DID_INS_2, 'uvw')
                    continue

            # Reference INS does not exist.  Compute reference from average INS.
            if refTime is None:
                for d in self.active_devs:
                    time = getTimeFromGpsTow(self.getData(d, DID_INS_2, 'timeOfWeek'), True)
                    uvw = self.getData(d, DID_INS_2, 'uvw')
                    if len(uvw) == 0:
                        continue

                    # Adjust data for attitude bias
                    uvw = quatRot(self.log.mount_bias_quat[d,:], uvw)

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
            time = getTimeFromGpsTow(self.getData(d, DID_INS_2, 'timeOfWeek'), True)
            uvw = self.getData(d, DID_INS_2, 'uvw')
            if len(uvw) == 0:
                continue
            # Adjust data for attitude bias
            uvw = quatRot(self.log.mount_bias_quat[d,:], uvw)
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

        self.legends_add(ax[0,0].legend(ncol=2))
        if self.residual:
            self.legends_add(ax[0,1].legend(ncol=2))
            for i in range(3):
                self.setPlotYSpanMin(ax[i,1], 1.0)
        for a in ax:
            for b in a:
                b.grid(True)

        self.setup_and_wire_legend()
        return self.saveFigJoinAxes(ax, axs, fig, 'velUVW')

    def attitude(self, fig=None, axs=None):
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
                    refTime = getTimeFromGpsTow(self.getData(d, DID_INS_2, 'timeOfWeek'), True)

            # Reference INS does not exist.  Compute reference from average INS.
            if refTime is None:
                for d in self.active_devs:
                    time = getTimeFromGpsTow(self.getData(d, DID_INS_2, 'timeOfWeek'), True)
                    qn2b = self.getData(d, DID_INS_2, 'qn2b')
                    if len(qn2b) == 0:
                        continue
                    # Adjust data for attitude bias
                    quat = mul_ConjQuat_Quat(self.log.mount_bias_quat[d,:], qn2b)
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
            qn2b = self.getData(d, DID_INS_2, 'qn2b')
            if len(qn2b) == 0:
                continue
            # Adjust data for attitude bias
            quat = mul_ConjQuat_Quat(self.log.mount_bias_quat[d,:], qn2b)
            euler = quat2euler(quat)
            tow = self.getData(d, DID_INS_2, 'timeOfWeek')
            time = getTimeFromGpsTow(tow, True)
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

        self.legends_add(ax[0,0].legend(ncol=2))
        if self.residual:
            self.legends_add(ax[0,1].legend(ncol=2))
            for i in range(3):
                self.setPlotYSpanMin(ax[i,1], 3.0)
        for a in ax:
            for b in a:
                b.grid(True)

        self.setup_and_wire_legend()
        return self.saveFigJoinAxes(ax, axs, fig, 'attINS')

    def gpx1Heading(self):
        filepath = self.log.directory + "/enu.out"
        if ~os.path.isfile(filepath):
            return [], [], []
        df = pd.read_csv(filepath, skiprows=2, header=None, index_col=None, names=[ 'date', 'time', 'e-baseline', 'n-baseline', 'u-baseline', 'Q', 'ns', 'sde', 'sdn', 'sdu', 'sden', 'sdnu', 'sdue', 'age', 'ratio', 'baseline'], delim_whitespace=True)

        df['datetime'] = df[['date','time']].apply(lambda row: ' '.join(row.values.astype(str)), axis=1)
        df['datetime'] = pd.to_datetime(df['datetime'] , format = '%Y/%m/%d %H:%M:%S.%f')

        baselineE = df['e-baseline'].values
        baselineN = df['n-baseline'].values
        baselineU = df['u-baseline'].values
        baselineNED = np.array([baselineN, baselineE, -baselineU]).T

        gpxTime = np.zeros_like(baselineNED[:,1])
        for index, elem in np.ndenumerate(gpxTime):
            i = index[0]
            gpxTime[i] = (df['datetime'][i] - datetime.datetime(1980,1,6)).total_seconds() % 604800.0

        # Throw away first sample
        n = 20
        gpxTime = gpxTime[n:]
        baselineNED = baselineNED[n:,:]

        gpxHeading = np.arctan2(baselineNED[:,1], baselineNED[:,0])
        return gpxTime, baselineNED, gpxHeading, 


    def heading(self, fig=None, axs=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(3, (2 if self.residual else 1), sharex=True, squeeze=False)
        fig.suptitle('Heading - ' + os.path.basename(os.path.normpath(self.log.directory)))
        self.configureSubplot(ax[0,0], 'Magnetic Heading', 'deg')
        self.configureSubplot(ax[1,0], 'RTK Compassing', 'deg')
        self.configureSubplot(ax[2,0], 'INS Heading', 'deg')

        refRtkTime = None
        refInsTime = None
        
        if self.residual:
            self.configureSubplot(ax[0,1], 'Heading Residual: Magnetic - INS', 'deg')
            self.configureSubplot(ax[1,1], 'Heading Residual: RTK - Mean', 'deg')
            self.configureSubplot(ax[2,1], 'Heading Residual: INS - Mean', 'deg')

            # Reference RTK heading does not exist.  Compute reference from average RTK heading.
            if refRtkTime is None:
                sumDelta = None
                sumCount = 1

                for d in self.active_devs:
                    gpsTime = getTimeFromGpsTowMs(self.getData(d, DID_GPS2_RTK_CMP_REL, 'timeOfWeekMs'))
                    gpsHdg = self.getData(d, DID_GPS2_RTK_CMP_REL, 'baseToRoverHeading')
                    if refRtkTime is None:
                        refRtkTime = gpsTime
                        refRtkHdg = np.copy(gpsHdg)
                        sumDelta = np.zeros_like(gpsHdg)
                    else:
                        unwrapRtkHdg = self.angle_unwrap(gpsHdg)
                        intRtkHdg = np.interp(refRtkTime, gpsTime, unwrapRtkHdg)
                        delta = self.angle_wrap(intRtkHdg - refRtkHdg)
                        sumDelta += delta
                        sumCount += 1
                refRtkHdg += sumDelta / sumCount

            # Reference INS does not exist.  Compute reference from average INS.
            if refInsTime is None:
                sumDelta = None
                sumCount = 1
                for d in self.active_devs:
                    time = getTimeFromGpsTow(self.getData(d, DID_INS_2, 'timeOfWeek'))
                    qn2b = self.getData(d, DID_INS_2, 'qn2b')
                    if len(qn2b) == 0:
                        continue
                    # Adjust data for attitude bias
                    quat = mul_ConjQuat_Quat(self.log.mount_bias_quat[d,:], qn2b)
                    euler = quat2euler(quat)
                    if refInsTime is None:
                        refInsTime = time
                        refEuler = np.copy(euler)
                        sumDelta = np.zeros_like(euler)
                    else:
                        unwrapEuler = self.vec3_unwrap(euler)
                        intEuler = np.empty_like(refEuler)
                        for i in range(3):
                            intEuler[:,i] = np.interp(refInsTime, time, unwrapEuler[:,i])
                        delta = self.vec3_wrap(intEuler - refEuler)
                        sumDelta += delta
                        sumCount += 1
                refEuler += sumDelta / sumCount

        for d in self.active_devs:
            magTime = getTimeFromGpsTowMs(self.getData(d, DID_INL2_MAG_OBS_INFO, 'timeOfWeekMs'), True)
            gpsTime = getTimeFromGpsTowMs(self.getData(d, DID_GPS2_RTK_CMP_REL, 'timeOfWeekMs'))
            insTime = getTimeFromGpsTow(self.getData(d, DID_INS_2, 'timeOfWeek'), True)
            magHdg = self.getData(d, DID_INL2_MAG_OBS_INFO, 'magHdg')
            gpsHdg = self.getData(d, DID_GPS2_RTK_CMP_REL, 'baseToRoverHeading')
            qn2b = self.getData(d, DID_INS_2, 'qn2b')
            if len(qn2b) == 0:
                continue
            euler = quat2euler(qn2b)
            insHdg = euler[:,2]
            if magTime.any():
                ax[0,0].plot(magTime, magHdg * RAD2DEG)
            if gpsTime.any():
                gpsHdg = self.angle_wrap(gpsHdg)
                ax[1,0].plot(gpsTime, gpsHdg*RAD2DEG, label='F9P')
            ax[2,0].plot(insTime, insHdg*RAD2DEG, label=self.log.serials[d])

            if self.residual: 
                if magTime.any():
                    unwrapMagHdg = self.angle_unwrap(magHdg)
                    intMagHdg = np.interp(insTime, magTime, unwrapMagHdg, right=np.nan, left=np.nan)
                    resMagHdg = self.angle_wrap(intMagHdg - insHdg)
                    ax[0,1].plot(insTime, resMagHdg*RAD2DEG)
                if gpsTime.any():
                    unwrapGpsHdg = self.angle_unwrap(gpsHdg)
                    intInsHdg = np.interp(refRtkTime, gpsTime, unwrapGpsHdg, right=np.nan, left=np.nan)
                    resInsHdg = self.angle_wrap(intInsHdg - refRtkHdg)
                    ax[1,1].plot(refRtkTime, resInsHdg*RAD2DEG)
                if not (refInsTime is None) and self.log.serials[d] != 'Ref INS': 
                    unwrapEuler = self.vec3_unwrap(euler)
                    intEuler = np.empty_like(refEuler)
                    for i in range(3):
                        intEuler[:,i] = np.interp(refInsTime, insTime, unwrapEuler[:,i], right=np.nan, left=np.nan)
                    resEuler = self.vec3_wrap(intEuler - refEuler)
                    ax[2,1].plot(refInsTime, resEuler[:,2]*RAD2DEG)

        # if 0:
        #     gpxTime, gpxBaselineNed, gpxHeading = self.gpx1Heading()
        #     if len(gpxTime) == 0 or len(gpxBaselineNED) == 0:
        #         continue

        #     gpsHdg_unwrap = self.angle_unwrap(gpsHdg)
        #     gpxHdg_unwrap = self.angle_unwrap(gpxHeading)
        #     gpxHeadingTruth = np.interp(gpxTime, gpsTime, gpsHdg_unwrap, right=np.nan, left=np.nan)
        #     gpsHeadingErrDeg = self.angle_wrap(gpxHdg_unwrap - gpxHeadingTruth)*RAD2DEG
        #     gpxRms = np.sqrt(np.mean(np.square(gpsHeadingErrDeg)))
        #     self.configureSubplot(ax[0], 'RTK Compassing Error: ' + f'{gpxRms:.3}' + ' deg RMS', 'deg')

        #     ax[1].plot(gpxTime, gpxHeading*RAD2DEG, label='GPX')
        #     self.legends_add(ax[1].legend(ncol=2))
        #     ax[0].plot(gpxTime, gpsHeadingErrDeg, label='GPX - F9P')
        #     self.legends_add(ax[0].legend(ncol=2))

        self.legends_add(ax[2,0].legend(ncol=2))
        for a in ax:
            for b in a:
                b.grid(True)

        self.setup_and_wire_legend()
        return self.saveFigJoinAxes(ax, axs, fig, 'heading')

    def imuStatus(self, fig=None, axs=None):
        try:
            if fig is None:
                fig = plt.figure()
            ax = fig.subplots(1, 1, sharex=True)

            for d in self.active_devs:
                r = d == self.active_devs[0]    # plot text w/ first device
                cnt = 0

                time   = self.getData(d, DID_PIMU, 'time')
                status = self.getData(d, DID_PIMU, 'status')
                title  = 'PIMU Status - '
                if not len(time):
                    time   = self.getData(d, DID_IMU, 'time')
                    status = self.getData(d, DID_IMU, 'status')
                    title  = 'IMU Status - '
                if not len(time):
                    time   = self.getData(d, DID_IMU3_RAW, 'time')
                    status = self.getData(d, DID_IMU3_RAW, 'status')
                    title  = 'IMU3-RAW Status - '
                if not len(time):
                    return

                fig.suptitle(title + os.path.basename(os.path.normpath(self.log.directory)))

                towOffset = self.getGpsTowOffset(d)
                if len(towOffset) > 0:
                    time = getTimeFromGpsTow(time + np.mean(towOffset))

                ax.plot(time, -cnt * 1.5 + ((status & 0x00000001) != 0))
                p1 = ax.get_xlim()[0] + 0.02 * (ax.get_xlim()[1] - ax.get_xlim()[0])
                if r: ax.text(p1, -cnt * 1.5, 'Gyr1 Saturation')
                cnt += 1
                ax.plot(time, -cnt * 1.5 + ((status & 0x00000002) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Gyr2 Saturation')
                cnt += 1
                ax.plot(time, -cnt * 1.5 + ((status & 0x00000004) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Gyr3 Saturation')
                cnt += 1
                ax.plot(time, -cnt * 1.5 + ((status & 0x00000008) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Acc1 Saturation')
                cnt += 1
                ax.plot(time, -cnt * 1.5 + ((status & 0x00000010) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Acc2 Saturation')
                cnt += 1
                ax.plot(time, -cnt * 1.5 + ((status & 0x00000020) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Acc3 Saturation')
                cnt += 1
                cnt += 1

                ax.plot(time, -cnt * 1.5 + ((status & 0x00000040) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Shock Detection')
                cnt += 1
                cnt += 1

                ax.plot(time, -cnt * 1.5 + ((status & 0x00000100) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Mag Update')
                cnt += 1
                ax.plot(time, -cnt * 1.5 + ((status & 0x00000200) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Ref IMU Present')
                cnt += 1
                cnt += 1

                ax.plot(time, -cnt * 1.5 + ((status & 0x00010000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Gyr1 OK')
                cnt += 1
                ax.plot(time, -cnt * 1.5 + ((status & 0x00020000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Gyr2 OK')
                cnt += 1
                ax.plot(time, -cnt * 1.5 + ((status & 0x00040000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Gyr3 OK')
                cnt += 1
                ax.plot(time, -cnt * 1.5 + ((status & 0x00080000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Acc1 OK')
                cnt += 1
                ax.plot(time, -cnt * 1.5 + ((status & 0x00100000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Acc2 OK')
                cnt += 1
                ax.plot(time, -cnt * 1.5 + ((status & 0x00200000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Acc3 OK')
                cnt += 1
                cnt += 1
                ax.plot(time, -cnt * 1.5 + ((status & 0x01000000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Gyr Reject')
                cnt += 1
                ax.plot(time, -cnt * 1.5 + ((status & 0x02000000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Acc Reject')
                cnt += 1
                cnt += 1

            ax.grid(True)

            self.setup_and_wire_legend()
            return self.saveFigJoinAxes(ax, axs, fig, 'imuStatus')
        except:
            print(RED + "problem plotting imuStatus: " + sys.exc_info()[0] + RESET)

    def insStatus(self, fig=None, axs=None):
        try:
            if fig is None:
                fig = plt.figure()
            ax = fig.subplots(1, 1, sharex=True)
            fig.suptitle('INS Status - ' + os.path.basename(os.path.normpath(self.log.directory)))

            for d in self.active_devs:
                r = d == self.active_devs[0]    # plot text w/ first device
                cnt = 0
                instime = getTimeFromGpsTow(self.getData(d, DID_INS_2, 'timeOfWeek'), True)
                iStatus = self.getData(d, DID_INS_2, 'insStatus')

                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x00000001) != 0))
                p1 = ax.get_xlim()[0] + 0.02 * (ax.get_xlim()[1] - ax.get_xlim()[0])
                if r: ax.text(p1, -cnt * 1.5, 'Hdg Coarse')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x00000010) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Hdg Fine')
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
                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x00000008) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Wheel Enc. aiding Vel')
                cnt += 1
                cnt += 1
                # ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x00001000) != 0))
                # if r: ax.text(p1, -cnt * 1.5, 'Nav Mode')
                # cnt += 1
                sol_status = (iStatus & 0x000F0000) >> 16
                aligning_or_high_variance = np.isin(sol_status, [1,4,6,8])    # Include aligning w/ high variance 
                solution_nav = np.isin(sol_status,  [3,4])
                solution_ahrs = np.isin(sol_status, [5,6])
                solution_vrs = np.isin(sol_status,  [7,8])
                # ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x000F0000) >> 16) / 4.0)
                # if r: ax.text(p1, -cnt * 1.5, 'Solution Status')
                # cnt += 1
                ax.plot(instime, -cnt * 1.5 + solution_nav)
                if r: ax.text(p1, -cnt * 1.5, 'Solution: Nav')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + solution_ahrs)
                if r: ax.text(p1, -cnt * 1.5, 'Solution: AHRS')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + solution_vrs)
                if r: ax.text(p1, -cnt * 1.5, 'Solution: VRS')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + aligning_or_high_variance)
                if r: ax.text(p1, -cnt * 1.5, 'Aligning/High Variance')
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

            self.setup_and_wire_legend()
            return self.saveFigJoinAxes(ax, axs, fig, 'iStatus')
        except:
            print(RED + "problem plotting insStatus: " + sys.exc_info()[0] + RESET)
        self.joinFigXAxes(ax,axs)
        return ax

    def hdwStatus(self, fig=None, axs=None):
        try:
            if fig is None:
                fig = plt.figure()
            ax = fig.subplots(1, 1, sharex=True)
            fig.suptitle('Hardware Status - ' + os.path.basename(os.path.normpath(self.log.directory)))

            for d in self.active_devs:
                r = d == self.active_devs[0]    # plot text w/ first device
                cnt = 0
                instime = getTimeFromGpsTow(self.getData(d, DID_INS_2, 'timeOfWeek'), True)
                hStatus = self.getData(d, DID_INS_2, 'hdwStatus')

                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x00000001) != 0))
                p1 = ax.get_xlim()[0] + 0.02 * (ax.get_xlim()[1] - ax.get_xlim()[0])
                if r: ax.text(p1, -cnt * 1.5, 'Motion Gyr')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x00000002) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Motion Acc')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x00000004) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Fault Detect Gyr')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x00000008) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Fault Detect Acc')
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

                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x00000010) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Satellite Rx')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x00000020) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Strobe In')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x00000040) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'GPS TOW Valid')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x00000080) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Ref IMU Rx')
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
                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x00F00000) >> 20) / 4)
                if r: ax.text(p1, -cnt * 1.5, 'Com Parse Error Count')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x02000000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Temperature error')
                cnt += 1
                cnt += 1

                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x00040000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'No GPS PPS')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x00080000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'GPS PPS Timesync')
                cnt += 1
                cnt += 1

                bit_status = (hStatus & 0x03000000) >> 24
                ax.plot(instime, -cnt * 1.5 + (bit_status == 1))
                if r: ax.text(p1, -cnt * 1.5, 'BIT Running')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + (bit_status == 2))
                if r: ax.text(p1, -cnt * 1.5, 'BIT Passed')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + (bit_status == 3))
                if r: ax.text(p1, -cnt * 1.5, 'BIT Fault')
                cnt += 1
                cnt += 1

                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x70000000) == 0x10000000))
                if r: ax.text(p1, -cnt * 1.5, 'Reset Backup Mode')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x70000000) == 0x20000000))
                if r: ax.text(p1, -cnt * 1.5, 'Watchdog Reset')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x70000000) == 0x30000000))
                if r: ax.text(p1, -cnt * 1.5, 'Software Reset')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x70000000) == 0x40000000))
                if r: ax.text(p1, -cnt * 1.5, 'Hardware Reset')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x80000000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Critical Sys Fault')
                cnt += 1
                cnt += 1
                
            ax.grid(True)

            self.setup_and_wire_legend()
            return self.saveFigJoinAxes(ax, axs, fig, 'Hardware Status')
        except:
            print(RED + "problem plotting hdwStatus: " + sys.exc_info()[0] + RESET)

    def genFaultCodes(self, fig=None, axs=None):
        try:
            if fig is None:
                fig = plt.figure()
            ax = fig.subplots(1, 1, sharex=True)
            fig.suptitle('Gen Fault Codes - ' + os.path.basename(os.path.normpath(self.log.directory)))

            for d in self.active_devs:
                r = d == self.active_devs[0]    # plot text w/ first device
                cnt = 0
                faultTime = getTimeFromGpsTowMs(self.getData(d, DID_SYS_PARAMS, 'timeOfWeekMs'), True)
                genFaultCode = self.getData(d, DID_SYS_PARAMS, 'genFaultCode')

                ax.plot(faultTime, -cnt * 1.5 + ((genFaultCode & 0x00000001) != 0))
                p1 = ax.get_xlim()[0] + 0.02 * (ax.get_xlim()[1] - ax.get_xlim()[0])
                if r: ax.text(p1, -cnt * 1.5, 'Overrun UWV')
                cnt += 1
                ax.plot(faultTime, -cnt * 1.5 + ((genFaultCode & 0x00000002) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Overrun Latitude')
                cnt += 1
                ax.plot(faultTime, -cnt * 1.5 + ((genFaultCode & 0x00000004) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Overrun Altitude')
                cnt += 1
                ax.plot(faultTime, -cnt * 1.5 + ((genFaultCode & 0x00000010) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Unhandled Interrupt')
                cnt += 1
                ax.plot(faultTime, -cnt * 1.5 + ((genFaultCode & 0x00000020) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'GNSS Sys Fault')
                cnt += 1
                ax.plot(faultTime, -cnt * 1.5 + ((genFaultCode & 0x00000040) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'GNSS Tx Limited')
                cnt += 1
                ax.plot(faultTime, -cnt * 1.5 + ((genFaultCode & 0x00000080) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'GNSS Rx Overrun')
                cnt += 1
                cnt += 1

                ax.plot(faultTime, -cnt * 1.5 + ((genFaultCode & 0x00000100) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Init Sensors')
                cnt += 1
                ax.plot(faultTime, -cnt * 1.5 + ((genFaultCode & 0x00000200) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Init SPI')
                cnt += 1
                ax.plot(faultTime, -cnt * 1.5 + ((genFaultCode & 0x00000400) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Config SPI')
                cnt += 1
                ax.plot(faultTime, -cnt * 1.5 + ((genFaultCode & 0x00000800) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'GNSS1 Init')
                cnt += 1
                ax.plot(faultTime, -cnt * 1.5 + ((genFaultCode & 0x00001000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'GNSS2 Init')
                cnt += 1
                ax.plot(faultTime, -cnt * 1.5 + ((genFaultCode & 0x00002000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Flash Invalid Values')
                cnt += 1
                ax.plot(faultTime, -cnt * 1.5 + ((genFaultCode & 0x00004000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Flash Checksum Failure')
                cnt += 1
                ax.plot(faultTime, -cnt * 1.5 + ((genFaultCode & 0x00008000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Flash Write Failure')
                cnt += 1
                cnt += 1

                ax.plot(faultTime, -cnt * 1.5 + ((genFaultCode & 0x00010000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Sys Fault General')
                cnt += 1
                ax.plot(faultTime, -cnt * 1.5 + ((genFaultCode & 0x00020000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Sys Fault Critical')
                cnt += 1
                cnt += 1

                ax.plot(faultTime, -cnt * 1.5 + ((genFaultCode & 0x00040000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Sensor Saturation')
                cnt += 1
                ax.plot(faultTime, -cnt * 1.5 + ((genFaultCode & 0x00100000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Init IMU')
                cnt += 1
                ax.plot(faultTime, -cnt * 1.5 + ((genFaultCode & 0x00200000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Init Barometer')
                cnt += 1
                ax.plot(faultTime, -cnt * 1.5 + ((genFaultCode & 0x00400000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Init Magnetometer')
                cnt += 1
                ax.plot(faultTime, -cnt * 1.5 + ((genFaultCode & 0x00800000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Init I2C')
                cnt += 1
                cnt += 1

                ax.plot(faultTime, -cnt * 1.5 + ((genFaultCode & 0x00080000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Ser Check Init')
                cnt += 1
                ax.plot(faultTime, -cnt * 1.5 + ((genFaultCode & 0x01000000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Chip Erase Invalid')
                cnt += 1
                ax.plot(faultTime, -cnt * 1.5 + ((genFaultCode & 0x02000000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'EKF GNSS Time Fault')
                cnt += 1
                ax.plot(faultTime, -cnt * 1.5 + ((genFaultCode & 0x02000000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'GNSS Rcvr Time Fault')
                cnt += 1
                cnt += 1

            ax.grid(True)

            self.setup_and_wire_legend()
            return self.saveFigJoinAxes(ax, axs, fig, 'genFaultCode')
        except:
            print(RED + "problem plotting insStatus: " + sys.exc_info()[0] + RESET)

    def gpxStatus(self, fig=None, axs=None):
        try:
            if fig is None:
                fig = plt.figure()
            ax = fig.subplots(1, 1, sharex=True)
            fig.suptitle('GPX Status - ' + os.path.basename(os.path.normpath(self.log.directory)))

            for d in self.active_devs:
                r = d == self.active_devs[0]    # plot text w/ first device
                cnt = 0
                time = getTimeFromGpsTowMs(self.getData(d, DID_GPX_STATUS, 'timeOfWeekMs'))
                status = self.getData(d, DID_GPX_STATUS, 'status')

                ax.plot(time, -cnt * 1.5 + ((status & 0x0000000F) >> 0))
                p1 = ax.get_xlim()[0] + 0.02 * (ax.get_xlim()[1] - ax.get_xlim()[0])
                if r: ax.text(p1, -cnt * 1.5, 'Com parse err count')
                cnt += 1
                ax.plot(time, -cnt * 1.5 + ((status & 0xFF000000) >> 24))
                if r: ax.text(p1, -cnt * 1.5, 'Fatal event')
                cnt += 1
                cnt += 1
                
            ax.grid(True)

            self.setup_and_wire_legend()
            return self.saveFigJoinAxes(ax, axs, fig, 'GPX Status')
        except:
            print(RED + "problem plotting GPX status: " + sys.exc_info()[0] + RESET)

    def gpxHdwStatus(self, fig=None, axs=None):
        try:
            if fig is None:
                fig = plt.figure()
            ax = fig.subplots(1, 1, sharex=True)
            fig.suptitle('GPX Hardware Status - ' + os.path.basename(os.path.normpath(self.log.directory)))

            for d in self.active_devs:
                r = d == self.active_devs[0]    # plot text w/ first device
                cnt = 0
                time = getTimeFromGpsTowMs(self.getData(d, DID_GPX_STATUS, 'timeOfWeekMs'))
                hStatus = self.getData(d, DID_GPX_STATUS, 'hdwStatus')

                ax.plot(time, -cnt * 1.5 + ((hStatus & 0x00000001) != 0))
                p1 = ax.get_xlim()[0] + 0.02 * (ax.get_xlim()[1] - ax.get_xlim()[0])
                if r: ax.text(p1, -cnt * 1.5, 'GNSS1 Sat RX')
                cnt += 1
                ax.plot(time, -cnt * 1.5 + ((hStatus & 0x00000002) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'GNSS2 Sat RX')
                cnt += 1
                ax.plot(time, -cnt * 1.5 + ((hStatus & 0x00000004) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'GNSS1 TOW Valid')
                cnt += 1
                ax.plot(time, -cnt * 1.5 + ((hStatus & 0x00000005) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'GNSS2 TOW Valid')
                cnt += 1
                cnt += 1

                ax.plot(time, -cnt * 1.5 + ((hStatus & 0x00000070) >> 4))
                if r: ax.text(p1, -cnt * 1.5, 'GNSS1 Reset Count')
                cnt += 1
                ax.plot(time, -cnt * 1.5 + ((hStatus & 0x00000070) >> 8))
                if r: ax.text(p1, -cnt * 1.5, 'GNSS2 Reset Count')
                cnt += 1
                ax.plot(time, -cnt * 1.5 + ((hStatus & 0x00000080) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'GNSS1 Fault')
                cnt += 1
                ax.plot(time, -cnt * 1.5 + ((hStatus & 0x00000800) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'GNSS2 Fault')
                cnt += 1
                cnt += 1

                ax.plot(time, -cnt * 1.5 + ((hStatus & 0x00001000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'FW Update Required')
                cnt += 1
                ax.plot(time, -cnt * 1.5 + ((hStatus & 0x00004000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Sys Reset Required')
                cnt += 1
                ax.plot(time, -cnt * 1.5 + ((hStatus & 0x00008000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Flash Write Pending')
                cnt += 1
                cnt += 1

                ax.plot(time, -cnt * 1.5 + ((hStatus & 0x00010000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Err Com Tx Limited')
                cnt += 1
                ax.plot(time, -cnt * 1.5 + ((hStatus & 0x00020000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Err Com Rx Overrun')
                cnt += 1
                ax.plot(time, -cnt * 1.5 + ((hStatus & 0x00040000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Err GPS1 PPS')
                cnt += 1
                ax.plot(time, -cnt * 1.5 + ((hStatus & 0x00080000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Err GPS2 PPS')
                cnt += 1
                cnt += 1

                ax.plot(time, -cnt * 1.5 + ((hStatus & 0x00100000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Err GPS1 low CN0')
                cnt += 1
                ax.plot(time, -cnt * 1.5 + ((hStatus & 0x00200000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Err GPS2 low CN0')
                cnt += 1
                ax.plot(time, -cnt * 1.5 + ((hStatus & 0x00400000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Err GPS1 CN0 IR')
                cnt += 1
                ax.plot(time, -cnt * 1.5 + ((hStatus & 0x00800000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Err GPS2 CN0 IR')
                cnt += 1
                cnt += 1

                ax.plot(time, -cnt * 1.5 + ((hStatus & 0x0300000) >> 24))
                if r: ax.text(p1, -cnt * 1.5, 'BIT: Off, Running, Passed, Fault')
                cnt += 1
                ax.plot(time, -cnt * 1.5 + ((hStatus & 0x04000000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Err Temperature')
                cnt += 1
                ax.plot(time, -cnt * 1.5 + ((hStatus & 0x08000000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'GPS PPS Timesync')
                cnt += 1
                cnt += 1

                ax.plot(time, -cnt * 1.5 + ((hStatus & 0x10000000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Reset Backup Mode')
                cnt += 1
                ax.plot(time, -cnt * 1.5 + ((hStatus & 0x20000000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Watchdog Reset')
                cnt += 1
                ax.plot(time, -cnt * 1.5 + ((hStatus & 0x30000000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Software Reset')
                cnt += 1
                ax.plot(time, -cnt * 1.5 + ((hStatus & 0x40000000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Hardware Reset')
                cnt += 1
                ax.plot(time, -cnt * 1.5 + ((hStatus & 0x80000000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Critical Sys Fault')
                cnt += 1
                cnt += 1
                
            ax.grid(True)

            self.setup_and_wire_legend()
            return self.saveFigJoinAxes(ax, axs, fig, 'GPX Hardware Status')
        except:
            print(RED + "problem plotting GPX hdwStatus: " + sys.exc_info()[0] + RESET)


    def gpsStats(self, fig=None, axs=None, did_gps_pos=DID_GPS1_POS):
        # try:
        if fig is None:
            fig = plt.figure()

        ax = fig.subplots(6, 1, sharex=True, gridspec_kw={'height_ratios': [1, 2, 2, 2, 1, 1]})
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
        self.configureSubplot(ax[5], 'CNO Sigma', '')

        plot_legend = 1
        for d in self.active_devs:
            r = d == self.active_devs[0]  # plot text w/ first device
            time = getTimeFromGpsTowMs(self.getData(d, did_gps_pos, 'timeOfWeekMs'))
            velTime = getTimeFromGpsTowMs(self.getData(d, did_gps_vel, 'timeOfWeekMs'))
            gStatus = self.getData(d, did_gps_pos, 'status')

            ax[0].plot(time, gStatus & 0xFF, label=self.log.serials[d])
            ax[1].plot(time, self.getData(d, did_gps_pos, 'cnoMean'), label=self.log.serials[d])
            ax[2].plot(time, self.getData(d, did_gps_pos, 'hAcc'), 'r', label="hAcc")
            ax[2].plot(time, self.getData(d, did_gps_pos, 'vAcc'), 'b', label="vAcc")
            ax[2].plot(time, self.getData(d, did_gps_pos, 'pDop'), 'm', label="pDop")
            if self.log.data[d, DID_GPS1_RTK_POS] is not []:
                rtktime = getTimeFromGpsTowMs(self.getData(d, DID_GPS1_RTK_POS, 'timeOfWeekMs'))
                ax[2].plot(rtktime, self.getData(d, DID_GPS1_RTK_POS, 'vAcc'), 'g', label="rtkHor")
            ax[3].plot(velTime, self.getData(d, did_gps_vel, 'sAcc'), label="sAcc")

            if plot_legend:
                plot_legend = 0
                self.legends_add(ax[2].legend(ncol=2))

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

            ax[5].plot(time, self.getData(d, did_gps_pos, 'cnoMeanSigma'), label=self.log.serials[d])

        self.setPlotYSpanMin(ax[1], 5)

        self.legends_add(ax[1].legend(ncol=2))
        for a in ax:
            a.grid(True)

        self.setup_and_wire_legend()
        return self.saveFigJoinAxes(ax, axs, fig, 'Gps Stats')

    def gps2Stats(self, fig=None, axs=None):
        self.gpsStats(fig=fig, axs=axs, did_gps_pos=DID_GPS2_POS)

    def rtkPosStats(self, fig=None, axs=None):
        self.rtkStats("Position", DID_GPS1_RTK_POS_REL, fig=fig, axs=axs)

    def rtkCmpStats(self, fig=None, axs=None):
        self.rtkStats("Compassing", DID_GPS2_RTK_CMP_REL, fig=fig, axs=axs)

    def rtkStats(self, name, relDid, fig=None, axs=None):
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
            rtkRelTime = getTimeFromGpsTowMs(self.getData(d, relDid, 'timeOfWeekMs'))
            if len(rtkRelTime) == 0:
                continue
            # rtkMiscTime = getTimeFromGpsTowMs(self.getData(d, DID_GPS2_RTK_CMP_MISC, 'timeOfWeekMs'))
            if not self.log.compassing:
                gps1PosTime = getTimeFromGpsTowMs(self.getData(d, DID_GPS1_POS, 'timeOfWeekMs'), 1)
                gps2PosTime = getTimeFromGpsTowMs(self.getData(d, DID_GPS2_POS, 'timeOfWeekMs'), 1)
                if len(gps1PosTime):
                    fixType = self.getData(d, DID_GPS1_POS, 'status') >> 8 & 0x1F
                    ax[0].plot(gps1PosTime, fixType, label=self.log.serials[d])
                elif len(gps2PosTime):
                    fixType = self.getData(d, DID_GPS2_POS, 'status') >> 8 & 0x1F
                    ax[0].plot(gps2PosTime, fixType, label=self.log.serials[d])
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
            self.legends_add(ax[0].legend(ncol=2))

        self.setPlotYSpanMin(ax[1], 0.5)    # Differential age
        self.setPlotYSpanMin(ax[3], 1.0)    # Distance to base

        for a in ax:
            a.grid(True)

        self.setup_and_wire_legend()
        return self.saveFigJoinAxes(ax, axs, fig, 'rtk'+name+'Stats')

    def rtkBaselineVector(self, fig=None, axs=None):
        name = "Compassing"
        relDid = DID_GPS2_RTK_CMP_REL

        # try:
        n_plots = 2
        if fig is None:
            fig = plt.figure()

        ax = fig.subplots(n_plots, 1, sharex=True)
        fig.suptitle('RTK ' + name + ' Stats - ' + os.path.basename(os.path.normpath(self.log.directory)))
        self.configureSubplot(ax[0], 'Base to Rover N', 'm')
        self.configureSubplot(ax[1], 'Base to Rover E', 'm')

        for i, d in enumerate(self.active_devs):
            rtkRelTime = getTimeFromGpsTowMs(self.getData(d, relDid, 'timeOfWeekMs'))
            gps1PosTime = getTimeFromGpsTowMs(self.getData(d, DID_GPS1_POS, 'timeOfWeekMs'))
            gpsLla = self.getData(d, DID_GPS1_POS, 'lla', True)
            if len(gpsLla) == 0:
                continue
            baseToRoverECEF = self.getData(d, relDid, 'baseToRoverVector')

            qe2n = quat_ecef2ned(gpsLla[-1,:]*np.pi/180.0)
            baselineNED = quatConjRot(qe2n, baseToRoverECEF)
            # gpsHeading = np.arctan2(baselineNED[:,1], baselineNED[:,0])

            ax[0].plot(rtkRelTime, baselineNED[:,0])
            ax[1].plot(rtkRelTime, baselineNED[:,1])

            gpxTime, gpxBaselineNED, gpxHeading = self.gpx1Heading()
            if len(gpxTime) == 0 or len(gpxBaselineNED) == 0:
                continue

            ax[0].plot(gpxTime, gpxBaselineNED[:,0], label="GPX")
            ax[1].plot(gpxTime, gpxBaselineNED[:,1], label="GPX")

            self.legends_add(ax[0].legend(ncol=2))

        for a in ax:
            a.grid(True)

        self.setup_and_wire_legend()
        return self.saveFigJoinAxes(ax, axs, fig, 'rtk'+name+'BaseToRoverVector')

    def rtkObsGPS1(self, fig=None, axs=None):
        self.rtkObs("Compassing", DID_GPS1_RAW, fig=fig, axs=axs)

    def rtkObsGPS2(self, fig=None, axs=None):
        self.rtkObs("Compassing", DID_GPS2_RAW, fig=fig, axs=axs)

    def rtkObs(self, name, relDid, fig=None, axs=None):
        Nf = 2
        n_plots = 8
        if fig is None:
            fig = plt.figure()

        ax = fig.subplots(n_plots, 1, sharex=True)
        fig.suptitle('GNSS Receiver Observations')
        self.configureSubplot(ax[0], 'L1 Pseudorange', 'm')
        self.configureSubplot(ax[1], 'L5 Pseudorange', 'm')
        self.configureSubplot(ax[2], 'L1 Carier Phase', 'cycles')
        self.configureSubplot(ax[3], 'L5 Carier Phase', 'cycles')
        self.configureSubplot(ax[4], 'L1 Doppler')
        self.configureSubplot(ax[5], 'L5 Doppler')
        self.configureSubplot(ax[6], 'L1 LLI')
        self.configureSubplot(ax[7], 'L5 LLI')

        for i, d in enumerate(self.active_devs):
            gps_data = self.log.data[d, relDid][0]
            N = len(gps_data)
            j0 = 0 # skip a few samples that may be corrupted

            # Build satellite array
            sat = np.empty(0, dtype=int)
            for j in range(j0, N):
                obs = gps_data[j]
                M = len(obs)
                for k in range(M):
                    sat_k = obs['sat'][k]
                    # add satellite if not in the list and if L1 observations are valid
                    if ( sat_k != 0 and (sat_k not in sat) and 
                         obs['time']['time'][k] > 0 and obs['P'][k][0] > 0 and obs['L'][k][0] > 0 ):
                        sat = np.append(sat, sat_k)

            Nsat = len(sat)
            tgps = np.zeros([N, Nsat])
            P = np.empty([Nf, N, Nsat])
            L = np.empty([Nf, N, Nsat])
            D = np.empty([Nf, N, Nsat])
            LLI  = np.empty([Nf, N, Nsat])
            P[:] = np.nan
            L[:] = np.nan
            D[:] = np.nan
            LLI[:]  = np.nan

            # Fill observation arrays
            for j in range(j0, N):
                obs = gps_data[j]
                M = len(obs)
                for k in range(M):
                    sat_k = obs['sat'][k]
                    if sat_k == 0:
                        continue
                    indo = np.where(obs['sat'] == sat_k)
                    inds = np.where(sat == sat_k)
                    if np.size(indo) == 0 or np.size(inds) == 0:
                        continue
                    indo = indo[0][0]
                    inds = inds[0][0]
                    tgps[j,inds] = obs['time']['time'][indo] + obs['time']['sec'][indo]
                    # Use only non-zero pseudorange and phase
                    indP = np.flatnonzero(obs['P'][indo])
                    indL = np.flatnonzero(obs['L'][indo])
                    indD = np.flatnonzero(obs['D'][indo])
                    if np.size(indP) > 0 and np.size(indL) > 0:
                        P[indP,j,inds] = obs['P'][indo][indP]
                        L[indL,j,inds] = obs['L'][indo][indL]
                        D[indD,j,inds] = obs['D'][indo][indD]
                        LLI[:,j,inds]  = obs['LLI'][indo]

            for k in range(len(sat)):
                ind = np.squeeze(np.where(tgps[:,k] != 0.0))
                # Do not plot satellites with invalid L1 pseudorange
                if np.isnan(P[0,ind,k]).all():
                    continue
                # Do not plot satellites with invalid L1 phase
                if np.isnan(L[0,ind,k]).all():
                    continue
                dt = np.diff(tgps[ind,k])
                t = np.squeeze(tgps[ind, k])
                Pk = P[:,ind,k]
                Lk = L[:,ind,k]
                Dk = D[:,ind,k]
                LLIk = LLI[:,ind,k]
                ind1 = np.squeeze(np.where(dt > 10*dt[0])) # jump in time
                if np.size(ind1) == 0:
                    ind1 = 0
                elif np.size(ind1) > 1:
                    ind1 = ind1[-1]
                ind1 = ind1 + 1
                ax[0].plot(t[ind1:,], Pk[0,ind1:], label=('Sat %s' % sat[k]))
                ax[1].plot(t[ind1:,], Pk[1,ind1:])
                ax[2].plot(t[ind1:,], Lk[0,ind1:])
                ax[3].plot(t[ind1:,], Lk[1,ind1:])
                ax[4].plot(t[ind1:,], Dk[0,ind1:])
                ax[5].plot(t[ind1:,], Dk[1,ind1:])
                ax[6].plot(t[ind1:,], LLIk[0,ind1:])
                ax[7].plot(t[ind1:,], LLIk[1,ind1:])
                self.legends_add(ax[0].legend(ncol=2))

        for a in ax:
            a.grid(True)

        self.setup_and_wire_legend()
        return self.saveFigJoinAxes(ax, axs, fig, 'rtk'+name+'obs_sd')

    def rtkObsSingleDiff(self, fig=None, axs=None):
        name = "Compassing"
        if len(self.log.data[0, DID_GPS1_RAW][0]) == 0:
            return
        
        Nf = len(self.log.data[0, DID_GPS1_RAW][0][0]['P'][0])
        n_plots = 8
        if fig is None:
            fig = plt.figure()

        ax = fig.subplots(n_plots, 1, sharex=True)
        fig.suptitle('RTK Rover-Base Single Differences')
        self.configureSubplot(ax[0], 'L1 Pseudorange difference', 'm')
        self.configureSubplot(ax[1], 'L5 Pseudorange difference', 'm')
        self.configureSubplot(ax[2], 'L1 Carier phase difference', 'cycles')
        self.configureSubplot(ax[3], 'L5 Carier phase difference', 'cycles')
        self.configureSubplot(ax[4], 'L1 GPS1 SNR', 'dB*Hz')
        self.configureSubplot(ax[5], 'L5 GPS1 SNR', 'dB*Hz')
        self.configureSubplot(ax[6], 'L1 GPS2 SNR', 'dB*Hz')
        self.configureSubplot(ax[7], 'L5 GPS2 SNR', 'dB*Hz')

        for id_, d in enumerate(self.active_devs):

            gps1_data = self.log.data[d, DID_GPS1_RAW][0]
            gps2_data = self.log.data[d, DID_GPS2_RAW][0]

            # Reassemble multiple chunks of data by timestamp
            j0 = 10 # skip a few first samples in case there is some odd data corruption
            t1 = np.empty(0)
            del_ind = range(j0) #np.empty(0, dtype=int)
            for j in range(j0, len(gps1_data)):
                ind = np.flatnonzero(gps1_data[j]['time']['time'])
                if len(ind) == 0:
                    # Empty chunk of data (all time stamps are zero)
                    del_ind = np.append(del_ind, j)
                    continue
                ind = ind[0]
                t_ = gps1_data[j]['time']['time'][ind] + gps1_data[j]['time']['sec'][ind]
                if j > j0 and t_ == t1[-1]:
                    # add chunk to the previous data and mark for deletion
                    gps1_data[j-1] = np.append(gps1_data[j-1], gps1_data[j])
                    del_ind = np.append(del_ind, j)
                    continue
                else:
                    # new data
                    t1 = np.append(t1, t_)
            gps1_data = np.asarray(gps1_data, dtype = object) # newer Numpy can't delete inhomogeneous arrays unless we set dtype=object
            gps1_data = np.delete(gps1_data, del_ind)

            t2 = np.empty(0)
            del_ind = range(j0) #np.empty(0, dtype=int)
            for j in range(j0, len(gps2_data)):
                ind = np.flatnonzero(gps2_data[j]['time']['time'])
                if len(ind) == 0:
                    # Empty chunk of data (all time stamps are zero)
                    del_ind = np.append(del_ind, j)
                    continue
                ind = ind[0]
                t_ = gps2_data[j]['time']['time'][ind] + gps2_data[j]['time']['sec'][ind]
                if j > j0 and t_ == t2[-1]:
                    # add chunk to the previous data and mark for deletion
                    gps2_data[j-1] = np.append(gps2_data[j-1], gps2_data[j])
                    del_ind = np.append(del_ind, j)
                    continue
                else:
                    # new data
                    t2 = np.append(t2, t_)
            gps2_data = np.asarray(gps2_data, dtype = object) # newer Numpy can't delete inhomogeneous arrays unless we set dtype=object
            gps2_data = np.delete(gps2_data, del_ind)

            N1 = len(gps1_data)
            N2 = len(gps2_data)

            # Build common satellite array for gps1 and gps2
            sat = np.empty(0, dtype=int)
            for j in range(N1):
                obs = gps1_data[j]
                M = len(obs)
                for k in range(M):
                    sat_k = obs['sat'][k]
                    # add satellite if not in the list and if L1 observations are valid
                    if ( sat_k != 0 and (sat_k not in sat) and 
                         obs['time']['time'][k] > 0 and obs['P'][k][0] > 0 and obs['L'][k][0] > 0 ):
                        sat = np.append(sat, sat_k)
            sat2 = np.empty(0, dtype=int)
            for j in range(N2):
                obs = gps2_data[j]
                M = len(obs)
                for k in range(M):
                    sat_k = obs['sat'][k]
                    # add satellite if not in the list and if L1 observations are valid
                    if ( sat_k != 0 and (sat_k not in sat2) and 
                         obs['time']['time'][k] > 0 and obs['P'][k][0] > 0 and obs['L'][k][0] > 0 ):
                        sat2 = np.append(sat2, sat_k)
            del_ind = np.empty(0, dtype = int)
            for is_, j in enumerate(sat):
                if j not in sat2:
                    del_ind = np.append(del_ind, is_)
            sat = np.delete(sat, del_ind)

            # Build array of common timestamps for gps1 and gps2
            del_ind = np.empty(0, dtype = int)
            for j in range(N1):
                if t1[j] not in t2:
                    del_ind = np.append(del_ind, j)
            gps1_data = np.asarray(gps1_data, dtype = object) # newer Numpy can't delete inhomogeneous arrays unless we set dtype=object
            gps1_data = np.delete(gps1_data, del_ind)
            t1 = np.delete(t1, del_ind)
            N1 = len(gps1_data)

            del_ind = np.empty(0, dtype = int)
            t = np.empty(0, dtype=int)
            for j in range(N2):
                if t2[j] not in t1:
                    del_ind = np.append(del_ind, j)
            gps2_data = np.asarray(gps2_data, dtype = object) # newer Numpy can't delete inhomogeneous arrays unless we set dtype=object
            gps2_data = np.delete(gps2_data, del_ind)
            t2 = np.delete(t2, del_ind)
            N2 = len(gps2_data)
            if (N1 != N2): 
                continue

            Nsat = len(sat)
            delta_P = np.empty([Nf, N1, Nsat])
            delta_L = np.empty([Nf, N1, Nsat])
            delta_P[:] = np.nan
            delta_L[:] = np.nan
            snr1 = np.empty([Nf, N1, Nsat])
            snr2 = np.empty([Nf, N1, Nsat])
            snr1[:] = np.nan
            snr2[:] = np.nan

            # Compute single differences
            for j in range(N1):
                obs1 = gps1_data[j]
                obs2 = gps2_data[j]

                for k in range(Nsat):
                    sat_k = sat[k]
                    # is this satellite present in both gps1 and gps2 data?
                    ind1 = np.where(obs1['sat'] == sat_k)
                    ind2 = np.where(obs2['sat'] == sat_k)
                    if np.size(ind1) == 0 or np.size(ind2) == 0:
                        continue
                    ind1 = ind1[0][0]
                    ind2 = ind2[0][0]

                    # Use only non-zero pseudorange and phase
                    indval1 = np.flatnonzero(obs1['P'][ind1])
                    indval2 = np.flatnonzero(obs2['P'][ind2])
                    if np.size(indval1) > 0 and np.size(indval2) > 0:
                        delta_P[:,j,k] = obs1['P'][ind1][indval1] - obs2['P'][ind2][indval2]
                        delta_L[:,j,k] = obs1['L'][ind1][indval1] - obs2['L'][ind2][indval2]
                        snr1[:,j,k] = obs1['SNR'][ind1][indval1] * 0.25
                        snr2[:,j,k] = obs2['SNR'][ind2][indval2] * 0.25

            for k in range(Nsat):
                # Do not plot satellites that appeared only for a short time
                ind = np.flatnonzero(~np.isnan(delta_P[0,:,k]))
                if (np.size(ind) / N1) < 0.1 and len(ind) < 100:
                    continue
                ax[0].plot(t1, delta_P[0,:,k], label=('Sat %s' % sat[k]))
                ax[1].plot(t1, delta_P[1,:,k])
                ax[2].plot(t1, delta_L[0,:,k])
                ax[3].plot(t1, delta_L[1,:,k])
                ax[4].plot(t1, snr1[0,:,k])
                ax[5].plot(t1, snr1[1,:,k])
                ax[6].plot(t1, snr2[0,:,k])
                ax[7].plot(t1, snr2[1,:,k])

                self.legends_add(ax[0].legend(ncol=2))

        for a in ax:
            a.grid(True)

        self.setup_and_wire_legend()
        return self.saveFigJoinAxes(ax, axs, fig, 'rtk'+name+'obs_sd')

    def rtkPosMisc(self, fig=None, axs=None):
        self.rtkMisc("Position", DID_GPS1_RTK_POS_MISC, fig=fig, axs=axs)

    def rtkCmpMisc(self, fig=None, axs=None):
        self.rtkMisc("Position", DID_GPS2_RTK_CMP_MISC, fig=fig, axs=axs)

    def rtkMisc(self, name, miscDid, fig=None, axs=None):
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
            # rtkRelTime = getTimeFromGpsTowMs(self.getData(d, DID_GPS1_RTK_POS_REL, 'timeOfWeekMs'))
            rtkMiscTime = getTimeFromGpsTowMs(self.getData(d, miscDid, 'timeOfWeekMs'))
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

            for a in ax:
                for b in a:
                    b.grid(True)


        self.setup_and_wire_legend()
        return self.saveFigJoinAxes(ax, axs, fig, 'rtk'+name+'Misc')

    def rtkRel(self, fig=None, axs=None):
        # try:
        n_plots = 3
        if fig is None:
            fig = plt.figure()

        ax = fig.subplots(3, 1, sharex=True)
        fig.suptitle('RTK Rel - ' + os.path.basename(os.path.normpath(self.log.directory)))
        self.configureSubplot(ax[0], 'GPS Base to Rover Heading', '')
        self.configureSubplot(ax[1], 'GPS Base to Rover Distance', '')

        for i, d in enumerate(self.active_devs):
            rtkRelTime = getTimeFromGpsTowMs(self.getData(d, DID_GPS1_RTK_POS_REL, 'timeOfWeekMs'))
            ax[0].plot(rtkRelTime, self.getData(d, DID_GPS1_RTK_POS_REL, 'baseToRoverHeading')*RAD2DEG)
            ax[1].plot(rtkRelTime, self.getData(d, DID_GPS1_RTK_POS_REL, 'baseToRoverDistance'))

            for a in ax:
                a.grid(True)


        self.setup_and_wire_legend()
        return self.saveFigJoinAxes(ax, axs, fig, 'rtkRel')

    def gnssEphemeris(self, fig=None, axs=None):
        if fig is None:
            fig = plt.figure()

        # Build array of SV present in the logs
        sv = np.empty(0, dtype = int)
        for d in self.active_devs:
            satData1 = self.log.data[d, DID_GPS1_SAT]
            if satData1.size == 0:
                continue
            for data in satData1:
                rng = range(data['numSats'])
                gnss = data['sat'][rng]['gnssId']
                sat = data['sat'][rng]['svId']
                sat[gnss==3] = sat[gnss==3] + 32
                ind_new = ~np.isin(sat, sv)
                if any(ind_new):
                    sv = np.append(sv, sat[ind_new])
        Nsat = len(sv)
        if Nsat == 0:
            return
        sv = np.sort(sv)

        # Array of ephemeris counts (Nsat x samples x Ndevices)
        ephData = np.zeros([Nsat, len(satData1), len(self.active_devs)])
        for d in self.active_devs:
            satData1 = self.log.data[d, DID_GPS1_SAT]
            time = getTimeFromGpsTowMs(satData1['timeOfWeekMs'], 1)
            for i, data in enumerate(satData1):
                rng = range(data['numSats'])
                status = data['sat'][rng]['status'] >> 12 & 0x7
                gnss = data['sat'][rng]['gnssId']
                sat = data['sat'][rng]['svId']
                # convert SV prn to RTKlib prn: add 32 (max number of GPS satellites) to Galileo, assuming no GLONASS satellites in the data (PRN sequence: [GPS, Galileo])
                sat[gnss==3] = sat[gnss==3] + 32
                ephData[np.isin(sv,sat),i,d] = status[rng]

        # Delete SV that have zero ephemeris entries
        ind = ephData > 0
        del_ind = []
        for j, sat in enumerate(sv):
            if not np.any(ind[j,:,:]):
                del_ind.append(j)
        sv = np.delete(sv, del_ind)
        ephData = np.delete(ephData, (del_ind), axis=0)
        Nsat = len(sv)
        if Nsat == 0:
            return

        cols = 4
        rows = math.ceil(Nsat/float(cols))
        ax = fig.subplots(rows, cols, sharex=True)
        fig.suptitle('Ephemeris Counters - ' + os.path.basename(os.path.normpath(self.log.directory)))

        for d in self.active_devs:
            for j, sat in enumerate(sv):
                ax[j % rows, j // rows].set_title('SV '+ str(sat))
                ax[j % rows, j // rows].title.set_fontsize(8)
                ax[j % rows, j // rows].plot(time, ephData[j,:,d], label=self.log.serials[d])

        self.legends_add(ax[0,0].legend(ncol=2))
        for a in ax:
            for b in a:
                b.grid(True)
                b.yaxis.set_major_locator(MaxNLocator(integer=True))


    def loadGyros(self, device, useImu3=False):
        return self.loadIMU(device, accelSensor=0, useImu3=useImu3)

    def loadAccels(self, device, useImu3=False):
        return self.loadIMU(device, accelSensor=1, useImu3=useImu3)

    def loadIMU(self, device, accelSensor, useImu3=False):   # 0 = gyro, 1 = accelerometer
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

        if np.shape(imu1)[0] != 0 and not useImu3:  # DID_PIMU
            name = "PIMU"
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
            name = "Reference PIMU"

            if time.size > 5: # DID_REFERENCE_PIMU, ignore data if there are just a few RefIMU data points (logging bug?)
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
                name = "IMU"

                if len(time) != 0 and not useImu3:  # DID_IMU
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
                    name = "IMU3"

                    if len(time) != 0: # DID_IMU3_RAW 
                        I = self.getData(device, DID_IMU3_RAW, 'I')
                        imuStatus = self.getData(device, DID_IMU3_RAW, 'status')
                        dt = time[1:] - time[:-1]
                        dt = np.append(dt, dt[-1])
                        imu1 = []
                        imu2 = []
                        imu3 = []
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
            towOffset = self.getGpsTowOffset(device)
            if len(towOffset):
                time = getTimeFromGpsTow(time + np.mean(towOffset))
        # else: # HACK: to correct for improper SPAN INS direction and gyro scalar
        #     tmp = np.copy(imu1)   
        #     tmp *= 125.0 
        #     imu1[:,0] =  tmp[:,1]
        #     imu1[:,1] =  tmp[:,0]
        #     imu1[:,2] = -tmp[:,2]

        return (name, time, dt, imu1, imu2, imu3, imuCount)

    def imu3PQR(self, fig=None, axs=None):
        self.imuPQR(fig, axs, useImu3=True)

    def imu3Acc(self, fig=None, axs=None):
        self.imuAcc(fig, axs, useImu3=True)

    def imu3PqrCombined(self, fig=None, axs=None):
        self.imuPQR(fig, axs, useImu3=True, combineImu3=True)

    def imu3AccCombined(self, fig=None, axs=None):
        self.imuAcc(fig, axs, useImu3=True, combineImu3=True)

    def imuPQR(self, fig=None, axs=None, useImu3=False, combineImu3=False):
        if fig is None:
            fig = plt.figure()

        refTime = []
        refSnr = []
        for d in self.active_devs:
            refTime_ = self.getData(d, DID_REFERENCE_PIMU, 'time')
            # Ignore data if there are just a few RefIMU data points (logging bug?)
            if refTime_.size > 5:
                refTheta = self.getData(d, DID_REFERENCE_PIMU, 'theta')
                refDt = self.getData(d, DID_REFERENCE_PIMU, 'dt')
                refSnr.append(refTheta / refDt[:,None])
                refTime.append(refTime_)

        (name, time, dt, snr0, snr1, snr2, sensorCnt) = self.loadGyros(0, useImu3)
        fig.suptitle(name + ' PQR - ' + os.path.basename(os.path.normpath(self.log.directory)))

        plotResidual = (sensorCnt==1 or combineImu3) and self.residual 
        if sensorCnt:
            ax = fig.subplots(3, (2 if plotResidual else 1 if combineImu3 else sensorCnt), sharex=True, squeeze=False)
        if plotResidual:
            for d in self.active_devs:
                if self.log.serials[d] == 'Ref INS' or combineImu3:
                    (name, time, dt, snr0, snr1, snr2, sensorCnt) = self.loadGyros(d, useImu3)
                    refTime = time
                    if combineImu3:
                        refSnr = (snr0 + snr1 + snr2) / 3
                    else:
                        refSnr = snr0
                    continue

        for dev_idx, d in enumerate(self.active_devs):
            (name, time, dt, snr0, snr1, snr2, sensorCnt) = self.loadGyros(d, useImu3)
            if sensorCnt:
                for i in range(3):
                    axislable = 'P' if (i == 0) else 'Q' if (i==1) else 'R'
                    for n, snr in enumerate([ snr0, snr1, snr2 ]):
                        if n<sensorCnt:
                            if np.all(snr) is not None:
                                snr = quatRot(self.log.mount_bias_quat[d,:], snr)
                                mean = np.mean(snr[:, i])
                                std = np.std(snr[:, i])
                                alable = 'Gyro'
                                if sensorCnt > 1 and not combineImu3:
                                    alable += '%d ' % n
                                else:
                                    alable += ' '
                                label = str(self.log.serials[d]) + (["-0", "-1", "-2"][n] if combineImu3 else "")
                                if combineImu3:
                                    n = 0
                                self.configureSubplot(ax[i, n], alable + axislable + ' (deg/s), mean: %.4g, std: %.3g' % (mean*180.0/np.pi, std*180.0/np.pi), 'deg/s')                                
                                ax[i, n].plot(time, snr[:, i] * 180.0/np.pi, label=label)
                                if plotResidual and (len(refTime) != 0) and self.log.serials[d] != 'Ref INS':
                                    self.configureSubplot(ax[i,1], 'Residual', 'deg/2')
                                    intSnr = np.empty_like(refSnr)
                                    intSnr[:,i] = np.interp(refTime, time, snr[:,i], right=np.nan, left=np.nan)
                                    resSnr = intSnr - refSnr
                                    ax[i,1].plot(refTime, resSnr[:,i]*RAD2DEG, label=(label if dev_idx==0 else None))

        if not plotResidual:
            for dev_idx, d in enumerate(self.active_devs):
                if len(refTime) > 0 and len(refTime[d]) > 0: # and dev_idx == 0:    # Only plot reference IMU for first device
                    for i in range(3):
                        if dev_idx == 0:
                            plabel = 'reference'
                        else:
                            plabel = ''
                        ax[i, 0].plot(refTime[d], refSnr[d][:, i] * 180.0/np.pi, color='black', linestyle = 'dashed', label = plabel)

        if not 'ax' in locals():
            return

        for i in range((1 if combineImu3 else sensorCnt)):
            self.legends_add(ax[0][i].legend(ncol=2))
            if plotResidual:
                self.legends_add(ax[0,1].legend(ncol=2))
                for i in range(3):
                    self.setPlotYSpanMin(ax[i,1], 1.0)
        for a in ax:
            for b in a:
                b.grid(True)

        self.setup_and_wire_legend()
        return self.saveFigJoinAxes(ax, axs, fig, 'pqrIMU')

    def imuAcc(self, fig=None, axs=None, useImu3=False, combineImu3=False):
        if fig is None:
            fig = plt.figure()

        refTime = []
        refSnr = []
        for d in self.active_devs:
            refTime_ = self.getData(d, DID_REFERENCE_PIMU, 'time')
            if refTime_.size > 5:
                refVel = self.getData(d, DID_REFERENCE_PIMU, 'vel')
                refDt = self.getData(d, DID_REFERENCE_PIMU, 'dt')
                refSnr.append(refVel / refDt[:,None])
                refTime.append(refTime_)

        (name, time, dt, snr0, snr1, snr2, sensorCnt) = self.loadAccels(0, useImu3)
        fig.suptitle(name + ' Accelerometer - ' + os.path.basename(os.path.normpath(self.log.directory)))

        plotResidual = (sensorCnt==1 or combineImu3) and self.residual 
        if sensorCnt:
            ax = fig.subplots(3, (2 if plotResidual else 1 if combineImu3 else sensorCnt), sharex=True, squeeze=False)
        if plotResidual:
            for d in self.active_devs:
                if self.log.serials[d] == 'Ref INS' or combineImu3:
                    (name, time, dt, snr0, snr1, snr2, sensorCnt) = self.loadAccels(d, useImu3)
                    refTime = time
                    if combineImu3:
                        refSnr = (snr0 + snr1 + snr2) / 3
                    else:
                        refSnr = snr0
                    continue

        for dev_idx, d in enumerate(self.active_devs):
            (name, time, dt, snr0, snr1, snr2, sensorCnt) = self.loadAccels(d, useImu3)
            if sensorCnt:
                for i in range(3):
                    axislable = 'X' if (i == 0) else 'Y' if (i==1) else 'Z'
                    for n, snr in enumerate([ snr0, snr1, snr2 ]):
                        if n<sensorCnt:
                            if np.all(snr) is not None:
                                mean = np.mean(snr[:, i])
                                std = np.std(snr[:, i])
                                alable = 'Accel'
                                if sensorCnt > 1 and not combineImu3:
                                    alable += '%d ' % n
                                else:
                                    alable += ' '
                                label = str(self.log.serials[d]) + (["-0", "-1", "-2"][n] if combineImu3 else "")
                                if combineImu3:
                                    n = 0
                                self.configureSubplot(ax[i, n], alable + axislable + ' (m/s^2), mean: %.4g, std: %.3g' % (mean, std), 'm/s^2')
                                ax[i, n].plot(time, snr[:, i], label=label)
                                if plotResidual and (len(refTime) != 0) and self.log.serials[d] != 'Ref INS':
                                    self.configureSubplot(ax[i,1], 'Residual', 'm/s^2')
                                    intSnr = np.empty_like(refSnr)
                                    intSnr[:,i] = np.interp(refTime, time, snr[:,i], right=np.nan, left=np.nan)
                                    resSnr = intSnr - refSnr
                                    ax[i,1].plot(refTime, resSnr[:,i], label=(label if dev_idx==0 else None))

        if not plotResidual:
            for dev_idx, d in enumerate(self.active_devs):
                if len(refTime) > 0 and len(refTime[d]) > 0: # and dev_idx == 0:    # Only plot reference IMU for first device
                    for i in range(3):
                        if dev_idx == 0:
                            plabel = 'reference'
                        else:
                            plabel = ''
                        ax[i, 0].plot(refTime[d], refSnr[d][:, i], color='black', linestyle = 'dashed', label = plabel)

        if not 'ax' in locals():
            return

        for i in range((1 if combineImu3 else sensorCnt)):
            self.legends_add(ax[0][i].legend(ncol=2))
            if plotResidual:
                self.legends_add(ax[0,1].legend(ncol=2))
                for i in range(3):
                    self.setPlotYSpanMin(ax[i,1], 1.0)

        for a in ax:
            for b in a:
                b.grid(True)

        self.setup_and_wire_legend()
        return self.saveFigJoinAxes(ax, axs, fig, 'accIMU')

    def allanVariancePQR(self, fig=None, axs=None):
        if fig is None:
            fig = plt.figure()

        (name, time, dt, snr0, snr1, snr2, snrCount) = self.loadGyros(0)
        ax = fig.subplots(3, snrCount, sharex=True, squeeze=False)
        fig.suptitle('Allan Variance: PQR - ' + os.path.basename(os.path.normpath(self.log.directory)))

        sumARW = []
        sumBI = []

        # Set up the statistics lists
        for i in range(3):
            sumARW.append([])
            sumBI.append([])
            for n, pqr in enumerate([ snr0, snr1, snr2 ]):
                sumARW[i].append([])
                sumBI[i].append([])

        for d in self.active_devs:
            (name, time, dt, snr0, snr1, snr2, snrCount) = self.loadGyros(d)

            if snrCount:
                dtMean = np.mean(dt)
                for i in range(3):
                    for n, pqr in enumerate([ snr0, snr1, snr2 ]):
                        if np.all(pqr) != None and n<snrCount:
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
            for n, pqr in enumerate([ snr0, snr1, snr2 ]):
                if np.all(pqr) != None and n<snrCount:
                    alable = 'Gyro'
                    if snrCount > 1:
                        alable += '%d ' % n
                    else:
                        alable += ' '
                    self.configureSubplot(ax[i, n], alable + axislable + ' ($deg/hr$), ARW: %.3g $deg/\sqrt{hr}$,  BI: %.3g $deg/hr$' % (np.mean(sumARW[i][n]) + np.std(sumARW[i][n]), np.mean(sumBI[i][n]) + np.std(sumBI[i][n])), 'deg/hr')

        for i in range(snrCount):
            for d in range(3):
                ax[d][i].grid(True, which='both')
                self.legends_add(ax[d][i].legend(ncol=2))

        self.setup_and_wire_legend()

        with open(self.log.directory + '/allan_variance_pqr.csv', 'w') as f:
            f.write('Hardware,Date,SN,BI-P,BI-Q,BI-R,ARW-P,ARW-Q,ARW-R,BI-X\n')
            f.write(',,,(deg/hr),(deg/hr),(deg/hr),(deg / rt hr),(deg / rt hr),(deg / rt hr)\n')
            today = date.today()
            for d in self.active_devs:
                hdwVer = self.getData(d, DID_DEV_INFO, 'hardwareVer')[d]
                f.write('%d.%d.%d,%s,%d,' % (hdwVer[0], hdwVer[1], hdwVer[2], str(today), self.log.serials[d]))
                for n, pqr in enumerate([ snr0, snr1, snr2 ]):
                    if np.all(pqr) != None and n<snrCount:
                        for i in range(3):
                            f.write('%f,' % (sumBI[i][n][d]))
                        for i in range(3):
                            f.write('%f,' % (sumARW[i][n][d]))
                f.write('\n')

        return self.saveFigJoinAxes(ax, axs, fig, 'pqrIMU')

    def allanVarianceAcc(self, fig=None, axs=None):
        if fig is None:
            fig = plt.figure()

        (name, time, dt, snr0, snr1, snr2, snrCount) = self.loadAccels(0)
        ax = fig.subplots(3, snrCount, sharex=True, squeeze=False)
        fig.suptitle('Allan Variance: Accelerometer - ' + os.path.basename(os.path.normpath(self.log.directory)))

        sumRW = []
        sumBI = []

        # Set up the statistics lists
        for i in range(3):
            sumRW.append([])
            sumBI.append([])
            for n, pqr in enumerate([ snr0, snr1, snr2 ]):
                sumRW[i].append([])
                sumBI[i].append([])


        for d in self.active_devs:
            (namae, time, dt, snr0, snr1, snr2, snrCount) = self.loadAccels(d)

            dtMean = np.mean(dt)
            for i in range(3):
                for n, acc in enumerate([ snr0, snr1, snr2 ]):
                    if np.all(acc) != None and n<snrCount:
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
            for n, pqr in enumerate([ snr0, snr1, snr2 ]):
                if np.all(pqr) != None and n<snrCount:
                    alable = 'Accel'
                    if snrCount > 1:
                        alable += '%d ' % n
                    else:
                        alable += ' '
                    self.configureSubplot(ax[i, n], alable + axislable + ' ($m/s^2$), RW: %.3g $m/s/\sqrt{hr}$, BI: %.3g $m/s^2$' % (np.mean(sumRW[i][n]) + np.std(sumRW[i][n]), np.mean(sumBI[i][n]) + np.std(sumBI[i][n])), 'm/s^2')

        for i in range(snrCount):
            for d in range(3):
                ax[d][i].grid(True, which='both')
                self.legends_add(ax[d][i].legend(ncol=2))

        self.setup_and_wire_legend()

        with open(self.log.directory + '/allan_variance_acc.csv', 'w') as f:
            f.write('Hardware,Date,SN,BI-X,BI-Y,BI-Z,ARW-X,ARW-Y,ARW-Z\n')
            f.write(',,,(m/s^2 / hr),(m/s^2 / hr),(m/s^2 / hr),(m/s / rt hr),(m/s / rt hr),(m/s / rt hr)\n')
            today = date.today()
            for d in self.active_devs:
                hdwVer = self.getData(d, DID_DEV_INFO, 'hardwareVer')[d]
                f.write('%d.%d.%d,%s,%d,' % (hdwVer[0], hdwVer[1], hdwVer[2], str(today), self.log.serials[d]))
                for n, acc in enumerate([ snr0, snr1, snr2 ]):
                    if np.all(acc) != None and n<snrCount:
                        for i in range(3):
                            f.write('%f,' % (sumBI[i][n][d]))
                        for i in range(3):
                            f.write('%f,' % (sumRW[i][n][d]))
                f.write('\n')

        return self.saveFigJoinAxes(ax, axs, fig, 'accIMU')

    def accelPSD(self, fig=None, axs=None):
        if fig is None:
            fig = plt.figure()

        (name, time, dt, snr0, snr1, snr2, snrCount) = self.loadAccels(0)
        ax = fig.subplots(3, snrCount, sharex=True, squeeze=False)
        fig.suptitle(name + ' Power Spectral Density - ' + os.path.basename(os.path.normpath(self.log.directory)))
        
        for d in self.active_devs:
            (name, time, dt, snr0, snr1, snr2, snrCount) = self.loadAccels(d)
            refTime = self.getData(d, DID_REFERENCE_PIMU, 'time')
            if refTime.size > 5:
                refVel = self.getData(d, DID_REFERENCE_PIMU, 'vel')
                refDt = self.getData(d, DID_REFERENCE_PIMU, 'dt')
                refAcc = refVel / refDt[:,None]

            N = time.size
            psd = np.zeros((N//2, 3))
            # 1/T = frequency
            Fs = 1 / np.mean(dt)
            f = np.linspace(0, 0.5*Fs, N // 2)

            for n, acc in enumerate([ snr0, snr1, snr2 ]):
                if np.all(acc) != None and n<snrCount:
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
                        if snrCount > 1:
                            alable += '%d ' % n
                        else:
                            alable += ' '
                        self.configureSubplot(ax[i, n], alable + axislable + ' PSD (dB (m/s^2)^2/Hz)', 'Hz')
                        ax[i][n].plot(f, 10*np.log10(psd[:, i]), label=self.log.serials[d])

        for i in range(snrCount):
            self.legends_add(ax[0][i].legend(ncol=2))
            for d in range(3):
                ax[d][i].grid(True)

        self.setup_and_wire_legend()
        return self.saveFigJoinAxes(ax, axs, fig, 'accelPSD')

    def gyroPSD(self, fig=None, axs=None):
        if fig is None:
            fig = plt.figure()

        (name, time, dt, snr0, snr1, snr2, snrCount) = self.loadGyros(0)
        ax = fig.subplots(3, snrCount, sharex=True, squeeze=False)
        fig.suptitle(name + ' Power Spectral Density - ' + os.path.basename(os.path.normpath(self.log.directory)))
        
        for d in self.active_devs:
            (name, time, dt, snr0, snr1, snr2, snrCount) = self.loadGyros(d)
            refTime = self.getData(d, DID_REFERENCE_PIMU, 'time')
            if refTime.size > 5:
                refTheta = self.getData(d, DID_REFERENCE_PIMU, 'theta')
                refDt = self.getData(d, DID_REFERENCE_PIMU, 'dt')
                refGyr = refTheta / refDt[:,None]

            N = time.size
            Nhalf = N // 2 + 1
            psd = np.zeros((Nhalf, 3))
            # 1/T = frequency
            Fs = 1 / np.mean(dt)
            f = np.linspace(0, 0.5*Fs, Nhalf)

            for n, pqr in enumerate([ snr0, snr1, snr2 ]):
                if np.all(pqr) != None and n<snrCount:
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
                        if snrCount > 1:
                            alable += '%d ' % n
                        else:
                            alable += ' '
                        self.configureSubplot(ax[i, n], alable + axislable + ' PSD (dB dps^2/Hz)', 'Hz')
                        ax[i][n].plot(f, 10*np.log10(psd[:, i]), label=self.log.serials[d])

        for i in range(snrCount):
            self.legends_add(ax[0][i].legend(ncol=2))
            for d in range(3):
                ax[d][i].grid(True)

        self.setup_and_wire_legend()
        return self.saveFigJoinAxes(ax, None, fig, 'gyroPSD')

    def altitude(self, fig=None, axs=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(4, 1, sharex=True)

        self.configureSubplot(ax[0], 'Altitude: Barometer', 'm')
        self.configureSubplot(ax[1], 'Altitude: GPS', 'm')
        self.configureSubplot(ax[2], 'Altitude: INS', 'm')
        self.configureSubplot(ax[3], 'Altitude: Combined', 'm')
        fig.suptitle('Altitude - ' + os.path.basename(os.path.normpath(self.log.directory)))
        
        for d in self.active_devs:
            timeBar = self.getData(d, DID_BAROMETER, 'time')
            towOffset = self.getGpsTowOffset(d)
            timeGps = getTimeFromGpsTowMs(self.getData(d, DID_GPS1_POS, 'timeOfWeekMs'))
            llaGps = self.getData(d, DID_GPS1_POS, 'lla')
            if len(llaGps) > 0:
                altGps = llaGps[:, 2]
            else:
                altGps = []
            timeIns = getTimeFromGpsTow(self.getData(d, DID_INS_2, 'timeOfWeek'), True)
            lla = self.getData(d, DID_INS_2, 'lla', True)
            if len(lla) > 0:
                altIns = lla[:, 2]
            else:
                altIns = []
            if np.shape(towOffset)[0] != 0:
                timeBar = timeBar + towOffset[-1]
            mslBar = self.getData(d, DID_BAROMETER, 'mslBar')

            ax[0].plot(timeBar, mslBar, label=self.log.serials[d])
            if len(timeGps) == len(altGps) and len(altGps) > 0:
                ax[1].plot(timeGps, altGps)
            if len(timeIns) == len(altIns) and len(altIns) > 0:
                ax[2].plot(timeIns, altIns)
            if len(altGps) > 0:
                ax[3].plot(timeBar, mslBar - (mslBar[0] - altGps[0]), label=("Bar %s" % self.log.serials[d]))
                ax[3].plot(timeGps, altGps, label=("GPS %s" % self.log.serials[d]))
            elif len(mslBar) > 0:
                ax[3].plot(timeBar, mslBar, label=("Bar %s" % self.log.serials[d]))

        self.legends_add(ax[0].legend(ncol=2))
        self.legends_add(ax[3].legend(ncol=2))
        for a in ax:
            a.grid(True)

        self.setup_and_wire_legend()
        return self.saveFigJoinAxes(ax, axs, fig, 'altitude')

    def climbRate(self, fig=None, axs=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(3, 1, sharex=True)

        self.configureSubplot(ax[0], 'Climb Rate: Barometer', 'm/s')
        self.configureSubplot(ax[1], 'Climb Rate: GPS', 'm/s')
        self.configureSubplot(ax[2], 'Climb Rate: INS', 'm/s')
        fig.suptitle('Climb Rate: ' + os.path.basename(os.path.normpath(self.log.directory)))
        for d in self.active_devs:
            timeBar = self.getData(d, DID_BAROMETER, 'time')
            mslBar  = self.getData(d, DID_BAROMETER, 'mslBar')
            timeGps = getTimeFromGpsTowMs(self.getData(d, DID_GPS1_POS, 'timeOfWeekMs'))
            llaGps = self.getData(d, DID_GPS1_POS, 'lla')
            if len(llaGps) > 0:
                altGps = llaGps[:, 2]
            else:
                altGps = []
            timeIns = getTimeFromGpsTow(self.getData(d, DID_INS_2, 'timeOfWeek'), True)
            llaIns = self.getData(d, DID_INS_2, 'lla', True)
            if len(llaIns) > 0:
                altIns = llaIns[:, 2]
            else:
                altIns = []
            towOffset = self.getGpsTowOffset(d)
            if len(towOffset) > 0:
                timeBar = timeBar + towOffset[-1]
            if len(timeBar) > 2:
                climbBar = np.gradient(mslBar, timeBar)
                ax[0].plot(timeBar, climbBar, label=self.log.serials[d])
            if len(timeGps) > 2 and len(altGps) == len(timeGps):
                climbGps = np.gradient(altGps, timeGps)
                ax[1].plot(timeGps, climbGps)
            if len(timeIns) > 2 and len(altIns) == len(timeIns):
                climbIns = np.gradient(altIns, timeIns)
                ax[2].plot(timeIns, climbIns)

        ax[0].legend(ncol=2)
        for a in ax:
            a.grid(True)
        return self.saveFigJoinAxes(ax, axs, fig, 'climbrate')

    def barometer(self, fig=None, axs=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(3, 1, sharex=True)

        self.configureSubplot(ax[0], 'Baro MSL', 'm')
        self.configureSubplot(ax[1], 'Baro Temp', 'C')
        self.configureSubplot(ax[2], 'Baro Humidity', '%%rH')
        fig.suptitle('Barometer - ' + os.path.basename(os.path.normpath(self.log.directory)))
        for d in self.active_devs:
            if 1:
                time = self.getData(d, DID_BAROMETER, 'time')
                towOffset = self.getGpsTowOffset(d)
                if np.shape(towOffset)[0] != 0:
                    time = getTimeFromGpsTow(time + np.mean(towOffset))
                mslBar = self.getData(d, DID_BAROMETER, 'mslBar')
                barTemp = self.getData(d, DID_BAROMETER, 'barTemp')
                humidity = self.getData(d, DID_BAROMETER, 'humidity')
            ax[0].plot(time, mslBar, label=self.log.serials[d])
            ax[1].plot(time, barTemp)
            ax[2].plot(time, humidity)

        ax[0].legend(ncol=2)
        for a in ax:
            a.grid(True)
        return self.saveFigJoinAxes(ax, axs, fig, 'barometer')

    def magnetometer(self, fig=None, axs=None):
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
                towOffset = self.getGpsTowOffset(d)
                if np.shape(towOffset)[0] != 0:
                    time = getTimeFromGpsTow(time + np.mean(towOffset))
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

        self.legends_add(ax[0].legend(ncol=2))
        for a in ax:
            a.grid(True)

        self.setup_and_wire_legend()
        return self.saveFigJoinAxes(ax, axs, fig, 'magnetometer')


    def temp(self, fig=None, axs=None):
        try:
            if fig is None:
                fig = plt.figure()
            ax = fig.subplots(3, 1, sharex=True)
            fig.suptitle('Temperature - ' + os.path.basename(os.path.normpath(self.log.directory)))

            self.configureSubplot(ax[0], 'IMU Temperature (C)')
            self.configureSubplot(ax[1], 'Barometer Temperature (C)')
            self.configureSubplot(ax[2], 'MCU Temperature (C)')

            for d in self.active_devs:
                time = getTimeFromGpsTowMs(self.getData(d, DID_SYS_PARAMS, 'timeOfWeekMs'), True)
                tempImu = self.getData(d, DID_SYS_PARAMS, 'imuTemp')
                tempBar = self.getData(d, DID_SYS_PARAMS, 'baroTemp')
                tempMcu = self.getData(d, DID_SYS_PARAMS, 'mcuTemp')
                towOffset = self.getGpsTowOffset(d)
                if np.shape(towOffset)[0] != 0:
                    tempImu = getTimeFromGpsTow(tempImu + np.mean(towOffset))
                    tempBar = getTimeFromGpsTow(tempBar + np.mean(towOffset))
                    tempMcu = getTimeFromGpsTow(tempMcu + np.mean(towOffset))
                
                ax[0].plot(time, tempImu, label=self.log.serials[d])
                ax[1].plot(time, tempBar)
                ax[2].plot(time, tempMcu)
            for a in ax:
                a.grid(True)

            self.setup_and_wire_legend()
            return self.saveFigJoinAxes(ax, axs, fig, 'Temp')
        except:
            print(RED + "problem plotting temp: " + sys.exc_info()[0] + RESET)

    def debugfArr(self, fig=None, axs=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(5,2, sharex=True)
        fig.suptitle('Debug float Array - ' + os.path.basename(os.path.normpath(self.log.directory)))
        for d in self.active_devs:
            debug_f = self.getData(d, DID_DEBUG_ARRAY, 'f')
            for i in range(9):
                ax[i%5, i//5].set_ylabel('f[' + str(i) +']')
                ax[i%5, i//5].plot(debug_f[:,i], label=self.log.serials[d])
        self.legends_add(ax[0,0].legend(ncol=2))
        for b in ax:
            for a in b:
                a.grid(True)

    def debugiArr(self, fig=None, axs=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(5,2, sharex=True)
        fig.suptitle('Debug int array - ' + os.path.basename(os.path.normpath(self.log.directory)))
        for d in self.active_devs:
            debug_i = self.getData(d, DID_DEBUG_ARRAY, 'i')
            for i in range(9):
                ax[i%5, i//5].set_ylabel('i[' + str(i) +']')
                ax[i%5, i//5].plot(debug_i[:,i], label=self.log.serials[d])
        self.legends_add(ax[0,0].legend(ncol=2))
        for b in ax:
            for a in b:
                a.grid(True)

    def debuglfArr(self, fig=None, axs=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(3,1, sharex=True)
        fig.suptitle('Debug double Array - ' + os.path.basename(os.path.normpath(self.log.directory)))
        for d in self.active_devs:
            debug_lf = self.getData(d, DID_DEBUG_ARRAY, 'lf')
            for i in range(3):
                ax[i].set_ylabel('lf[' + str(i) +']')
                ax[i].plot(debug_lf[:,i], label=self.log.serials[d])
        self.legends_add(ax[0].legend(ncol=2))
        for a in ax:
            a.grid(True)

    def gpxDebugfArray(self, fig=None, axs=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(5,2, sharex=True)
        fig.suptitle('GPX Debug float Array - ' + os.path.basename(os.path.normpath(self.log.directory)))
        for d in self.active_devs:
            debug_f = self.getData(d, DID_GPX_DEBUG_ARRAY, 'f')
            for i in range(9):
                ax[i%5, i//5].set_ylabel('f[' + str(i) +']')
                ax[i%5, i//5].plot(debug_f[:,i], label=self.log.serials[d])
        self.legends_add(ax[0,0].legend(ncol=2))
        for b in ax:
            for a in b:
                a.grid(True)

    def gpxDebugiArray(self, fig=None, axs=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(5,2, sharex=True)
        fig.suptitle('GPX Debug int array - ' + os.path.basename(os.path.normpath(self.log.directory)))
        for d in self.active_devs:
            debug_i = self.getData(d, DID_GPX_DEBUG_ARRAY, 'i')
            for i in range(9):
                ax[i%5, i//5].set_ylabel('i[' + str(i) +']')
                ax[i%5, i//5].plot(debug_i[:,i], label=self.log.serials[d])
        self.legends_add(ax[0,0].legend(ncol=2))
        for b in ax:
            for a in b:
                a.grid(True)

    def magDec(self, fig=None, axs=None):
        if fig is None:
            fig = plt.figure()

        ax = fig.subplots(2, 1, sharex=True)
        fig.suptitle('Magnetometer Declination - ' + os.path.basename(os.path.normpath(self.log.directory)))
        self.configureSubplot(ax[0], 'Declination', 'deg')
        self.configureSubplot(ax[1], 'Inclination', 'deg')

        for d in self.active_devs:
            time = getTimeFromGpsTow(self.getData(d, DID_INL2_STATES, 'timeOfWeek'), True)
            mag_declination = 180.0/np.pi * self.getData(d, DID_INL2_STATES, 'magDec')
            mag_inclination = 180.0/np.pi * self.getData(d, DID_INL2_STATES, 'magInc')
            ax[0].plot(time, mag_declination, label=self.log.serials[d])
            ax[1].plot(time, mag_inclination)
        self.legends_add(ax[0].legend(ncol=2))

        for a in ax:
            a.grid(True)

        self.setup_and_wire_legend()
        return self.saveFigJoinAxes(ax, axs, fig, 'magDec')

    def deltatime(self, fig=None, axs=None):
        if fig is None:
            fig = plt.figure()

        refImuPresent = False
        for d in self.active_devs:
            timeRef = self.getData(d, DID_REFERENCE_PIMU, 'time')
            if np.any(timeRef):
                refImuPresent = True

        N = 6
        if refImuPresent:
            N = N + 2
        ax = fig.subplots(N, 1, sharex=(self.xAxisSample==0))

        fig.suptitle('Timestamps - ' + os.path.basename(os.path.normpath(self.log.directory)))
        self.configureSubplot(ax[0], 'INS dt', 's')
        self.configureSubplot(ax[1], 'GPS1 dt', 's')
        self.configureSubplot(ax[2], 'GPS2 dt', 's')
        self.configureSubplot(ax[3], 'IMU3 Delta Timestamp', 's')
        self.configureSubplot(ax[4], 'PIMU Delta Timestamp', 's')
        self.configureSubplot(ax[5], 'PIMU Integration Period', 's', xlabel = 'Message Index' if self.xAxisSample else 'Time of Week')

        for d in self.active_devs_no_ref:
            dtIns = self.getData(d, DID_INS_2, 'timeOfWeek')[1:] - self.getData(d, DID_INS_2, 'timeOfWeek')[0:-1]
            dtIns = dtIns / self.d
            timeIns = getTimeFromGpsTow(self.getData(d, DID_INS_2, 'timeOfWeek')[1:], True)

            towMsGps1 = self.getData(d, DID_GPS1_POS, 'timeOfWeekMs')[1:]
            towMsGps2 = self.getData(d, DID_GPS2_POS, 'timeOfWeekMs')[1:]
            dtGps1 = 0.001*(towMsGps1 - self.getData(d, DID_GPS1_POS, 'timeOfWeekMs')[0:-1])
            dtGps2 = 0.001*(towMsGps2 - self.getData(d, DID_GPS2_POS, 'timeOfWeekMs')[0:-1])
            dtGps1 = dtGps1 / self.d
            dtGps2 = dtGps2 / self.d
            timeGps1 = getTimeFromGpsTowMs(towMsGps1)
            timeGps2 = getTimeFromGpsTowMs(towMsGps2)

            towOffset = self.getGpsTowOffset(d)
            if np.size(towOffset) > 0:
                towOffset = towOffset[-1]
            else:
                towOffset = 0

            deltaTimestamp = 0
            timeImu  = 0
            timePimu = self.getData(d, DID_PIMU, 'time')
            timeIMU  = self.getData(d, DID_IMU, 'time')
            timeImu3 = self.getData(d, DID_IMU3_RAW, 'time')
            if timePimu.size:
                deltaTimestamp = timePimu[1:] - timePimu[0:-1]
                deltaTimestamp = deltaTimestamp / self.d
                timeImu = getTimeFromGpsTow(timePimu[1:] + towOffset)            
                dtPimu = self.getData(d, DID_PIMU, 'dt')[1:]
            elif timeIMU.size:
                deltaTimestamp = timeIMU[1:] - timeIMU[0:-1]
                deltaTimestamp = deltaTimestamp / self.d
                timeImu = getTimeFromGpsTow(timeIMU[1:] + towOffset)
            if timeImu3.size:
                deltaImu3Timestamp = timeImu3[1:] - timeImu3[0:-1]
                deltaImu3Timestamp = deltaImu3Timestamp / self.d
                timeImu3 = getTimeFromGpsTow(timeImu3[1:] + towOffset)

            if self.xAxisSample:
                xIns  = np.arange(0, np.shape(dtIns)[0])
                xGps1 = np.arange(0, np.shape(dtGps1)[0])
                xGps2 = np.arange(0, np.shape(dtGps2)[0])
                xImu3 = np.arange(0, np.shape(deltaImu3Timestamp)[0])
                xImu  = np.arange(0, np.shape(deltaTimestamp)[0])
            else:
                xIns  = timeIns
                xGps1 = timeGps1
                xGps2 = timeGps2
                xImu3 = timeImu3
                xImu  = timeImu

            ax[0].plot(xIns, dtIns, label=self.log.serials[d])
            ax[1].plot(xGps1, dtGps1)
            ax[2].plot(xGps2, dtGps2)
            if xImu3.size > 0:
                ax[3].plot(xImu3, deltaImu3Timestamp)
            ax[4].plot(xImu, deltaTimestamp)
            if 'dtPimu' in locals() and dtPimu.size:
                ax[5].plot(xImu, dtPimu)

            self.configureSubplot(ax[0],  f'INS dt: {np.mean(dtIns):.3f}s', 's')
            self.configureSubplot(ax[1], f'GPS1 dt: {np.mean(dtGps1):.3f}s', 's')
            self.configureSubplot(ax[2], f'GPS2 dt: {np.mean(dtGps2):.3f}s', 's')
            if 'deltaImu3Timestamp' in locals() and deltaImu3Timestamp.size > 0:
                self.configureSubplot(ax[3], f'IMU3 Delta Timestamp: {np.mean(deltaImu3Timestamp):.3f}s', 's')
            self.configureSubplot(ax[4], f'PIMU Delta Timestamp: {np.mean(deltaTimestamp):.3f}s', 's')
            if 'dtPimu' in locals() and dtPimu.size:
                self.configureSubplot(ax[5], f'PIMU Integration Period: {np.mean(deltaTimestamp):.3f}s', 's', xlabel = 'Message Index' if self.xAxisSample else 'Time of Week')

        # Don't zoom in closer than 0.005s so we can easily see that the delta time is clean
        for i in range(len(ax)):
            self.setPlotYSpanMin(ax[i], 0.005)

        if refImuPresent:
            self.configureSubplot(ax[6], 'Reference IMU Integration Period', 's')
            self.configureSubplot(ax[7], 'Reference IMU Delta Timestamp', 's')
            for d in self.active_devs:
                deltaTimestampRef = 0
                timeImuRef = 0
                timeRef = self.getData(d, DID_REFERENCE_PIMU, 'time')
                if np.any(timeRef):
                    integrationPeriodRef = self.getData(d, DID_REFERENCE_PIMU, 'dt')[1:]
                    deltaTimestampRef = timeRef[1:] - timeRef[0:-1]
                    deltaTimestampRef = deltaTimestampRef / self.d
                    timeImuRef = getTimeFromGpsTow(timeRef[1:] + towOffset)
                    ax[5].plot(timeImuRef, integrationPeriodRef)
                    ax[6].plot(timeImuRef, deltaTimestampRef)

        self.legends_add(ax[0].legend(ncol=2))
        for a in ax:
            a.grid(True)

        self.setup_and_wire_legend()
        return self.saveFigJoinAxes(ax, axs, fig, 'deltatime')

    def gpsTime(self, fig=None, axs=None):
        if fig is None:
            fig = plt.figure()

        refImuPresent = False
        for d in self.active_devs:
            timeRef = self.getData(d, DID_REFERENCE_PIMU, 'time')
            if np.any(timeRef):
                refImuPresent = True

        N = 5
        ax = fig.subplots(N, 1, sharex=(self.xAxisSample==0))

        fig.suptitle('Timestamps - ' + os.path.basename(os.path.normpath(self.log.directory)))
        self.configureSubplot(ax[0], 'GPS1 dt', 's')
        self.configureSubplot(ax[1], 'GPS2 dt', 's')
        self.configureSubplot(ax[2], 'RTK Compassing dt', 's')
        self.configureSubplot(ax[3], 'GPS1 TOW Offset', 's')
        self.configureSubplot(ax[4], 'GPS2 TOW Offset', 's')

        for d in self.active_devs_no_ref:
            towMsGps1 = self.getData(d, DID_GPS1_POS, 'timeOfWeekMs')[1:]
            towMsGps2 = self.getData(d, DID_GPS2_POS, 'timeOfWeekMs')[1:]
            towMsRtk2 = self.getData(d, DID_GPS2_RTK_CMP_REL, 'timeOfWeekMs')[1:]
            dtGps1 = 0.001*(towMsGps1 - self.getData(d, DID_GPS1_POS, 'timeOfWeekMs')[0:-1])
            dtGps2 = 0.001*(towMsGps2 - self.getData(d, DID_GPS2_POS, 'timeOfWeekMs')[0:-1])
            dtRtk2 = 0.001*(towMsRtk2 - self.getData(d, DID_GPS2_RTK_CMP_REL, 'timeOfWeekMs')[0:-1])
            dtGps1 = dtGps1 / self.d
            dtGps2 = dtGps2 / self.d
            dtRtk2 = dtRtk2 / self.d
            timeGps1 = getTimeFromGpsTowMs(towMsGps1)
            timeGps2 = getTimeFromGpsTowMs(towMsGps2)
            timeRtk2 = getTimeFromGpsTowMs(towMsRtk2)

            towOffsetGps1 = self.getData(d, DID_GPS1_POS, 'towOffset')[1:]
            towOffsetGps2 = self.getData(d, DID_GPS2_POS, 'towOffset')[1:]

            if self.xAxisSample:
                xGps1 = np.arange(0, np.shape(dtGps1)[0])
                xGps2 = np.arange(0, np.shape(dtGps2)[0])
                xRtk2 = np.arange(0, np.shape(dtRtk2)[0])
            else:
                xGps1 = timeGps1
                xGps2 = timeGps2
                xRtk2 = timeRtk2

            ax[0].plot(xGps1, dtGps1)
            ax[1].plot(xGps2, dtGps2)
            ax[2].plot(xRtk2, dtRtk2)
            ax[3].plot(xGps1, towOffsetGps1)
            ax[4].plot(xGps2, towOffsetGps2)

            self.configureSubplot(ax[0],  f'GPS1 dt: {np.mean(dtGps1):.3f}s', 's')
            self.configureSubplot(ax[1],  f'GPS2 dt: {np.mean(dtGps2):.3f}s', 's')
            self.configureSubplot(ax[2],  f'RTK Compassing dt: {np.mean(dtRtk2):.3f}s', 's')


        # Don't zoom in closer than 0.005s so we can easily see that the delta time is clean
        for i in range(len(ax)):
            self.setPlotYSpanMin(ax[i], 0.005)

        self.legends_add(ax[0].legend(ncol=2))
        for a in ax:
            a.grid(True)

        self.setup_and_wire_legend()
        return self.saveFigJoinAxes(ax, axs, fig, 'gpstime')

    def gpsRawTime(self, fig=None, axs=None):
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

        self.legends_add(ax[0].legend(ncol=2))
        for a in ax:
            a.grid(True)

        self.setup_and_wire_legend()
        return self.saveFigJoinAxes(ax, axs, fig, 'gpsRawTime')

    def ekfBiases(self, fig=None, axs=None):
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
            time = getTimeFromGpsTow(self.getData(d, DID_INL2_STATES, 'timeOfWeek'), True)
            ax[0,0].plot(time, self.getData(d, DID_INL2_STATES, 'biasPqr')[:, 0]*180.0/np.pi, label=self.log.serials[d])
            ax[1,0].plot(time, self.getData(d, DID_INL2_STATES, 'biasPqr')[:, 1]*180.0/np.pi)
            ax[2,0].plot(time, self.getData(d, DID_INL2_STATES, 'biasPqr')[:, 2]*180.0/np.pi)
            ax[3,0].plot(time, self.getData(d, DID_INL2_STATES, 'biasBaro'), label=self.log.serials[d])

            ax[0,1].plot(time, self.getData(d, DID_INL2_STATES, 'biasAcc')[:, 0], label=self.log.serials[d])
            ax[1,1].plot(time, self.getData(d, DID_INL2_STATES, 'biasAcc')[:, 1])
            ax[2,1].plot(time, self.getData(d, DID_INL2_STATES, 'biasAcc')[:, 2])

        self.legends_add(ax[0,0].legend(ncol=2))
        for a in ax:
            for b in a:
                b.grid(True)

        self.setup_and_wire_legend()
        return self.saveFigJoinAxes(ax, axs, fig, 'ekfBiases')

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
        self.legends_add(ax[0].legend(ncol=2))

    def rtkDebugP1(self, fig=None, axs=None):
        if fig is None:
            fig = plt.figure()

        rtkData = self.log.data[0, DID_RTK_DEBUG]
        if rtkData.size == 0:
            return 
        fields = list(rtkData.dtype.names)
        fields.remove('time')
        num_plots = 0
        for f in range(0, math.ceil(len(fields)/2)):
            field = fields[f]
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
            for f in range(0, math.ceil(len(fields)/2)):
                field = fields[f]
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
        self.legends_add(ax[0,0].legend(ncol=2))
        for a in ax:
            for b in a:
                b.grid(True)

    def rtkDebugP2(self, fig=None, axs=None):
        if fig is None:
            fig = plt.figure()

        rtkData = self.log.data[0, DID_RTK_DEBUG]
        if rtkData.size == 0:
            return 
        fields = list(rtkData.dtype.names)
        fields.remove('time')
        num_plots = 0
        for f in range(math.ceil(len(fields)/2), len(fields)):
            field = fields[f]
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
            for f in range(math.ceil(len(fields)/2), len(fields)):
                field = fields[f]
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
        self.legends_add(ax[0,0].legend(ncol=2))
        for a in ax:
            for b in a:
                b.grid(True)

    def rtkDebug2(self, fig=None, axs=None):
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
        self.legends_add(ax[r1,c1].legend(ncol=2))

        for a in ax:
            for b in a:
                b.grid(True)

    def rtkDebug2Sat(self, fig=None, axs=None):
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
        self.legends_add(ax[r1,c1].legend(ncol=2))

        for a in ax:
            for b in a:
                b.grid(True)

    def rtkDebug2Std(self, fig=None, axs=None):
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
        self.legends_add(ax[r1,c1].legend(ncol=2))

        for a in ax:
            for b in a:
                b.grid(True)

    def rtkDebug2Lock(self, fig=None, axs=None):
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
        self.legends_add(ax[r1,c1].legend(ncol=2))

        for a in ax:
            for b in a:
                b.grid(True)

    def wheelEncoder(self, fig=None, axs=None):
        if fig is None:
            fig = plt.figure()

        fig.suptitle('Wheel Encoder - ' + os.path.basename(os.path.normpath(self.log.directory)))
        ax = fig.subplots(4, 1, sharex=True)
        titles = ['Left Wheel Angle', 'Right Wheel Angle', 'Left Wheel Velocity', 'Right Wheel Velocity']
        fields = ['theta_l', 'theta_r', 'omega_l', 'omega_r']

        for d in self.active_devs:
            time = getTimeFromGpsTow(self.getData(d, DID_WHEEL_ENCODER, 'timeOfWeek'), True)
            if len(time) == 0:
                # No sensor data in the log
                continue
            for i, a in enumerate(ax):
                a.plot(time, self.getData(d, DID_WHEEL_ENCODER, fields[i]), label=self.log.serials[d])
                if i == 0:
                    self.legends_add(a.legend(ncol=2))

        for i, a in enumerate(ax):
            a.set_ylabel(fields[i])
            a.set_title(titles[i])
            a.grid(True)

    def groundVehicleStatus(self, fig=None, axs=None):
        if fig is None:
            fig = plt.figure()

        fig.suptitle('Ground Vehicle - ' + os.path.basename(os.path.normpath(self.log.directory)))
        ax = fig.subplots(3, 2, sharex=True)

        ax[0,0].set_title('Learning Enabled')
        ax[0,1].set_title('Dead Reckoning')
        ax[1,0].set_title('Kinematic Calibration Good')
        ax[1,1].set_title('Learning Converged')
        ax[2,0].set_title('Learning Needed')
        ax[2,1].set_title('Mode')

        for d in self.active_devs:
            time = getTimeFromGpsTowMs(self.getData(d, DID_GROUND_VEHICLE, 'timeOfWeekMs'), True)
            if len(time) == 0:
                # No sensor data in the log
                continue
            wheelConfig = self.getData(d, DID_GROUND_VEHICLE, 'wheelConfig')
            iStatus = self.getData(d, DID_GROUND_VEHICLE, 'status')
            ax[0,0].plot(time, (iStatus & 0x00000001) != 0) # Kinematic learing is solving for the translation from IMU to wheel (wheel_config)
            ax[0,1].plot(time, (iStatus & 0x01000000) != 0) # Navigation is running without GPS input
            ax[1,0].plot(time, (iStatus & 0x02000000) != 0) # Vehicle kinematic parameters agree with GPS
            ax[1,1].plot(time, (iStatus & 0x04000000) != 0) # Vehicle kinematic learning has converged and is complete
            ax[2,0].plot(time, (iStatus & 0x08000000) != 0) # Vehicle kinematic learning data (wheel_config_t) is missing
            ax[2,1].plot(time, self.getData(d, DID_GROUND_VEHICLE, 'mode'))

        # Show serial numbers
        ax[0,0].legend(ncol=2)

        for a in ax:
            for b in a:
                b.grid(True)

    def groundVehicle(self, fig=None, axs=None):
        if fig is None:
            fig = plt.figure()

        fig.suptitle('Ground Vehicle - ' + os.path.basename(os.path.normpath(self.log.directory)))
        ax = fig.subplots(7, 2, sharex=True)

        ax[0,0].set_title('e_b2w')
        ax[0,1].set_title('e_b2w_sigma')
        ax[3,0].set_title('t_b2w')
        ax[3,1].set_title('t_b2w_sigma')
        ax[6,0].set_title('Radius')
        ax[6,1].set_title('Track Width')

        for d in self.active_devs:
            time = getTimeFromGpsTowMs(self.getData(d, DID_GROUND_VEHICLE, 'timeOfWeekMs'), True)
            if len(time) == 0:
                # No sensor data in the log
                continue
            wheelConfig = self.getData(d, DID_GROUND_VEHICLE, 'wheelConfig')
            ax[0,0].plot(time, wheelConfig['transform']['e_b2w'][:, 0], label=self.log.serials[d])
            ax[1,0].plot(time, wheelConfig['transform']['e_b2w'][:, 1])
            ax[2,0].plot(time, wheelConfig['transform']['e_b2w'][:, 2])
            ax[0,1].plot(time, wheelConfig['transform']['e_b2w_sigma'][:, 0], label=self.log.serials[d])
            ax[1,1].plot(time, wheelConfig['transform']['e_b2w_sigma'][:, 1])
            ax[2,1].plot(time, wheelConfig['transform']['e_b2w_sigma'][:, 2])

            ax[3,0].plot(time, wheelConfig['transform']['t_b2w'][:, 0], label=self.log.serials[d])
            ax[4,0].plot(time, wheelConfig['transform']['t_b2w'][:, 1])
            ax[5,0].plot(time, wheelConfig['transform']['t_b2w'][:, 2])
            ax[3,1].plot(time, wheelConfig['transform']['t_b2w_sigma'][:, 0], label=self.log.serials[d])
            ax[4,1].plot(time, wheelConfig['transform']['t_b2w_sigma'][:, 1])
            ax[5,1].plot(time, wheelConfig['transform']['t_b2w_sigma'][:, 2])

            ax[6,0].plot(time, wheelConfig['radius'])
            ax[6,1].plot(time, wheelConfig['track_width'])

        # Show serial numbers
        self.legends_add(ax[0,0].legend(ncol=2))

        for a in ax:
            for b in a:
                b.grid(True)

    def wheelControllerTime(self, fig=None, axs=None):
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
            if len(time) == 0:
                # No data
                continue
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

    def wheelControllerVel(self, fig=None, axs=None):
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
            if len(time) == 0:
                # No data
                continue
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

    def sensorCompGyrTemp(self, fig=None, axs=None):
        if fig is None:
            fig = plt.figure()
        self.sensorCompGen(fig, 'pqr', useTemp=True)

    def sensorCompAccTemp(self, fig=None, axs=None):
        if fig is None:
            fig = plt.figure()
        self.sensorCompGen(fig, 'acc', useTemp=True)

    def sensorCompMagTemp(self, fig=None, axs=None):
        if fig is None:
            fig = plt.figure()
        self.sensorCompGen(fig, 'mag', useTemp=True)

    def sensorCompGyr(self, fig=None, axs=None):
        if fig is None:
            fig = plt.figure()
        self.sensorCompGen(fig, 'pqr')

    def sensorCompAcc(self, fig=None, axs=None):
        if fig is None:
            fig = plt.figure()
        self.sensorCompGen(fig, 'acc')

    def sensorCompMag(self, fig=None, axs=None):
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
            if useTemp:
                ax[3, i].set_title('Temperature %s %d' % (name, i))
            else:
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
                    if not useTemp:
                        ax[3,i].plot(x, temp)
                    else:
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
        self.legends_add(ax[0,0].legend(ncol=2))

        for a in ax:
            for b in a:
                b.grid(True)
                lim = b.get_ylim()
                mid = 0.5 * (lim[0] + lim[1])
                span = lim[1] - lim[0]
                span = max(span, yspan)
                b.set_ylim([mid-span/2, mid+span/2])

    def linearityAcc(self, fig=None, axs=None):
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
        self.legends_add(ax[0,0].legend(ncol=2))

        yspan = 0.5 # m/s^2 

        for a in ax:
            for b in a:
                b.grid(True)
                # lim = b.get_ylim()
                # mid = 0.5 * (lim[0] + lim[1])
                # span = lim[1] - lim[0]
                # span = max(span, yspan)
                # b.set_ylim([mid-span/2, mid+span/2])

    def linearityGyr(self, fig=None, axs=None):
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
        self.legends_add(ax[0,0].legend(ncol=2))

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
    file = open(home + "/Documents/Inertial_Sense/log_inspector.yaml", 'r')
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

import math
from typing import List, Any, Union

import numpy as np
import matplotlib.pyplot as plt
import sys
import yaml
import os
from os.path import expanduser

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
        self.directory = log.directory
        self.format = format
        self.log = log
        self.d = 1
        self.setActiveSerials(self.log.serials)

        if len(self.log.data[0, DID_INS_2]):
            setGpsWeek(self.log.data[0, DID_INS_2]['week'][-1])
            
    def setDownSample(self, dwns):
        self.d = dwns

    def setActiveSerials(self, serials):
        self.active_devs = []
        for d, ser in enumerate(self.log.serials):
            if ser in serials:
                self.active_devs.append(d)

    def configureSubplot(self, ax, title, xlabel):
        ax.set_title(title)
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
        ax = fig.subplots(3,1, sharex=True)
        self.configureSubplot(ax[0], 'North', 'm')
        self.configureSubplot(ax[1], 'East', 'm')
        self.configureSubplot(ax[2], 'Down', 'm')
        fig.suptitle('INS NED - ' + os.path.basename(os.path.normpath(self.log.directory)))
        refLla = None
        for d in self.active_devs:
            if refLla is None:
                refLla = self.getData(d, DID_INS_2, 'lla')[0]
            ned = lla2ned(refLla, self.getData(d, DID_INS_2, 'lla'))
            time = getTimeFromTow(self.getData(d, DID_INS_2, 'timeOfWeek'))
            ax[0].plot(time, ned[:,0], label=self.log.serials[d])
            ax[1].plot(time, ned[:,1])
            ax[2].plot(time, ned[:,2])

            if(np.shape(self.active_devs)[0]==1):
                timeGPS = getTimeFromTowMs(self.getData(d, DID_GPS1_POS, 'timeOfWeekMs'))
                nedGps = lla2ned(self.getData(d, DID_INS_2, 'lla')[0], self.getData(d, DID_GPS1_POS, 'lla'))
                ax[0].plot(timeGPS, nedGps[:, 0], label='GPS')
                ax[1].plot(timeGPS, nedGps[:, 1])
                ax[2].plot(timeGPS, nedGps[:, 2])

        ax[0].legend(ncol=2)
        for a in ax:
            a.grid(True)
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

            if(np.shape(self.active_devs)[0]==1):
                self.drawNEDMapArrow(ax, ned, euler[:, 2])

                nedGps = lla2ned(self.getData(d, DID_INS_2, 'lla')[0], self.getData(d, DID_GPS1_POS, 'lla'))
                ax.plot(nedGps[:, 1], nedGps[:, 0], label='GPS')


        ax.set_aspect('equal', 'datalim')
        ax.legend(ncol=2)
        ax.grid(True)

    def posLLA(self, fig=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(3,1, sharex=True)
        self.configureSubplot(ax[0], 'Latitude', 'deg')
        self.configureSubplot(ax[1], 'Longitude', 'deg')
        self.configureSubplot(ax[2], 'Altitude', 'deg')
        fig.suptitle('INS LLA - ' + os.path.basename(os.path.normpath(self.log.directory)))
        for d in self.active_devs:
            time = getTimeFromTow(self.getData(d, DID_INS_2, 'timeOfWeek'))
            ax[0].plot(time, self.getData(d, DID_INS_2, 'lla')[:,0], label=self.log.serials[d])
            ax[1].plot(time, self.getData(d, DID_INS_2, 'lla')[:,1])
            ax[2].plot(time, self.getData(d, DID_INS_2, 'lla')[:,2], label=self.log.serials[d])

            if(np.shape(self.active_devs)[0]==1):
                timeGPS = getTimeFromTowMs(self.getData(d, DID_GPS1_POS, 'timeOfWeekMs'))
                ax[0].plot(timeGPS, self.getData(d, DID_GPS1_POS, 'lla')[:, 0], label='GPS')
                ax[1].plot(timeGPS, self.getData(d, DID_GPS1_POS, 'lla')[:, 1])
                ax[2].plot(timeGPS, self.getData(d, DID_GPS1_POS, 'lla')[:, 2], label='GPS')

                timeBaro = getTimeFromTow(self.getData(d, DID_BAROMETER, 'time')+ self.getData(d, DID_GPS1_POS, 'towOffset')[-1])
                ax[2].plot(timeBaro, self.getData(d, DID_BAROMETER, 'mslBar'), label='Baro')

        ax[0].legend(ncol=2)
        ax[2].legend(ncol=2)
        for a in ax:
            a.grid(True)
        self.saveFig(fig, 'insLLA')

    def llaGps(self, fig=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(3,1, sharex=True)
        self.configureSubplot(ax[0], 'Latitude', 'deg')
        self.configureSubplot(ax[1], 'Longitude', 'deg')
        self.configureSubplot(ax[2], 'Altitude', 'deg')
        fig.suptitle('GPS LLA - ' + os.path.basename(os.path.normpath(self.log.directory)))
        for d in self.active_devs:
            time = getTimeFromTowMs(self.getData(d, DID_GPS1_POS, 'timeOfWeekMs'))
            ax[0].plot(time, self.getData(d, DID_GPS1_POS, 'lla')[:,0], label=self.log.serials[d])
            ax[1].plot(time, self.getData(d, DID_GPS1_POS, 'lla')[:,1])
            ax[2].plot(time, self.getData(d, DID_GPS1_POS, 'lla')[:,2])
        ax[0].legend(ncol=2)
        for a in ax:
            a.grid(True)
        self.saveFig(fig, 'llaGPS')

    def velNED(self, fig=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(3,1, sharex=True)
        self.configureSubplot(ax[0], 'North', 'm/s')
        self.configureSubplot(ax[1], 'East', 'm/s')
        self.configureSubplot(ax[2], 'Down', 'm/s')
        fig.suptitle('NED Vel - ' + os.path.basename(os.path.normpath(self.log.directory)))
        for d in self.active_devs:
            time = getTimeFromTow(self.getData(d, DID_INS_2, 'timeOfWeek'))
            insVelNed = quatRot(self.getData(d, DID_INS_2, 'qn2b'), self.getData(d, DID_INS_2, 'uvw'))
            ax[0].plot(time, insVelNed[:,0], label=self.log.serials[d])
            ax[1].plot(time, insVelNed[:,1])
            ax[2].plot(time, insVelNed[:,2])

            if np.shape(self.active_devs)[0] == 1:  # Show GPS if #devs is 1
                timeGPS = getTimeFromTowMs(self.getData(d, DID_GPS1_VEL, 'timeOfWeekMs'))
                gpsVelEcef = self.getData(d, DID_GPS1_VEL, 'velEcef')
                qe2n = quat_ecef2ned(self.getData(d, DID_GPS1_POS, 'lla')[0,0:2]*np.pi/180.0)
                if len(gpsVelEcef) > 0:
                    gpsVelNed = quatConjRot(qe2n, gpsVelEcef)
                    #R = rotmat_ecef2ned(self.getData(d, DID_GPS1_POS, 'lla')[0,0:2]*np.pi/180.0)
                    #gpsVelNed = R.dot(gpsVelEcef.T).T
                    ax[0].plot(timeGPS, gpsVelNed[:, 0], label='GPS')
                    ax[1].plot(timeGPS, gpsVelNed[:, 1])
                    ax[2].plot(timeGPS, gpsVelNed[:, 2])

        ax[0].legend(ncol=2)
        for a in ax:
            a.grid(True)
        self.saveFig(fig, 'velNED')

    def velUVW(self, fig=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(3,1, sharex=True)
        self.configureSubplot(ax[0], 'Vel-X', 'm/s')
        self.configureSubplot(ax[1], 'Vel-Y', 'm/s')
        self.configureSubplot(ax[2], 'Vel-Z', 'm/s')
        fig.suptitle('INS uvw - ' + os.path.basename(os.path.normpath(self.log.directory)))
        for d in self.active_devs:
            time = getTimeFromTow(self.getData(d, DID_INS_2, 'timeOfWeek'))
            ax[0].plot(time, self.getData(d, DID_INS_2, 'uvw')[:,0], label=self.log.serials[d])
            ax[1].plot(time, self.getData(d, DID_INS_2, 'uvw')[:,1])
            ax[2].plot(time, self.getData(d, DID_INS_2, 'uvw')[:,2])
        ax[0].legend(ncol=2)
        for a in ax:
            a.grid(True)
        self.saveFig(fig, 'velUVW')

    def attitude(self, fig=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(3, 1, sharex=True)
        fig.suptitle('INS Attitude - ' + os.path.basename(os.path.normpath(self.log.directory)))
        self.configureSubplot(ax[0], 'Roll', 'deg')
        self.configureSubplot(ax[1], 'Pitch', 'deg')
        self.configureSubplot(ax[2], 'Yaw', 'deg')
        for d in self.active_devs:
            euler = quat2euler(self.getData(d, DID_INS_2, 'qn2b'))
            time = getTimeFromTow(self.getData(d, DID_INS_2, 'timeOfWeek'))
            ax[0].plot(time, euler[:,0]*RAD2DEG, label=self.log.serials[d])
            ax[1].plot(time, euler[:,1]*RAD2DEG)
            ax[2].plot(time, euler[:,2]*RAD2DEG)
        ax[0].legend(ncol=2)
        for a in ax:
            a.grid(True)
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
            if magTime:
                ax[0].plot(magTime, magHdg * RAD2DEG)
            if gpsTime:
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

        ax = fig.subplots(4, 1, sharex=True)
        did_gps_vel = did_gps_pos+(DID_GPS1_VEL-DID_GPS1_POS)
        if did_gps_pos==DID_GPS1_POS:
            gps_num = 1
        else:
            gps_num = 2
        fig.suptitle('GPS ' + str(gps_num) + ' Stats - ' + os.path.basename(os.path.normpath(self.log.directory)))
        self.configureSubplot(ax[0], 'Satellites Used in Solution', '')
        self.configureSubplot(ax[1], 'Accuracy', 'm')
        self.configureSubplot(ax[2], 'CNO', 'dBHz')
        self.configureSubplot(ax[3], 'Status', '')

        for d in self.active_devs:
            r = d == self.active_devs[0]  # plot text w/ first device
            time = getTimeFromTowMs(self.getData(d, did_gps_pos, 'timeOfWeekMs'))
            velTime = getTimeFromTowMs(self.getData(d, did_gps_vel, 'timeOfWeekMs'))
            gStatus = self.getData(d, did_gps_pos, 'status')

            ax[0].plot(time, gStatus & 0xFF, label=self.log.serials[d])
            ax[1].plot(time, self.getData(d, did_gps_pos, 'pDop'), 'm', label="pDop")
            ax[1].plot(time, self.getData(d, did_gps_pos, 'hAcc'), 'r', label="hAcc")
            ax[1].plot(time, self.getData(d, did_gps_pos, 'vAcc'), 'b', label="vAcc")
            ax[1].plot(velTime, self.getData(d, did_gps_vel, 'sAcc'), 'c', label="sAcc")
            if self.log.data[d, DID_GPS1_RTK_POS] is not []:
                rtktime = getTimeFromTowMs(self.getData(d, DID_GPS1_RTK_POS, 'timeOfWeekMs'))
                ax[1].plot(rtktime, self.getData(d, DID_GPS1_RTK_POS, 'vAcc'), 'g', label="rtkHor")
            if d == 0:
                ax[1].legend(ncol=2)
            ax[2].plot(time, self.getData(d, did_gps_pos, 'cnoMean'))

            cnt = 0
            ax[3].plot(time, -cnt * 1.5 + ((gStatus & 0x04000000) != 0))
            p1 = ax[3].get_xlim()[0] + 0.02 * (ax[3].get_xlim()[1] - ax[3].get_xlim()[0])
            if r: ax[3].text(p1, -cnt * 1.5, 'RTK Positioning Valid')
            cnt += 1
            ax[3].plot(time, -cnt * 1.5 + ((gStatus & 0x08000000) != 0))
            if r: ax[3].text(p1, -cnt * 1.5, 'RTK Compassing Valid (fix & hold)')
            cnt += 1
            ax[3].plot(time, -cnt * 1.5 + ((gStatus & 0x00002000) != 0))
            if r: ax[3].text(p1, -cnt * 1.5, 'GPS Compass Baseline BAD')
            cnt += 1
            ax[3].plot(time, -cnt * 1.5 + ((gStatus & 0x00004000) != 0))
            if r: ax[3].text(p1, -cnt * 1.5, 'GPS Compass Baseline UNSET')
            cnt += 1

        self.setPlotYSpanMin(ax[2], 5)

        ax[0].legend(ncol=2)
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

    def loadGyros(self, d):
        return self.loadIMU(d, 0)

    def loadAccels(self, d):
        return self.loadIMU(d, 1)

    def loadIMU(self, d, index):   # 0 = gyro, 1 = accelerometer
        imu0 = []
        imu1 = []
        time = None
        dt = None

        I1 = self.getData(d, DID_DUAL_IMU, 'I')[:, 0]
        I2 = self.getData(d, DID_DUAL_IMU, 'I')[:, 1]
        time = self.getData(d, DID_DUAL_IMU, 'time')
        if np.shape(I1)[0] == 0:
            I1 = self.getData(d, DID_DUAL_IMU_RAW, 'I')[:, 0]
            I2 = self.getData(d, DID_DUAL_IMU_RAW, 'I')[:, 1]
            time = self.getData(d, DID_DUAL_IMU_RAW, 'time')

        if np.shape(I1)[0] != 0:  # DID_DUAL_IMU or DID_DUAL_IMU_RAW
            dt = time[1:] - time[:-1]
            dt = np.append(dt, dt[-1])
            for i in range(0, len(I1)):
                imu0.append(I1[i][index])
                imu1.append(I2[i][index])
            imu0 = np.array(imu0)
            imu1 = np.array(imu1)
        else:  # DID_PREINTEGRATED_IMU
            if index==0:
                imu0 = np.copy(self.getData(d, DID_PREINTEGRATED_IMU, 'theta1'))
                imu1 = np.copy(self.getData(d, DID_PREINTEGRATED_IMU, 'theta2'))
            else:
                imu0 = np.copy(self.getData(d, DID_PREINTEGRATED_IMU, 'vel1'))
                imu1 = np.copy(self.getData(d, DID_PREINTEGRATED_IMU, 'vel2'))
            time = self.getData(d, DID_PREINTEGRATED_IMU, 'time')
            if len(time)!=0:
                # dt = self.getData(d, DID_PREINTEGRATED_IMU, 'dt') # this doesn't account for LogInspector downsampling
                dt = time[1:] - time[:-1]
                dt = np.append(dt, dt[-1])
                for i in range(3):
                    imu0[:, i] *= self.d/dt
                    imu1[:, i] *= self.d/dt
            else:
                imu0 = None
                imu0 = None
                time = None
                dt = None

        return (imu0, imu1, time, dt)

    def imuPQR(self, fig=None):
        if fig is None:
            fig = plt.figure()

        for d in self.active_devs:
            (pqr0, pqr1, time, dt) = self.loadGyros(d)

            refTime = self.getData(d, DID_REFERENCE_IMU, 'time')
            if len(refTime)!=0:
                refImu = self.getData(d, DID_REFERENCE_IMU, 'I')
                refImu = refImu
                refPqr = refImu['pqr']

        ax = fig.subplots(3, 2, sharex=True)
        fig.suptitle('PQR - ' + os.path.basename(os.path.normpath(self.log.directory)))
        for d in self.active_devs:
            for i in range(3):
                axislable = 'P' if (i == 0) else 'Q' if (i==1) else 'R'
                for n, pqr in enumerate([ pqr0, pqr1 ]):
                    if (pqr is not None) and pqr.any(None):
                        mean = np.mean(pqr[:, i])
                        std = np.std(pqr[:, i])
                        self.configureSubplot(ax[i, n], 'Gyro%d ' % n + axislable + ' (deg/s), mean: %.4g, std: %.3g' % (mean, std), 'sec')
                        ax[i, n].plot(time, pqr[:, i] * 180.0/np.pi, label=self.log.serials[d])

                if len(refTime) != 0:
                    ax[i, 0].plot(refTime, refPqr[:, i] * 180.0/np.pi, color='red', label="reference")
                    ax[i, 1].plot(refTime, refPqr[:, i] * 180.0/np.pi, color='red', label="reference")

        ax[0,0].legend(ncol=2)
        for i in range(3):
            for j in range(2):
                ax[i,j].grid(True)
        self.saveFig(fig, 'pqrIMU')

    def imuAcc(self, fig=None):
        if fig is None:
            fig = plt.figure()

        for d in self.active_devs:
            (acc0, acc1, time, dt) = self.loadAccels(d)

            refTime = self.getData(d, DID_REFERENCE_IMU, 'time')
            if len(refTime)!=0:
                refImu = self.getData(d, DID_REFERENCE_IMU, 'I')
                refImu = refImu
                refAcc = refImu['acc']

        ax = fig.subplots(3, 2, sharex=True)
        fig.suptitle('Accelerometer - ' + os.path.basename(os.path.normpath(self.log.directory)))
        for d in self.active_devs:
            for i in range(3):
                axislable = 'X' if (i == 0) else 'Y' if (i==1) else 'Z'
                for n, acc in enumerate([ acc0, acc1 ]):
                    if (acc is not None) and acc.any(None):
                        mean = np.mean(acc[:, i])
                        std = np.std(acc[:, i])
                        self.configureSubplot(ax[i, n], 'Accel%d ' % n + axislable + ' (m/s^2), mean: %.4g, std: %.3g' % (mean, std), 'sec')
                        ax[i, n].plot(time, acc[:, i], label=self.log.serials[d])

                if len(refTime) != 0:
                    ax[i, 0].plot(refTime, refAcc[:, i], color='red', label="reference")
                    ax[i, 1].plot(refTime, refAcc[:, i], color='red', label="reference")

        ax[0,0].legend(ncol=2)
        for i in range(3):
            for j in range(2):
                ax[i,j].grid(True)
        self.saveFig(fig, 'accIMU')

    def accelPSD(self, fig=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(3, 2, sharex=True)
        self.configureSubplot(ax[0,0], 'AccX 0 PSD (dB (m/s^2)^2/Hz)', 'Hz')
        self.configureSubplot(ax[0,1], 'AccX 1 PSD (dB (m/s^2)^2/Hz)', 'Hz')
        self.configureSubplot(ax[1,0], 'AccY 0 PSD (dB (m/s^2)^2/Hz)', 'Hz')
        self.configureSubplot(ax[1,1], 'AccY 1 PSD (dB (m/s^2)^2/Hz)', 'Hz')
        self.configureSubplot(ax[2,0], 'AccZ 0 PSD (dB (m/s^2)^2/Hz)', 'Hz')
        self.configureSubplot(ax[2,1], 'AccZ 1 PSD (dB (m/s^2)^2/Hz)', 'Hz')
        fig.suptitle('Power Spectral Density - ' + os.path.basename(os.path.normpath(self.log.directory)))
        for d in self.active_devs:
            (acc0, acc1, time, dt) = self.loadAccels(d)

            N = time.size
            psd0 = np.zeros((N//2, 3))
            psd1 = np.zeros((N//2, 3))
            # 1/T = frequency
            Fs = 1 / np.mean(dt)
            f = np.linspace(0, 0.5*Fs, N // 2)

            for i in range(3):
                sp0 = np.fft.fft(acc0[:,i] / 9.8)
                sp0 = sp0[:N // 2]
                # psd = abssp*abssp
                # freq = np.fft.fftfreq(time.shape[-1])
#                    np.append(psd0, [1/N/Fs * np.abs(sp0)**2], axis=1)
                psd0[:,i] = 1/N/Fs * np.abs(sp0)**2
                psd0[1:-1,i] = 2 * psd0[1:-1,i]
                sp1 = np.fft.fft(acc1[:,i] / 9.8)
                sp1 = sp1[:N // 2]
                # psd = abssp*abssp
                # freq = np.fft.fftfreq(time.shape[-1])
#                    np.append(psd0, [1/N/Fs * np.abs(sp0)**2], axis=1)
                psd1[:,i] = 1/N/Fs * np.abs(sp1)**2
                psd1[1:-1,i] = 2 * psd1[1:-1,i]

            for i in range(3):
                # ax[i, 0].loglog(f, psd0[:, i])
                # ax[i, 1].loglog(f, psd1[:, i])
                ax[i, 0].plot(f, 10*np.log10(psd0[:, i]))
                ax[i, 1].plot(f, 10*np.log10(psd1[:, i]))

        ax[0,0].legend(ncol=2)
        for i in range(3):
            for j in range(2):
                ax[i,j].grid(True)
        self.saveFig(fig, 'accelPSD')

    def gyroPSD(self, fig=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(3, 2, sharex=True)
        self.configureSubplot(ax[0,0], 'Gyr0 X PSD (dB dps^2/Hz)', 'Hz')
        self.configureSubplot(ax[0,1], 'Gyr1 X PSD (dB dps^2/Hz)', 'Hz')
        self.configureSubplot(ax[1,0], 'Gyr0 Y PSD (dB dps^2/Hz)', 'Hz')
        self.configureSubplot(ax[1,1], 'Gyr1 Y PSD (dB dps^2/Hz)', 'Hz')
        self.configureSubplot(ax[2,0], 'Gyr0 Z PSD (dB dps^2/Hz)', 'Hz')
        self.configureSubplot(ax[2,1], 'Gyr1 Z PSD (dB dps^2/Hz)', 'Hz')
        fig.suptitle('Power Spectral Density - ' + os.path.basename(os.path.normpath(self.log.directory)))
        for d in self.active_devs:
            (pqr0, pqr1, time, dt) = self.loadGyros(d)

            N = time.size
            Nhalf = N // 2 + 1
            psd0 = np.zeros((Nhalf, 3))
            psd1 = np.zeros((Nhalf, 3))
            # 1/T = frequency
            Fs = 1 / np.mean(dt)
            f = np.linspace(0, 0.5*Fs, Nhalf)
            
            for i in range(3):
                sp0 = np.fft.fft(pqr0[:,i] * 180.0/np.pi)
                sp0 = sp0[:Nhalf]
                # psd = abssp*abssp
                # freq = np.fft.fftfreq(time.shape[-1])
    #                    np.append(psd0, [1/N/Fs * np.abs(sp0)**2], axis=1)
                psd0[:,i] = 1/N/Fs * np.abs(sp0)**2
                psd0[1:-1,i] = 2 * psd0[1:-1,i]

                sp1 = np.fft.fft(pqr1[:,i] * 180.0/np.pi)
                sp1 = sp1[:Nhalf]
                # psd = abssp*abssp
                # freq = np.fft.fftfreq(time.shape[-1])
    #                    np.append(psd0, [1/N/Fs * np.abs(sp0)**2], axis=1)
                psd1[:,i] = 1/N/Fs * np.abs(sp1)**2
                psd1[1:-1,i] = 2 * psd1[1:-1,i]

            for i in range(3):
                ax[i, 0].plot(f, 10*np.log10(psd0[:, i]))
                ax[i, 1].plot(f, 10*np.log10(psd1[:, i]))

        ax[0,0].legend(ncol=2)
        for i in range(3):
            for j in range(2):
                ax[i,j].grid(True)
        self.saveFig(fig, 'gyroPSD')

    def magnetometer(self, fig=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(6, 1, sharex=True)

        self.configureSubplot(ax[0], 'Mag0 X', 'gauss')
        self.configureSubplot(ax[1], 'Mag1 X', 'gauss')
        self.configureSubplot(ax[2], 'Mag0 Y', 'gauss')
        self.configureSubplot(ax[3], 'Mag1 Y', 'gauss')
        self.configureSubplot(ax[4], 'Mag0 Z', 'gauss')
        self.configureSubplot(ax[5], 'Mag1 Z', 'gauss')
        fig.suptitle('Magnetometer - ' + os.path.basename(os.path.normpath(self.log.directory)))
        for d in self.active_devs:
            time0 = self.getData(d, DID_MAGNETOMETER, 'time')
            towOffset = self.getData(d, DID_GPS1_POS, 'towOffset')
            if np.shape(towOffset)[0] != 0:
                time0 = time0 + towOffset[-1]
            mag0 = self.getData(d, DID_MAGNETOMETER, 'mag')
            mag0x = mag0[:,0]
            mag0y = mag0[:,1]
            mag0z = mag0[:,2]
            ax[0].plot(time0, mag0x, label=self.log.serials[d])
            ax[2].plot(time0, mag0y)
            ax[4].plot(time0, mag0z)

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
            ax = fig.subplots(2, 1, sharex=True)
            fig.suptitle('Temperature - ' + os.path.basename(os.path.normpath(self.log.directory)))

            for d in self.active_devs:
                time = getTimeFromTowMs(self.getData(d, DID_SYS_PARAMS, 'timeOfWeekMs'))
                ax[0].plot(time, self.getData(d, DID_SYS_PARAMS, 'imuTemp'), label=self.log.serials[d])
                ax[1].plot(time, self.getData(d, DID_SYS_PARAMS, 'baroTemp'))
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

    def magDec(self, fig=None):
        if fig is None:
            fig = plt.figure()

        ax = fig.subplots(2, 1, sharex=True)
        fig.suptitle('Magnetometer Declination - ' + os.path.basename(os.path.normpath(self.log.directory)))
        self.configureSubplot(ax[0], 'Declination', 'deg')
        self.configureSubplot(ax[1], 'Inclination', 'deg')

        for d in self.active_devs:
            time = getTimeFromTow(self.getData(d, DID_INL2_STATES, 'timeOfWeek'))
            declination = 180.0/np.pi * self.getData(d, DID_INL2_STATES, 'magDec')
            inclination = 180.0/np.pi * self.getData(d, DID_INL2_STATES, 'magInc')
            ax[0].plot(time, declination, label=self.log.serials[d])
            ax[1].plot(time, inclination)
        ax[0].legend(ncol=2)
        self.saveFig(fig, 'magDec')
        for a in ax:
            a.grid(True)

    def deltatime(self, fig=None):
        if fig is None:
            fig = plt.figure()

        ax = fig.subplots(4, 1, sharex=True)
        fig.suptitle('Timestamps - ' + os.path.basename(os.path.normpath(self.log.directory)))
        self.configureSubplot(ax[0], 'INS dt', 's')
        self.configureSubplot(ax[1], 'GPS dt', 's')
        self.configureSubplot(ax[2], 'IMU Integration Period', 's')
        self.configureSubplot(ax[3], 'IMU Delta Timestamp', 's')

        for d in self.active_devs:
            dtIns = self.getData(d, DID_INS_2, 'timeOfWeek')[1:] - self.getData(d, DID_INS_2, 'timeOfWeek')[0:-1]
            dtIns = dtIns / self.d
            timeIns = getTimeFromTow(self.getData(d, DID_INS_2, 'timeOfWeek')[1:])

            dtGps = 0.001*(self.getData(d, DID_GPS1_POS, 'timeOfWeekMs')[1:] - self.getData(d, DID_GPS1_POS, 'timeOfWeekMs')[0:-1])
            dtGps = dtGps / self.d
            timeGps = getTimeFromTowMs(self.getData(d, DID_GPS1_POS, 'timeOfWeekMs')[1:])

            dtPImu = self.getData(d, DID_PREINTEGRATED_IMU, 'dt')[1:]
            dtPImu = dtPImu / self.d

            dtImu = self.getData(d, DID_PREINTEGRATED_IMU, 'time')[1:] - self.getData(d, DID_PREINTEGRATED_IMU, 'time')[0:-1]
            dtImu = dtImu / self.d

            towOffset = self.getData(d, DID_GPS1_POS, 'towOffset')
            if np.size(towOffset) > 0:
                towOffset = towOffset[-1]
            else:
                towOffset = 0
            timeImu = getTimeFromTow(self.getData(d, DID_PREINTEGRATED_IMU, 'time')[1:] + towOffset)

            ax[0].plot(timeIns, dtIns, label=self.log.serials[d])
            ax[1].plot(timeGps, dtGps)
            ax[2].plot(timeImu, dtPImu)
            ax[3].plot(timeImu, dtImu)

        self.setPlotYSpanMin(ax[0], 0.01)
        self.setPlotYSpanMin(ax[1], 0.01)
        self.setPlotYSpanMin(ax[2], 0.05)

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

        fields = list(self.log.data[0, DID_RTK_DEBUG].dtype.names)
        fields.remove('time')
        num_plots = 0
        for field in fields:
            dat = self.log.data[0, DID_RTK_DEBUG][field][0]
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

        #max_num_biases = 22 #np.array(self.getData(self.active_devs[0], DID_RTK_DEBUG_2, 'num_biases'))
        max_num_biases = self.getData(0, DID_RTK_DEBUG_2, 'num_biases')[-1]
        for r in range(0,6):
            for c in range(0,4):
                self.configureSubplot(ax[r,c], '', '')

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

        max_num_biases = self.getData(0, DID_RTK_DEBUG_2, 'num_biases')[-1]
        for r in range(0,6):
            for c in range(0,4):
                self.configureSubplot(ax[r,c], '', '')

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

        max_num_biases = self.getData(0, DID_RTK_DEBUG_2, 'num_biases')[-1]
        for r in range(0,6):
            for c in range(0,4):
                self.configureSubplot(ax[r,c], '', '')

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

        max_num_biases = self.getData(0, DID_RTK_DEBUG_2, 'num_biases')[-1]
        for r in range(0,6):
            for c in range(0,4):
                self.configureSubplot(ax[r,c], '', '')

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

    def sensorCompGyrTime(self, fig=None):
        if fig is None:
            fig = plt.figure()

        self.sensorCompGen(fig, 'pqr', useTime=True)

    def sensorCompAccTime(self, fig=None):
        if fig is None:
            fig = plt.figure()
        self.sensorCompGen(fig, 'acc', useTime=True)

    def sensorCompMagTime(self, fig=None):
        if fig is None:
            fig = plt.figure()
        self.sensorCompGen(fig, 'mag', useTime=True)

    def sensorCompGen(self, fig, name, useTime=False):
        fig.suptitle('Sensor Comp ' + name + ' - ' + os.path.basename(os.path.normpath(self.log.directory)))
        ax = fig.subplots(4, 2, sharex=True)

        for i in range(2):
            ax[0, i].set_title('X %s %d' % (name, i))
            ax[1, i].set_title('Y %s %d' % (name, i))
            ax[2, i].set_title('Z %s %d' % (name, i))
            ax[3, i].set_title('Magnitude %s %d' % (name, i))
            for d in range(3):
                if useTime:
                    ax[d,i].set_xlabel("Time (s)")
                else:
                    ax[d,i].set_xlabel("Temperature (C)")
                if name=='pqr':
                    ax[d,i].set_ylabel("Gyro (deg/s)")
                elif name=='acc':
                    ax[d,i].set_ylabel("Accel (m/s^2)")
                elif name=='mag':
                    ax[d,i].set_ylabel("Mag")

        for d in self.active_devs:
            time = 0.001 * self.getData(d, DID_SCOMP, 'timeMs')
            mpu = self.getData(d, DID_SCOMP, 'mpu')
            status = self.getData(d, DID_SCOMP, 'status')

            if name=='mag':
                refTime = self.getData(d, DID_REFERENCE_MAGNETOMETER, 'time')
                if len(refTime)!=0:
                    refVal = self.getData(d, DID_REFERENCE_MAGNETOMETER, 'mag')
            else:
                refTime = self.getData(d, DID_REFERENCE_IMU, 'time')
                if len(refTime)!=0:
                    refImu = self.getData(d, DID_REFERENCE_IMU, 'I')
                    refImu = refImu
                    refVal = refImu[name]

            for i in range(2):
                temp = mpu[:,i]['lpfLsb']['temp']
                sensor = mpu[:,i]['lpfLsb'][name]

                if name=='pqr':
                    scalar = RAD2DEG
                else:
                    scalar = 1.0

                if useTime:
                    temp = time

                # ax[0,i].plot(temp, sensor[:,0], label=self.log.serials[d] if i==0 else None )
                ax[0,i].plot(temp, sensor[:,0]*scalar, label=self.log.serials[d] )
                ax[1,i].plot(temp, sensor[:,1]*scalar)
                ax[2,i].plot(temp, sensor[:,2]*scalar)
                if name=='acc':
                    ax[3,i].plot(temp, np.linalg.norm(sensor, axis=1)*scalar)

                if useTime and 1:
                    # Show sensor valid status bit
                    if name=='acc':
                        valid = 0.0 + ((status & 0x00000200) != 0) * scalar * 0.25
                    else:
                        valid = 0.0 + ((status & 0x00000100) != 0) * scalar * 0.25
                    ax[0,i].plot(time, valid * np.max(sensor[:,0]), color='y', label="Sensor Valid")
                    ax[1,i].plot(time, valid * np.max(sensor[:,1]), color='y')
                    ax[2,i].plot(time, valid * np.max(sensor[:,2]), color='y')

                    if len(refTime) != 0:
                        for j in range(3):
                            ax[j, i].plot(refTime, refVal[:, j] * scalar, color='red', label="reference")

        # Show serial numbers
        ax[0,0].legend(ncol=2)

        for a in ax:
            for b in a:
                b.grid(True)


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

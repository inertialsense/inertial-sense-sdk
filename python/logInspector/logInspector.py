#!/usr/bin/python3

import sys, os, signal, ctypes, shutil, json, io, traceback, yaml, subprocess, re
from PyQt5 import QtCore
from PyQt5.QtWidgets import QWidget, QDialog, QApplication, QPushButton, QVBoxLayout, QLineEdit, QTreeView, QFileSystemModel,\
    QHBoxLayout, QMainWindow, QSizePolicy, QSpacerItem, QFileDialog, QMessageBox, QLabel, QAbstractItemView, QMenu,\
    QTableWidget,QTableWidgetItem, QSpinBox, QCheckBox, QGroupBox, QListView, QStyle
from PyQt5.QtGui import QMovie, QIcon, QPixmap, QImage, QStandardItemModel, QStandardItem
from PyQt5.QtCore import pyqtSignal, QItemSelectionModel, Qt

import matplotlib
matplotlib.use('Agg')

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
import matplotlib.pyplot as plt
import numpy as np

from logReader import Log
from logPlotter import logPlot

file_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(file_path + '/..'))
sys.path.append(os.path.normpath(file_path + '/../ci_hdw'))
sys.path.append(os.path.normpath(file_path + '/../math/src'))
sys.path.append(os.path.normpath(file_path + '/../supernpp'))

from pylib.data_sets import *

START_MODE_HOT = 0
START_MODE_COLD = 1
START_MODE_FACTORY = 2
startModes = ["HOT", "COLD", "FACTORY"]

def openFolderWithFileBrowser(path):
    if sys.platform == 'darwin':
        subprocess.check_call(['open', '--', path])
    elif sys.platform == 'linux':
        subprocess.check_call(['xdg-open', path])
    elif sys.platform == 'win32':
        # subprocess.check_call(['explorer', path])     # Not stable
        subprocess.Popen(r'explorer '+path)

def cleanFolder(path, toplevel=True):
    containsDAT = False
    containsSDAT = False
    for fname in os.listdir(path):
        if fname.endswith('.dat'):
            containsDAT = True
        if fname.endswith('.sdat'):
            containsSDAT = True

    for fname in os.listdir(path):
        fpath = os.path.join(path, fname)
        if os.path.isdir( fpath ):
            if fname == 'post_processed':
                removeDirectory(fpath)
            else:
                cleanFolder(fpath, False)
        else:
            # Remove .csv if .dat or .sdat exist
            if containsDAT or containsSDAT:
                if fname.endswith('.csv'):
                    print('Deleting: ' + fpath)
                    os.remove(fpath)
            # Remove .sdat if .dat exist
            if containsDAT:
                if fname.endswith('.sdat'):
                    print('Deleting: ' + fpath)
                    os.remove(fpath)
    if toplevel:
        print('Finished Cleaning!')

def removeDirectory(fpath):
    print('Removing Directory: ' + fpath)
    shutil.rmtree(fpath)

# startMode 0=hot, 1=cold, 2=factory
def setDataInformationDirectory(path, startMode=START_MODE_HOT):
    settings_filename = os.path.expanduser("~") + '/Documents/Inertial_Sense/dataInformation.json'

    if settings_filename is not None:
        # with open(settings_filename, 'r') as f:
        #     data = json.load(f)
        #     f.close()
        data = {}
        data['dataInfo'] = {}
        data['dataInfo']['dataDirectory'] = os.path.dirname(path).replace('\\','/')
        data['dataInfo']['subDirectories'] = [os.path.basename(path)]
        serialnumbers = []
        logtype = 'RAW'
        for filename in os.listdir(path):
            print(filename)
            if "LOG_SN" in filename:
                serialnum = re.search(r'\d+', filename).group()
                if serialnum not in serialnumbers:
                    serialnumbers += [serialnum]
            if ".dat" in filename.lower():
                logtype = "DAT"

        data['processData'] = {}
        data['processData']['datasets'] = [{}]
        data['processData']['datasets'][0]['SerialNumbers'] = serialnumbers
        data['processData']['datasets'][0]['folder'] = os.path.basename(path)
        data['processData']['datasets'][0]['logType'] = logtype
        if startMode == START_MODE_HOT:
            data['processData']['datasets'][0]['startMode'] = 'HOT'
        elif startMode == START_MODE_COLD:
            data['processData']['datasets'][0]['startMode'] = 'COLD'
        else:
            data['processData']['datasets'][0]['startMode'] = 'FACTORY'

        # os.remove(settings_filename)
        with open(settings_filename, 'w') as f:
            json.dump(data, f, indent=4)

def verArrayToString(array):
    return str(array[0]) + '.' + str(array[1]) + '.' + str(array[2]) + '.' + str(array[3])

def dateTimeArrayToString(info):
    year   = info['buildYear']
    month  = info['buildMonth']
    day    = info['buildDay']
    hour   = info['buildHour']
    minute = info['buildMinute']
    second = info['buildSecond']
    return str(int(year)+2000) + '-' + f'{month:02}' + '-' + f'{day:02}' + ' ' + f'{hour:02}' + ':' + f'{minute:02}' + ':' + f'{second:02}'

class DeviceInfoDialog(QDialog):
    def __init__(self, log, parent=None):
        super(DeviceInfoDialog, self).__init__(parent)
        self.setWindowTitle("Device Info")

        if np.shape(log.data[0,DID_DEV_INFO])[0] == 0:
            self.label = QLabel('No DID_DEV_INFO data available.')
            self.mainlayout = QVBoxLayout()
            self.mainlayout.addWidget(self.label)
            self.setLayout(self.mainlayout)
            self.resize(400, 200)
            return

        self.table = QTableWidget()
        nfields = len(log.data[0, DID_DEV_INFO].dtype.names)
        field_names = []
        vals = []

        self.table.setColumnCount(9)
        self.table.setHorizontalHeaderLabels(['Serial#','Hardware','Firmware','Build','Protocol','Repo','Build Date','Manufacturer','AddInfo'])

        for d, dev in enumerate(log.data):
            devInfo = dev[DID_DEV_INFO][0]
            self.table.setRowCount(d+1)
            self.table.setItem(d, 0, QTableWidgetItem(str(devInfo['serialNumber'])))
            self.table.setItem(d, 1, QTableWidgetItem(verArrayToString(devInfo['hardwareVer'])))
            self.table.setItem(d, 2, QTableWidgetItem(verArrayToString(devInfo['firmwareVer'])))
            self.table.setItem(d, 3, QTableWidgetItem(str(devInfo['buildNumber']))) 
            self.table.setItem(d, 4, QTableWidgetItem(verArrayToString(devInfo['protocolVer'])))
            self.table.setItem(d, 5, QTableWidgetItem(str(devInfo['repoRevision'])))   # Repo
            self.table.setItem(d, 6, QTableWidgetItem(dateTimeArrayToString(devInfo))) # Build Date & Time
            self.table.setItem(d, 7, QTableWidgetItem(devInfo['manufacturer'].decode('UTF-8')))         # Manufacturer
            self.table.setItem(d, 8, QTableWidgetItem(devInfo['addInfo'].decode('UTF-8')))        # Additional Info

        self.mainlayout = QVBoxLayout()
        self.mainlayout.addWidget(self.table)
        self.setLayout(self.mainlayout)
        self.resize(1280, 800)

class FlashConfigDialog(QDialog):
    def __init__(self, log, parent=None):
        super(FlashConfigDialog, self).__init__(parent)
        self.setWindowTitle("Flash Config")

        if np.shape(log.data[0,DID_FLASH_CONFIG])[0] == 0:
            self.label = QLabel('No DID_FLASH_CONFIG data available.')
            self.mainlayout = QVBoxLayout()
            self.mainlayout.addWidget(self.label)
            self.setLayout(self.mainlayout)
            self.resize(400, 200)
            return

        self.table = QTableWidget()
        nfields = len(log.data[0, DID_FLASH_CONFIG].dtype.names)
        field_names = []
        vals = []

        for d, dev in enumerate(log.data):
            vals.append([])
            for f, field in enumerate(dev[DID_FLASH_CONFIG].dtype.names):
                if isinstance(dev[DID_FLASH_CONFIG][field][0], np.ndarray):
                    length = len(dev[DID_FLASH_CONFIG][field][0])
                    if d == 0: nfields +=  length-1 # add extra rows for arrays in flash config
                    for i in range(length):
                        if d == 0: field_names.append(field + "[" + str(i) + "]")
                        vals[d].append(dev[DID_FLASH_CONFIG][field][0][i])
                else:
                    if d == 0: field_names.append(field)
                    vals[d].append(dev[DID_FLASH_CONFIG][field][0])

        self.table.setRowCount(nfields)
        self.table.setColumnCount(log.numDev)

        self.table.setHorizontalHeaderLabels([str(ser) for ser in log.serials])
        self.table.setVerticalHeaderLabels(field_names)

        hex_fields = ['ioConfig', 'platformConfig', 'RTKCfgBits', 'sysCfgBits', 'gnssSatSigConst', 'sensorConfig']
        for d in range(log.numDev):
            for f, field in enumerate(field_names):
                if field in hex_fields:
                    self.table.setItem(f, d, QTableWidgetItem(hex(vals[d][f])))
                else:
                    self.table.setItem(f, d, QTableWidgetItem(str(vals[d][f])))

        self.mainlayout = QVBoxLayout()
        self.mainlayout.addWidget(self.table)
        self.setLayout(self.mainlayout)
        self.resize(1280, 900)

class MPlotter(QDialog):
    # Define a signal that will be emitted when the dialog is closed
    dialogClosed = pyqtSignal(int)
    
    def __init__(self, index=0, parentDialog=None, popup=False, title=None):
        self.index = index
        self.figure = plt.figure()
        self.canvas = FigureCanvas(self.figure)
        self.toolbar = NavigationToolbar(self.canvas, parentDialog)
        self.func = None
        if popup:
            super(MPlotter, self).__init__(parentDialog)
            layout = QVBoxLayout()
            layout.addWidget(self.canvas)
            layout.addWidget(self.toolbar)
            layout.setStretchFactor(self.canvas, 1)

            self.setLayout(layout)
            if title:
                self.setWindowTitle(title)
            self.setParent(parentDialog)
            self.resize(1110, 900)
            self.setWindowFlags(Qt.Window)  # Allow this window to go on top or behind main dialog

        self.figure.subplots_adjust(left=0.05, right=0.99, bottom=0.05, top=0.91, wspace=0.2, hspace=0.2)
        self.plotter = logPlot()

    def closeEvent(self, event):
        # Emit the dialogClosed signal when the dialog is closed
        self.dialogClosed.emit(self.index)
        print("MPlotter is closing")
        # Call the base class implementation
        super(MPlotter, self).closeEvent(event)

class LogInspectorWindow(QMainWindow):
    def __init__(self, configFilePath):
        super(LogInspectorWindow, self).__init__()
        self.mplots = [MPlotter(self)]
        self.configFilePath = configFilePath
        self.exePath = __file__
        self.directory = None

        folder = os.path.dirname(self.configFilePath)
        if not os.path.exists(folder):
            os.makedirs(folder)

        self.loadConfigFromFile()
        self.selectedIndex = 0
        self.downsample = 5
        self.plotargs = None
        self.log = None

    def closeEvent(self, event):
        self.updateRootPathHist()
        self.saveConfigToFile()
        super().closeEvent(event)  

    def popPlot(self):
        print("New Plot")
        mp = MPlotter(len(self.mplots), self, True, self.nameList[self.selectedIndex] + " - " + self.deviceInfo())
        mp.dialogClosed.connect(self.on_mplotter_closed)  # Connect the custom signal to a slot
        mp.show()
        mp.plotter.setLog(self.log)
        mp.plotter.setDownSample(self.downsample)
        self.mplots.append(mp)
        self.updatePlot()

    def on_mplotter_closed(self, index):
        if index < len(self.mplots):
            del self.mplots[index]      # Remove element
        for index, mplot in enumerate(self.mplots):
            mplot.index = index

    def addButton(self, name, function, layout=None, tooltip=None):
        buttonName = name + "button"
        setattr(self, buttonName, QPushButton(name))
        getattr(self, buttonName).clicked.connect(function)
        # getattr(self, buttonName).setMinimumWidth(220)
        if layout is None:
            layout = self.buttonLayoutColIns
        if tooltip:
            getattr(self, buttonName).setToolTip(tooltip)

        if type(layout) is list:
            for i in range(len(layout)):
                if (i == len(layout)-1 
                    or layout[i].count() <= layout[i+1].count()):
                    layout[i].addWidget(getattr(self, name + "button"))
                    break
        else:
            layout.addWidget(getattr(self, name + "button"))

    def addListSection(self, name):
        self.addListItem("==========  " + name + "  ==========", None)

    def addListItem(self, name, function):
        funcName = None
        if type(function) == str:
            funcName = function
            function = lambda: self.plot(funcName)
        self.modelList.appendRow(QStandardItem(name))
        self.nameList.append(name)
        self.funcNameList.append(funcName)
        self.functionList.append(function)

    def selectedPlot(self):
        if self.selectedIndex < len(self.funcNameList):
            return self.funcNameList[self.selectedIndex]
        else:
            return None

    def updatePlot(self):
        self.plot(self.selectedPlot(), self.plotargs)
        self.updateWindowTitle()

    def deviceInfo(self):
        info = self.log.data[0,DID_DEV_INFO][0]
        return 'SN' + str(info['serialNumber']) + ', H:' + verArrayToString(info['hardwareVer']) + ', F:' + verArrayToString(info['firmwareVer']) + ' build ' + str(info['buildNumber']) + ', ' + dateTimeArrayToString(info) + ', ' + info['addInfo'].decode('UTF-8')

    def updateWindowTitle(self):
        try:
            size = self.log.numDev
            if  size != 0:
                if size == 1:
                    infoStr = self.deviceInfo()
                else:
                    infoStr = 'Devices: [' + " ".join([str(x) for x in self.log.serials]) + "]"
                if self.log.using_mounting_bias:
                    infoStr += ', Mounting Corrected'
                self.setWindowTitle("LogInspector  -  " + infoStr)
        except:
            self.setWindowTitle("DID_DEV_INFO missing")

    def reload(self):
        if 'directory' in self.config:
            self.load(self.config['directory'])

    def load(self, directory):
        self.updateRootPathHist()
        self.saveConfigToFile()
        self.config['directory'] = directory
        print("\nLoading files from " + directory)
        self.setStatus("Loading...")
        # if self.log is None:
        self.log = Log()
        self.log.load(directory)
        print("done loading")
        for mplot in self.mplots:
            mplot.plotter.setLog(self.log)
            mplot.plotter.setDownSample(self.downsample)
        self.updatePlot()
        self.setStatus("")
        self.expandAndSelectDirectory(directory)

    def setupUi(self):
        self.setObjectName("LogInspector")
        self.setWindowTitle("LogInspector")
        self.resize(1280, 900)
        self.setWindowIcon(QIcon('assets/Magnifying_glass_icon.png'))

        self.controlLayout = QVBoxLayout()
        self.controlWidget = QWidget(self)
        self.controlWidget.setLayout(self.controlLayout)
        
        self.createPlotSelection()
        self.createPlotSelectionPost()
        self.createFileTree()
        self.createStatus()
        self.controlLayout.setStretch(0, 2)     # Plot selection
        self.controlLayout.setStretch(3, 1)     # File tree
        self.createBottomToolbar()

        self.figureLayout = QVBoxLayout()
        self.figureLayout.addWidget(self.mplots[0].canvas)
        self.figureLayout.addLayout(self.toolLayout)
        self.figureLayout.setStretchFactor(self.mplots[0].canvas, 1)

        layout = QHBoxLayout()
        layout.addWidget(self.controlWidget)
        layout.addLayout(self.figureLayout)
        layout.setStretch(1, 1)
        widget = QWidget()
        widget.setLayout(layout)
        self.setCentralWidget(widget)
        self.resize(1450, 1000)
        self.setAcceptDrops(True)

    def createListSystem(self):
        self.addListItem('Flash Config', lambda: self.showFlashConfig())
        self.addListItem('Device Info', lambda: self.showDeviceInfo())
        self.addListItem('IMU Status', 'imuStatus')
        self.addListItem('INS Status', 'insStatus')
        self.addListItem('HDW Status', 'hdwStatus')

    def createListIns(self):
        self.addListItem('Pos NED Map', 'posNEDMap')
        self.addListItem('Pos NED', 'posNED')
        self.addListItem('Pos LLA', 'posLLA')
        self.addListItem('Vel NED', 'velNED')
        self.addListItem('Vel UVW', 'velUVW')
        self.addListItem('Attitude', 'attitude')
        self.addListItem('Heading', 'heading')
        self.addListItem('Altitude', 'altitude')
        self.addListItem('Climb Rate', 'climbRate')

    def createListSensors(self):
        self.addListItem('IMU Gyro', 'imuPQR')
        self.addListItem('IMU Accel', 'imuAcc')
        self.addListItem('PSD Gyro', 'gyroPSD')
        self.addListItem('PSD Accel', 'accelPSD')
        self.addListItem('Barometer', 'barometer')
        self.addListItem('Magnetometer', 'magnetometer')
        self.addListItem('Temp', 'temp')

    def createListGps(self):
        self.addListItem('GPS LLA', 'gpsLLA')
        self.addListItem('GPS 1 Stats', 'gpsStats')
        self.addListItem('GPS 2 Stats', 'gps2Stats')
        self.addListItem('GPS Time', 'gpsTime')
        self.addListItem('GPX Status', 'gpxStatus')
        self.addListItem('GPX HDW Status', 'gpxHdwStatus')
        self.addListItem('RTK Pos Stats', 'rtkPosStats')
        self.addListItem('RTK Cmp Stats', 'rtkCmpStats')
        self.addListItem('RTK Cmp BaseVector', 'rtkBaselineVector')
        self.addListItem('RTK Obs GPS1', 'rtkObsGPS1')
        self.addListItem('RTK Obs GPS2', 'rtkObsGPS2')
        self.addListItem('RTK Obs Rover-Base Single Diff', 'rtkObsSingleDiff')
        self.addListItem('GPS Position NED Map', 'gpsPosNEDMap')
        self.addListItem('GPS Position NED', 'gpsPosNED')
        self.addListItem('GPS Velocity NED', 'gpsVelNED')

    def createListGeneral(self):
        None

    def setCurrentListRow(self, row):
        if row < self.modelList.rowCount() and row < len(self.functionList):
            if self.functionList[row]:
                self.selectedIndex = row
                index = self.modelList.createIndex(self.selectedIndex, 0)
                self.listView.setCurrentIndex(index)
                return True
        return False

    def onSelectListItem(self, index):
        row = index.row()
        if row < len(self.functionList):
            if self.functionList[row]==None and self.selectedIndex>0:
                self.setCurrentListRow(self.selectedIndex)
            else:
                self.selectedIndex = index.row()
                if self.log != None:    # Don't attempt to plot if log wasn't selected
                    self.functionList[row]()
    
    def createPlotSelection(self):
        groupBox = QGroupBox("Select Plot")
        self.listView = QListView()
        self.modelList = QStandardItemModel()
        self.nameList = []
        self.funcNameList = []
        self.functionList = []
        self.listView.setModel(self.modelList)
        self.listView.clicked.connect(self.onSelectListItem)
        LayoutList = QHBoxLayout()
        LayoutList.addWidget(self.listView)
        groupBox.setLayout(LayoutList)
        self.controlLayout.addWidget(groupBox)        

        self.addListSection('SYSTEM')
        self.createListSystem()

        self.addListSection('INS/AHRS')
        self.createListIns()

        self.addListSection('SENSORS')
        self.createListSensors()

        self.addListSection('GPS')
        self.createListGps()

        self.addListSection('GENERAL')
        self.createListGeneral()

        self.checkboxResidual = QCheckBox("Residual", self)
        self.checkboxResidual.setToolTip("Show residual plots")
        self.checkboxResidual.stateChanged.connect(self.changeResidualCheckbox)
        self.checkboxTime = QCheckBox("Timestamp", self)
        self.checkboxTime.setToolTip("Display timestamps in Pos NED Map plot")
        self.checkboxTime.stateChanged.connect(self.changeTimeCheckbox)
        self.xAxisSample = QCheckBox("XAxis Index", self)
        self.xAxisSample.stateChanged.connect(self.changeXAxisSampleCheckbox)
        self.checkboxUtc = QCheckBox("UTC", self)
        self.checkboxUtc.setToolTip("Display UTC time")
        self.checkboxUtc.stateChanged.connect(self.changeUtcCheckbox)

        self.saveAllPushButton = QPushButton("Save All Plots")
        self.saveAllPushButton.setToolTip("Save all plots to file")
        self.saveAllPushButton.clicked.connect(self.saveAllPlotsToFile)

        self.VLayoutOptions1 = QVBoxLayout()
        self.VLayoutOptions1.addWidget(self.checkboxResidual)
        self.VLayoutOptions1.addWidget(self.checkboxTime)
        self.VLayoutOptions1.addWidget(self.xAxisSample)
        self.VLayoutOptions1.addWidget(self.checkboxUtc)
        self.VLayoutOptions1.setSpacing(0)
        self.VLayoutOptions2 = QVBoxLayout()
        self.VLayoutOptions2.addWidget(self.saveAllPushButton)
        self.VLayoutOptions2.setSpacing(0)

        group_box = QGroupBox("")
        self.LayoutVTests = QVBoxLayout()
        self.LayoutVTests.setSpacing(0)
        group_box.setLayout(self.LayoutVTests)


        self.LayoutBelowPlotSelection = QHBoxLayout()
        self.LayoutBelowPlotSelection.addLayout(self.VLayoutOptions1)
        self.LayoutBelowPlotSelection.addLayout(self.VLayoutOptions2)
        self.LayoutBelowPlotSelection.addItem(QSpacerItem(0, 0, QSizePolicy.Expanding, QSizePolicy.Minimum))
        self.LayoutBelowPlotSelection.addWidget(group_box)

        self.controlLayout.addLayout(self.LayoutBelowPlotSelection)

    def createPlotSelectionPost(self):
        self.VLayoutOptions1.addItem(QSpacerItem(0, 0, QSizePolicy.Minimum, QSizePolicy.Expanding))
        self.VLayoutOptions2.addItem(QSpacerItem(0, 0, QSizePolicy.Minimum, QSizePolicy.Expanding))
        self.LayoutVTests.addItem(QSpacerItem(0, 0, QSizePolicy.Minimum, QSizePolicy.Expanding))

    def createFileTree(self):
        self.dirModel = QFileSystemModel()
        self.dirModel.setRootPath(self.config["root_path"])
        self.dirModel.setFilter(QtCore.QDir.Dirs | QtCore.QDir.NoDotAndDotDot)
        self.recentDirsPushButton = QPushButton("⋯")
        self.recentDirsPushButton.setFixedWidth(25)
        self.recentDirsPushButton.clicked.connect(self.showRootPathsHistMenu)
        self.upDirPushButton = QPushButton()
        self.upDirPushButton.setIcon(self.style().standardIcon(QStyle.SP_ArrowUp))
        self.upDirPushButton.clicked.connect(self.fileTreeUpDir)
        self.dirLineEdit = QLineEdit()
        self.dirLineEdit.setText(self.config["root_path"])
        self.dirLineEdit.setFixedHeight(25)
        self.dirLineEdit.returnPressed.connect(self.handleTreeDirChange)
        self.fileTree = QTreeView()
        self.fileTree.setModel(self.dirModel)
        self.fileTree.setRootIndex(self.dirModel.index(self.config['root_path']))        
        self.fileTree.setColumnHidden(1, True)
        self.fileTree.setColumnHidden(2, True)
        self.fileTree.setColumnHidden(3, True)
        self.fileTree.setMinimumWidth(300)
        self.fileTree.clicked.connect(self.handleTreeViewClick)
        self.fileTree.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        self.fileTree.setSelectionMode(QAbstractItemView.SingleSelection) 
        self.fileTree.customContextMenuRequested.connect(self.handleTreeViewRightClick)
        self.fileTree.doubleClicked.connect(self.setTreeViewDirectoryRoot)
        # self.populateRMSCheck(self.config['root_path'])

        self.controlDirLayout = QHBoxLayout()
        self.controlDirLayout.addWidget(self.recentDirsPushButton)
        self.controlDirLayout.addWidget(self.upDirPushButton)
        self.controlDirLayout.addWidget(self.dirLineEdit)
        self.controlLayout.addLayout(self.controlDirLayout)
        self.controlLayout.addWidget(self.fileTree)

        self.setCurrentListRow(self.nameList.index('Pos NED Map'))   # Default to NED Map
        # self.setCurrentListRow(self.nameList.index('Delta Time'))   # Default to NED Map        

    def createStatus(self):
        self.statusLabel = QLabel()
        self.controlLayout.addWidget(self.statusLabel)
        # self.statusLabel.setVisible(False)

    def setStatus(self, str):
        # self.statusLabel.setVisible(str != "")     # Hide status if string is empty
        self.statusLabel.setText(str)
        QtCore.QCoreApplication.processEvents() # refresh UI

    def hideControl(self):
        self.controlWidget.setVisible(not self.controlWidget.isVisible())
        if self.controlWidget.isVisible():
            self.hideControlButton.setText("Hide Panel")
        else:
            self.hideControlButton.setText("Show Panel")
            
    def newWindow(self):
        subprocess.Popen([sys.executable,  self.exePath, self.config['directory']])

    def createBottomToolbar(self):
        self.toolLayout = QHBoxLayout()
        self.toolLayout.addWidget(self.mplots[0].toolbar)

        self.hideControlButton = QPushButton("Hide Panel")
        self.hideControlButton.setToolTip("Hide/show left side control panel of the LogInspector.")
        self.hideControlButton.clicked.connect(self.hideControl)
        self.toolLayout.addWidget(self.hideControlButton)

        self.popPlotButton = QPushButton("New Plot")
        self.popPlotButton.setToolTip("Open current plot in separate dialog window.")
        self.popPlotButton.clicked.connect(self.popPlot)
        self.toolLayout.addWidget(self.popPlotButton)

        self.newAppButton = QPushButton("New App")
        self.newAppButton.setToolTip("Open a new instance of the LogInspector.")
        self.newAppButton.clicked.connect(self.newWindow)
        self.toolLayout.addWidget(self.newAppButton)

        self.toolLayout.addItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding))
        # self.toolLayout.addWidget(QSpacerItem(150, 10, QSizePolicy.Expanding))

        self.copyImagePushButton = QPushButton()
        self.copyImagePushButton.setToolTip("Copy the current plot to the system clipboard.")
        self.copyImagePushButton.setIcon(self.style().standardIcon(QStyle.SP_DialogSaveButton))
        self.toolLayout.addWidget(self.copyImagePushButton)
        self.copyImagePushButton.clicked.connect(self.copyPlotToClipboard)

        downsampleLabel = QLabel()
        downsampleLabel.setText("DS")
        self.downSampleInput = QSpinBox()
        self.downSampleInput.setToolTip("Adjust downsample rate, reducing the number of the displayed data samples to increase plotting speed.")
        self.downSampleInput.setMinimum(1)
        self.downSampleInput.setValue(self.downsample)
        self.toolLayout.addWidget(downsampleLabel)
        self.toolLayout.addWidget(self.downSampleInput)
        self.downSampleToOne = QPushButton()
        self.downSampleToOne.setToolTip("Set data downsample rate to 1.")
        self.downSampleToOne.setMinimumWidth(1)
        self.downSampleToOne.setMaximumWidth(20)
        self.downSampleToOne.setSizePolicy(QSizePolicy(QSizePolicy.Minimum, QSizePolicy.Fixed))
        self.downSampleToOne.setText("1")
        self.toolLayout.addWidget(self.downSampleToOne)
        self.downSampleToOne.clicked.connect(self.setDownSampleToOne)
        self.downSampleInput.valueChanged.connect(self.changeDownSample)

    def changeResidualCheckbox(self, state):
        for mplot in self.mplots:
            if mplot.plotter:
                mplot.plotter.enableResidualPlot(state)
                self.updatePlot()

    def changeTimeCheckbox(self, state):
        for mplot in self.mplots:
            if mplot.plotter:
                mplot.plotter.enableTimestamp(state)
                self.updatePlot()

    def changeXAxisSampleCheckbox(self, state):
        for mplot in self.mplots:
            if mplot.plotter:
                mplot.plotter.enableXAxisSample(state)
                self.updatePlot()

    def changeUtcCheckbox(self, state):
        for mplot in self.mplots:
            if mplot.plotter:
                mplot.plotter.enableUtcTime(state)
                self.updatePlot()

    def saveAllPlotsToFile(self):
        if self.log == None:
            print("Log not opened.  Please select a log directory.")
            return
        print("Saving all plots file")
        for mplot in self.mplots:
            mplot.plotter.save = True
        for i in range(len(self.funcNameList)):
            if self.setCurrentListRow(i) and self.funcNameList[i] != None:
                self.plot(self.selectedPlot(), self.plotargs)
            QtCore.QCoreApplication.processEvents()
        for mplot in self.mplots:
            mplot.plotter.save = False

    def changeDownSample(self, val):
        self.downsample = max(val, 1)
        for mplot in self.mplots:
            mplot.plotter.setDownSample(self.downsample)
        self.reload()

    def setDownSampleToOne(self):
        self.downSampleInput.setValue(1)

    def copyPlotToClipboard(self):
        # pixmap = QPixmap.grabWidget(self.mplot.canvas)
        # QApplication.clipboard().setPixmap(pixmap)
        # pixmap.save('test.png')

        # store the image in a buffer using savefig(), this has the
        # advantage of applying all the default savefig parameters
        # such as background color; those would be ignored if you simply
        # grab the canvas using Qt
        buf = io.BytesIO()
        self.mplots[0].figure.savefig(buf)

        QApplication.clipboard().setImage(QImage.fromData(buf.getvalue()))
        buf.close()

    def dragEnterEvent(self, e):
        if (e.mimeData().hasUrls()):
            e.acceptProposedAction()

    def dropEvent(self, e):
        try:
            directory = e.mimeData().urls()[0].toLocalFile()
            self.load(directory)
        except Exception as e:
            self.showError(e)

    def showError(self, e):
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Critical)
        msg.setText("Unable to load log: " + e.__str__())
        msg.setDetailedText(traceback.format_exc())
        msg.exec()

    def updateFileTree(self):
        self.dirModel.setRootPath(self.config["root_path"])
        self.fileTree.setRootIndex(self.dirModel.index(self.config['root_path']))

    def expandAndSelectDirectory(self, directory):
        # Find the index for the folder
        index = self.dirModel.index(directory)
        # Expand the tree view to this folder
        self.fileTree.expand(index)
        # Select the folder
        self.fileTree.setCurrentIndex(index)
        self.fileTree.selectionModel().select(index, QItemSelectionModel.Select)        
        # Scroll to the selected item
        self.fileTree.scrollTo(index)
        
    def populateRMSCheck(self, directory):
        for subdir in os.listdir(directory):
            path = os.path.join(directory, subdir)
            if os.path.isdir(path):
                self.populateRMSCheck(path)
            elif 'RMS' in subdir:
                f = open(path)
                rms_report = f.read()
                p = re.compile(r'(?<=^PASS/FAIL).*\n', re.M)
                line = re.search(p, rms_report).group()
                failed = True if "FAIL" in line else False
                if failed:
                    pass
                else:
                    pass

    def loadConfigFromFile(self):
        if os.path.exists(self.configFilePath):
            # log_inspector.yaml found.  Read from file.
            file = open(self.configFilePath, 'r')
            self.config = yaml.load(file, Loader=yaml.FullLoader)
            file.close()
            if 'root_path' not in self.config and 'logs_directory' in self.config:
                self.config['root_path'] = self.config['logs_directory']
            if 'root_path_hist' not in self.config:
                self.config['root_path_hist'] = []
        else:
            # log_inspector.yaml not found.  Create new file.
            self.config = {}
            self.config['root_path'] = os.path.join(os.path.expanduser("~"), "Documents", "Inertial_Sense", "Logs")
            self.config['root_path_hist'] = []
            self.config['directory'] = ""
            self.config['serials'] = ["ALL"]
            self.saveConfigToFile()

    def saveConfigToFile(self):
        file = open(self.configFilePath, 'w')
        yaml.dump(self.config, file)
        file.close()

    def handleTreeDirChange(self):
        self.config["root_path"] = os.path.normpath(self.dirLineEdit.text())
        self.updateFileTree()

    def handleTreeViewClick(self):
        self.config['directory'] = self.fileTree.model().filePath(self.fileTree.selectedIndexes()[0])
        self.saveConfigToFile()

        for fname in os.listdir(self.config['directory']):
            fname = fname.lower()
            if fname.endswith('.dat') or \
               fname.endswith('.raw') or \
               fname.endswith('.sdat'):
                try:
                    self.load(self.config['directory'])
                except Exception as e:
                    self.showError(e)
                break
            
    def fileTreeUpDir(self):
        self.setRootPath(os.path.abspath(os.path.join(self.dirLineEdit.text(), '..')))

    def setRootPath(self, directory):
        self.dirLineEdit.setText(os.path.normpath(directory))
        self.config["root_path"] = self.dirLineEdit.text()
        self.handleTreeDirChange()        

    def updateRootPathHist(self):
        directory = os.path.normpath(self.config["root_path"])
        root_path_hist = self.config["root_path_hist"]

        # Remove the directory if it already exists
        if directory in root_path_hist:
            root_path_hist.remove(directory)

        # Insert it at the beginning
        root_path_hist.insert(0, directory)

        # Keep only the first 5 entries (most recent)
        self.config["root_path_hist"] = root_path_hist[:5]

        self.dirLineEdit.setText(directory)

    def showRootPathsHistMenu(self):
        menu = QMenu()
        current_root = self.config.get("root_path")

        # Add each item to the menu, skipping the current root path
        for item in self.config["root_path_hist"]:
            if item == current_root:
                continue
            action = menu.addAction(item)
            action.triggered.connect(lambda checked=False, text=item: self.on_select_recent_dir(text))

        # Show the menu just below the button
        menu.exec_(self.recentDirsPushButton.mapToGlobal(self.recentDirsPushButton.rect().bottomLeft()))     

    def on_select_recent_dir(self, text):
        # Handle the selected item
        self.setRootPath(text)
        self.updateFileTree()
        self.saveConfigToFile()

    def runNpp(self, directory, startMode):
        cleanFolder(directory)
        setDataInformationDirectory(directory, startMode=startMode)
        sys.path.insert(1, '../../../../python/src')
        params_flename = directory + '/params.yaml'
        data = {
            'name': 'test_imx',
            'results_directory': ".",
            'directory': directory,
            'logs': ['.'],
            'blacklist_logs': [],
            'options': [],
            'run_test': [],
            'reprocess': True
        }
        with open(params_flename, 'w') as file:
            yaml.dump(data, file, default_flow_style=False)
        from supernpp.supernpp import SuperNPP
        spp = SuperNPP(params_flename, serials=self.config['serials'], startMode=startMode)
        self.setStatus(("NPP %s running..." % (startModes[startMode])))
        spp.run_reprocess()
        
        # Expand file tree so "post_processed" directory is visable
        self.updateFileTree()
        QtCore.QCoreApplication.processEvents() # refresh UI
        index = self.fileTree.selectedIndexes()[0]
        self.fileTree.expand(index)
        QtCore.QCoreApplication.processEvents() # refresh UI
        
        # Select post_processed directory in file tree view
        for row in range(index.model().rowCount(index)):
            if self.fileTree.model().fileName(index.child(row, 0)) == "post_processed":
                self.fileTree.setCurrentIndex(index.child(row, 0))
                QtCore.QCoreApplication.processEvents() # refresh UI
                self.handleTreeViewClick()
                continue
            
        self.setStatus("NPP done.")

    def selectedDirectory(self):
        directory = os.path.normpath(self.fileTree.model().filePath(self.fileTree.selectedIndexes()[0]))
        return directory

    def setTreeViewDirectoryRoot(self, event):
        directory = self.fileTree.model().filePath(self.fileTree.selectedIndexes()[0])
        self.dirLineEdit.setText(directory)
        self.handleTreeDirChange()

    def handleTreeViewRightClick(self, event):
        directory = self.selectedDirectory()
        menu = QMenu(self)
        copyAction                  = menu.addAction("Copy path")
        nppActionHot                = menu.addAction("Run NPP, HOT")
        nppActionCold               = menu.addAction("Run NPP, COLD")
        nppActionFactory            = menu.addAction("Run NPP, FACTORY")
        setDataInfoDirHotAction     = menu.addAction("Set dataInfo.json directory, HOT")
        setDataInfoDirColdAction    = menu.addAction("Set dataInfo.json directory, COLD")
        setDataInfoDirFactoryAction = menu.addAction("Set dataInfo.json directory, FACTORY")
        exploreAction               = menu.addAction("Explore folder")
        cleanFolderAction           = menu.addAction("Clean folder")
        deleteFolderAction          = menu.addAction("Delete folder")
        action = menu.exec_(self.fileTree.viewport().mapToGlobal(event))
        if action == copyAction:
            cb = QApplication.clipboard()
            cb.clear(mode=cb.Clipboard )
            cb.setText(directory, mode=cb.Clipboard)
        if action == nppActionHot:
            self.runNpp(directory, START_MODE_HOT)
        if action == nppActionCold:
            self.runNpp(directory, START_MODE_COLD)
        if action == nppActionFactory:
            self.runNpp(directory, START_MODE_FACTORY)
        if action == setDataInfoDirHotAction:
            setDataInformationDirectory(directory, startMode=START_MODE_HOT)
        if action == setDataInfoDirColdAction:
            setDataInformationDirectory(directory, startMode=START_MODE_COLD)
        if action == setDataInfoDirFactoryAction:
            setDataInformationDirectory(directory, startMode=START_MODE_FACTORY)
        if action == exploreAction:
            openFolderWithFileBrowser(directory)
        if action == cleanFolderAction:
            cleanFolder(directory)
        if action == deleteFolderAction:
            msg = QMessageBox(self)
            msg.setIcon(QMessageBox.Question)
            msg.setText("Are you sure you want to delete this folder?\n\n" + directory)
            msg.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
            result = msg.exec()
            if result == QMessageBox.Yes:
                removeDirectory(directory)

    def showDeviceInfo(self):
        dlg = DeviceInfoDialog(self.log, self)
        dlg.show()
        dlg.exec_()

    def showFlashConfig(self):
        dlg = FlashConfigDialog(self.log, self)
        dlg.show()
        dlg.exec_()

    def plotMPlot(self, mplot, func, args):
        mplot.figure.clear()
        if hasattr(mplot, 'plotter'):
            if args is not None:
                getattr(mplot.plotter, func)(*args, mplot.figure)
            else:
                getattr(mplot.plotter, func)(mplot.figure)
        mplot.canvas.draw()

    def plot(self, func, args=None):
        if func is None:
            return
        print("plotting " + func)
        self.selectedPlotFunc = func
        self.plotargs = args
        ax = None

        for mplot in self.mplots:
            mplot.figure.clear()

            if mplot == self.mplots[0] or mplot.func is None:
                # Set plot function if first in list or if not defined.  This keeps the popup plots from changing
                mplot.func = func

            if hasattr(mplot, 'plotter'):
                if args is not None:
                    ax = getattr(mplot.plotter, mplot.func)(*args, mplot.figure, axs=ax)
                else:
                    ax = getattr(mplot.plotter, mplot.func)(mplot.figure, axs=ax)
            mplot.canvas.draw()

        print("done plotting")

def kill_handler(*args):
    instance = QApplication.instance()
    instance.quit()

if __name__ == '__main__':
    if sys.version[0] != '3':
        raise Exception("You must use Python 3. The current version is " + sys.version)

    if os.name == 'nt':
        # On Windows, this is required to get the icon changed in the taskbar
        myappid = 'InertialSense.PythonTools.LogInspector.Any'
        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(myappid)

    app = QApplication(sys.argv)
    
    configFilePath = os.path.join(os.path.expanduser("~"), "Documents", "Inertial_Sense", "log_inspector.yaml")

    main = LogInspectorWindow(configFilePath)
    main.setupUi()

    # Allow the process to be killed with Ctrl-C from terminal
    timer = QtCore.QTimer()
    timer.start(200)
    timer.timeout.connect(lambda: None)
    signal.signal(signal.SIGINT, kill_handler)

    main.show()

    if len(sys.argv) > 1:
        directory = sys.argv[1]
        main.load(directory)

    app.exec()

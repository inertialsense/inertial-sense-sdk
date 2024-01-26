#!/usr/bin/python3

import sys, os, signal, ctypes, shutil, json, io, traceback, yaml, subprocess, re
from PyQt5 import QtCore
from PyQt5.QtWidgets import QWidget, QDialog, QApplication, QPushButton, QVBoxLayout, QLineEdit, QTreeView, QFileSystemModel,\
    QHBoxLayout, QMainWindow, QSizePolicy, QSpacerItem, QFileDialog, QMessageBox, QLabel, QAbstractItemView, QMenu,\
    QTableWidget,QTableWidgetItem, QSpinBox, QCheckBox, QGroupBox, QListView, QStyle
from PyQt5.QtGui import QMovie, QIcon, QPixmap, QImage, QStandardItemModel, QStandardItem

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
        for root, dirs, files in os.walk(path):
            for filename in files:
                if "LOG_SN" in filename:
                    serialnum = filename[4:11]
                    if serialnum not in serialnumbers:
                        serialnumbers += [serialnum]

        data['processData'] = {}
        data['processData']['datasets'] = [{}]
        data['processData']['datasets'][0]['SerialNumbers'] = serialnumbers
        data['processData']['datasets'][0]['folder'] = os.path.basename(path)
        data['processData']['datasets'][0]['logType'] = 'DAT'
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
    return str(array[0]) + '.' + str(array[1]) + '.' + str(array[2]) #+ '.' + str(array[3])

def dateTimeArrayToString(date, time):
    return str(date[1]+2000) + '-' + f'{date[2]:02}' + '-' + f'{date[3]:02}' + ' ' + f'{time[0]:02}' + ':' + f'{time[1]:02}' + ':' + f'{time[2]:02}'

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
            data = dev[DID_DEV_INFO][0]
            self.table.setRowCount(d+1)
            self.table.setItem(d, 0, QTableWidgetItem(str(data[1])))                         # Serial#
            self.table.setItem(d, 1, QTableWidgetItem(verArrayToString(data[2])))  # Hardware version
            self.table.setItem(d, 2, QTableWidgetItem(verArrayToString(data[3])))  # Firmware version
            self.table.setItem(d, 3, QTableWidgetItem(str(data[4])))   # Build
            self.table.setItem(d, 4, QTableWidgetItem(verArrayToString(data[5])))  # Protocol
            self.table.setItem(d, 5, QTableWidgetItem(str(data[6])))   # Repo
            self.table.setItem(d, 6, QTableWidgetItem(dateTimeArrayToString(data[8], data[9]))) # Build Date & Time
            self.table.setItem(d, 7, QTableWidgetItem(data[7].decode('UTF-8')))         # Manufacturer
            self.table.setItem(d, 8, QTableWidgetItem(data[10].decode('UTF-8')))        # Additional Info

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

        hex_fields = ['ioConfig', 'platformConfig', 'RTKCfgBits', 'sysCfgBits']
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




class LogInspectorWindow(QMainWindow):
    def __init__(self, configFilePath):
        super(LogInspectorWindow, self).__init__()
        self.initMatPlotLib()
        self.configFilePath = configFilePath

        folder = os.path.dirname(self.configFilePath)
        if not os.path.exists(folder):
            os.makedirs(folder)

        if os.path.exists(self.configFilePath):
            # config.yaml found.  Read from file.
            file = open(self.configFilePath, 'r')
            self.config = yaml.load(file, Loader=yaml.FullLoader)
            file.close()
        else:
            # config.yaml not found.  Create new file.
            self.config = {}
            self.config['logs_directory'] = os.path.join(os.path.expanduser("~"), "Documents", "Inertial_Sense", "Logs")
            self.config['directory'] = ""
            self.config['serials'] = ["ALL"]
            file = open(self.configFilePath, 'w')
            yaml.dump(self.config, file)
            file.close()

        self.selectedIndex = 0
        self.downsample = 5
        self.plotargs = None
        self.log = None
        self.plotter = logPlot(False, False, 'svg', None)

    def initMatPlotLib(self):
        self.figure = plt.figure()
        self.canvas = FigureCanvas(self.figure)
        self.toolbar = NavigationToolbar(self.canvas, self)
        self.figure.subplots_adjust(left=0.05, right=0.99, bottom=0.05, top=0.91, wspace=0.2, hspace=0.2)

    def addButton(self, name, function, layout=None):
        setattr(self, name + "button", QPushButton(name))
        getattr(self, name + "button").clicked.connect(function)
        # getattr(self, name + "button").setMinimumWidth(220)
        if layout is None:
            layout = self.buttonLayoutColIns

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

    def updateWindowTitle(self):
        try:
            size = self.log.numDev
            if  size != 0:
                info = self.log.data[0,DID_DEV_INFO][0]
                if size == 1:
                    infoStr = 'SN' + str(info[1]) + ', H:' + verArrayToString(info[2]) + ', F:' + verArrayToString(info[3]) + ' build ' + str(info[4]) + ', ' + dateTimeArrayToString(info[8], info[9]) + ', ' + info[10].decode('UTF-8')
                else:
                    infoStr = 'Devices: [' + " ".join([str(x) for x in self.log.serials]) + "]"
                if self.log.using_mounting_bias:
                    infoStr += ', Mounting Corrected'
                self.setWindowTitle("LogInspector  -  " + infoStr)
        except:
            self.setWindowTitle("DID_DEV_INFO missing")

    def choose_directory(self):
        log_dir = config['logs_directory']
        config['directory'] = QFileDialog.getExistingDirectory(parent=self, caption='Choose Log Directory', directory=log_dir)

        if directory != '':
            try:
                self.load(config['directory'])
            except Exception as e:
                msg = QMessageBox()
                msg.setIcon(QMessageBox.Critical)
                msg.setText("Unable to load log: " + e.__str__())
                msg.setDetailedText(traceback.format_exc())
                msg.exec()

    def load(self, directory):
        print("loading files from " + directory)
        self.setStatus("Loading...")
        # if self.log is None:
        self.log = Log()
        self.log.load(directory)
        print("done loading")
        self.plotter.setLog(self.log)
        self.plotter.setDownSample(self.downsample)
        self.updatePlot()
        self.setStatus("")

    def setupUi(self):
        self.setObjectName("LogInspector")
        self.setWindowTitle("LogInspector")
        self.resize(1280, 900)
        self.setWindowIcon(QIcon('assets/Magnifying_glass_icon.png'))

        self.controlLayout = QVBoxLayout()
        self.createPlotSelection()
        self.createFileTree()
        self.createStatus()
        self.controlLayout.setStretch(0, 2)     # Plot selection
        self.controlLayout.setStretch(3, 1)     # File tree
        self.createBottomToolbar()

        self.figureLayout = QVBoxLayout()
        self.figureLayout.addWidget(self.canvas)
        self.figureLayout.addLayout(self.toolLayout)
        self.figureLayout.setStretchFactor(self.canvas, 1)

        layout = QHBoxLayout()
        layout.addLayout(self.controlLayout)
        layout.addLayout(self.figureLayout)
        layout.setStretch(1, 1)
        widget = QWidget()
        widget.setLayout(layout)
        self.setCentralWidget(widget)
        self.resize(1450, 1000)
        self.setAcceptDrops(True)

    def createListIns(self):
        self.addListSection('INS/AHRS')
        self.addListItem('Pos NED Map', 'posNEDMap')
        self.addListItem('Pos NED', 'posNED')
        self.addListItem('Pos LLA', 'posLLA')
        self.addListItem('Vel NED', 'velNED')
        self.addListItem('Vel UVW', 'velUVW')
        self.addListItem('Attitude', 'attitude')
        self.addListItem('Altitude', 'altitude')
        self.addListItem('Heading', 'heading')
        self.addListItem('INS Status', 'insStatus')
        self.addListItem('HDW Status', 'hdwStatus')

    def createListSensors(self):
        self.addListSection('SENSORS')
        self.addListItem('IMU PQR', 'imuPQR')
        self.addListItem('IMU Accel', 'imuAcc')
        self.addListItem('PSD PQR', 'gyroPSD')
        self.addListItem('PSD Accel', 'accelPSD')
        self.addListItem('Magnetometer', 'magnetometer')
        self.addListItem('Temp', 'temp')

    def createListGps(self):
        self.addListSection('GPS')
        self.addListItem('GPS LLA', 'gpsLLA')
        self.addListItem('GPS 1 Stats', 'gpsStats')
        self.addListItem('GPS 2 Stats', 'gps2Stats')
        self.addListItem('RTK Pos Stats', 'rtkPosStats')
        self.addListItem('RTK Cmp Stats', 'rtkCmpStats')
        self.addListItem('GPS Position NED Map', 'gpsPosNEDMap')
        self.addListItem('GPS Position NED', 'gpsPosNED')
        self.addListItem('GPS Velocity NED', 'gpsVelNED')

    def createListSystem(self):
        self.addListSection('SYSTEM')
        self.addListItem('Flash Config', lambda: self.showFlashConfig())
        self.addListItem('Device Info', lambda: self.showDeviceInfo())

    def createListGeneral(self):
        self.addListSection('GENERAL')

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
        self.functionList = []
        self.funcNameList = []
        self.listView.setModel(self.modelList)
        self.listView.clicked.connect(self.onSelectListItem)
        LayoutList = QHBoxLayout()
        LayoutList.addWidget(self.listView)
        groupBox.setLayout(LayoutList)
        self.controlLayout.addWidget(groupBox)        

        self.createListIns()
        self.createListSensors()
        self.createListGps()
        self.createListSystem()
        self.createListGeneral()
        self.checkboxResiduals = QCheckBox("Residuals", self)
        self.checkboxResiduals.stateChanged.connect(self.changeResidualsCheckbox)
        self.LayoutBelowPlotSelection = QHBoxLayout()
        self.LayoutBelowPlotSelection.addWidget(self.checkboxResiduals)

        self.saveAllPushButton = QPushButton(" Save All Plots ")
        self.saveAllPushButton.clicked.connect(self.saveAllPlotsToFile)
        hSpacer = QSpacerItem(0, 0, QSizePolicy.Expanding, QSizePolicy.Minimum) 
        self.LayoutBelowPlotSelection.addItem(hSpacer)
        self.LayoutBelowPlotSelection.addWidget(self.saveAllPushButton)

        self.controlLayout.addLayout(self.LayoutBelowPlotSelection)
    
    # def findTreeIndex(self, directory):
        

    def createFileTree(self):
        self.dirModel = QFileSystemModel()
        self.dirModel.setRootPath(self.config["logs_directory"])
        self.dirModel.setFilter(QtCore.QDir.Dirs | QtCore.QDir.NoDotAndDotDot)
        self.upDirPushButton = QPushButton()
        self.upDirPushButton.setIcon(self.style().standardIcon(QStyle.SP_ArrowUp))
        self.upDirPushButton.clicked.connect(self.fileTreeUpDir)
        self.dirLineEdit = QLineEdit()
        self.dirLineEdit.setText(self.config["logs_directory"])
        self.dirLineEdit.setFixedHeight(25)
        self.dirLineEdit.returnPressed.connect(self.handleTreeDirChange)
        self.fileTree = QTreeView()
        self.fileTree.setModel(self.dirModel)
        self.fileTree.setRootIndex(self.dirModel.index(self.config['logs_directory']))        
        self.fileTree.setColumnHidden(1, True)
        self.fileTree.setColumnHidden(2, True)
        self.fileTree.setColumnHidden(3, True)
        self.fileTree.setMinimumWidth(300)
        self.fileTree.clicked.connect(self.handleTreeViewClick)
        self.fileTree.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        self.fileTree.setSelectionMode(QAbstractItemView.SingleSelection) 
        self.fileTree.customContextMenuRequested.connect(self.handleTreeViewRightClick)
        self.fileTree.doubleClicked.connect(self.setTreeViewDirectoryRoot)
        # self.populateRMSCheck(self.config['logs_directory'])

        self.controlDirLayout = QHBoxLayout()
        self.controlDirLayout.addWidget(self.upDirPushButton)
        self.controlDirLayout.addWidget(self.dirLineEdit)
        self.controlLayout.addLayout(self.controlDirLayout)
        self.controlLayout.addWidget(self.fileTree)

        # self.buttonLayout.addItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding))
        # self.addButton('load', self.choose_directory)

        self.setCurrentListRow(1)   # Default to NED Map

    def createStatus(self):
        self.statusLabel = QLabel()
        self.controlLayout.addWidget(self.statusLabel)
        # self.statusLabel.setVisible(False)

    def setStatus(self, str):
        # self.statusLabel.setVisible(str != "")     # Hide status if string is empty
        self.statusLabel.setText(str)
        QtCore.QCoreApplication.processEvents() # refresh UI

    def createBottomToolbar(self):
        self.toolLayout = QHBoxLayout()
        self.toolLayout.addWidget(self.toolbar)

        self.toolLayout.addItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding))
        # self.toolLayout.addWidget(QSpacerItem(150, 10, QSizePolicy.Expanding))

        self.copyImagePushButton = QPushButton()
        # self.copyImagePushButton.setText("Copy")
        # self.copyImagePushButton.setMinimumWidth(1)
        # self.copyImagePushButton.style().standardIcon(QStyle.SP_DialogOpenButton)
        self.copyImagePushButton.setIcon(self.style().standardIcon(QStyle.SP_DialogSaveButton))
        self.toolLayout.addWidget(self.copyImagePushButton)
        self.copyImagePushButton.clicked.connect(self.copyPlotToClipboard)

        downsampleLabel = QLabel()
        downsampleLabel.setText("DS")
        self.downSampleInput = QSpinBox()
        self.downSampleInput.setValue(self.downsample)
        self.toolLayout.addWidget(downsampleLabel)
        self.toolLayout.addWidget(self.downSampleInput)
        self.downSampleInput.valueChanged.connect(self.changeDownSample)

    def changeResidualsCheckbox(self, state):
        if self.plotter:
            self.plotter.enableResidualPlot(state)
            self.updatePlot()

    def saveAllPlotsToFile(self):
        if self.log == None:
            print("Log not opened.  Please select a log directory.")
            return
        print("Saving all plots file")
        self.plotter.save = True
        for i in range(len(self.funcNameList)):
            if self.setCurrentListRow(i) and self.funcNameList[i] != None:
                self.plot(self.selectedPlot(), self.plotargs)
            QtCore.QCoreApplication.processEvents()
        self.plotter.save = False

    def changeDownSample(self, val):
        self.downsample = val
        self.plotter.setDownSample(self.downsample)
        self.updatePlot()

    def copyPlotToClipboard(self):
        # pixmap = QPixmap.grabWidget(self.canvas)
        # QApplication.clipboard().setPixmap(pixmap)
        # pixmap.save('test.png')

        # store the image in a buffer using savefig(), this has the
        # advantage of applying all the default savefig parameters
        # such as background color; those would be ignored if you simply
        # grab the canvas using Qt
        buf = io.BytesIO()
        self.figure.savefig(buf)

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
        self.dirModel.setRootPath(self.config["logs_directory"])
        self.fileTree.setRootIndex(self.dirModel.index(self.config['logs_directory']))

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

    def handleTreeDirChange(self):
        self.config["logs_directory"] = self.dirLineEdit.text()
        self.updateFileTree()

        file = open(self.configFilePath, 'w')
        yaml.dump(self.config, file)
        file.close()

    def handleTreeViewClick(self):
        self.config['directory'] = self.fileTree.model().filePath(self.fileTree.selectedIndexes()[0])
        file = open(self.configFilePath, 'w')
        yaml.dump(self.config, file)
        file.close()

        for fname in os.listdir(self.config['directory']):
            if fname.endswith('.dat'):
                try:
                    self.load(self.config['directory'])
                except Exception as e:
                    self.showError(e)
                break
            
    def fileTreeUpDir(self):
        self.dirLineEdit.setText(os.path.abspath(os.path.join(self.dirLineEdit.text(), '..')))
        self.config["logs_directory"] = self.dirLineEdit.text()
        self.handleTreeDirChange()        
            
    def runNpp(self, directory, startMode):
        cleanFolder(directory)
        setDataInformationDirectory(directory, startMode=startMode)
        sys.path.insert(1, '../../../../python/src')
        from supernpp.supernpp import SuperNPP
        spp = SuperNPP(directory, self.config['serials'], startMode=startMode)
        self.setStatus(("NPP %s running..." % (startModes[startMode])))
        spp.run()
        
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

    def setTreeViewDirectoryRoot(self, event):
        directory = self.fileTree.model().filePath(self.fileTree.selectedIndexes()[0])
        self.dirLineEdit.setText(directory)
        self.handleTreeDirChange()

    def handleTreeViewRightClick(self, event):
        directory = os.path.normpath(self.fileTree.model().filePath(self.fileTree.selectedIndexes()[0]))
        menu = QMenu(self)
        copyAction                  = menu.addAction("Copy path")
        nppActionHot                = menu.addAction("Run NPP, HOT start")
        nppActionCold               = menu.addAction("Run NPP, COLD start")
        nppActionFactory            = menu.addAction("Run NPP, FACTORY start")
        setDataInfoDirHotAction     = menu.addAction("Set dataInfo.json directory, HOT start")
        setDataInfoDirColdAction    = menu.addAction("Set dataInfo.json directory, COLD start")
        setDataInfoDirFactoryAction = menu.addAction("Set dataInfo.json directory, FACTORY start")
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


    def plot(self, func, args=None):
        print("plotting " + func)
        self.selectedPlotFunc = func
        self.plotargs = args

        self.figure.clear()

        if hasattr(self, 'plotter'):
            if args is not None:
                getattr(self.plotter, func)(*args, self.figure)
            else:
                getattr(self.plotter, func)(self.figure)

        self.canvas.draw()
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

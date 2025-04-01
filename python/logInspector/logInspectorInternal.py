#!/usr/bin/env python3

from logInspector import LogInspectorWindow

import subprocess
import sys, os, signal, ctypes, yaml
from PyQt5 import QtCore
from PyQt5.QtWidgets import QDialog, QApplication, QPushButton, QVBoxLayout, QCheckBox
from PyQt5.QtWidgets import QApplication


class ChooseDevsDialog(QDialog):
    def __init__(self, parent):
        super(ChooseDevsDialog, self).__init__(parent)
        self.setWindowTitle("Choose Devices")
        self.parent = parent
        self.mainLayout = QVBoxLayout()

        self.selectAllButton = QPushButton()
        self.selectAllButton.setText("Select All")
        self.selectAllButton.clicked.connect(self.selectAll)
        self.mainLayout.addWidget(self.selectAllButton)

        self.selectNoneButton = QPushButton()
        self.selectNoneButton.setText("Select None")
        self.selectNoneButton.clicked.connect(self.selectNone)
        self.mainLayout.addWidget(self.selectNoneButton)

        self.checkboxes = []
        for i in range(parent.log.numDev):
            checkbox = QCheckBox()
            checkbox.setText(str(parent.log.serials[i]))
            checkbox.setChecked(i in parent.mplots[0].plotter.active_devs)
            checkbox.clicked.connect(self.updatePlot)
            self.checkboxes.append(checkbox)
            self.mainLayout.addWidget(checkbox)

        self.okbutton = QPushButton()
        self.okbutton.setText("OK")
        self.okbutton.clicked.connect(self.clickedOk)
        self.mainLayout.addWidget(self.okbutton)


        self.setLayout(self.mainLayout)

    def updatePlot(self):
        active_serials = []
        for i, checkbox in enumerate(self.checkboxes):
            if checkbox.isChecked():
                active_serials.append(self.parent.log.serials[i])
        for mplot in self.parent.mplots:
            mplot.plotter.setActiveSerials(active_serials)
        self.parent.updatePlot()

    def clickedOk(self):
        self.close()

    def selectAll(self):
        for checkbox in self.checkboxes:
            checkbox.setChecked(True)
        self.updatePlot()

    def selectNone(self):
        for checkbox in self.checkboxes:
            checkbox.setChecked(False)
        self.updatePlot()


class logInspectorInternal(LogInspectorWindow):
    def __init__(self, config):
        super(logInspectorInternal, self).__init__(config)
        self.page = 0
        self.exePath = __file__

    def createListSystem(self):
        super(logInspectorInternal, self).createListSystem()
        self.addListItem('General Fault Codes', 'genFaultCodes')

    def createListIns(self):
        super(logInspectorInternal, self).createListIns()
        self.addListItem('EKF Biases', 'ekfBiases')

    def createListSensors(self):
        self.addListItem('IMU3 Gyro', 'imu3PQR')
        self.addListItem('IMU3 Accel', 'imu3Acc')
        self.addListItem('IMU3 Gyro Combined', 'imu3PqrCombined')
        self.addListItem('IMU3 Accel Combined', 'imu3AccCombined')
        super(logInspectorInternal, self).createListSensors()
        self.addListItem('Allan Var. Gyro', 'allanVariancePQR')
        self.addListItem('Allan Var. Accel', 'allanVarianceAcc')
        self.addListItem('Mag Decl.', 'magDec')
        self.addListItem('Wheel Encoder', 'wheelEncoder')
        self.addListItem('Ground Vehicle Status', 'groundVehicleStatus')
        self.addListItem('Ground Vehicle', 'groundVehicle')
        self.addListItem('Whl Ctrl Time', 'wheelControllerTime')
        self.addListItem('Whl Ctrl Vel', 'wheelControllerVel')

    def createListGeneral(self):
        super(logInspectorInternal, self).createListGeneral()
        self.addListItem('Delta Time', 'deltatime')
        self.addListItem('Debug Int', 'debugiArr')
        self.addListItem('Debug Float', 'debugfArr')
        self.addListItem('Debug Double', 'debuglfArr')
        self.addListItem('SComp Gyr v Temp', 'sensorCompGyrTemp')
        self.addListItem('SComp Acc v Temp', 'sensorCompAccTemp')
        self.addListItem('SComp Mag v Temp', 'sensorCompMagTemp')
        self.addListItem('SComp Gyr', 'sensorCompGyr')
        self.addListItem('SComp Acc', 'sensorCompAcc')
        self.addListItem('SComp Mag', 'sensorCompMag')
        self.addListItem('SComp Gyr Resid', 'linearityGyr')
        self.addListItem('SComp Acc Resid', 'linearityAcc')
        self.addListItem('Phase Residuals', lambda: self.plot('rtkResiduals', ('phase', self.page)))
        self.addListItem('Code Residuals', lambda: self.plot('rtkResiduals', ('code', self.page)))
        self.addListItem('RTK Debug p1', 'rtkDebugP1')
        self.addListItem('RTK Debug p2', 'rtkDebugP2')
        self.addListItem('RTK Dbg 2', 'rtkDebug2')
        self.addListItem('RTK Dbg 2 Sat', 'rtkDebug2Sat')
        self.addListItem('RTK Dbg 2 STD', 'rtkDebug2Std')
        self.addListItem('RTK Dbg 2 Lock', 'rtkDebug2Lock')
        self.addListItem('RTK Pos Misc', 'rtkPosMisc')
        self.addListItem('RTK Cmp Misc', 'rtkCmpMisc')
        self.addListItem('GPS Raw Time', 'gpsRawTime')
        #self.addButton('RTK Rel', lambda: self.plot('rtkRel'))

    def createBottomToolbar(self):
        super(logInspectorInternal, self).createBottomToolbar()
        # pageLabel = QLabel()
        # pageLabel.setText("Page")
        # self.pageInput = QSpinBox()
        # self.pageInput.setValue(self.page)
        # self.toolLayout.addWidget(pageLabel)
        # self.toolLayout.addWidget(self.pageInput)
        # self.pageInput.valueChanged.connect(self.changePage)
        # self.toolLayout.addWidget(self.pageInput)

    def changePage(self, val):
        self.page = val
        if self.plotargs is not None:
            self.plotargs = (self.plotargs[0], self.page)
        self.updatePlot()

    def chooseDevs(self):
        try:
            dlg = ChooseDevsDialog(self)
            dlg.show()
            dlg.exec_()
        except Exception as e:
            self.showError(e)

    def isLogDirectory(self, directory):
        for filename in os.listdir(directory):
            if filename.endswith('.dat') or filename.endswith('.raw'):
                return True

    def openTextFile(self, filename=None):
        if filename is None:
            return        
        if 'win' in sys.platform:
            subprocess.Popen(["notepad.exe", filename])
        if 'linux' in sys.platform:
            subprocess.Popen(['gedit', filename])

    def TestImx(self):
        directory = self.selectedDirectory()
        if self.isLogDirectory(directory):
            self.RunTest("IMX", self.log.runImxPerformanceReport)
        else:
            self.runSuperNppTest(directory, "imx")

    def TestGpx(self):
        directory = self.selectedDirectory()
        if self.isLogDirectory(directory):
            self.RunTest("GPX", self.log.runGpxPerformanceReport)
        else:
            self.runSuperNppTest(directory, "gpx")

    def RunTest(self, name, reportFunc):
        if self.log is not None:
            self.setStatus("Running "+ name +" report...")
            result = reportFunc()
            self.log.openReport()
            self.updatePlot()
            self.setStatus(name +" Test: " + ("FAILED" if result else "PASSED"))

    def file_contains(self, filepath, text):
        with open(filepath, 'r') as f:
            for line in f:
                if text.lower() in line.lower():
                    return True
        return False

    def runSuperNppTest(self, directory, test):
        sys.path.insert(1, '../../../../python/src')
        params_filename = directory + '/params.yaml'
        params = {
            'name': 'test_' + test,
            'results_directory': ".",
            'directory': directory,
            'logs': ['.'],
            'blacklist_logs': [],
            'options': [],
            'run_test': [test],
            'reprocess': self.reprocess.isChecked(),
        }
        with open(params_filename, 'w') as file:
            yaml.dump(params, file, default_flow_style=False)
        from supernpp.supernpp import SuperNPP
        spp = SuperNPP(params_filename, serials=self.config['serials'])
        self.setStatus(("Running %s test..." % (test)))
        spp.run_reprocess()
        spp.run_tests()
        results_filename = spp.resultsFilename()
        self.openTextFile(results_filename)
        self.setStatus(test.upper() + " test: " + ("FAILED" if self.file_contains(results_filename, "FAILED") else "PASSED"))

    def createPlotSelection(self):
        super(logInspectorInternal, self).createPlotSelection()
        self.addButton('Devices', self.chooseDevs, layout=self.VLayoutOptions2, tooltip="Show/Hide devices")
        self.addButton('IMX Test', self.TestImx, layout=self.LayoutVTests, tooltip="Run IMX Test")
        self.addButton('GPX Test', self.TestGpx, layout=self.LayoutVTests, tooltip="Run GPX Test")

        self.reprocess = QCheckBox("Reprocess", self)
        self.reprocess.setToolTip("Reprocess data using NPP.  Requires pre-compiled NavProcess.")
        self.LayoutVTests.addWidget(self.reprocess)
        # self.reprocess.stateChanged.connect(self.changeReprocess)

    def createListGps(self):
        super(logInspectorInternal, self).createListGps()
        self.addListItem('GNSS Ephemeris', 'gnssEphemeris')
        self.addListItem('GPX Debug Float', 'gpxDebugfArray')
        self.addListItem('GPX Debug Int', 'gpxDebugiArray')

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

    configFilePath = os.path.join(os.path.expanduser("~"), "Documents", "Inertial_Sense", "config.yaml")

    main = logInspectorInternal(configFilePath)
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

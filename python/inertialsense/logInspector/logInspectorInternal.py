#!/usr/bin/env python3

try:
    from .logInspector import LogInspectorWindow
except ImportError:
    # Fallback for direct script execution
    from logInspector import LogInspectorWindow

import subprocess
import sys, os, signal, ctypes, yaml
from PyQt6 import QtCore
from PyQt6.QtWidgets import QDialog, QApplication, QPushButton, QVBoxLayout, QCheckBox, \
    QTableWidget, QTableWidgetItem, QAbstractItemView
from PyQt6.QtCore import Qt

from inertialsense.tools.data_sets import DID_DEV_INFO

# import logInspector as logInspector

DEVICE_TABLE_COL_CHECKBOX  = 0
DEVICE_TABLE_COL_HARDWARE  = 1
DEVICE_TABLE_COL_FIRMWARE  = 2
DEVICE_TABLE_COL_BUILD     = 3
DEVICE_TABLE_COL_PROTOCOL  = 4
DEVICE_TABLE_COL_COMMIT    = 5
DEVICE_TABLE_COL_BUILDDATE = 6
DEVICE_TABLE_MORE_INFO_COLS = (DEVICE_TABLE_COL_HARDWARE, DEVICE_TABLE_COL_FIRMWARE, DEVICE_TABLE_COL_BUILD,
                                DEVICE_TABLE_COL_PROTOCOL, DEVICE_TABLE_COL_COMMIT, DEVICE_TABLE_COL_BUILDDATE)

DEVICE_TABLE_AUTO_FIT_ROWS = 30

class ChooseDevsDialog(QDialog):
    def __init__(self, parent):
        super(ChooseDevsDialog, self).__init__(parent)
        self.setWindowTitle("Devices")
        self.parent = parent
        self.mainLayout = QVBoxLayout()

        self.selectAllButton = QPushButton()
        self.selectAllButton.setText("Select All")
        self.selectAllButton.clicked.connect(self.selectAll)
        self.mainLayout.addWidget(self.selectAllButton, 0, Qt.AlignmentFlag.AlignLeft)

        self.selectNoneButton = QPushButton()
        self.selectNoneButton.setText("Select None")
        self.selectNoneButton.clicked.connect(self.selectNone)
        self.mainLayout.addWidget(self.selectNoneButton, 0, Qt.AlignmentFlag.AlignLeft)

        self.table = QTableWidget()
        self.table.setColumnCount(7)
        self.table.setHorizontalHeaderLabels(['Serial#', 'Hardware', 'Firmware', 'Build', 'Protocol', 'Commit', 'Build Date'])
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.setSelectionMode(QAbstractItemView.SelectionMode.NoSelection)
        self.table.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.table.setRowCount(parent.log.numDev)

        self.checkboxes = []
        for i in range(parent.log.numDev):
            checkbox = QCheckBox()
            checkbox.setText(str(parent.log.serials[i]))
            checkbox.setChecked(i in parent.mplots[0].plotter.active_devs)
            self.checkboxes.append(checkbox)
            self.table.setCellWidget(i, DEVICE_TABLE_COL_CHECKBOX, checkbox)

            hardwareStr, firmwareStr, buildStr, protocolStr, commitStr, buildDateStr = self.deviceInfoStrings(i)
            self.table.setItem(i, DEVICE_TABLE_COL_HARDWARE, self.readOnlyItem(hardwareStr))
            self.table.setItem(i, DEVICE_TABLE_COL_FIRMWARE, self.readOnlyItem(firmwareStr))
            self.table.setItem(i, DEVICE_TABLE_COL_BUILD, self.readOnlyItem(buildStr))
            self.table.setItem(i, DEVICE_TABLE_COL_PROTOCOL, self.readOnlyItem(protocolStr))
            self.table.setItem(i, DEVICE_TABLE_COL_COMMIT, self.readOnlyItem(commitStr))
            self.table.setItem(i, DEVICE_TABLE_COL_BUILDDATE, self.readOnlyItem(buildDateStr))

        self.table.resizeColumnsToContents()
        for col in DEVICE_TABLE_MORE_INFO_COLS:
            self.table.setColumnHidden(col, True)
        self.fitTableWidth()
        self.mainLayout.addWidget(self.table)

        self.moreInfoCheckbox = QCheckBox()
        self.moreInfoCheckbox.setText("More info")
        self.moreInfoCheckbox.setChecked(False)
        self.moreInfoCheckbox.stateChanged.connect(self.toggleMoreInfo)
        self.mainLayout.addWidget(self.moreInfoCheckbox)

        self.applyButton = QPushButton()
        self.applyButton.setText("Apply")
        self.applyButton.clicked.connect(self.updatePlot)
        self.mainLayout.addWidget(self.applyButton, 0, Qt.AlignmentFlag.AlignLeft)

        self.okbutton = QPushButton()
        self.okbutton.setText("OK")
        self.okbutton.clicked.connect(self.clickedOk)
        self.mainLayout.addWidget(self.okbutton, 0, Qt.AlignmentFlag.AlignLeft)

        buttonWidth = max(b.sizeHint().width() for b in
                           (self.selectAllButton, self.selectNoneButton, self.applyButton, self.okbutton))
        for b in (self.selectAllButton, self.selectNoneButton, self.applyButton, self.okbutton):
            b.setFixedWidth(buttonWidth)

        self.setLayout(self.mainLayout)
        self.fitDialogWidth()
        self.fitDialogHeight()

    @staticmethod
    def readOnlyItem(text):
        item = QTableWidgetItem(text)
        item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsEditable)
        return item

    def deviceInfoStrings(self, devIdx):
        devInfoArray = self.parent.log.data[devIdx, DID_DEV_INFO]
        if len(devInfoArray) == 0:
            return '', '', '', '', '', ''
        devInfo = devInfoArray[0]
        hv = devInfo['hardwareVer']
        fv = devInfo['firmwareVer']
        pv = devInfo['protocolVer']
        buildNumber = int(devInfo['buildNumber'])
        hardwareStr = 'IMX-%d.%d.%d' % (hv[0], hv[1], hv[2])
        firmwareStr = '%d.%d.%d' % (fv[0], fv[1], fv[2])
        buildStr = '%05X.%d' % ((buildNumber >> 12) & 0xFFFFF, buildNumber & 0xFFF)
        protocolStr = '%d.%d.%d' % (pv[0], pv[1], pv[2])
        commitStr = '%08X' % int(devInfo['repoRevision'])
        buildDateStr = '%04d-%02d-%02d' % (2000 + int(devInfo['buildYear']), devInfo['buildMonth'], devInfo['buildDay'])
        return hardwareStr, firmwareStr, buildStr, protocolStr, commitStr, buildDateStr

    def toggleMoreInfo(self):
        showMoreInfo = self.moreInfoCheckbox.isChecked()
        for col in DEVICE_TABLE_MORE_INFO_COLS:
            self.table.setColumnHidden(col, not showMoreInfo)
        self.table.resizeColumnsToContents()
        self.fitDialogWidth()

    def fitDialogWidth(self):
        self.fitTableWidth()
        self.setFixedWidth(self.sizeHint().width())

    def fitDialogHeight(self):
        numDev = len(self.checkboxes)
        rowsToShow = min(numDev, DEVICE_TABLE_AUTO_FIT_ROWS)
        rowHeight = self.table.rowHeight(0) if numDev > 0 else self.table.verticalHeader().defaultSectionSize()
        headerHeight = self.table.horizontalHeader().sizeHint().height()
        tableFitHeight = headerHeight + rowsToShow * rowHeight + self.table.frameWidth() * 2 + 2

        # Temporarily pin the table to its up-to-N-row height so the dialog's
        # sizeHint (and adjustSize) reflect that, not the full row count.
        self.table.setFixedHeight(tableFitHeight)
        self.adjustSize()

        screen = QApplication.primaryScreen()
        if screen is not None:
            maxHeight = screen.availableGeometry().height()
            if self.height() > maxHeight:
                self.resize(self.width(), maxHeight)
            self.setMaximumHeight(maxHeight)

        fittedHeight = self.height()

        # Release the table's height so it can grow/shrink again when the
        # user manually resizes the dialog (e.g. for >30 devices).
        self.table.setMinimumHeight(0)
        self.table.setMaximumHeight(16777215)
        self.resize(self.width(), fittedHeight)

    def fitTableWidth(self):
        width = self.table.frameWidth() * 2
        if self.table.verticalHeader().isVisible():
            width += self.table.verticalHeader().width()
        for col in range(self.table.columnCount()):
            if not self.table.isColumnHidden(col):
                width += self.table.columnWidth(col)
        self.table.setFixedWidth(width + 4)

    def updatePlot(self):
        active_serials = []
        for i, checkbox in enumerate(self.checkboxes):
            if checkbox.isChecked():
                active_serials.append(self.parent.log.serials[i])
        for mplot in self.parent.mplots:
            mplot.plotter.setActiveSerials(active_serials)
        self.parent.updatePlot()

    def clickedOk(self):
        self.updatePlot()
        self.close()

    def selectAll(self):
        for checkbox in self.checkboxes:
            checkbox.setChecked(True)

    def selectNone(self):
        for checkbox in self.checkboxes:
            checkbox.setChecked(False)


class logInspectorInternal(LogInspectorWindow):
    def __init__(self, config):
        super(logInspectorInternal, self).__init__(config)
        self.page = 0
        self.exePath = __file__

    def createListSystem(self):
        super(logInspectorInternal, self).createListSystem()
        self.addListItem('General Fault Codes', 'genFaultCodes')
        self.addListItem('Port Monitor (General)', 'portMonitor')
        self.addListItem('GPX Port Monitor (behind IMX)', 'gpxPortMonitor')

    def createListIns(self):
        super(logInspectorInternal, self).createListIns()
        self.addListItem('EKF Biases', 'ekfBiases')

    def createListSensors(self):
        self.addListItem('IMUs Uncal Gyro',  'imusUncalPqr')
        self.addListItem('IMUs Uncal Accel', 'imusUncalAcc')
        self.addListItem('IMUs Raw Gyro',  'imusRawPqr')
        self.addListItem('IMUs Raw Accel', 'imusRawAcc')
        self.addListItem('IMUs Raw Gyro Combined',  'imusRawPqrCombined')
        self.addListItem('IMUs Raw Accel Combined', 'imusRawAccCombined')
        self.addListItem('IMUs Raw Gyro FFT', 'gyroFFT')
        self.addListItem('IMUs Raw Gyro PSD', 'gyroRawPSD')
        self.addListItem('IMUs Raw Accel FFT', 'accelFFT')
        self.addListItem('IMUs Raw Accel PSD', 'accelRawPSD')
        self.addListItem('IMUs Gyro',  'imusPqr')
        self.addListItem('IMUs Accel', 'imusAcc')
        super(logInspectorInternal, self).createListSensors()
        self.addListItem('Allan Dev. Gyro', 'allanDeviationPqr')
        self.addListItem('Allan Dev. Accel', 'allanDeviationAcc')
        self.addListItem('Allan Dev. Imus Gyro',  'allanDeviationImusPqr')
        self.addListItem('Allan Dev. Imus Accel', 'allanDeviationImusAcc')
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
        self.addListItem('GNSS Raw Time', 'gnssRawTime')
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
            dlg.exec()
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

    def changeShowReferenceCheckbox(self, state):
        enabled = (state == int(Qt.CheckState.Checked))
        for mplot in self.mplots:
            if mplot.plotter:
                mplot.plotter.enableReference(enabled)
        self.updatePlot()

    def changeShowUcalCheckbox(self, state):
        for mplot in self.mplots:
            if mplot.plotter:
                mplot.plotter.enableUcal(state)
                self.updatePlot()

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

        self.showReference = QCheckBox("Reference", self)
        self.showReference.setToolTip("Show reference and Sensor Valid lines on SComp v Temp plots")
        self.showReference.setChecked(False)
        self.showReference.stateChanged.connect(self.changeShowReferenceCheckbox)
        self.VLayoutOptions3.addWidget(self.showReference)

        self.showUcal = QCheckBox("UCAL", self)
        self.showUcal.setToolTip("Show DID_SENSORS_UCAL data on SComp v Temp plots")
        self.showUcal.setChecked(False)
        self.showUcal.stateChanged.connect(self.changeShowUcalCheckbox)
        self.VLayoutOptions3.addWidget(self.showUcal)

        self.reprocess = QCheckBox("Reprocess", self)
        self.reprocess.setToolTip("Reprocess data using NPP.  Requires pre-compiled NavProcess.")
        self.LayoutVTests.addWidget(self.reprocess)
        # self.reprocess.stateChanged.connect(self.changeReprocess)

    def createListGnss(self):
        super(logInspectorInternal, self).createListGnss()
        self.addListItem('GNSS Ephemeris', 'gnssEphemeris')
        self.addListItem('GPX Debug Float', 'gpxDebugfArray')
        self.addListItem('GPX Debug Int', 'gpxDebugiArray')

def kill_handler(*args):
    instance = QApplication.instance()
    instance.quit()


def main():
    if sys.version[0] != '3':
        raise Exception("You must use Python 3. The current version is " + sys.version)

    if os.name == 'nt':
        # On Windows, this is required to get the icon changed in the taskbar
        myappid = 'InertialSense.PythonTools.LogInspector.Any'
        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(myappid)

    app = QApplication(sys.argv)
    app.styleHints().setColorScheme(Qt.ColorScheme.Light)
    if os.name == 'nt':
        app.setStyle('Fusion')

    configFilePath = os.path.join(os.path.expanduser("~"), "Documents", "Inertial_Sense", "log_inspector.yaml")

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

if __name__ == '__main__':
    main()

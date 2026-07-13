from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qtagg import NavigationToolbar2QT as NavigationToolbar
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
from PyQt6.QtWidgets import QMainWindow, QApplication, QPushButton, QWidget, QTabWidget,QVBoxLayout
from PyQt6.QtGui import QIcon, QAction
from PyQt6.QtCore import pyqtSlot, QTimer
import sys, os

class plotWindow():
    # Lazy Qt instantiation: QApplication / MainWindow / per-tab widgets are
    # not constructed in __init__. They are deferred to show() so that scripts
    # which only generate and save figures (no on-screen display) never pay
    # the cost of a Qt6 + X11 client per child process. See SN-8031.
    def __init__(self, title="Plot Window", parent=None):
        self._title = title
        self._pending_plots = []   # list of (title, figure, threeD)
        # Realized Qt objects are created in show(); kept as attributes for
        # _poll_close_request() to reach them after materialization.
        self.app = None
        self.MainWindow = None
        self.tabs = None
        self.canvases = []
        self.figure_handles = []
        self.toolbar_handles = []
        self.tab_handles = []
        self.current_window = -1
        self._close_marker_path = os.environ.get('IMX_CLOSE_PLOTS_MARKER', '')
        self._close_marker_mtime = 0.0
        self._close_timer = None

    def _poll_close_request(self):
        if not self._close_marker_path:
            return
        try:
            marker_mtime = os.path.getmtime(self._close_marker_path)
        except OSError:
            return

        if marker_mtime > self._close_marker_mtime:
            self._close_marker_mtime = marker_mtime
            plt.close('all')
            if self.MainWindow is not None:
                self.MainWindow.close()
            if self.app is not None:
                self.app.quit()

    def addPlot(self, title, figure, threeD=False):
        # subplots_adjust must run eagerly so subsequent saveFig() output has
        # the right margins regardless of whether show() is ever called.
        figure.subplots_adjust(left=0.07, right=0.95, bottom=0.05, top=0.91, wspace=0.25, hspace=0.25)
        if threeD:
            figure.axes[0].mouse_init()
        self._pending_plots.append((title, figure, threeD))
        self.figure_handles.append(figure)

    def show(self):
        # Build the Qt UI now -- first time we actually need a display.
        self.app = QApplication.instance() or QApplication(sys.argv)
        self.MainWindow = QMainWindow()
        self.MainWindow.setWindowTitle(self._title)
        self.tabs = QTabWidget()
        self.MainWindow.setCentralWidget(self.tabs)
        self.MainWindow.resize(1200, 980)

        if self._close_marker_path:
            try:
                self._close_marker_mtime = os.path.getmtime(self._close_marker_path)
            except OSError:
                self._close_marker_mtime = 0.0
            self._close_timer = QTimer(self.MainWindow)
            self._close_timer.timeout.connect(self._poll_close_request)
            self._close_timer.start(250)

        for title, figure, _threeD in self._pending_plots:
            new_tab = QWidget()
            layout = QVBoxLayout()
            new_tab.setLayout(layout)
            new_canvas = FigureCanvas(figure)
            new_toolbar = NavigationToolbar(new_canvas, new_tab)
            layout.addWidget(new_canvas)
            layout.addWidget(new_toolbar)
            self.tabs.addTab(new_tab, title)
            self.toolbar_handles.append(new_toolbar)
            self.canvases.append(new_canvas)
            self.tab_handles.append(new_tab)

        self.MainWindow.show()
        return self.app.exec()

    def saveFig(self, fig, filepath, format='svg', sizeInches=[]):
        if fig == None:
            return
        allaxes = fig.get_axes()
        for ax in allaxes:
            ax.autoscale()      # Reset to default zoom

        restoreSize = fig.get_size_inches()
        if not sizeInches:
            if format == 'png':     # Increase size for saved png
                sizeInches = [16,11]
                # sizeInches = [20,14]
            else: # svg or png
                sizeInches = [11,8]
        fig.set_size_inches(sizeInches)
        directory = os.path.dirname(filepath)
        if not os.path.exists(directory):
            os.makedirs(directory)
        fig.savefig(os.path.join(filepath + '.' + format), bbox_inches='tight')
        fig.set_size_inches(restoreSize)

if __name__ == '__main__':
    import numpy as np


    pw = plotWindow()

    x = np.arange(0, 10, 0.001)

    f = plt.figure()
    ysin = np.sin(x)
    plt.plot(x, ysin, '--')
    pw.addPlot("sin", f)

    f = plt.figure()
    ycos = np.cos(x)
    plt.plot(x, ycos, '--')
    pw.addPlot("cos", f)
    pw.show()

    # sys.exit(app.exec_())

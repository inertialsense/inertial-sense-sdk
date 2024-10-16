from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
from PyQt5.QtWidgets import QMainWindow, QApplication, QPushButton, QWidget, QAction, QTabWidget,QVBoxLayout
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import pyqtSlot
import sys, os

class plotWindow():
    def __init__(self, title="Plot Window", parent=None):
        self.app = QApplication(sys.argv)
        self.MainWindow = QMainWindow()
        self.MainWindow.__init__()
        self.MainWindow.setWindowTitle(title)
        self.canvases = []
        self.figure_handles = []
        self.toolbar_handles = []
        self.tab_handles = []
        self.current_window = -1
        self.tabs = QTabWidget()
        self.MainWindow.setCentralWidget(self.tabs)
        # self.MainWindow.resize(1920, 1080)
        self.MainWindow.resize(1200, 980)
        self.MainWindow.show()

    def addPlot(self, title, figure, threeD=False):
        new_tab = QWidget()
        layout = QVBoxLayout()
        new_tab.setLayout(layout)

        figure.subplots_adjust(left=0.07, right=0.95, bottom=0.05, top=0.91, wspace=0.25, hspace=0.25)
        new_canvas = FigureCanvas(figure)

        new_toolbar = NavigationToolbar(new_canvas, new_tab)

        layout.addWidget(new_canvas)
        layout.addWidget(new_toolbar)
        self.tabs.addTab(new_tab, title)

        self.toolbar_handles.append(new_toolbar)
        self.canvases.append(new_canvas)
        self.figure_handles.append(figure)
        if threeD:
            figure.axes[0].mouse_init()
        self.tab_handles.append(new_tab)

    def show(self):        
        return self.app.exec_()

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







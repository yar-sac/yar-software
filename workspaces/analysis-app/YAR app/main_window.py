import sys

import pyqtgraph as pg
from PySide6.QtWidgets import (QComboBox, QFrame, QGridLayout, QGroupBox,
                               QHBoxLayout, QLabel, QMainWindow, QPushButton,
                               QVBoxLayout, QWidget)


class Graph(pg.PlotWidget):
    def __init__(self, parent=None, plotItem=None, title="", xaxis="", yaxis="", **kargs):
        super().__init__(parent=parent, background="w", plotItem=plotItem, **kargs)

        # standard dictionary of font for text
        styles = {"color": "k", "font-size": "12px"}

        self.setLabel("left", yaxis, **styles)
        self.setLabel("bottom", xaxis, **styles)
        self.setTitle(title, **styles)

        # colour for the axes and the axes labels
        self.getAxis('left').setPen('k')
        self.getAxis('bottom').setPen('k')
        self.getAxis('left').setTextPen('k')
        self.getAxis('bottom').setTextPen('k')

        self.showGrid(x=True, y=True)  # showing the grid
        # self.setXRange(0, 1, padding=0.01)           #setting an x and y range for the graph
        #self.setYRange(0, 5, padding=0.01)

        self.pen_1 = pg.mkPen(color="r", width=2)  # pen to plot data
        self.plot_1([0, 1], [0, 0])

    def plot_1(self, x, y):
        self.data_line_1 = self.plot(x, y, fillLevel=0, brush=(
            200, 50, 50, 50), pen=self.pen_1)  # initial plot

    def update_1(self, x, y):
        self.data_line_1.setData(x, y)  # plot after data update


# box containing data acquisition controllers like start and stop
class AcquisitionBox(QGroupBox):
    def __init__(self, parent=None):
        super().__init__("Data Acquisition", parent=parent)

        layout = QHBoxLayout()
        self.setLayout(layout)

        self.button_run = QPushButton("RUN")
        self.button_single = QPushButton("PUASE")

        layout.addWidget(self.button_run)
        layout.addWidget(self.button_single)

        # self.button_single.clicked.connect()               #function for data acq through single stepping
        # self.button_run.clicked.connect()                  #continous run function

# The dashboard for required approxiamte values as the data is updated


class Dashboard(QGroupBox):
    def __init__(self, parent=None):
        super().__init__("Main Dashboard", parent=parent)

        layout = QVBoxLayout()
        self.setLayout(layout)

        self.alt_label = QLabel(f"Height = {None} m")  # setting label titles
        self.vel_label = QLabel(f"Velocity = {None} m/s")
        self.acc_label = QLabel(f"Acceleration = {None} m/s²")
        self.temp_label = QLabel(f"Temperature = {None} °C")
        self.long_label = QLabel(f"Longitude = {None} °")
        self.lat_label = QLabel(f"Latitude = {None} °")
        if
        self.apogee = QLabel(f"Apogee = {None} m")

        layout.addWidget(self.alt_label)  # adding labels to the groupbox
        layout.addWidget(self.vel_label)
        layout.addWidget(self.acc_label)
        layout.addWidget(self.temp_label)
        layout.addWidget(self.long_label)
        layout.addWidget(self.lat_label)

# panel of all the graphs


class GraphControl(QFrame):
    def __init__(self, parent=None):
        super().__init__(parent=parent)

        self.setFrameStyle(QFrame.StyledPanel)  # styled frame panel

        self.screen1 = Graph(title="Height v/s Time",
                             yaxis="Height (m)", xaxis="Time (s)")
        self.screen2 = Graph(title="Velocity v/s Time",
                             yaxis="Velocity (m/s)", xaxis="Time (s)",)
        self.screen3 = Graph(title="Accelaration v/s Time",
                             yaxis="Acceleration (m/s²)", xaxis="Time (s)")
        self.screen4 = Graph(title="Temperature v/s Time",
                             yaxis="Temperature (°C)", xaxis="Time (s)")

        self.content_layout = QGridLayout()
        self.content_layout.addWidget(
            self.screen1, 0, 0)  # add Graph1 to top left
        self.content_layout.addWidget(
            self.screen2, 0, 1)  # add Graph2 to top right
        # add Graph3 to bottom left
        self.content_layout.addWidget(self.screen3, 1, 0)
        # add Graph4 to bottom right
        self.content_layout.addWidget(self.screen4, 1, 1)

        self.setLayout(self.content_layout)

# control panel of all the controller widgets


class ControlPanel(QFrame):
    def __init__(self, parent=None):
        super().__init__(parent=parent)
        self.setFrameStyle(QFrame.StyledPanel)
        self.acq_box = AcquisitionBox()
        self.dash_box = Dashboard()

        self.layout = QVBoxLayout()
        # adding the dashboard and acquisition box to the frame
        self.layout.addWidget(self.dash_box)
        self.layout.addStretch()  # space between 2 widgets
        self.layout.addWidget(self.acq_box)

        self.setLayout(self.layout)  # set layout to Vertical orientation

# main window for UI display in app


class MainWindow(QMainWindow):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.setupUi()

    def setupUi(self):
        self.setWindowTitle("YAR App")  # Title of app

        self.graph_control = GraphControl()
        self.control_panel = ControlPanel()

        self.content_layout = QHBoxLayout()
        self.content_layout.addWidget(
            self.control_panel)  # Adding contoller panel
        self.content_layout.addWidget(self.graph_control)  # adding graph panel

        self.setCentralWidget(QWidget())
        self.centralWidget().setLayout(self.content_layout)

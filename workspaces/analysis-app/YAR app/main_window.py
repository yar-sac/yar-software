import sys
from PySide6.QtWidgets import (
    QComboBox,
    QFrame,
    QMainWindow,
    QPushButton,
    QWidget,
    QHBoxLayout,
    QVBoxLayout,
    QGridLayout,
    QGroupBox,
    QLabel,
)
import pyqtgraph as pg

class Graph1(pg.PlotWidget):                                               
    def __init__(self, parent=None, plotItem=None, **kargs):
        super().__init__(parent=parent, background="w", plotItem=plotItem, **kargs)

        styles = {"color": "k", "font-size": "12px"}  #standard dictionary of font for text
        self.setLabel("left", "Height (m)", **styles)
        self.setLabel("bottom", "Time (s)", **styles)
        self.setTitle("Height v/s Time",**styles)
        
        self.getAxis('left').setPen('k')              #colour for the axes and the axes labels
        self.getAxis('bottom').setPen('k')
        self.getAxis('left').setTextPen('k')
        self.getAxis('bottom').setTextPen('k')       
        

        self.showGrid(x=True, y=True)                 #showing the grid
        #self.setXRange(0, 1, padding=0.01)           #setting an x and y range for the graph
        #self.setYRange(0, 5, padding=0.01)

        self.pen_1 = pg.mkPen(color="r", width=2)     #pen to plot data
        self.plot_1([0,1],[0,0])
        

    def plot_1(self, x, y):
        self.data_line_1 = self.plot(x, y, fillLevel =0, brush=(200,50,50,50), pen=self.pen_1) #initial plot

    def update_1(self, x, y):
        self.data_line_1.setData(x, y) #plot after data update

#graph 2,3,4 classes are identical to 1 but for different components
class Graph2(pg.PlotWidget):
    def __init__(self, parent=None, plotItem=None, **kargs):
        super().__init__(parent=parent, background="w", plotItem=plotItem, **kargs)

        styles = {"color": "k", "font-size": "12px"}
        self.setLabel("left", "Velocity (m/s)", **styles)
        self.setLabel("bottom", "Time (s)", **styles)
        self.setTitle("Velocity v/s Time",**styles)
        
        self.getAxis('left').setPen('k')
        self.getAxis('bottom').setPen('k')
        self.getAxis('left').setTextPen('k')
        self.getAxis('bottom').setTextPen('k')
        

        self.showGrid(x=True, y=True)
        #self.setXRange(0, 1, padding=0.02)
        #self.setYRange(0, 5, padding=0.02)
        

        self.pen_2 = pg.mkPen(color="r", width=2)
        self.plot_2([0,1],[0,0])
        

    def plot_2(self, x, y):
        self.data_line_2 = self.plot(x, y, fillLevel =0, brush=(200,50,50,50), pen=self.pen_2)

    def update_2(self, x, y):
        self.data_line_2.setData(x, y)


class Graph3(pg.PlotWidget):
    def __init__(self, parent=None, plotItem=None, **kargs):
        super().__init__(parent=parent, background="w", plotItem=plotItem, **kargs)

        styles = {"color": "k", "font-size": "12px"}
        self.setLabel("left", "Acceleration (m/s²)", **styles)
        self.setLabel("bottom", "Time (s)", **styles)
        self.setTitle("Accelaration v/s Time",**styles)
        
        self.getAxis('left').setPen('k')
        self.getAxis('bottom').setPen('k')
        self.getAxis('left').setTextPen('k')
        self.getAxis('bottom').setTextPen('k')
        

        self.showGrid(x=True, y=True)
        #self.setXRange(0, 1, padding=0.02)
        #self.setYRange(0, 5, padding=0.02)
        

        self.pen_3 = pg.mkPen(color="r", width=2)
        self.plot_3([0,1],[0,0])
        

    def plot_3(self, x, y, ch=1):
        self.data_line_3 = self.plot(x, y, fillLevel =0, brush=(200,50,50,50), pen=self.pen_3)

    def update_3(self, x, y, ch=1):
        self.data_line_3.setData(x, y)

class Graph4(pg.PlotWidget):
    def __init__(self, parent=None, plotItem=None, **kargs):
        super().__init__(parent=parent, background="w", plotItem=plotItem, **kargs)

        styles = {"color": "k", "font-size": "12px"}
        self.setLabel("left", "Temperature (°C)", **styles)
        self.setLabel("bottom", "Time (s)", **styles)
        self.setTitle("Temperature v/s Time",**styles)
        
        self.getAxis('left').setPen('k')
        self.getAxis('bottom').setPen('k')
        self.getAxis('left').setTextPen('k')
        self.getAxis('bottom').setTextPen('k')
        

        self.showGrid(x=True, y=True)
        #self.setXRange(0, 1, padding=0.02)
        #self.setYRange(0, 5, padding=0.02)
        

        self.pen_3 = pg.mkPen(color="r", width=2)
        self.plot_3([0,1],[0,0])
        

    def plot_3(self, x, y, ch=1):
        self.data_line_3 = self.plot(x, y, fillLevel =0, brush=(200,50,50,50), pen=self.pen_3)

    def update_3(self, x, y, ch=1):
        self.data_line_3.setData(x, y)

#box containing data acquisition controllers like start and stop
class AcquisitionBox(QGroupBox):
    def __init__(self, parent=None):
        super().__init__("Data Acquisition", parent=parent)

        layout = QHBoxLayout()
        self.setLayout(layout)

        self.button_run = QPushButton("RUN")
        self.button_single = QPushButton("SINGLE")

        layout.addWidget(self.button_run)
        layout.addWidget(self.button_single)

        #self.button_single.clicked.connect()               #function for data acq through single stepping
        #self.button_run.clicked.connect()                  #continous run function

#The dashboard for required approxiamte values as the data is updated
class Dashboard(QGroupBox):
    def __init__(self, parent=None):
        super().__init__("Main Dashboard", parent=parent)

        layout = QVBoxLayout()
        self.setLayout(layout)

        self.alt_label = QLabel(f"Height = {None} m")         #setting label titles
        self.vel_label = QLabel(f"Velocity = {None} m/s")
        self.acc_label = QLabel(f"Acceleration = {None} m/s²")
        self.temp_label = QLabel(f"Temperature = {None} °C")
        self.long_label = QLabel(f"Longitude = {None} °")
        self.lat_label = QLabel(f"Latitude = {None} °")

        layout.addWidget(self.alt_label)                     #adding labels to the groupbox
        layout.addWidget(self.vel_label)
        layout.addWidget(self.acc_label)
        layout.addWidget(self.temp_label)
        layout.addWidget(self.long_label)
        layout.addWidget(self.lat_label)
        
#control panel of all the graphs 
class ControlPanel1(QFrame):
    def __init__(self, parent=None):
        super().__init__(parent=parent)

        self.setFrameStyle(QFrame.StyledPanel)              #styled frame panel

        self.screen1 = Graph1()
        self.screen2 = Graph2()
        self.screen3 = Graph3()
        self.screen4 = Graph4()

        self.content_layout = QGridLayout()
        self.content_layout.addWidget(self.screen1,0,0)     #add Graph1 to top left
        self.content_layout.addWidget(self.screen2,0,1)     #add Graph2 to top right
        self.content_layout.addWidget(self.screen3,1,0)     #add Graph3 to bottom left
        self.content_layout.addWidget(self.screen4,1,1)     #add Graph4 to bottom right

        self.setLayout(self.content_layout)

#control panel of all the controller widgets
class ControlPanel2(QFrame):
    def __init__(self, parent=None):
        super().__init__(parent=parent)
        self.setFrameStyle(QFrame.StyledPanel)
        self.acq_box = AcquisitionBox()
        self.dash_box = Dashboard()

        self.layout = QVBoxLayout()
        self.layout.addWidget(self.dash_box)                 #adding the dashboard and acquisition box to the frame
        self.layout.addStretch()                             #space between 2 widgets
        self.layout.addWidget(self.acq_box)

        self.setLayout(self.layout)                          #set layout to Vertical orientation

#main window for UI display in app
class MainWindow(QMainWindow):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.setupUi()

    def setupUi(self):
        self.setWindowTitle("YAR App")                         #Title of app 

        self.control_panel_1 = ControlPanel1()                  
        self.control_panel_2 = ControlPanel2()

        self.content_layout = QHBoxLayout()
        self.content_layout.addWidget(self.control_panel_2)    #Adding contoller panel
        self.content_layout.addWidget(self.control_panel_1)    #adding graph panel

        self.setCentralWidget(QWidget())
        self.centralWidget().setLayout(self.content_layout)

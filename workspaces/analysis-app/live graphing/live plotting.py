import sys
from PySide6.QtWidgets import QWidget, QApplication
from pyqtgraph import PlotWidget
from PySide6 import QtCore
import numpy as np
import pyqtgraph as pg

class Window(PlotWidget):
    def __init__(self):
        super().__init__(background = 'w')
        
        styles = {"color": "k", "font-size": "12px"}
        self.setLabel("left", "Height (m)", **styles)
        self.setLabel("bottom", "Time (s)", **styles)
        self.setTitle("Height v/s Time",**styles)
        
        self.getAxis('left').setPen('k')
        self.getAxis('bottom').setPen('k')
        self.getAxis('left').setTextPen('k')
        self.getAxis('bottom').setTextPen('k')

        self.showGrid(x=True, y=True)
        

        self.pen = pg.mkPen(color="r", width=2)

        
        # Copy the data in the mode1 code
        self.data1 = [0,0,0,0,0]                           
        self.curve2 = self.plot(self.data1, fillLevel =0, brush=(200,50,50,50), pen=self.pen)
        self.ptr1 = 0

        # Set timer
        self.timer = pg.QtCore.QTimer()                                     # Timer signal binding update_data function
        self.timer.timeout.connect(self.update_data)
        self.timer.start(100)                                                # The timer interval is 50ms, which can be understood as refreshing data once in 50ms

    def get_data(self,i):
        try:
            with open('txt_file_experiment.txt','r') as file:
                data = file.read().split('\n')
                try:
                    x,y = data[i].split(',')
                except ValueError:
                    x = 0
                    y = 0
                return int(y)
                file.flush()
        except FileNotFoundError:
            print("no file present")
            raise SystemExit
            
    # Data shift left
    def update_data(self):
        try:
            self.data1.append(self.get_data(self.ptr1))
        except IndexError:
            self.data1.append(0)
            pass
        self.curve2.setData(self.data1)
        self.ptr1 += 1                                                      # x axis record point
        self.curve2.setPos(0,0)                                             # Reset x-related coordinate origin

        
def main():
    app = QApplication(sys.argv)
    window = Window()                                                       # Instantiate and display the window bound to the drawing control
    window.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()

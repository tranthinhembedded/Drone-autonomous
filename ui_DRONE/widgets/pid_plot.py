# widgets/pid_plot.py
from collections import deque
import pyqtgraph as pg
from utils.colors import ACCENT

class PIDPlot(pg.PlotWidget):
    def __init__(self, title):
        super().__init__()
        self.setBackground("#0D0F14")
        self.setTitle(title, color=ACCENT)
        self.showGrid(x=True,y=True,alpha=0.1)

        self.data = deque(maxlen=200)
        self.curve = self.plot(pen=pg.mkPen(ACCENT, width=2))

    def update(self, val):
        self.data.append(val)
        self.curve.setData(self.data)
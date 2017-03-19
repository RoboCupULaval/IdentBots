from threading import Thread
import matplotlib.pyplot as plt
import numpy as np

class MotorSubPlot():

    def __init__(self, ax, maxSamples):
        self.ax1 = ax
        self.ax2 = self.ax1.twinx()
        self.maxSamples = maxSamples
        self.motorsNames = []
        self.dataNames = []
        self.lines = []
        self.dataBuffers = []
        self.dataBuffersIdx = []
        self.axArray = []
        self.time = np.linspace(0, maxSamples, maxSamples)

    def addLineAx1(self, motorName, dataName, color, label):
        self.motorsNames.append(motorName)
        self.dataNames.append(dataName)
        buffer = np.zeros(self.maxSamples)
        self.dataBuffers.append(buffer)
        self.dataBuffersIdx.append(0)
        self.axArray.append(self.ax1)
        self.lines.append(self.ax1.plot(self.time, buffer, color, label=label)[0])

    def addLineAx2(self, motorName, dataName, color, label):
        self.motorsNames.append(motorName)
        self.dataNames.append(dataName)
        buffer = np.zeros(self.maxSamples)
        self.dataBuffers.append(buffer)
        self.dataBuffersIdx.append(0)
        self.axArray.append(self.ax2)
        self.lines.append(self.ax2.plot(self.time, buffer, color, label=label)[0])

    def newData(self, data):
        for i in range(len(self.lines)):
            self.dataBuffers[i][self.dataBuffersIdx[i]] = data[self.motorsNames[i]][self.dataNames[i]]
            self.lines[i].set_ydata(self.dataBuffers[i])
            self.axArray[i].relim()
            self.axArray[i].autoscale_view()
            self.dataBuffersIdx[i] += 1
            if(self.dataBuffersIdx[i] >= self.maxSamples):
                self.dataBuffersIdx[i] = 0

class PlotterThread(Thread):

    def __init__(self, plotter):
        Thread.__init__(self, name='Plotter')
        self.daemon = True
        self.plotter = plotter

    def run(self):
        while True:
            data = self.plotter.logger.getData()
            data = self.plotter.logger.getData()
            data = self.plotter.logger.getData()
            data = self.plotter.logger.getData()
            data = self.plotter.logger.getData()

            for plot in self.plotter.plots:
                plot.newData(data)

class DataPlotter():

    def __init__(self, loopType, logger):
        self.logger = logger
        plt.ion()
        self.fig, self.ax = plt.subplots(2, 2)
        self.plots = []
        self.plots.append(MotorSubPlot(self.ax[0, 0], 100)) # Motor 1...
        self.plots.append(MotorSubPlot(self.ax[0, 1], 100))
        self.plots.append(MotorSubPlot(self.ax[1, 0], 100))
        self.plots.append(MotorSubPlot(self.ax[1, 1], 100))

        if(loopType == 'open_loop'):
            self.fig.canvas.set_window_title("Open loop data")

            self.plots[0].addLineAx1('motor1', 'cmd', '-b', 'Command')
            self.plots[0].addLineAx2('motor1', 'speed', '-g', 'Speed')
            self.plots[1].addLineAx1('motor2', 'cmd', '-b', 'Command')
            self.plots[1].addLineAx2('motor2', 'speed', '-g', 'Speed')
            self.plots[2].addLineAx1('motor3', 'cmd', '-b', 'Command')
            self.plots[2].addLineAx2('motor3', 'speed', '-g', 'Speed')
            self.plots[3].addLineAx1('motor4', 'cmd', '-b', 'Command')
            self.plots[3].addLineAx2('motor4', 'speed', '-g', 'Speed')

        else:
            self.fig.canvas.set_window_title("Close loop data")

            self.plots[0].addLineAx1('motor1', 'ref', '-b', 'Reference')
            self.plots[0].addLineAx1('motor1', 'speed', '-r', 'Speed')
            self.plots[0].addLineAx1('motor1', 'error', '*r', 'Error')
            self.plots[0].addLineAx2('motor1', 'cmd', '-g', 'Command')
            self.plots[1].addLineAx1('motor2', 'ref', '-b', 'Reference')
            self.plots[1].addLineAx1('motor2', 'speed', '-r', 'Speed')
            self.plots[1].addLineAx1('motor2', 'error', '*r', 'Error')
            self.plots[1].addLineAx2('motor2', 'cmd', '-g', 'Command')
            self.plots[2].addLineAx1('motor3', 'ref', '-b', 'Reference')
            self.plots[2].addLineAx1('motor3', 'speed', '-r', 'Speed')
            self.plots[2].addLineAx1('motor3', 'error', '*r', 'Error')
            self.plots[2].addLineAx2('motor3', 'cmd', '-g', 'Command')
            self.plots[3].addLineAx1('motor4', 'ref', '-b', 'Reference')
            self.plots[3].addLineAx1('motor4', 'speed', '-r', 'Speed')
            self.plots[3].addLineAx1('motor4', 'error', '*r', 'Error')
            self.plots[3].addLineAx2('motor4', 'cmd', '-g', 'Command')

        self.thread = PlotterThread(self)
        self.thread.start()

    def update(self):
        plt.pause(0.01)
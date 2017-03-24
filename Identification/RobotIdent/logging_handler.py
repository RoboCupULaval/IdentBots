#!/usr/bin/env python
import serial
import sys
from queue import Queue
from threading import Thread, RLock, Event

class LoggingListener(Thread):

    def __init__(self, loggingHandler):
        Thread.__init__(self, name='LoggingListener')
        self.daemon = True
        self.loggingHandler = loggingHandler
        
    def run(self):
        while True:
            self.loggingHandler.parseLine()

class LoggingHandler:

    """ Class that handles logs from one robot over a serial connection """

    def __init__(self):
        self.ser = serial.Serial()
        self.serLock = RLock()
        self.connected = Event()
        self.connected.clear()
        self.listener = LoggingListener(self)
        self.listener.start()
        self.dataQueue = Queue()
        self.dataArray = []
        self.logQueue = Queue()
        self.loopType = 'open_loop'

    def connect(self, port, baudrate=115200):
        with self.serLock:
            if not self.connected.is_set():
                #self.ser = open('data-robot.csv', 'r')
                try:
                    self.ser = serial.Serial(port, baudrate)
                except serial.serialutil.SerialException:
                    print('\nSorry connection to specified port is impossible')
                    sys.exit(1)
                self.connected.set()

    def disconnect(self):
        with self.serLock:
            if self.connected.is_set():
                self.connected.clear()
                self.ser.close()

    def setLoopType(self, loopType):
        self.loopType = loopType

    def getLoopType(self):
        return self.loopType

    def getData(self):
        """ Get one parsed data line (blocking) """
        return self.dataQueue.get()

    def clearData(self):
        """Clear the logged data"""
        with self.dataQueue.mutex:
            self.dataQueue = Queue()
            self.dataArray.clear()

    def getLog(self):
        """ Get one parsed log line (blocking) """
        return self.logQueue.get()
    
    def saveDataCsv(self, fileName):
        file = open(fileName + '_' + self.loopType + '.csv', 'w')
        if(self.loopType == 'open_loop'):
            file.write("time,robotID,motor1_cmd,motor1_speed,motor2_cmd,motor2_speed,motor3_cmd,motor3_speed,motor4_cmd,motor4_speed\n")
            for line in self.dataArray:
                file.write("{},{},{},{},{},{},{},{},{},{}\n".format(line["time"], line["robotID"],
                                                    line["motor1"]["cmd"], line["motor1"]["speed"],
                                                    line["motor2"]["cmd"], line["motor2"]["speed"],
                                                    line["motor3"]["cmd"], line["motor3"]["speed"],
                                                    line["motor4"]["cmd"], line["motor4"]["speed"]))
        else:
            file.write("time,robotID,vx,vy,vt,"
                       "motor1_ref,motor1_speed,motor1_error,motor1_cmd,"
                       "motor2_ref,motor2_speed,motor2_error,motor2_cmd,"
                       "motor3_ref,motor3_speed,motor3_error,motor3_cmd,"
                       "motor4_ref,motor4_speed,motor4_error,motor4_cmd\n")
            for line in self.dataArray:
                file.write("{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n".format(line["time"], line["robotID"], line["receivedSpeed"]["vx"], line["receivedSpeed"]["vy"], line["receivedSpeed"]["vt"],
                                                     line["motor1"]["ref"], line["motor1"]["speed"], line["motor1"]["error"], line["motor1"]["cmd"],
                                                     line["motor2"]["ref"], line["motor2"]["speed"], line["motor2"]["error"], line["motor2"]["cmd"],
                                                     line["motor3"]["ref"], line["motor3"]["speed"], line["motor3"]["error"], line["motor3"]["cmd"],
                                                     line["motor4"]["ref"], line["motor4"]["speed"], line["motor4"]["error"], line["motor4"]["cmd"]))

        self.clearData()

    def isDataAvailable(self):
        return not self.dataQueue.empty()

    def isLogAvailable(self):
        return not self.logQueue.empty()

    def parseLine(self):
        self.connected.wait()
        try:
            with self.serLock:
                line = str(self.ser.readline().decode('ascii'))
            line = line.strip('\n')
            line = line.split('|')
            d = {'type': line[0], 'time': int(line[1])}
            d['robotID'] = int(line[2].strip('R'))
            d['battVoltage'] = float(line[3].strip('B'))
            if d['type'] == 'DATA':
                if(self.loopType == 'open_loop'):
                    d['loopType'] = 'open_loop'
                    d['motor1'] = {'cmd': float(line[4]), 'speed': float(line[5])}
                    d['motor2'] = {'cmd': float(line[6]), 'speed': float(line[7])}
                    d['motor3'] = {'cmd': float(line[8]), 'speed': float(line[9])}
                    d['motor4'] = {'cmd': float(line[10]), 'speed': float(line[11])}
                else:
                    d['loopType'] = 'close_loop'
                    d['receivedSpeed'] = {'vx': float(line[4]), 'vy': float(line[5]), 'vt': float(line[6])}
                    d['motor1'] = {'ref': float(line[7]), 'speed': float(line[8]), 'error': float(line[9]), 'cmd': float(line[10])}
                    d['motor2'] = {'ref': float(line[11]), 'speed': float(line[12]), 'error': float(line[13]), 'cmd': float(line[14])}
                    d['motor3'] = {'ref': float(line[15]), 'speed': float(line[16]), 'error': float(line[17]), 'cmd': float(line[18])}
                    d['motor4'] = {'ref': float(line[19]), 'speed': float(line[20]), 'error': float(line[21]), 'cmd': float(line[22])}
                self.dataQueue.put(d)
                self.dataArray.append(d)
            else:
                d['msg'] = line[4]
                self.logQueue.put(d)
        except IndexError:
            pass #print("Bad line format")
        except serial.serialutil.SerialException as err:
            print('Serial exception : ' + str(err))
            exit()
        except:
            print(line)

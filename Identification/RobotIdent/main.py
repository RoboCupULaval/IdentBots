import code
import argparse
from threading import Thread
import time

from logging_handler import LoggingHandler
from plotter import DataPlotter

log = LoggingHandler()


class ConsoleInteractThread(Thread):
    def __init__(self):
        Thread.__init__(self, name='Console interract')
        self.daemon = True

    def run(self):
        code.interact(banner='', local=globals())

class ConsoleLogThread(Thread):
    def __init__(self):
        Thread.__init__(self, name='Console log')
        self.daemon = True

    def run(self):
        while True:
            line = log.getLog()
            #print("Log from Robot #{} | {} | Batt {} V | {} : {}".format(line["robotID"], line["time"], line["battVoltage"],
             #
            pass#                                                              line["type"], line["msg"]))

if __name__ == '__main__':
    try:
        print('Robocup - Identification and PID')

        main_parser = argparse.ArgumentParser(description='Robocup utility for robot motors identification and PID tuning')
        main_parser.add_argument('log_serial_port', help='Serial port where the robot logs data')
        main_parser.add_argument('loop_type', help='open_loop or close_loop')

        consoleInteractThread = ConsoleInteractThread()
        consoleInteractThread.start()

        consoleLogThread = ConsoleLogThread()
        consoleLogThread.start()

        args = main_parser.parse_args()

        log.setLoopType(args.loop_type)
        log.connect(args.log_serial_port)

        plotter = DataPlotter(args.loop_type, log)

        while True:
            time.sleep(0.001)
            #plotter.update()
    except KeyboardInterrupt:
        log.disconnect()

log.disconnect()
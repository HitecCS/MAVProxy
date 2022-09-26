#!/usr/bin/env python
'''
nmea GPS connector
connect to a NMEA GPS on a serial port and provide this as location position
'''

import sys, os, serial, time
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib import mp_util
import multiprocessing
from time import sleep

try:
    from setting_utils import *
except ImportError as e:
    raise(e)


try:
    import pynmea2
except ImportError as e:
    raise(e)


'''
class NMEAGPSstatus():
    def __init__(self,port, position):
        self.port = port
        self.position = position

    def set_port(self, port):
        self.port = port

    def set_position(self, position):
        self.position = position


    def update_status(self):
        status = ["No status", 'warning']
        if self.port is None:
            status = ["GPS is not connected", 'warning']
        elif self.position.timestamp is None:
            status = ["No position", 'warning']
        else:
            try:
                res = str(self.position)
                status = [res, 'info']
            except Exception as e:
                status = ["No position", 'warning']
        data = {"gps_status" : status}
        set_status("gps_status", data)
'''


class NMEAGPSModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(NMEAGPSModule, self).__init__(mpstate, "NMEAGPS", "NMEA input")
        self.nmeagps_settings = mp_settings.MPSettings([
            ("port", str, "/dev/ttyAMA0"),
            ("baudrate", int, 9600)
            ])
        self.add_completion_function('(NMEAGPSSETTING)',
                                     self.nmeagps_settings.completion)
        self.add_command('nmeagps', self.cmd_nmeagps, "nmea GPS input control",
                         ["<status|connect|disconnect>", "set (NMEAGPSSETTING)"])
        self.port = None
        self.position = mp_util.mp_position()
        self.nmeagps_settings.load_json(get_modules_json_file("nmeagps.json"))
        #self.interval_time = 0

        #self.gpsstatus = NMEAGPSstatus(self.port, self.position)


    def cmd_nmeagps(self, args):
        '''nmeagps commands'''
        usage = "nmeagps <set|connect|disconnect|status>"
        if len(args) == 0:
            print(usage)
            return
        if args[0] == "set":
            self.nmeagps_settings.command(args[1:])
        elif args[0] == "connect":
            self.cmd_connect()
        elif args[0] == "disconnect":
            self.cmd_disconnect()
        elif args[0] == "status":
            self.cmd_status()
        else:
            print(usage)

    def cmd_connect(self):
        '''connect to GPS'''
        try:
            self.port = serial.Serial(self.nmeagps_settings.port, self.nmeagps_settings.baudrate)
        except Exception as ex:
            print("Failed to open %s : %s" % (self.nmeagps_settings.port, ex))

    def cmd_disconnect(self):
        '''disconnect from GPS'''
        if self.port is not None:
            self.port.close()
            self.port = None
        else:
            print("GPS not connected")

    def cmd_status(self):
        '''status'''
        if self.port is None:
            print("GPS not connected")
            return
        if self.position.timestamp is None:
            print("No position")
            return
        print(self.position)


    def idle_task(self):
        '''check for new data'''
        if self.port is None:
            try:
                self.cmd_connect()
            except Exception as ex:
                return
        line = self.port.readline()
        try:
            line = line.decode("ascii")
        except UnicodeDecodeError as e:
            return
        if not line.startswith("$"):
            return
        if line[3:6] == "GGA":
            msg = pynmea2.parse(line)
            self.position.num_sats = int(msg.num_sats)
            self.position.latitude = msg.latitude
            self.position.longitude = msg.longitude
            self.position.altitude = msg.altitude
            self.position.timestamp = time.time()
            self.mpstate.position = self.position
        if line[3:6] == "RMC":
            msg = pynmea2.parse(line)
            if msg.true_course is not None:
                self.position.ground_course = msg.true_course
            else:
                self.position.ground_course = 0.0
            self.position.ground_speed = msg.spd_over_grnd
            self.position.timestamp = time.time()
            self.mpstate.position = self.position
        #if time.time() > self.interval_time:
        #    self.gpsstatus.set_position(self.position)
        #    self.gpsstatus.set_port(self.port)
        #    self.gpsstatus.update_status()
        #    self.interval_time = time.time()+self.nmeagps_settings.status_period


def init(mpstate):
    '''initialise module'''
    return NMEAGPSModule(mpstate)

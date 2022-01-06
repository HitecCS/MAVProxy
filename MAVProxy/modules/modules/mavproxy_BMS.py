#!/usr/bin/env python3

from smbus2 import SMBus
from time import sleep
from pymavlink.dialects.v10 import ardupilotmega
import multiprocessing
from MAVProxy.modules.lib import mp_module



class BMSOutput(mp_module.MPModule):
	def __init__(self, mpstate):
		super(BMSOutput, self).__init__(mpstate, "Battery Management System Output", "Sends controller BMS data to connected IPS")
		self.bms_info_queue = multiprocessing.Queue()
		bms = BMS(self.bms_info_queue, test=True)
		bms.start()


	def idle_task(self):
		try:
			RSOC, TTE = self.bms_info_queue.get(False)
			if RSOC and TTE and RSOC != -1 and TTE != -1:
				msg = ardupilotmega.MAVLink_battery2_message(RSOC, TTE)
				self.mpstate.additional_msgs.append(msg)
		except Exception as e:
			pass


class BMS(multiprocessing.Process):
	def __init__(self, bms_info_queue, test=False, period=1, addr=0x0b, bus=1):
		multiprocessing.Process.__init__(self)
		self.addr = addr
		self.dev_bus = bus
		self.bms_info_queue = bms_info_queue
		self.period = period
		self.test = test
		self.bus = SMBus(bus)

	def read_word(self, comm):
		try:
			data = self.bus.read_word_data(self.addr, comm)
			return data
		except OSError as e:
			return -1


	def getRelativeStateOfCharge(self):
		if self.test: return 100
		comm = 0x0D
		return self.read_word(comm)


	def getRunTimeToEmpty(self):
		if self.test: return 100
		comm = 0x11
		return self.read_word(comm)


	def run(self):
		while True:
			RSOC = self.getRelativeStateOfCharge()
			TTE = self.getRunTimeToEmpty()
			self.bms_info_queue.put((RSOC, TTE))
			sleep(self.period)


def init(mpstate):
	return BMSOutput(mpstate)



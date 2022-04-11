#!/usr/bin/env python3
from smbus2 import SMBus
from time import sleep
from pymavlink.dialects.v10 import ardupilotmega
import multiprocessing
from MAVProxy.modules.lib import mp_module
import sys
import json
import os


MODEM_STATUS_FILE = "/home/pi/modem/modem_status.json"


class RadioStatusOutput(mp_module.MPModule):
	def __init__(self, mpstate):
		super(RadioStatusOutput, self).__init__(mpstate, "Radio Status Output", "Sends FC radio status messages, turn off if radio already includes them")
		self.radio_status_queue = multiprocessing.Queue()
		radio_status = RadioStatus(self.radio_status_queue)
		radio_status.start()


	def idle_task(self):
		try:
			gnd_RSSI, air_RSSI, gnd_noise, air_noise = self.radio_status_queue.get(False)
			msg = ardupilotmega.MAVLink_radio_status_message(gnd_RSSI, air_RSSI, 100, gnd_noise, air_noise, 0, 0)
			self.mpstate.additional_msgs.append(msg)
		except Exception as e:
			pass



class RadioStatus(multiprocessing.Process):

	def __init__(self, radio_status_queue, test=False, period=1):
		multiprocessing.Process.__init__(self)
		self.radio_status_queue = radio_status_queue
		self.period = period
		self.test = test

	#TODO: might want to something more sophisticated in the future
	def scale(self, rssi):
		scaled_value = 254 if rssi > -50 else 0
		return scaled_value


	def calculate_noise_value(self, snr, rssi):
		return rssi-snr;

	def get_status_values(self, data):
		gnd_rssi = int(data["gnd_RSSI"])
		air_rssi = int(data["air_RSSI"])

		gnd_snr = int(data["gnd_SNR"])
		air_snr = int(data["air_SNR"])

		gnd_noise = self.scale(self.calculate_noise_value(gnd_snr, gnd_rssi))
		air_noise = self.scale(self.calculate_noise_value(air_snr, air_rssi))
		scaled_gnd_rssi = self.scale(gnd_rssi)
		scaled_air_rssi = self.scale(air_rssi)
		return (scaled_gnd_rssi, scaled_air_rssi, gnd_noise, air_noise)




	#Radio status messages won't send until connection to both ground and airside are established
	def run(self):
		while True:
			try:
				with open(MODEM_STATUS_FILE) as f:
					data = json.load(f)
				values = self.get_status_values(data)
				self.radio_status_queue.put(values)
			except Exception as e:
				pass
			sleep(self.period)


def init(mpstate):
	#Remove modem status file from previous power up,
	if(os.path.exists(MODEM_STATUS_FILE)):
		os.remove(MODEM_STATUS_FILE)
	return RadioStatusOutput(mpstate)



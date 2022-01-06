#!/usr/bin/python3


'''
	Python driver for [GY-63 MS5611 pressure sensor]
'''

__license__ = 'GPL'
__version__ = '1.0'
__date__  = '10-April-2018'
__status__ = 'Production'

import RPi.GPIO as GPIO
import time
import numpy
from pymavlink.dialects.v10 import ardupilotmega
from MAVProxy.modules.lib import mp_module
import multiprocessing


class Baro(multiprocessing.Process):
	# MS5611 commands and addresses
	__MS5611_RESET             = 0x1E        # 

	__MS5611_CONVERT_D1_256    = 0x40        #
	__MS5611_CONVERT_D1_512    = 0x42        #
	__MS5611_CONVERT_D1_1024   = 0x44        #
	__MS5611_CONVERT_D1_2048   = 0x46        #
	__MS5611_CONVERT_D1_4096   = 0x48        #

	__MS5611_CONVERT_D2_256    = 0x50        #
	__MS5611_CONVERT_D2_512    = 0x52        #
	__MS5611_CONVERT_D2_1024   = 0x54        #
	__MS5611_CONVERT_D2_2048   = 0x56        #
	__MS5611_CONVERT_D2_4096   = 0x58        #

	__MS5611_ADC_READ          = 0x00        #

	__MS5611_C1                = 0xA2
	__MS5611_C2                = 0xA4
	__MS5611_C3                = 0xA6
	__MS5611_C4                = 0xA8
	__MS5611_C5                = 0xAA
	__MS5611_C6                = 0xAC

	def __init__(self, baro_info_queue, cs_pin=8, clock_pin=11, data_in_pin=9, data_out_pin=10, board = GPIO.BCM, period = 1):

		'''Initialize Soft (Bitbang) SPI bus
		Parameters:
		- cs_pin:    Chip Select (CS) / Slave Select (SS) pin (Any GPIO)  
		- clock_pin: Clock (SCLK / SCK) pin (Any GPIO)
		- data_in_pin:  Data input (SO / MOSI) pin (Any GPIO)
		  - data_out_pin: Data output (MISO) pin (Any GPIO)
		- board:     (optional) pin numbering method as per RPi.GPIO library (GPIO.BCM (default) | GPIO.BOARD)
		'''
		multiprocessing.Process.__init__(self)
		self.baro_info_queue = baro_info_queue
		self.cs_pin = cs_pin
		self.clock_pin = clock_pin
		self.data_in_pin = data_in_pin
		self.data_out_pin = data_out_pin
		self.board = board
		self.period = period

		self.dT = 0
		self.TEMP = 0        # INT32
		self.OFF = 0         # INT64
		self.SENS = 0        # INT64
		self.P = 0          # INT32  

		# Initialize needed GPIO
		GPIO.setmode(self.board)
		GPIO.setup(self.cs_pin, GPIO.OUT)
		GPIO.setup(self.clock_pin, GPIO.OUT)
		GPIO.setup(self.data_in_pin, GPIO.IN)
		GPIO.setup(self.data_out_pin, GPIO.OUT)

		# Pull chip select high to make chip inactive
		GPIO.output(self.cs_pin, GPIO.HIGH)

		# Reset sensor
		self._send_command(self.__MS5611_RESET)
		time.sleep(4.0)

		# Load compensation parameters
		self._read_coefficients()
		time.sleep(1.0)

		# Updating data                
		self.update() 
		

	def _spixfer(self, x):
		'''SPI send/recieve function'''
		reply = 0
		for i in range(7,-1,-1):
			reply <<= 1
			GPIO.output(self.clock_pin, GPIO.LOW)
			GPIO.output(self.data_out_pin, x & (1<<i))
			GPIO.output(self.clock_pin, GPIO.HIGH)
			if (GPIO.input(self.data_in_pin)):
				reply |= 1
		return reply

	def _read16(self, register):
		'''Reads 16-bits from specified register'''
		GPIO.output(self.cs_pin, GPIO.LOW)  
		self._spixfer(register)    # send request to read from register
		value = (self._spixfer(0) << 8) | self._spixfer(0) 
		GPIO.output(self.cs_pin, GPIO.HIGH)
		return value

	def _read24(self, register):
		'''Reads 24-bits from specified register'''
		GPIO.output(self.cs_pin, GPIO.LOW)
		self._spixfer(register)    # send request to read from register
		value = (self._spixfer(0) << 16) | (self._spixfer(0) << 8) | self._spixfer(0)  
		GPIO.output(self.cs_pin, GPIO.HIGH)
		return value

	def _send_command(self, command):
		'''Sends command via SPI'''
		GPIO.output(self.cs_pin, GPIO.LOW)
		self._spixfer(command)
		GPIO.output(self.cs_pin, GPIO.HIGH)

	def _read_coefficients(self):
		'''Reads the factory-set coefficients'''
		self.C1 = self._read16(self.__MS5611_C1)   # UINT16
		self.C2 = self._read16(self.__MS5611_C2)   # UINT16
		self.C3 = self._read16(self.__MS5611_C3)   # UINT16
		self.C4 = self._read16(self.__MS5611_C4)   # UINT16
		self.C5 = self._read16(self.__MS5611_C5)   # UINT16
		self.C6 = self._read16(self.__MS5611_C6)   # UINT16


	def _read_adc(self):
		GPIO.output(self.cs_pin, GPIO.LOW)
		dump = self._spixfer(self.__MS5611_ADC_READ)    # send request to read from register
		#time.sleep(0.1)
		byteH = self._spixfer(0)    # send request to read from register
		byteM = self._spixfer(0)    # send request to read from register
		byteL = self._spixfer(0)    # send request to read from register
		value = (byteH << 16) | (byteM << 8) | byteL 
		GPIO.output(self.cs_pin, GPIO.HIGH)
		return value  

	def _refreshPressure(self, OSR = __MS5611_CONVERT_D1_4096):
		self._send_command(OSR)

	def _refreshTemperature(self, OSR = __MS5611_CONVERT_D2_4096):
		self._send_command(OSR)

	def _readPressure(self):
		self.D1 = self._read_adc()

	def _readTemperature(self):
		self.D2 = self._read_adc()

	def check_working_coeffs(self):
		if self.C1 == 65535 and self.C2 == 65535 and self.C3 == 65535 and self.C4 == 65535:
			return False
		return True

	def update(self):
		self._refreshPressure()
		time.sleep(0.01) # Waiting for pressure data ready
		self._readPressure()
		self._refreshTemperature()
		time.sleep(0.01) # Waiting for temperature data ready
		self._readTemperature()
		self.calculatePressureAndTemperature()

	def returnPressure(self):
		return '{:.3f}'.format(self.PRES)

	def returnTemperature(self):
		return '{:.2f}'.format(self.TEMP)

	def calculatePressureAndTemperature(self):
		dT = self.D2 - self.C5 * 2**8
		self.TEMP = 2000 + dT * self.C6 / 2**23
		OFF = self.C2 * 2**16 + (self.C4 * dT) / 2**7
		SENS = self.C1 * 2**15 + (self.C3 * dT) / 2**8

		if (self.TEMP >= 2000):
			T2 = 0
			OFF2 = 0
			SENS2 = 0
		elif (self.TEMP < 2000):
			T2 = dT * dT / 2**31
			OFF2 = 5 * ((self.TEMP - 2000) ** 2) / 2
			SENS2 = OFF2 / 2
		elif (self.TEMP < -1500):
			OFF2 = OFF2 + 7 * ((self.TEMP + 1500) ** 2)
			SENS2 = SENS2 + 11 * (self.TEMP + 1500) ** 2 / 2

		self.TEMP = self.TEMP - T2
		OFF = OFF - OFF2
		SENS = SENS - SENS2

		self.PRES = (self.D1 * SENS / 2**21 - OFF) / 2**15
		#print 'PRES = ', self.PRES

		self.TEMP = self.TEMP / 100.0 # Temperature, C
		self.PRES = self.PRES / 1000.0 # Pressure, kPa


	def returnAltitude(self, seaLevel_kPa = 101.325):
		altitude = 44330 * (1.0 - numpy.power(self.PRES / seaLevel_kPa, 0.1903))
		return '{:.2f}'.format(altitude)

	def run(self):
		while True:
			ms.update()
			temp = ms.returnTemperature()
			pressure = ms.returnPressure()
			alt = ms.returnAltitude(101.7)
			self.baro_info_queue.put((temp, pressure, alt, self.check_working_coeffs()))
			time.sleep(self.period)



class BaroOutput(mp_module.MPModule):
	def __init__(self, mpstate):
		super(BaroOutput, self).__init__(mpstate, "Barometer GCS output", "Sends barometer data to connected IPS")
		self.baro_info_queue = multiprocessing.Queue()
		baro = Baro(self.baro_info_queue)
		baro.start()


	def idle_task(self):
		try:
			temp, pressure, alt, valid_coeffs = self.baro_info_queue.get(False)
			if valid_coeffs:
				msg = ardupilotmega.MAVLink_scaled_pressure_message(0, pressure, 0, temp)
				self.mpstate.additional_msgs.append(msg)
		except Exception as e:
			pass


def init(mpstate):
	return BaroOutput(mpstate)
    


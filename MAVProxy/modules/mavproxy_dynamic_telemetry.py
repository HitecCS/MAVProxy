#!/usr/bin/env python3
import multiprocessing
import time
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
import asyncio
import traceback



WHITELISTED_MSGS = {
	20: "PARAM_REQUEST_READ",
	21: "PARAM_REQUEST_LIST",
	22: "PARAM_VALUE",
	0:  "HEARTBEAT",
	66: "REQUEST_DATA_STREAM",
	2:  "SYSTEM_TIME",
	43: "MISSION_REQUEST_LIST",
	37: "MISSION_REQUEST_PARTIAL_LIST",
        40: "MISSION_REQUEST",
	37: "MISSION_ACK",
	51: "MISSION_REQUEST_INT"
}


class DynamicTelemetry(mp_module.MPModule):
	def __init__(self, mpstate, telem_port=14550, main_dev_ip="192.168.42.246", cmd_port=5060):
		super(DynamicTelemetry, self).__init__(mpstate, "dynamic_telemetry", "automatically adds all clients on network, filters messages from non-main devices")
		self.telem_port = telem_port
		self.main_dev_ip = main_dev_ip
		self.links = {}
		self.mpstate.whitelisted_msgs = WHITELISTED_MSGS
		self.add(main_dev_ip)
		self.cmd_queue = multiprocessing.Queue()
		command_proc = CommandProc(self.cmd_queue, cmd_port)
		command_proc.start()


	def idle_task(self):
		try:
			msg = self.cmd_queue.get(False)
			cmd, ip = msg.split(':')
			if(cmd == 'add'):
				self.add(ip)
			elif(cmd == 'del'):
				self.remove(ip)
		except Exception as e:
			pass

	def add(self, ip):
		if not self.linked(ip):
			print("Adding {}".format(ip))
			self.links[ip] = Link(ip, self)
			self.links[ip].add_connection()
		else:
			print("{} already in links can't add".format(ip))


	def remove(self, ip):
		if self.linked(ip):
			print("Deleting {}".format(ip))
			self.links[ip].remove_connection()
			del self.links[ip]
		else:
			print("{} not in links can't remove".format(ip))


	def linked(self, ip):
		return ip in self.links.keys()


class Link:
	def __init__(self, ip, dt):
		device = ip + ":" + str(dt.telem_port)
		self.conn = mavutil.mavlink_connection(device, input=False, source_system=dt.settings.source_system)
		self.conn.mav.srcComponent = dt.settings.source_component
		self.dt = dt
		self.ip = ip

	def remove_connection(self):
		if self.ip == self.dt.main_dev_ip:
			self.dt.mpstate.mav_outputs.remove(self.conn)
		else:
			self.dt.mpstate.mav_filtered_outputs.remove(self.conn)
		mp_util.child_fd_list_remove(self.conn.port.fileno())


	def add_connection(self):
		if self.ip == self.dt.main_dev_ip:
			self.dt.mpstate.mav_outputs.append(self.conn)
		else:
			self.dt.mpstate.mav_filtered_outputs.append(self.conn)
		mp_util.child_fd_list_add(self.conn.port.fileno())


class CommandProtocol:
	def __init__(self, cmd_queue, on_con_lost):
		self.on_con_lost = on_con_lost
		self.cmd_queue = cmd_queue

	def connection_made(self, transport):
		self.transport = transport

	def datagram_received(self, data, addr):
		message = data.decode()
		self.cmd_queue.put(message)

	def connection_lost(self, exc):
		self.on_con_lost.set_result(True)


class CommandProc(multiprocessing.Process):
	def __init__(self, cmd_queue, port=5060):
		multiprocessing.Process.__init__(self)
		self.cmd_queue = cmd_queue
		self.port = port

	def run(self):
		asyncio.run(self.get_commands())

	async def get_commands(self):
		loop = asyncio.get_running_loop()
		on_con_lost = loop.create_future()
		transport, protocol = await loop.create_datagram_endpoint(
				lambda: CommandProtocol(self.cmd_queue, on_con_lost), local_addr=('127.0.0.1', self.port))

		try:
			await on_con_lost
		finally:
			transport.close()

def init(mpstate):
	return DynamicTelemetry(mpstate)




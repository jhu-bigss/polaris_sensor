# -*- coding: utf-8 -*-
import sys
import io
import numpy
import os
import serial
import struct
import logging
import time
from absl import flags

FLAGS = flags.FLAGS

LOGGER = logging.getLogger(__name__)

class Bundle(object):
	"""
	This class bundles many fields, similar to a record or a mutable
	namedtuple.
	"""
	def __init__(self, variables):
		for var, val in variables.items():
			object.__setattr__(self, var, val)

	# Freeze fields so new ones cannot be set.
	# def __setattr__(self, key, value):
	#     if not hasattr(self, key):
	#         raise AttributeError("%r has no attribute %s" % (self, key))
	#     object.__setattr__(self, key, value)


class PolarisTracker:

	def __init__(self):

		self.port_handles = {}
		self.n_port_handles = 0
		self.state = 'ready' # can be: ready, tracking,

		self.comm = serial.Serial('/dev/ttyUSB0', baudrate=9600)

		# PHSR Reply options (source: Polaris_API_Guide page 101)
		self.PHSR_HANDLES_ALL                      = '00'
		self.PHSR_HANDLES_TO_BE_FREED              = '01'
		self.PHSR_HANDLES_OCCUPIED                 = '02'
		self.PHSR_HANDLES_OCCUPIED_AND_INITIALIZED = '03'
		self.PHSR_HANDLES_ENABLED                  = '04'
		self.initialize()

		#~ self.initialize()


	#~ def check_for_errors(self, message): # Need to work on this!!! Need some type of error reporting!
		#~ errnum = ndicapy.ndiGetError(self._device)
		#~ if errnum != ndicapy.NDI_OKAY:
			#~ ndicapy.ndiClose(self._device)
			#~ raise IOError('error when {}. the error was: {}'
						  #~ .format(message, ndicapy.ndiErrorString(errnum)))

	def initialize(self):
		self.INIT()
		self.BEEP(2)
		self.detect_tools()

	def INIT(self):
		return self.send_command_get_reply('INIT ')

	def read_data(self):
		buffer = ""
		while True:
			oneByte = self.comm.read(1)
			if oneByte == b"\r":    #method should returns bytes
				return buffer
			else:
				buffer += oneByte.decode("ascii")

	def close(self):
		self.comm.close()

	def detect_tools(self):
		"""
		Retrieves the list of all available Port Handles, with their ID
		and Status, and initialize the port_handles member variable.
		"""
		reply = self.PHSR(self.PHSR_HANDLES_ALL)
		# LOGGER.debug('reply: {} | reply[1:2]: {}'.format(reply, reply[1:2]))
		self.n_port_handles = self.hex2dec(reply[1:2])

		# LOGGER.debug('self.n_port_handles ', self.n_port_handles)

		for i in range(self.n_port_handles):
			s = 3 + 5*(i - 1)
			id = reply[s:s+1]
			status = reply[s+2:s+4]
			self.port_handles[id] = status

	def get_frame(self):
		"""Gets a frame of tracking data from the NDI device.

		:return:

			port_numbers : list of port handles, one per tool

			time_stamps : list of timestamps (cpu clock), one per tool

			frame_numbers : list of framenumbers (tracker clock) one per tool

			tracking : list of 4x4 tracking matrices, rotation and position,
			or if use_quaternions is true, a list of tracking quaternions,
			column 0-2 is x,y,z column 3-6 is the rotation as a quaternion.

			tracking_quality : list the tracking quality, one per tool.

		Note: The time stamp is based on the host computer clock. Read the
		following extract from NDI's API Guide for advice on what to use:
		"Use the frame number, and not the host computer clock, to identify when
		data was collected. The frame number is incremented by 1 at a constant
		rate of 60 Hz. Associating a time from the host computer clock to
		replies from the system assumes that the duration of time between raw
		data collection and when the reply is received by the host computer is
		constant. This is not necessarily the case."
		"""

		self.BX('0001')

		(start_sequence, reply_length, header_crc) = struct.unpack('HHH', self.comm.read(6))
		(number_of_handles, handle_n, handle_status) = struct.unpack('BBB', self.comm.read(3))

		reply = {
			"start_sequence": start_sequence,
			"reply_length": reply_length,
			"header_crc": header_crc,
			"number_of_handles": number_of_handles,
			"handle_n": handle_n,
			"handle_status": handle_status
		}

		body = self.comm.read(reply_length-1)

		if handle_status == 1:
			(q0, qx, qy, qz, tx, ty, tz, error, port_status, frame_number, system_status, end_crc) = struct.unpack('ffffffffIIHH', body)

			reply.update({
				"q0": q0,
				"qx": qx,
				"qy": qy,
				"qz": qz,
				"tx": tx,
				"ty": ty,
				"tz": tz,
				"error": error,
				"port_status": port_status,
				"frame_number": frame_number,
				"system_status": system_status,
				"end_crc": end_crc
			})

			return Bundle(reply)

		elif handle_status == 2:
			LOGGER.debug("TOOL MISSING")
		elif handle_status == 4:
			LOGGER.debug("TOOL DISABLED")
		else:
			LOGGER.debug("ERROR")

	def get_position(self, option):
		"""Gets a frame of tracking data from the NDI device.

		:return:

			port_numbers : list of port handles, one per tool

			time_stamps : list of timestamps (cpu clock), one per tool

			frame_numbers : list of framenumbers (tracker clock) one per tool

			tracking : list of 4x4 tracking matrices, rotation and position,
			or if use_quaternions is true, a list of tracking quaternions,
			column 0-2 is x,y,z column 3-6 is the rotation as a quaternion.

			tracking_quality : list the tracking quality, one per tool.

		Note: The time stamp is based on the host computer clock. Read the
		following extract from NDI's API Guide for advice on what to use:
		"Use the frame number, and not the host computer clock, to identify when
		data was collected. The frame number is incremented by 1 at a constant
		rate of 60 Hz. Associating a time from the host computer clock to
		replies from the system assumes that the duration of time between raw
		data collection and when the reply is received by the host computer is
		constant. This is not necessarily the case."
		"""

		if option == "tools":
			self.BX("0001")
		elif option == "markers":
			self.BX("0003")
		else:
			self.stop_tracking()
			self.close()
			sys.exit("Error! Option not found.")

		(start_sequence, reply_length, header_crc) = struct.unpack('HHH', self.comm.read(6))
		(number_of_handles, handle_n, handle_status) = struct.unpack('BBB', self.comm.read(3))

		# if FLAGS.verbose:
		# 	LOGGER.debug('start_sequence: {}, reply_length: {}, header_crc: {}'.format(
		# 					start_sequence, reply_length, header_crc ))
		# 	LOGGER.debug('number_of_handles: {}, handle_n: {}, handle_status: {}\n\n'.format(
		# 			number_of_handles, handle_n, handle_status))
		# 	LOGGER.debug()

		reply = {
			"start_sequence": start_sequence,
			"reply_length": reply_length,
			"header_crc": header_crc,
			"number_of_handles": number_of_handles,
			"handle_n": handle_n,
			"handle_status": handle_status
		}

		body = self.comm.read(reply_length-1)

		if handle_status == 1:

			if option == "tools":
				# if FLAGS.verbose:
				#     LOGGER.debug(reply_length,len(body))
				(q0, qx, qy, qz, tx, ty, tz, error, port_status, frame_number, system_status, end_crc) = struct.unpack('ffffffffIIHH', body)

				reply.update({
					"q0": q0,
					"qx": qx,
					"qy": qy,
					"qz": qz,
					"tx": tx,
					"ty": ty,
					"tz": tz,
					"error": error,
					"port_status": port_status,
					"frame_number": frame_number,
					"system_status": system_status,
					"end_crc": end_crc
				})
			elif option == "markers":
				# if FLAGS.verbose:
				#     LOGGER.debug(reply_length,len(body))
				(marker_num, marker_out_vol)  = struct.unpack('bbffffffffffffHH', body)

				# if FLAGS.verbose:
				#     LOGGER.debug('marker num: ', marker_num)
			return Bundle(reply)

		elif handle_status == 2:
			LOGGER.debug("TOOL MISSING")
		elif handle_status == 4:
			LOGGER.debug("TOOL DIABLED")
		else:
			LOGGER.debug("ERROR")

	def quaternions(self, bundle):
		'return a bare bones quaternion message'
		q = dict(q0=bundle['q0'], qx=bundle['qx'], qy=bundle['qy'],
				 qz=bundle['qz'], tx=bundle['tx'], ty=bundle['ty'],
				 tz=bundle['tz'], frame_id=bundle['frame_number'])

		return Bundle(q)

	def get_markers3D(self):

		#self.TSTOP()
		#self.send_command('DSTART')

		self.send_command('3D 015')
		LOGGER.debug(self.comm.read(12))

	def get_markers(self):
		"""Gets a 3D position of markers on tool from the NDI device.

		:return:


		"""

		#~ self.comm.reset_input_buffer()
		#~ self.comm.reset_output_buffer()

		self.BX('0008')

		(start_sequence, reply_length, header_crc) = struct.unpack('HHH', self.comm.read(6))
		(number_of_handles, handle_n, handle_status, number_of_markers) = struct.unpack('BBBB', self.comm.read(4))

		reply = {
			"start_sequence": start_sequence,
			"reply_length": reply_length,
			"header_crc": header_crc,
			"number_of_handles": number_of_handles,
			"handle_n": handle_n,
			"handle_status": handle_status,
			"number_of_markers" : number_of_markers
		}

		body = self.comm.read(reply_length-2)

		if handle_status == 1:
			return reply
			(q0, qx, qy, qz, tx, ty, tz, error, port_status, frame_number, system_status, end_crc) = struct.unpack('ffffffffIIHH', body)

			reply.update({
				"q0": q0,
				"qx": qx,
				"qy": qy,
				"qz": qz,
				"tx": tx,
				"ty": ty,
				"tz": tz,
				"error": error,
				"port_status": port_status,
				"frame_number": frame_number,
				"system_status": system_status,
				"end_crc": end_crc
			})

			return reply

		elif handle_status == 2:
			LOGGER.debug("TOOL MISSING")
		elif handle_status == 4:
			LOGGER.debug("TOOL DISABLED")
		else:
			LOGGER.debug("ERROR")

	def hex2dec(self, s):
		"""return the integer value of a hexadecimal string s"""
		return int(s, 16)

	def start_tracking(self):
		"""
		Tells the NDI devices to start tracking.
		:raises Exception: ValueError
		"""
		if self.state != 'ready':
			raise ValueError("""Called start tracking before device ready,
			try calling connect first""")
		self.PINIT('01')
		self.PENA('01','D')
		self.PHSR(self.PHSR_HANDLES_ALL)
		self.TSTART('80')
		#~ self.check_for_errors('starting tracking.')
		self.state = 'tracking'

	def stop_tracking(self):
		"""
		Tells the NDI devices to stop tracking.
		:raises Exception: ValueError
		"""
		self.TSTOP()
		self.check_for_errors('stopping tracking.')
		self.state = 'ready'

	def send_command(self, command):
		command = command + '\r'
		# command = command + '\0'
		# command = '{}'.format(command).encode('utf-8', 'strict')
		command = '{}'.format(command).encode()
		return self.comm.write(command)

	def send_command_get_reply(self, command):
		self.send_command(command)
		return self.read_data()

	def add_wireless_tool(self, file_name):

		argv = FLAGS(sys.argv)
		reply = self.PHSR(self.PHSR_HANDLES_ALL)
		self.n_port_handles = self.hex2dec(reply[1:2])

		hardware_device = '********'
		system_type = '*'
		tool_type = '1'
		port_number = '**'

		reply = self.PHRQ(hardware_device, system_type, tool_type, port_number)

		handle = reply[0:2]

		file_size = os.path.getsize(file_name)
		tool_data = numpy.fromfile(file_name, dtype=numpy.uint8)
		if file_size % 64:
			tool_data = numpy.pad(tool_data, (0, 64 - file_size % 64), 'constant')
			file_size = file_size + (64 - file_size % 64)

		for i in range(0, len(tool_data), 64):
			start = "{:04x}".format(i)
			tool = "".join("{:02x}".format(x) for x in tool_data[i:i + 64].tolist())
			if FLAGS.verbose:
				LOGGER.debug('{}, {}, {}'.format(handle, start, tool))
			self.PVWR(handle, start, tool)

   #
   # NDI API commands
   #

	def APIREV(self):
		return self.send_command_get_reply('APIREV ')

	def BEEP(self, n_beep):
		return self.send_command_get_reply('BEEP {}'.format(n_beep))

	def BX(self, reply_option):
		return self.send_command('BX {}'.format(reply_option))

	def COMM(self, baud_rate, data_bits, parity, stop_bits, hardware_handshaking):
		return self.send_command_get_reply('COMM {}{}{}{}{}'.format(baud_rate, data_bits, parity, stop_bits, hardware_handshaking))

	def ECHO(self, message):
		return self.send_command_get_reply('ECHO {}'.format(message))

	def PENA(self, port_handle, tool_tracking_priority):
		return self.send_command_get_reply('PENA {}{}'.format(port_handle, tool_tracking_priority))

	def PHRQ(self, hardware_device, system_type, tool_type, port_number):
		"""
		Reply: 01D4D5
		Detected a passive tool.
		"""
		return self.send_command_get_reply('PHRQ {}{}{}{}**'.format(hardware_device, system_type, tool_type, port_number))

	def PHSR(self, reply_option):
		"""
		Reply: 001414
		In this case, there are no occupied port handles.

		Reply:
		0101031F1AF
		In this case, there is one occupied port handle, which is initialized and enabled.
		"""
		return self.send_command_get_reply('PHSR {}'.format(reply_option))

	def PINIT(self, port_handle):
		return self.send_command_get_reply('PINIT {}'.format(port_handle))

	def PVWR(self, port_handle, start_address, tool_definition_data):
		return self.send_command_get_reply('PVWR {}{}{}'.format(port_handle, start_address, tool_definition_data))

	def TSTART(self, reply_option):
		return self.send_command_get_reply('TSTART {}'.format(reply_option))

	def TSTOP(self):
		return self.send_command_get_reply('TSTOP ')

	def VER(self):
		return self.send_command_get_reply('VER 0')

"""
Class for connecting to vicon and the APM
This is currently the main file used
"""
from pymavlink import mavutil
import sys, math, time
import socket, struct, threading
import logging

###################################################################################
## A simple module for retrieving data from a Vicon motion capture system
###################################################################################
## Copyright (c) 2014, Cameron Finucane <cpf37@cornell.edu>
## All rights reserved.
##
## Redistribution and use in source and binary forms, with or without modification,
## are permitted provided that the following conditions are met:
##
## 1. Redistributions of source code must retain the above copyright notice, this
## list of conditions and the following disclaimer.
##
## 2. Redistributions in binary form must reproduce the above copyright notice,
## this list of conditions and the following disclaimer in the documentation and/or
## other materials provided with the distribution.
##
## 3. Neither the name of the copyright holder nor the names of its contributors
## may be used to endorse or promote products derived from this software without
## specific prior written permission.
##
## THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
## ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
## WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
## DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
## ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
## (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
## LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
## ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
## (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
## SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
###################################################################################

class ViconStreamer:
    # based on example from http://docs.python.org/howto/sockets.html

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._streamNames = None
        self._desiredStreams = []
        self._streaming = False
        self._verbose = False
        self.data = None

    def connect(self, host, port):
        print ">> Connecting..."
        self.sock.connect((host, port))
        print ">> Requesting stream info..."
        # Mysteriously, the ordering of the following two bytes is the opposite
        # of what the Vicon documentation claims it is
        self._viconSend([1,0])
        print ">> Receiving stream info..."
        self._streamNames = self._viconReceive()

    def _send(self, msg):
        totalsent = 0
        while totalsent < len(msg):
            sent = self.sock.send(msg[totalsent:])
            if sent == 0:
                raise RuntimeError("socket connection broken")
            totalsent = totalsent + sent

    def close(self):
        self.stopStreams()

        print ">> Disconnecting..."
        self.sock.close()

    def getData(self):
        # returns None if no data is available yet

        if self.data is None:
            return None

        return [self.data[i] for i in self._desiredStreams]

    def printStreamInfo(self):
        if self._streamNames is None:
            raise RuntimeError("stream info is not available because you are not connected")
        else:
            print "Available streams:"
            print "   " + "\n   ".join(["(%d) %s" % (i,n) for i,n in enumerate(self._streamNames)])

    def selectStreams(self, names):
        # For each name passed in, finds and subscribes to all streams whose name contains
        # the input name.
        # Returns the full stream names.

        if self._streamNames is None:
            raise RuntimeError("cannot set streams because you are not connected")

        matchingStreamNames = []
        self._desiredStreams = []
        for m in names:
            found = False
            for i,n in enumerate(self._streamNames):
                if m in n:
                    matchingStreamNames.append(n)
                    self._desiredStreams.append(i)
                    found = True
            if not found:
                raise RuntimeError("could not find stream matching name '%s'" % m)

        print ">> Subscribed to streams: " + ", ".join(matchingStreamNames)
        return matchingStreamNames

    def startStreams(self, verbose=False):
        if self._desiredStreams == []:
            raise RuntimeError("cannot start streaming because no streams are selected")

        print ">> Starting streams..."
        self._viconSend([3,0])

        self._verbose = verbose
        self._streaming = True
        self.listenThread = threading.Thread(target = self._processStream)
        self.listenThread.daemon = True
        self.listenThread.start()

    def stopStreams(self):
        if not self._streaming:
            return

        print ">> Stopping streams..."
        self._streaming = False
        self.listenThread.join()

        self._viconSend([4,0])

    def _viconSend(self, data):
        msg = struct.pack('<' + str(len(data)) + 'L', *data)
        self._send(msg)

    def _viconReceive(self):
        # get header
        msg = self._receive(2*4)
        header = struct.unpack("<2L", msg)

        if header[0] not in [1,2]:
            # packet we are not set up to process
            return header

        msg = self._receive(1*4)
        length = struct.unpack("<1L", msg)

        if header[0] == 1:
            # info packet
            strs = []
            for i in xrange(length[0]):
                msg = self._receive(1*4)
                strlen = struct.unpack("<1L", msg)
                msg = self._receive(strlen[0])
                strs.append(msg)
            return strs
        elif header[0] == 2:
            # data packet
            msg = self._receive(length[0]*8)
            body = struct.unpack("<" + str(length[0]) + "d", msg)
            return body

    def _receive(self, msglen):
        msg = ''
        while len(msg) < msglen:
            chunk = self.sock.recv(msglen-len(msg))
            if chunk == '':
                raise RuntimeError("socket connection broken")
            msg = msg + chunk
        return msg

    def _processStream(self):
        while self._streaming:
            self.data = self._viconReceive()
            if self._verbose:
                print "  ".join([self._streamNames[i] for i in self._desiredStreams])
                for i in self._desiredStreams:
                    print self.data[i], "  ",
                print

class Vidro:

	def __init__(self, sitl, vicon_num):
		self.sitl = sitl
		if self.sitl == True:
			self.baud = 115200
			self.device = "127.0.0.1:14551"
			
			self.base_rc_roll = 1535
			self.base_rc_pitch = 1535
			self.base_rc_throttle = 1370
			self.base_rc_yaw = 1470
		else:
			#for GCS with wireless radio
			#self.baud = 57600 
			#self.device = '/dev/ttyUSB0'
			
			#for raspberry pi
			self.baud = 115200 
			self.device = '/dev/ttyACM0'
			
			self.base_rc_roll = 1519
			self.base_rc_pitch = 1519
			self.base_rc_throttle = 1516
			self.base_rc_yaw = 1520

		#Home x,y,x position
		self.home_x = 0
		self.home_y = 0
		self.home_z = 0

		#Home lat,lon, and alt for sitl
		self.home_lat = 0
		self.home_lon = 0
		self.home_alt = 0

		#Last updated lat,lon, and alt for sitl
		self.current_lat = None
		self.current_lon = None
		self.current_alt = None

		self.ground_alt = 0

		#Last updated position
		self.current_x = None
		self.current_y = None
		self.current_z = None

		#Last updated rc channel's values'
		self.current_rc_channels = [None] * 6

		#Last updated rc overrides
		self.current_rc_overrides = [0] * 6

		#Last updated attitude
		self.current_pitch = None
		self.current_yaw = None
		self.current_roll = None

		#Fence for safety (Not implemented yet)
		self.fence_x_min = None
		self.fence_x_max = None
		self.fence_y_min = None
		self.fence_y_max = None
		self.fence_z_min = None
		self.fence_z_max = None

		self.clock = time.time()

		#Flag for vicon error. May not be needed
		self.vicon_error = False

		#The number of vicon objects that are being streamed. Can currently handle only two
		self.num_vicon_objs = vicon_num
		
		self.vicon_time = 0

		#Start of a log
		logging.basicConfig(filename='vidro.log', level=logging.DEBUG)

	def connect_mavlink(self):
		"""
		Initialize connection to pixhawk and make sure to get first heartbeat message
		"""
		#Initialize connection
		self.master = mavutil.mavlink_connection(self.device, self.baud)
		print "Attempting to get HEARTBEAT message from APM..."

		#Request heartbeat from APM
		msg = self.master.recv_match(type='HEARTBEAT', blocking=True)
		print("Heartbeat from APM (system %u component %u)" % (self.master.target_system, self.master.target_system))

		#The max rate (the second to last argument in the line below) is 25 Hz. You must change the firmware to get a fast rate than that.
		#It may be possible to get up to 500 Hz??
		#This may be useful later down the road to decrease latency
		#It also may be helpful to only stream needed data instead of all data
		if self.sitl == True:
			#setup data stream
			self.master.mav.request_data_stream_send(self.master.target_system, self.master.target_component, 0, 25, 1) 
			
			#Get intial values from the APM
			print "Getting inital values from APM..."
			while (self.current_rc_channels[0] == None) or (self.current_alt == None) or (self.current_yaw == None):
				self.update_mavlink()
			print("Got RC channels, global position, and attitude")
			
			#Set constants for SITl
			self.ground_alt = self.current_alt
			self.current_alt = 0
			print("Successfully set ground altitude")
			self.home_lat = self.current_lat
			self.home_lon = self.current_lon
			print("Successfully set home latitude and longitude")
		else:
			#setup data stream
			self.master.mav.request_data_stream_send(self.master.target_system, self.master.target_component, 0, 1, 0) #All
			self.master.mav.request_data_stream_send(self.master.target_system, self.master.target_component, 3, 25, 1) #RC channels
			self.master.mav.request_data_stream_send(self.master.target_system, self.master.target_component, 6, 25, 1) #Position
			
			#Get intial values from the APM
			print "Getting inital values from APM..."
			while (self.current_rc_channels[0] == None):
				self.update_mavlink()
			print("Got RC channels")

	def update_mavlink(self):
		"""
		Function for getting the general mavlink message and changing class variables based on that message.
		It looks for 5 different message types:
		-'BAD_DATA'
		-'RC_CHANNELS_RAW'
		-'GLOBAL_POSITION_INT' (for SITL only)
		-'ATTITUDE'
		-'HEARTBEAT'
		This is non-blocking so does not gaurantee to chnage any values.
		"""
		self.msg = self.master.recv_match(blocking=False)

		if self.msg:
			#print self.msg.get_type()

			if self.msg.get_type() == "RC_CHANNELS_RAW":
				try:
					self.current_rc_channels[0] = self.msg.chan1_raw
					self.current_rc_channels[1] = self.msg.chan2_raw
					self.current_rc_channels[2] = self.msg.chan3_raw
					self.current_rc_channels[3] = self.msg.chan4_raw
					self.current_rc_channels[4] = self.msg.chan5_raw
					self.current_rc_channels[5] = self.msg.chan6_raw
				except:
					pass
				
				if self.vicon_time >= self.get_vicon()[0]:
					logging.error('Vicon system values are remainng the same. Stop the system and restart the vicon values')
					self.set_rc_throttle(self.base_rc_throttle)
					self.set_rc_roll(self.base_rc_roll)
					self.set_rc_pitch(self.base_rc_pitch)
					self.set_rc_yaw(self.base_rc_yaw)
					self.vicon_error = True
				self.vicon_time = self.get_vicon()[0]

				self.send_rc_overrides()

			if self.sitl == True:
				if self.msg.get_type() == "ATTITUDE":
					self.current_yaw = self.msg.yaw*180/math.pi

				if self.msg.get_type() == "GLOBAL_POSITION_INT":
					self.current_lat = self.msg.lat * 1.0e-7
					self.current_lon = self.msg.lon * 1.0e-7
					self.current_alt = self.msg.alt-self.ground_alt

	def connect_vicon(self):
		"""
		Connects to vicon. This is needed to scream vicon data.
		"""
		self.s = ViconStreamer()
		self.s.connect("Vicon", 800)
		self.streams = self.s.selectStreams(["Time", "t-", "a-"])
		self.s.startStreams(verbose=False)
		print "checking values..."
		while self.s.getData() == None:
			pass
			
		print "Got inital vicon position"
		self.home_x = self.get_position()[0]
		self.home_y = self.get_position()[1]
		self.home_z = self.get_position()[2]
		
		timer = time.time()
		self.vicon_time = self.get_vicon()[0]
		time.sleep(1)
		while self.vicon_time >= self.get_vicon()[0]:
			if (time.time() - timer) > 10:
				print "unable to connect to the vicon system. Needs to be reset"
				logging.error('Unable to connect to the vicon system. Needs to be reset')
				return False
		print "Vicon Connected..."
		return True
		"""
		if len(self.s.getData()) < 51:
			self.num_vicon_objs = 1
		elif len(self.s.getData()) > 50:
			self.num_vicon_objs = 2
		else:
			logging.error('Number of Vicon objects was not set. This means that length of s.getData() was not correct')
		"""

	def disconnect_vicon(self):
		"""
		Properly closes vicon connection. Call this when finished using vicon.
		"""
		self.s.close()

	def connect(self):
		"""
		Connects to mavlink and vicon
		"""
		flight_ready = True
		if self.sitl == False:
			flight_ready = self.connect_vicon()
		self.connect_mavlink()
		return flight_ready

	def close(self):
		"""
		Call at the end of all programs.
		"""
		if self.sitl == False:
			self.disconnect_vicon()

	def get_vicon(self):
		"""
		Gets vicon data in the folling format:

		if num_vicon_objs == 1:
			vicon_data()[0] = time
			vicon_data()[1] = x
			vicon_data()[2] = y
			vicon_data()[3] = z
			vicon_data()[4] = x rotation
			vicon_data()[5] = y rotation
			vicon_data()[6] = z rotation

		if num_vicon_objs == 2:
			vicon_data()[0] = time
			vicon_data()[1] = x_1
			vicon_data()[2] = y_1
			vicon_data()[3] = z_1
			vicon_data()[4] = x_2
			vicon_data()[5] = y_2
			vicon_data()[6] = z_2
			vicon_data()[7] = x_rotation_1
			vicon_data()[8] = y_rotation_1
			vicon_data()[9] = z_rotation_1 (yaw)
			vicon_data()[10] = x_rotation_2
			vicon_data()[11] = y_rotation_2
			vicon_data()[12] = z_rotation_2
		"""
		return self.s.getData()

	def set_vicon_home(self):
		"""
		Set the home coordinate for the vicon data.
		"""
		self.home_x = self.get_vicon()[1]
		self.home_y = self.get_vicon()[2]
		self.home_z = self.get_vicon()[3]

	def set_fence(min_x, max_x, min_y, max_y, min_z, max_z):
		"""
		Set the fence for the quadcopter to stay within. This currently just sets global fence variables.
		"""
		self.fence_x_min = mix_x
		self.fence_x_max = max_x
		self.fence_y_min = min_y
		self.fence_y_max = max_y
		self.fence_z_min = min_z
		self.fence_z_max = max_z

	def rc_filter(self, rc_value, rc_min, rc_max):
		"""
		Filter for the RC values to filter out RC values out of RC range. Returns filtered RC value
		"""
		if rc_value > rc_max:
			rc_value = rc_max
		if rc_value < rc_min:
			rc_value = rc_min
		return rc_value

	def send_rc_overrides(self):
		self.master.mav.rc_channels_override_send(self.master.target_system, self.master.target_component, self.current_rc_overrides[0], self.current_rc_overrides[1], self.current_rc_overrides[2], self.current_rc_overrides[3], self.current_rc_overrides[4], self.current_rc_overrides[5], 0, 0)
		#self.master.mav.file.fd.flush()

	## Set RC Channels ##
	def set_rc_roll(self, rc_value):
		rc_value = self.rc_filter(rc_value,1519-270,1519+270)
		self.current_rc_overrides[0] = rc_value

	def set_rc_pitch(self, rc_value):
		rc_value = self.rc_filter(rc_value, 1519-270,1519+270)
		self.current_rc_overrides[1] = rc_value


	def set_rc_throttle(self, rc_value):
		rc_value = self.rc_filter(rc_value, 1110, 1741)
		self.current_rc_overrides[2] =  rc_value


	def set_rc_yaw(self, rc_value):
		rc_value = self.rc_filter(rc_value, 1277, 1931)
		self.current_rc_overrides[3] = rc_value



	## Reset RC Channels ##
	def rc_roll_reset(self):
		self.current_rc_overrides[0] = 0


	def rc_pitch_reset(self):
		self.current_rc_overrides[1] = 0


	def rc_throttle_reset(self):
		self.current_rc_overrides[2] = 0


	def rc_yaw_reset(self):
		self.current_rc_overrides[3] = 0


	def rc_channel_five_reset(self):
		self.current_rc_overrides[4] = 0


	def rc_channel_six_reset(self):
		self.current_rc_overrides[5] = 0

	def rc_all_reset(self):
		self.rc_roll_reset()
		self.rc_pitch_reset()
		self.rc_throttle_reset()
		self.rc_yaw_reset()

	def rc_check_dup(self, channel, value):
		"""
		Check for duplicate RC value. This is used to check to see if the RC value is already set to the value being passed in.
		"""
		if self.v.channel_readback[channel] == value:
			return True
		return False

	def get_alt(self):
		"""
		Returns the altitude in mm in SITL
		"""
		return self.current_alt

	def get_lat(self):
		"""
		Returns the latitude of the copter in SITL
		"""
		return self.current_lat

	def get_lon(self):
		"""
		Returns the longitude of the copter in SITL
		"""
		return self.current_lon

	def get_roll(self):
		"""
		Returns the roll in radians
		"""
		return self.current_roll

	def get_yaw_radians(self):
		"""
		Returns the current yaw in radians from -pi to pi
		Works for both SITL and Vicon
		For SITL it returns that yaw givn by the copter and for the Vicon system it returns the yaw given by the Vicon
		"""
		yaw = None

		if self.sitl == True:
			yaw = self.current_yaw

		else:
			try:
				#Depending on different number of objects yaw located in different place in the vicon data stream
				if self.num_vicon_objs == 1:
					yaw = self.get_vicon()[6]*(1.0)
				if self.num_vicon_objs == 2:
					yaw = self.get_vicon()[9]*(1.0)
				self.vicon_error = False
			except:
				logging.error('Unable to get the yaw(radians) from vicon')
				yaw = None
				self.vicon_error = True
		return yaw

	def get_yaw_degrees(self):
		"""
		Returns the current yaw in degrees from 0 to 360
		Works for SITL and Vicon
		For SITL it returns that yaw givn by the copter and for the Vicon system it returns the yaw given by the Vicon
		"""
		try:
			if self.num_vicon_objs == 1:
				yaw = math.degrees((self.get_vicon()[6]*(1.0)) % ((2*math.pi)*(1.0)))*-1
			if self.num_vicon_objs == 2:
				yaw = math.degrees((self.get_vicon()[9]*(1.0)) % ((2*math.pi)*(1.0)))*-1
			self.vicon_error = False
			if yaw < 0.0:
				yaw += 360

		except:
			logging.error('Unable to get the yaw(radians) from vicon')
			yaw = None
			self.vicon_error = True

		return yaw

	def get_pitch(self):
		"""
		Returns the pitch of the copter in radians
		"""
		return self.current_pitch

	def get_position(self):
		"""
		Will return position in millimeters. (X,Y,Z)
		Use this for Vicon and SITL in the loop.
		"""
		position=[None]*3

		if self.sitl == True:
			position[0] = self.calc_sitl_distance_x()
			position[1] = self.calc_sitl_distance_y()
			position[2] = self.get_alt()

			#Assign distance with appropriate sign
			if self.get_lat() < self.home_lat:
				position[0] *= -1
			if self.get_lon() < self.home_lon:
				position[1] *= -1

		else:
			try:
				position[0] = self.get_vicon()[1]
				position[1] = self.get_vicon()[2]
				position[2] = self.get_vicon()[3]
				self.vicon_error = False
			except:
				logging.error('Unable to get position data from vicon')
				position = None
				self.vicon_error = True
		return position

	def get_distance_xy(self):
		"""
		Returns the distance traveled from home in millimeters
		"""
		if self.sitl == True:
			distance = self.calc_sitl_distance(self.home_lat, self.home_lon, selt.get_lat(), self.get_lon())
		else:
			distance = math.sqrt(self.get_position()[0]*self.getposition()[0] + self.get_position()[1]*self.get_position()[1])

		return distance

	def set_sitl_home(self):
		"""
		Sets home for SITL
		"""
		self.home_lat = self.get_lat()
		self.home_lon = self.get_lon()
		self.home_alt = self.get_alt()

	def set_home(self):
		"""
		Sets the home for he quadcopter.
		Use this for Vicon and SITL
		"""
		if self.sitl == True:
			self.set_sitl_home()
		else:
			self.set_vicon_home()

	def calc_sitl_distance(self, lat1, lon1, lat2, lon2):
		"""
		Calculates the distance from one point of lat-lon to another point of lat-lon.
		Uses the 'haversine' formula to calculte the distance between lat-lon points
		Returns distance in mm
		"""
		radius = 6371

		lat1_rad = math.radians(lat1)
		lat2_rad = math.radians(lat2)

		delta_lat_rad = math.radians(lat2-lat1)
		delta_lon_rad = math.radians(lon2-lon1)

		a = ( math.sin(delta_lat_rad/2) * math.sin(delta_lat_rad/2) ) + ( math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lon_rad/2) * math.sin(delta_lon_rad/2) )

		c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

		#Returns a distance in meters
		return radius * c * 1000 * 1000

	def calc_sitl_distance_x(self):
		"""
		Calculate x distance in SITL (lat)
		X axis is north/south (change in lat)
		"""
		return self.calc_sitl_distance(self.home_lat, self.home_lon, self.home_lat, self.get_lon())

	def calc_sitl_distance_y(self):
		"""
		Calculate y distance in SITL (lon)
		Y axis is east/west (change in lon)
		"""
		return self.calc_sitl_distance(self.home_lat, self.home_lon, self.get_lat(), self.home_lon)


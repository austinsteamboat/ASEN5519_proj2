#!/usr/bin/env python2

import serial
import pymavlink.dialects.v10.common as mavlink
import sys, math, time
import socket, struct, threading

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

f = serial.Serial('/dev/ttyUSB0', 57600, timeout=0, dsrdtr=False, rtscts=False, xonxoff=False)

mav = mavlink.MAVLink(f)

# Flag system and component as unknown
target_system = -1
target_component = -1

"""
s = ViconStreamer()
s.connect("Vicon", 800)
s.streams = s.selectStreams(["Time", "t-", "a-"])
s.startStreams(verbose=False)
print "Vicon Connected..."
while s.getData() == None:
	pass
print "Got inital vicon position"

home_z = s.getData()[3]
"""
goal_z = 1000
rc_base = 1500
gain = .05

try:
    while target_system == -1:
        try:
            a = f.read(1)
            msg = mav.parse_char(str(a))
            if msg is not None:
                if msg.get_type() == "HEARTBEAT":
                    print "HEARTBEAT: %g %g" % (msg.get_srcSystem(), msg.get_srcComponent())
                    if ( target_system == -1 ) :
                        target_system = msg.get_srcSystem()
                        target_component = msg.get_srcComponent()
                        mav.request_data_stream_send(target_system, target_component, mavlink.MAV_DATA_STREAM_ALL, 1, 0) # Turn off all telemetry first
                        mav.request_data_stream_send(target_system, target_component, mavlink.MAV_DATA_STREAM_RC_CHANNELS, 20, 1) # Turn on R/C servo reads at this rate
               # print "Got Message type %s!" % msg.get_type()
        except mavlink.MAVError as err:
            #None
            print "Oops, MAVLink Error: %s!" % err.message
	
    while 1 :
        try:
            a = f.read(1)
            msg = mav.parse_char(str(a))
            if msg is not None:
                if msg.get_type() == "RC_CHANNELS_RAW":
                    print "SRV: %g %g %g %g %g %g" % (msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw, msg.chan5_raw, msg.chan6_raw)
                  #  print s.getData()[3] - home_z
                    if msg.chan6_raw < 1600:
                   #     error = goal_z - (s.getData()[3] - home_z)
                   #     throttle_override = (gain * error) + rc_base
                  #      print throttle_override
                        print "hello"
                        mav.rc_channels_override_send(target_system, target_component, 1000, 1000, 1000, 1000, 0, 0, 0, 0)
                    else:
                        mav.rc_channels_override_send(target_system, target_component, 0, 0, 0, 0, 0, 0, 0, 0)
                #print "Got Message type %s!" % msg.get_type()
        except mavlink.MAVError as err:
            #None
            print "Oops, MAVLink Error: %s!" % err.message
except KeyboardInterrupt:
    f.close()

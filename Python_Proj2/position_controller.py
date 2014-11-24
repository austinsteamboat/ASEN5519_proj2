"""
Class for handling the PID controller
"""
from vidro import Vidro, ViconStreamer
import sys, math, time
import socket, struct, threading
import matplotlib.pyplot as plot
import os
import logging

class PositionController:
    def __init__(self,vidro):

        #vidro object
        self.vidro = vidro

        #clock
        self.timer = time.time()

        #Previous errors for calculating I and D
        self.previous_time_alt = (time.time()-self.timer)*10
        self.I_error_alt = 0
        self.D_error_alt = 0
        self.previous_error_alt = None
        self.error_alt = 0

        self.previous_time_yaw = (time.time()-self.timer)*10
        self.I_error_yaw = 0
        self.D_error_yaw = 0
        self.error_yaw = 0
        self.previous_error_yaw = None

        self.previous_time_xy = (time.time()-self.timer)*10
        self.previous_error_pitch = None
        self.previous_error_roll = None
        self.D_error_roll = 0
        self.D_error_pitch = 0
        self.I_error_roll = 0
        self.I_error_pitch = 0
        self.error_pitch = 0
        self.error_roll = 0
        self.error_x = 0
        self.error_y = 0


        #Gains for PID controller
        self.alt_K_P = .005
        self.alt_K_I = .00003
        self.alt_K_D = 0

        self.yaw_K_P = 10
        self.yaw_K_I = 0
        self.yaw_K_D = 0

        self.roll_K_P = .01
        self.roll_K_I = .0006
        self.roll_K_D = .05

        self.pitch_K_P = .01
        self.pitch_K_I = .0006
        self.pitch_K_D = .05


        #Initalization of log
        logging.basicConfig(filename='controller.log', level=logging.DEBUG)


        #Base RC values and path for gains file
        if self.vidro.sitl == True:
            self.gains_file_path = os.path.join(os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__))), 'gains/gains_sitl.txt')
        else:
            self.gains_file_path = os.path.join(os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__))), 'gains/gains.txt')

    def update_gains(self):
        """
        Update gains from the gains file
        """
        #Get fill lines with the lines from the gain file
        gainfile = open(self.gains_file_path, "r")
        lines = gainfile.readlines()
        gainfile.close()

        #Set the gain files from the values from the file
        self.alt_K_P = float(lines[1])
        self.alt_K_I = float(lines[3])
        self.alt_K_D = float(lines[5])
        self.yaw_K_P = float(lines[7])
        self.yaw_K_I = float(lines[9])
        self.yaw_K_D = float(lines[11])
        self.roll_K_P = float(lines[13])
        self.roll_K_I = float(lines[15])
        self.roll_K_D = float(lines[17])
        self.pitch_K_P = float(lines[19])
        self.pitch_K_I = float(lines[21])
        self.pitch_K_D = float(lines[23])

    def rc_alt(self, target_alt):
        """
        Will send copter based off of throttle to 'target_alt'
        """

        #Calculate delta t and set previous time ot current time
        current_time = ((time.time()-self.timer)*10)
        delta_t = current_time - self.previous_time_alt
        self.previous_time_alt = current_time

        #Get error
        try:
            self.error_alt = target_alt - self.vidro.get_position()[2]
        except:
            logging.error('Unable to get the error for alt. This means vicon data is likely `None`')
            self.vidro.set_rc_throttle(self.vidro.base_rc_throttle)
            self.D_error_alt = 0
            self.P_error_alt = 0
            self.I_error_alt = 0
            return

        #Get error I
        if abs(self.error_alt) < 1000:
            self.I_error_alt = self.I_error_alt + self.error_alt*delta_t
        else:
            pass

        #Get error D
        if self.previous_error_alt == None:
            self.previous_error_alt = self.error_alt
        #Check to make sure error and previous error are not the same so D is not 0
        if self.error_alt != self.previous_error_alt:
            self.D_error_alt = (self.error_alt-self.previous_error_alt)/delta_t
            self.previous_error_alt = self.error_alt

        #filter for D values
        if self.D_error_alt != self.filter_value(5000,-5000,self.D_error_alt):
            self.vidro.set_rc_throttle(self.vidro.base_rc_throttle)
            self.P_error_alt = 0
            self.I_error_alt = 0
            return

        #Send RC value
        self.vidro.set_rc_throttle(round(self.vidro.base_rc_throttle + self.error_alt*self.alt_K_P + self.I_error_alt*self.alt_K_I + self.D_error_alt*self.alt_K_D))

        return self.error_alt

    def rc_yaw(self, target_heading):
        """
        Sends quad to given yaw
        Imput is in radians from -pi to pi
        """

        #Get rid of bad inputs
        if target_heading > math.pi or target_heading < math.pi*-1:
            return 0

        #Calculate delta t and set previous time ot current time
        current_time = ((time.time()-self.timer)*10)
        delta_t = current_time - self.previous_time_yaw
        self.previous_time_yaw = current_time

        #Get error
        try:
            yaw = self.vidro.get_yaw_radians() - 0
        except:
            logging.error('Unable to get the error for yaw. This means vicon data is likely `None`')
            self.vidro.set_rc_yaw(self.vidro.base_rc_yaw)
            self.D_error_yaw = 0
            self.P_error_yaw = 0
            self.I_error_yaw = 0
            return

        self.error_yaw = target_heading - yaw

        #Find the smallest error which makes shortest path to error
        if abs(target_heading - (yaw+2*math.pi)) < abs(self.error_yaw):
            self.error_yaw = target_heading - (yaw+2*math.pi)
        if abs(target_heading - (yaw-2*math.pi)) < abs(self.error_yaw):
            self.error_yaw = target_heading - (yaw-2*math.pi)
        self.error_yaw = self.error_yaw * -1

        #Get error I
        self.I_error_yaw = self.I_error_yaw + self.error_yaw*delta_t

        #Get error D
        if self.previous_error_yaw == None:
            self.previous_error_yaw = self.error_yaw

        if self.previous_error_yaw != self.error_yaw:
            self.D_error_yaw = (self.error_yaw-self.previous_error_yaw)/delta_t
            self.previous_error_yaw = self.error_yaw

        #filter for D values
        if self.D_error_yaw != self.filter_value(5000,-5000,self.D_error_yaw):
            self.vidro.set_rc_yaw(self.vidro.base_rc_yaw)
            self.P_error_yaw = 0
            self.I_error_yaw = 0
            return

        #Send RC value
        self.vidro.set_rc_yaw(self.vidro.base_rc_yaw + self.error_yaw*self.yaw_K_P + self.I_error_yaw*self.yaw_K_I + self.D_error_yaw*self.yaw_K_D)

        return self.error_yaw


    def rc_xy(self, target_x, target_y):
        """
        Sends quad copter to given x-y point
        """

        #Get current heading for shifting axis
        if self.vidro.sitl == True:
            heading = self.vidro.get_yaw_degrees()
        else:
            try:
                heading = self.vidro.get_yaw_degrees() - 0.0
            except:
                logging.error('Unable to get the error for yaw in rc_xy. This means vicon data is likely `None`')
                self.vidro.set_rc_pitch(self.vidro.base_rc_pitch)
                self.vidro.set_rc_roll(self.vidro.base_rc_roll)
                self.P_error_roll = 0
                self.I_error_roll = 0
                self.D_error_roll = 0
                self.P_error_pitch = 0
                self.I_error_pitch = 0
                self.D_error_pitch = 0
                return

        #Calculate current position and error
        try:
            self.x_current = self.vidro.get_position()[0]
            self.y_current = self.vidro.get_position()[1]
            self.error_x = target_x - self.x_current * 1.0
            self.error_y = target_y - self.y_current * 1.0
        except:
            logging.error('Unable to get either roll or pitch data from vicon. This means vicon data is likely `None`')
            self.vidro.set_rc_pitch(self.vidro.base_rc_pitch)
            self.vidro.set_rc_roll(self.vidro.base_rc_roll)
            self.P_error_roll = 0
            self.I_error_roll = 0
            self.D_error_roll = 0
            self.P_error_pitch = 0
            self.I_error_pitch = 0
            self.D_error_pitch = 0
            return

        #Make error not zero so you don't get a divid by zero error (Need to test to see if needed)
        if self.error_x == 0:
            self.error_x += .000000000001

        if self.error_y == 0:
            self.error_y += .000000000001

        #Total error from current point to target point
        total_error = math.sqrt(self.error_x*self.error_x+self.error_y*self.error_y)

        #Angle on x-y(lat/lon) axis to point
        waypoint_angle = math.degrees(math.atan2(self.error_y,self.error_x))

        #Calculate the offset of the vehicle from the x-y (lat-lon) axis
        vehicle_angle = 90 - (waypoint_angle + heading)

        #Calculate the error for the roll and pitch
        self.error_roll = total_error * math.sin(math.radians(vehicle_angle))
        self.error_pitch = total_error * math.cos(math.radians(vehicle_angle))*-1

        #Calculate delta-t for integration
        current_time = ((time.time()-self.timer)*10)
        delta_t = current_time - self.previous_time_xy
        self.previous_time_xy = current_time

        #Calculate the I error for roll and pitch
        if abs(self.error_roll) < 300:
            self.I_error_roll = self.I_error_roll + self.error_roll*delta_t
        else:
            self.I_eror_roll = 0

        if abs(self.error_pitch) < 300:
            self.I_error_pitch = self.I_error_pitch + self.error_pitch*delta_t
        else:
            self.I_error_pitch = 0

        #Calculate the D error for roll and pitch
        if self.previous_error_roll == None:
            self.previous_error_roll = self.error_roll
        if self.previous_error_pitch == None:
            self.previous_error_pitch = self.error_pitch
        #Check to make sure that the errors are different or else D will be 0
        if self.previous_error_roll != self.error_roll:
            self.D_error_roll = (self.error_roll-self.previous_error_roll)/delta_t
            self.previous_error_roll = self.error_roll
        if self.previous_error_pitch != self.error_pitch:
            self.D_error_pitch = (self.error_pitch-self.previous_error_pitch)/delta_t
            self.previous_error_pitch = self.error_pitch

        #filter for D values
        if self.D_error_roll != self.filter_value(7000,-7000,self.D_error_roll) or self.D_error_pitch != self.filter_value(7000,-7000,self.D_error_pitch):
            self.vidro.set_rc_pitch(self.vidro.base_rc_pitch)
            self.vidro.set_rc_roll(self.vidro.base_rc_roll)
            self.P_error_roll = 0
            self.I_error_roll = 0
            self.P_error_pitch = 0
            self.I_error_pitch = 0
            return

        #Send RC values
        self.vidro.set_rc_pitch( self.vidro.base_rc_pitch + (self.error_pitch*self.pitch_K_P) + (self.I_error_pitch*self.pitch_K_I) + (self.D_error_pitch*self.pitch_K_D) )
        self.vidro.set_rc_roll(  self.vidro.base_rc_roll + (self.error_roll*self.roll_K_P) + (self.I_error_roll*self.roll_K_I) + (self.D_error_roll*self.roll_K_D) )

        return self.error_pitch, self.error_roll

    def filter_value(self, high, low, value):
        if high < low:
            return None
        if value > high:
            value = high
        if value < low:
            value = low
        return value

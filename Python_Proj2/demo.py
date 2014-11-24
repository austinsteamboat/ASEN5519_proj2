from vidro import Vidro, ViconStreamer
from position_controller import PositionController
import sys, math, time
import socket, struct, threading
import curses
import matplotlib.pyplot as plot
import logging

#Plot arrays to start previous data for plotting
plot_error_yaw=[]
plot_error_yaw_I=[]
plot_time_yaw=[]

plot_error_throttle=[]
plot_error_throttle_I=[]
plot_time_throttle=[]

plot_error_pitch=[]
plot_error_pitch_I=[]
plot_error_pitch_D = []
plot_time_pitch=[]
plot_rc_pitch=[]

plot_error_roll=[]
plot_error_roll_I=[]
plot_error_roll_D = []
plot_time_roll=[]
plot_rc_roll=[]

plot_x_current=[]
plot_y_current=[]
plot_x_goal=[]
plot_y_goal=[]

target_x = 0
target_y = 0
target_z = 0
target_x_previous = 0
target_y_previous = 0
target_z_previous = 0

def curses_print(string, line, col):
	"""
	Function to do a simple curses print.
	"""

	#Check for bad inputs
	if col > 1 or col < 0:
		return

	if line > 22 or line < 0:
		return

	#Print to screen using curses
	if col == 0:
		screen.addstr(line, 0, string)
	if col == 1:
		screen.addstr(line, 40, string)

	screen.refresh()

#Filter for fencing values given by wand
def filter_value(low,high,value):
	if value < low:
		value = low
	if value > high:
		value = high
	return value

#Initialization of log
logging.basicConfig(filename='demo.log', level=logging.DEBUG)

#Creation of vidro and controller objects
vidro = Vidro(False, 2)
flight_ready = vidro.connect()
controller = PositionController(vidro)

#Setup the screen for curses
screen = curses.initscr()
screen.clear()
screen.refresh()

#switch for knowing if first time out of control loop
switch = False

#needed for live plotting
plot.ion()

#Setup of timer
timer = time.time()

while (vidro.current_rc_channels[4] > 1600) and flight_ready == True:

	#Reset of errors after each time control loop finishes
	controller.I_error_alt = 0
	controller.I_error_pitch = 0
	controller.I_error_roll = 0
	controller.I_error_yaw = 0
	controller.previous_time_alt = (time.time()-controller.timer)*10
	controller.previous_time_yaw = (time.time()-controller.timer)*10
	controller.previous_time_xy = (time.time()-controller.timer)*10
	vidro.previous_error_alt = 0
	vidro.previous_error_yaw = 0
	vidro.previous_error_roll = 0
	vidro.previous_error_pitch = 0

	#Update of gains before going into control loop
	if vidro.current_rc_channels[5] > 1600:
		controller.update_gains()

	#control loop
	while vidro.current_rc_channels[5] > 1600:

		#Get the position of the wand
		try:
			target_x = vidro.get_vicon()[4]
			target_y = vidro.get_vicon()[5]
			target_z = vidro.get_vicon()[6]
		except:
			logging.error('Unable to get position data from the vicon for wand')
			pass

		#filter position of wand between values
		target_z = filter_value(1000,5000,target_z)
		target_x = filter_value(-2000,2000,target_x)
		target_y = filter_value(-2000,2000,target_y)

		#Send control values
		try:
			controller.rc_alt(target_z)
			controller.rc_yaw(0)
			controller.rc_xy(target_x,target_y)
		except:
			logging.error('Something went wrong in the control system. Setting to hover')
			vidro.set_rc_throttle(vidro.base_rc_throttle)
			vidro.set_rc_roll(vidro.base_rc_roll)
			vidro.set_rc_pitch(vidro.base_rc_pitch)
			vidro.set_rc_yaw(vidro.base_rc_yaw)
			curses_print("ERROR",4,0)

		#Printing to screen.
		if round((round(time.time(),3) % .05),2) == 0:

			screen.clear()
			screen.refresh()
			
			try:
				curses_print("Position: X: " + str(vidro.get_position()[0]) + " Y: " + str(vidro.get_position()[1]) + " Z: " + str(vidro.get_position()[2]),0,0)
				curses_print("Wand:     X: " + str(vidro.get_vicon()[4]) + " Y: " + str(vidro.get_vicon()[5]) + " Z: " + str(vidro.get_vicon()[6]),1,0)
				curses_print("Target:     X: " + str(target_x) + " Y: " + str(target_y) + " Z: " + str(target_z),2,0)
				curses_print("Vicon Error: " + str(vidro.vicon_error),3,0)

				#Print alt data
				curses_print("Throttle RC Override: " + str(vidro.current_rc_overrides[2]), 5, 1)
				curses_print("Throttle RC Level: " + str(vidro.current_rc_channels[2]), 6, 1)
				curses_print("Error: " + str(controller.error_alt), 7, 1)
				curses_print("Altitude:" + str(vidro.get_position()[2]), 8, 1)
				curses_print("T: "+ str(int(vidro.base_rc_throttle+controller.error_alt*controller.alt_K_P+controller.I_error_alt*controller.alt_K_I)) + " = "+ str(vidro.base_rc_throttle) + " + " + str(controller.error_alt*controller.alt_K_P) + " + " + str(controller.I_error_alt*controller.alt_K_I) + " + " + str(controller.D_error_alt*controller.alt_K_D), 19, 0)

				#Print yaw data
				curses_print("Yaw RC Level: " + str(vidro.current_rc_channels[3]), 5, 0)
				curses_print("Error: " + str(controller.error_yaw), 6, 0)
				curses_print("raw vicon : " + str(vidro.get_vicon()[9]), 7, 0)
				curses_print("Heading Radians: " + str(vidro.get_yaw_radians()), 8, 0)
				curses_print("Heading Degrees: " + str(vidro.get_yaw_degrees()), 9, 0)
				curses_print("Y: "+ str(int(vidro.base_rc_yaw+controller.error_yaw*controller.yaw_K_P+controller.I_error_yaw*controller.yaw_K_I)) + " = "+ str(vidro.base_rc_yaw) + " + " + str(controller.error_yaw*controller.yaw_K_P) + " + " + str(controller.I_error_yaw*controller.yaw_K_I) + " + " + str(controller.D_error_yaw*controller.yaw_K_D), 20, 0)

				#Print pitch and roll
				curses_print("Pitch RC Level: " + str(vidro.current_rc_channels[1]), 11, 0)
				curses_print("Roll RC Level: " + str(vidro.current_rc_channels[0]), 11, 1)
				curses_print("Pitch: " + str(vidro.get_pitch()), 12, 0)
				curses_print("Roll: " + str(vidro.get_roll()), 12, 1)
				curses_print("X Error: " + str(round(controller.error_x)), 15, 0)
				curses_print("Y Error: " + str(round(controller.error_y)), 15, 1)
				curses_print("Roll Error: " + str(round(controller.error_roll)), 13, 1)
				curses_print("Pitch Error: " + str(round(controller.error_pitch)), 13, 0)
				curses_print("P: " +  str(int(vidro.base_rc_pitch+controller.error_pitch*controller.pitch_K_P+controller.I_error_pitch*controller.pitch_K_I+controller.D_error_pitch*controller.pitch_K_D)) + " = " + str(vidro.base_rc_pitch) + " + " + str(controller.error_pitch*controller.pitch_K_P) + " + " + str(controller.I_error_pitch*controller.pitch_K_I) + " + " + str(controller.D_error_pitch*controller.pitch_K_D), 21, 0)
				curses_print("R: " +  str(int(vidro.base_rc_roll+controller.error_roll*controller.roll_K_P+controller.I_error_roll*controller.roll_K_I+controller.D_error_roll*controller.roll_K_D)) + " = " + str(vidro.base_rc_roll) + " + " + str(controller.error_roll*controller.roll_K_P) + " + " + str(controller.I_error_roll*controller.roll_K_I) + " + " + str(controller.D_error_roll*controller.roll_K_D), 22, 0)
				
			except:
				logging.error('Unable to print values')
		
		try:
			#Add values to arrays for plotting
			plot_error_yaw.append(controller.error_yaw)
			plot_error_yaw_I.append(controller.I_error_yaw)
			plot_time_yaw.append(controller.previous_time_yaw)

			plot_error_throttle.append(controller.error_alt)
			plot_error_throttle_I.append(controller.I_error_alt)
			plot_time_throttle.append(controller.previous_time_alt)

			plot_error_pitch.append(controller.error_pitch)
			plot_error_pitch_I.append(controller.I_error_pitch)
			plot_time_pitch.append(controller.previous_time_xy)
			plot_error_pitch_D.append(controller.D_error_pitch)
			plot_rc_pitch.append(vidro.current_rc_channels[1])

			plot_error_roll.append(controller.error_roll)
			plot_error_roll_I.append(controller.I_error_roll)
			plot_time_roll.append(controller.previous_time_xy)
			plot_rc_roll.append(vidro.current_rc_channels[0])
			plot_error_roll_D.append(controller.D_error_roll)

			plot_x_current.append(vidro.get_position()[0])
			plot_y_current.append(vidro.get_position()[1])

			plot_x_goal.append(target_x)
			plot_y_goal.append(target_y)
		except:
			logging.error('Unable to add values to plot lists')

		#reset switch
		switch = True

		#update mavlink
		vidro.update_mavlink()

	#reset of rc channels to go back to transmitter control
	vidro.rc_all_reset()
	vidro.update_mavlink()

	#Erase Plots
	if switch == True:
		"""
		plot.figure(1).clf()
		plot.figure(1)
		plot.xlabel("Time(sec)")
		plot.ylabel("Error(rads)")
		plot.title("Yaw")
		plot.plot(plot_time_yaw,plot_error_yaw)
		"""
		"""
		plot.figure(2).clf()
		plot.figure(2)
		plot.xlabel("Time(sec)")
		plot.ylabel("Error(mm)")
		plot.title("Throttle")
		plot.plot(plot_time_throttle,plot_error_throttle)
		"""
		"""
		plot.figure(3).clf()
		plot.figure(3)
		plot.xlabel("Time(sec)")
		plot.ylabel("Error(mm) | RC Value")
		plot.title("Pitch Error PD | RC Value")
		plot.plot(plot_time_pitch,plot_error_pitch)
		#plot.plot(plot_time_pitch,plot_rc_pitch)
		#plot.plot(plot_time_pitch,plot_error_pitch_D)

		plot.figure(4).clf()
		plot.figure(4)
		plot.xlabel("Time(sec)")
		plot.ylabel("Error(mm) | RC Value")
		plot.title("Roll Error PD | RC Value")
		plot.plot(plot_time_roll,plot_error_roll)
		#plot.plot(plot_time_roll,plot_rc_roll)
		#plot.plot(plot_time_roll,plot_error_roll_D)
		"""
		"""
		plot.figure(5).clf()
		plot.figure(5)
		plot.xlabel("Time(sec)")
		plot.ylabel("Error(rads)")
		plot.title("Yaw")
		plot.plot(plot_time_yaw,plot_error_yaw)
		plot.plot(plot_time_yaw,plot_error_yaw_I)
		"""
		"""
		plot.figure(6).clf()
		plot.figure(6)
		plot.xlabel("Time(sec)")
		plot.ylabel("Error(mm)")
		plot.title("Throttle")
		plot.plot(plot_time_throttle,plot_error_throttle)
		plot.plot(plot_time_throttle,plot_error_throttle_I)
		"""
		"""
		plot.figure(7).clf()
		plot.figure(7)
		plot.xlabel("Time(sec)")
		plot.ylabel("Error(mm)")
		plot.title("Pitch Error PI")
		plot.plot(plot_time_pitch,plot_error_pitch)
		plot.plot(plot_time_pitch,plot_error_pitch_I)
		"""
		"""
		plot.figure(8).clf()
		plot.figure(8)
		plot.xlabel("Time(sec)")
		plot.ylabel("Error(mm)")
		plot.title("Roll Error PI")
		plot.plot(plot_time_roll,plot_error_roll)
		plot.plot(plot_time_roll,plot_error_roll_I)
		"""
		"""	
		plot.figure(9).clf()
		plot.figure(9)
		plot.xlabel("x Location(mm)")
		plot.ylabel("y Location(mm)")
		plot.title("Location")
		plot.plot(plot_x_current, plot_y_current)
		plot.plot(plot_x_goal,plot_y_goal)
		"""
		#plot.draw()
		#plot.pause(.0001)
		time.sleep(.02)


		plot_error_yaw[:]=[]
		plot_error_yaw_I[:]=[]
		plot_time_yaw[:]=[]

		plot_error_throttle[:]=[]
		plot_error_throttle_I[:]=[]
		plot_time_throttle[:]=[]

		plot_error_pitch[:]=[]
		plot_error_pitch_I[:]=[]
		plot_error_pitch_D[:]= []
		plot_time_pitch[:]=[]
		plot_rc_pitch[:]=[]

		plot_error_roll[:]=[]
		plot_error_roll_I[:]=[]
		plot_error_roll_D[:] = []
		plot_time_roll[:]=[]
		plot_rc_roll[:]=[]

		plot_x_current[:]=[]
		plot_y_current[:]=[]
		
		plot_x_goal[:]= []
		plot_y_goal[:]= []

		switch = False

	screen.clear()
	screen.refresh()
	curses_print("Under transmitter controll", 0, 0)

	curses_print(str(vidro.current_rc_channels[0]),2,0)
	curses_print(str(vidro.current_rc_channels[1]),3,0)
	curses_print(str(vidro.current_rc_channels[2]),4,0)
	curses_print(str(vidro.current_rc_channels[3]),5,0)
	curses_print(str(vidro.current_rc_channels[4]),6,0)
	curses_print(str(vidro.current_rc_channels[5]),7,0)

vidro.close()
curses.endwin()

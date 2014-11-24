from vidro_class import Vidro, ViconStreamer
from position_controller import PositionController
import sys, math, time
import socket, struct, threading
import curses
import matplotlib.pyplot as plot

#Plot arrays to start previous data fro plotting
plot_error_yaw=[]
plot_error_yaw_I=[]
plot_error_yaw_D=[]
plot_time_yaw=[]

plot_error_throttle=[]
plot_error_throttle_I=[]
plot_error_throttle_D=[]
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

vidro = Vidro(True)
vidro.connect()
controller = PositionController(vidro)

screen = curses.initscr()
screen.clear()
screen.refresh()

while vidro.current_rc_channels[4] > 1600:

	controller.update_gains()

	try:
		controller.rc_alt(500)
		#controller.rc_yaw(0)
		#controller.rc_xy(500,500)
		curses_print("No errors",2,0)
	except:
		controller.vidro.set_rc_throttle(controller.base_rc_throttle)
		controller.vidro.set_rc_roll(controller.base_rc_roll)
		controller.vidro.set_rc_pitch(controller.base_rc_pitch)
		controller.vidro.set_rc_yaw(controller.base_rc_yaw)
		curses_print("ERROR",2,0)

	if round((round(time.clock(),3) % .05),2) == 0:

		screen.clear()
		screen.refresh()

		curses_print("Position: X: " + str(vidro.get_position()[0]) + " Y: " + str(vidro.get_position()[1]) + " Z: " + str(vidro.get_position()[2]),0,0)

		#Print alt data
		curses_print("Throttle RC Override: " + str(vidro.current_rc_overrides[2]), 5, 1)
		curses_print("Throttle RC Level: " + str(vidro.current_rc_channels[2]), 6, 1)
		curses_print("Error: " + str(controller.error_alt), 7, 1)
		curses_print("Altitude:" + str(vidro.get_position()[2]), 8, 1)
		curses_print("T: "+ str(int(controller.base_rc_throttle+controller.error_alt*controller.alt_K_P+controller.I_error_alt*controller.alt_K_I)) + " = "+ str(controller.base_rc_throttle) + " + " + str(controller.error_alt*controller.alt_K_P) + " + " + str(controller.I_error_alt*controller.alt_K_I) + " + " + str(controller.D_error_alt*controller.alt_K_D), 19, 0)

		#Print yaw data
		curses_print("Yaw RC Level: " + str(vidro.current_rc_channels[3]), 6, 0)
		curses_print("Error: " + str(controller.error_yaw), 7, 0)
		curses_print("Heading Radians: " + str(vidro.get_yaw_radians()), 8, 0)
		curses_print("Heading Degrees: " + str(vidro.get_yaw_degrees()), 9, 0)
		curses_print("Y: "+ str(int(controller.base_rc_yaw+controller.error_yaw*controller.yaw_K_P+controller.I_error_yaw*controller.yaw_K_I)) + " = "+ str(controller.base_rc_yaw) + " + " + str(controller.error_yaw*controller.yaw_K_P) + " + " + str(controller.I_error_yaw*controller.yaw_K_I) + " + " + str(controller.D_error_yaw*controller.yaw_K_D), 20, 0)

		#Print pitch and roll
		curses_print("Pitch RC Level: " + str(vidro.current_rc_channels[1]), 11, 0)
		curses_print("Roll RC Level: " + str(vidro.current_rc_channels[0]), 11, 1)
		curses_print("Pitch: " + str(vidro.get_pitch()), 12, 0)
		curses_print("Roll: " + str(vidro.get_roll()), 12, 1)
		curses_print("X Error: " + str(round(controller.error_x)), 15, 0)
		curses_print("Y Error: " + str(round(controller.error_y)), 15, 1)
		curses_print("Roll Error: " + str(round(controller.error_roll)), 13, 1)
		curses_print("Pitch Error: " + str(round(controller.error_pitch)), 13, 0)
		curses_print("P: " +  str(int(controller.base_rc_pitch+controller.error_pitch*controller.pitch_K_P+controller.I_error_pitch*controller.pitch_K_I+controller.D_error_pitch*controller.pitch_K_D)) + " = " + str(controller.base_rc_pitch) + " + " + str(controller.error_pitch*controller.pitch_K_P) + " + " + str(controller.I_error_pitch*controller.pitch_K_I) + " + " + str(controller.D_error_pitch*controller.pitch_K_D), 21, 0)
		curses_print("R: " +  str(int(controller.base_rc_roll+controller.error_roll*controller.roll_K_P+controller.I_error_roll*controller.roll_K_I+controller.D_error_roll*controller.roll_K_D)) + " = " + str(controller.base_rc_roll) + " + " + str(controller.error_roll*controller.roll_K_P) + " + " + str(controller.I_error_roll*controller.roll_K_I) + " + " + str(controller.D_error_roll*controller.roll_K_D), 22, 0)

		screen.refresh()

	#Add values to arrays for plotting
	plot_error_yaw.append(controller.error_yaw)
	plot_error_yaw_I.append(controller.I_error_yaw)
	plot_error_yaw_D.append(controller.D_error_yaw)
	plot_time_yaw.append(controller.previous_time_yaw)

	plot_error_throttle.append(controller.error_alt)
	plot_error_throttle_I.append(controller.I_error_alt)
	plot_error_throttle_D.append(controller.D_error_alt*controller.alt_K_D)
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

	vidro.get_mavlink()

vidro.get_mavlink()
vidro.close()

#Plots
plot.figure(1)
plot.xlabel("Time(sec)")
plot.ylabel("Error(rads)")
plot.title("Yaw")
plot.plot(plot_time_yaw,plot_error_yaw)
plot.figure(2)
plot.xlabel("Time(sec)")
plot.ylabel("Error(mm)")
plot.title("Throttle")
plot.plot(plot_time_throttle,plot_error_throttle)

plot.figure(3)
plot.xlabel("Time(sec)")
plot.ylabel("Error(mm) | RC Value")
plot.title("Pitch Error PD | RC Value")
plot.plot(plot_time_pitch,plot_error_pitch)
#plot.plot(plot_time_pitch,plot_rc_pitch)
#plot.plot(plot_time_pitch,plot_error_pitch_D)
plot.figure(4)
plot.xlabel("Time(sec)")
plot.ylabel("Error(mm) | RC Value")
plot.title("Roll Error PD | RC Value")
plot.plot(plot_time_roll,plot_error_roll)
#plot.plot(plot_time_roll,plot_rc_roll)
#plot.plot(plot_time_roll,plot_error_roll_D)

"""
plot.figure(5)
plot.xlabel("Time(sec)")
plot.ylabel("Error(rads)")
plot.title("Yaw")
plot.plot(plot_time_yaw,plot_error_yaw)
plot.plot(plot_time_yaw,plot_error_yaw_I)
plot.figure(6)
plot.xlabel("Time(sec)")
plot.ylabel("Error(mm)")
plot.title("Throttle")
plot.plot(plot_time_throttle,plot_error_throttle)
plot.plot(plot_time_throttle,plot_error_throttle_I)
"""
"""
plot.figure(7)
plot.xlabel("Time(sec)")
plot.ylabel("Error(mm)")
plot.title("Pitch Error PI")
plot.plot(plot_time_pitch,plot_error_pitch)
plot.plot(plot_time_pitch,plot_error_pitch_I)
plot.figure(8)
plot.xlabel("Time(sec)")
plot.ylabel("Error(mm)")
plot.title("Roll Error PI")
plot.plot(plot_time_roll,plot_error_roll)
plot.plot(plot_time_roll,plot_error_roll_I)
"""

plot.figure(9)
plot.xlabel("x Location(mm)")
plot.ylabel("y Location(mm)")
plot.title("Location")
plot.plot(plot_x_current, plot_y_current)

plot.show()

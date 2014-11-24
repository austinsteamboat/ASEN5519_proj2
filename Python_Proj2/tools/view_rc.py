from vidro_class import Vidro, ViconStreamer
from position_controller import PositionController
import sys, math, time
import socket, struct, threading
import curses
import utm
import matplotlib.pyplot as plot

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

vidro = Vidro(False, 1)
vidro.connect()
controller = PositionController(vidro)

screen = curses.initscr()
screen.clear()
screen.refresh()

while True:

	curses_print("Roll: " + str(vidro.current_rc_channels[0]),0,0)
	curses_print("Pitch: " + str(vidro.current_rc_channels[1]),1,0)
	curses_print("Throttle: " + str(vidro.current_rc_channels[2]),2,0)
	curses_print("Yaw: " + str(vidro.current_rc_channels[3]),3,0)
	time.sleep(.005)
	screen.refresh()
	screen.clear()
	screen.refresh()
	vidro.update_mavlink()

vidro.close()

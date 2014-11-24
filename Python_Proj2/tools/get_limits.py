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

vidro = Vidro(False)
vidro.connect()
controller = PositionController(vidro)

screen = curses.initscr()
screen.clear()
screen.refresh()

RC_yaw_min = controller.base_rc_yaw
RC_yaw_max = controller.base_rc_yaw
RC_throttle_min = controller.base_rc_throttle
RC_throttle_max = controller.base_rc_throttle
RC_pitch_min = controller.base_rc_pitch
RC_pitch_max = controller.base_rc_pitch
RC_roll_min = controller.base_rc_roll
RC_roll_max = controller.base_rc_roll

while vidro.current_rc_channels[4] > 1600:

	screen.erase()
	if vidro.current_rc_channels[0] < RC_roll_min:
		RC_roll_min = vidro.current_rc_channels[0]
	if vidro.current_rc_channels[0] > RC_roll_max:
		RC_roll_max = vidro.current_rc_channels[0]

	if vidro.current_rc_channels[1] < RC_pitch_min:
		RC_pitch_min = vidro.current_rc_channels[1]
	if vidro.current_rc_channels[1] > RC_pitch_max:
		RC_pitch_max = vidro.current_rc_channels[1]

	if vidro.current_rc_channels[2] < RC_throttle_min:
		RC_throttle_min = vidro.current_rc_channels[2]
	if vidro.current_rc_channels[2] > RC_throttle_max:
		RC_throttle_max = vidro.current_rc_channels[2]

	if vidro.current_rc_channels[3] < RC_yaw_min:
		RC_yaw_min = vidro.current_rc_channels[3]
	if vidro.current_rc_channels[3] > RC_yaw_max:
		RC_yaw_max = vidro.current_rc_channels[3]

	curses_print("Roll: " + str(vidro.current_rc_channels[0]),0,0)
	curses_print("Pitch: " + str(vidro.current_rc_channels[1]),1,0)
	curses_print("Throttle: " + str(vidro.current_rc_channels[2]),2,0)
	curses_print("Yaw: " + str(vidro.current_rc_channels[3]),3,0)

	screen.refresh()
	screen.clear()
	screen.refresh()

	vidro.get_mavlink()

vidro.close()

screen.clear()
screen.refresh()

curses_print("Roll Min: " + str(RC_roll_min), 0,0)
curses_print("Roll Max: " + str(RC_roll_max), 1,0)

curses_print("Pitch Min " + str(RC_pitch_min), 3,0)
curses_print("Pitch Max: " + str(RC_pitch_max), 4,0)

curses_print("Throttle Min: " + str(RC_throttle_min), 6,0)
curses_print("Throttle Max: " + str(RC_throttle_max), 7,0)

curses_print("Yaw Min: " + str(RC_yaw_min), 9,0)
curses_print("Yaw Max: " + str(RC_yaw_max), 10,0)

screen.refresh()

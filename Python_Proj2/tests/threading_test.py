import threading
import time
import curses

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
	
#Setup the screen for curses
#screen = curses.initscr()
#screen.clear()
#screen.refresh()


def daemon(event):
	start_time = time.time()
	while time.time() - start_time < 10:
		print event.is_set()

event = threading.Event()

d = threading.Thread(name='daemon', target=daemon, args=(event,))
#d.setDaemon(True)

d.start()

start_time = time.time()

while (time.time()-start_time) < 5:
	
	if time.time() % .5 == 0:
		event.set()
	if time.time() % 1 == 0:
		event.clear()

d.join()

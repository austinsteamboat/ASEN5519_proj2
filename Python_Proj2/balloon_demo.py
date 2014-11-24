#!/usr/bin/env python
# -*- coding: utf-8 -*-

from vidro import Vidro, ViconStreamer
from position_controller import PositionController
import sys, math, time
import socket, struct, threading
import curses
import matplotlib.pyplot as plot
import logging
import numpy as np
import cv2
import picamera
import picamera.array

H_lower = 157
S_lower = 149
V_lower = 67

H_upper = 186
S_upper = 255
V_upper = 255

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

def get_object(frame):
    try:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower = np.array([H_lower, S_lower, V_lower], dtype=np.uint8)
        upper = np.array([H_upper,S_upper,V_upper], dtype=np.uint8)

        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        kernel_open = np.ones((4,4),np.uint8)
        kernel_close = np.ones((25,25),np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(frame,frame, mask= mask)

        contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(frame, contours, -1, (0,255,0), 3)

        try:
            cnt = contours[0]

            area = cv2.contourArea(cnt)
            #perimeter = cv2.arcLength(cnt, True)

            moment = cv2.moments(cnt)
            cx = int(moment['m10']/moment['m00'])
            cy = int(moment['m01']/moment['m00'])

            min_con_x = min(x[0][0] for x in cnt)
            min_con_y = min(y[0][1] for y in cnt)
            max_con_x = max(x[0][0] for x in cnt)
            max_con_y = max(y[0][1] for y in cnt)

            out_of_bounds = False
            if min_con_x == 1 or min_con_y == 1 or max_con_x == 638 or max_con_y == 478:
            out_of_bounds = True
            num_objects = len(contours)

            image_data = [cx ,cy, area, min_con_x, max_con_x, min_con_y, max_con_y, out_of_bounds, num_objects]

        except:
            area = None
            perimeter = None
            cx = None
            cy = None
            min_con_x = None
            min_con_y = None
            max_con_x = None
            max_con_y = None
            out_of_bounds = None
            image_data = None

    return image_data


def camera_thread(event):
    global frame
    global new_frame
    global plot_camera_time
    global Plot_camera_cycle
    global start_time
    previous_time = time.time()
    cycle = 0
    with picamera.PiCamera() as camera:
        with picamera.array.PiRGBArray(camera) as stream:
            camera.resolution = (320, 240)
            camera.hflip = True
            camera.vflip = True

            while not event.is_set():
                current_time = time.time()
                plot_camera_time.append(current_time-start_time)
                plot_camera_cycle.append(current_time-previous_time)
                plot_camera_cycle_number.append(cycle)
                cycle += 1
                previous_time = current_time
                curses_print("In camera loop",3,0)
                camera.capture(stream, 'bgr', use_video_port=True)
                # stream.array now contains the image data in BGR order
        frame = stream.array
                new_frame = True
                stream.truncate(0)

#Setup of vidro and controller
vidro = Vidro(False, 1)
flight_ready = vidro.connect()
controller = PositionController(vidro)

event = threading.Event()

d = threading.Thread(name='camera', target=camera_thread, args=(event,))

d.start()

start_time = time.time()
previous_time = time.time()

cycle = 0

controller.update_gains()

sequence = 0
seq0_cnt = 0
seq1_cnt = 0
seq2_cnt = 0
seq3_cnt = 0

yaw = 0

while vidro.current_rc_channels[5] > 1600:

    #On ground
    if sequence = 0:
        error_z = controller.rc_alt(1000)
        error_yaw = controller.rc_yaw(0)
        error_pitch, error_roll = controller.rc_xy(0,0)
        if error_z < 50 and error_yaw < 1 and error_pitch < 50 and error_roll < 50:
            if (time.time() % 1) == 0:
                seq0_cnt += 1
                if seq0_cnt == 3:
                    sequence = 1

    #Hover in center of space
    if sequence = 1:
        error_z = controller.rc_alt(1000)
        error_yaw = controller.rc_yaw(0)
        error_x_y = controller.rc_xy(0,0)
        if error_z < 50 and error_yaw < 1 and error_pitch < 50 and error_roll < 50:
            if (time.time() % 1) == 0:
                seq1_cnt += 1
                if seq1_cnt == 3:
                    sequence = 2

    #Yaw around to find red balloon
    if sequence = 2:
        error_z = controller.rc_alt(1000)
        error_yaw = controller.rc_yaw(yaw)
        error_x_y = controller.rc_xy(0,0)
        if new_frame == True:
            image_data = get_object(frame)
            #0: Centroid x
            #1: Centroid y
            #2: Area
            #3: Min x value of object 1
            #4: Max x value of object 1
            #5: Min y value of object 1
            #6: Max y value of object 1
            #7: Out of bounds
            #8: Number of objects
            new_frame = False
        if image_data[8] == 1:
            if image_data[0] != center_of_image
                if (time.time() % .5) == 0:
                    yaw += .1
            else:
                sequence = 3
        if image_data[8] > 1:
            sequence = 0
        if image_data[8] < 1:
            if (time.time() % 1) == 0:
                yaw += 1
                if yaw > 2 * math.pi:
                    yaw -= (2 * math.pi)

    #Adjust height
    if sequence = 3:
        error_z = controller.rc_alt(1000)
        error_yaw = controller.rc_yaw(1)
        error_x_y = controller.rc_xy(0,0)
        if error_z < 50 and error_yaw < 1 and error_pitch < 50 and error_roll < 50:
            if (time.time() % 1) == 0:
                seq3_cnt += 1
                if seq3_cnt == 3:
                    sequence = 0

    #Aproach balloon
    if sequence = 3:
        error_z = controller.rc_alt(1000)
        error_yaw = controller.rc_yaw(1)
        error_x_y = controller.rc_xy(0,0)
        if error_z < 50 and error_yaw < 1 and error_pitch < 50 and error_roll < 50:
            if (time.time() % 1) == 0:
                seq3_cnt += 1
                if seq3_cnt == 3:
                    sequence = 0


    current_time = time.time()
    curses_print(str(current_time-start_time),0,0)
    curses_print("New Frame: " + str(new_frame),7,0)
    plot_main_time.append(current_time-start_time)
    plot_main_cycle.append((current_time-previous_time))
    plot_main_cycle_number.append(cycle)
    cycle += 1
    previous_time = current_time


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    screen.clear()
    screen.refresh()

event.set()
d.join()

cv2.destroyAllWindows()
curses.endwin()

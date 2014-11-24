import cv2
import picamera
import picamera.array
import time
import matplotlib.pyplot as plot

plot_time=[]
plot_cycle=[]

previous_time = time.time()
start_time = time.time()
with picamera.PiCamera() as camera:
    with picamera.array.PiRGBArray(camera) as stream:
        camera.resolution = (320, 240)

	while (time.time() - start_time) < 15:
            current_time = time.time()
            plot_cycle.append(current_time - previous_time)
            plot_time.append(current_time - start_time)
	    previous_time = current_time
	    camera.capture(stream, 'bgr', use_video_port=True)
	    # stream.array now contains the image data in BGR order
            #cv2.imshow('frame', stream.array)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            # reset the stream before the next capture
            #stream.seek(0)
            stream.truncate(0)

        cv2.destroyAllWindows()

plot.figure(1)
plot.xlabel("Time (sec)")
plot.ylabel("Time/cycle")
plot.title("Main loop cycle times without threading")
plot.plot(plot_time, plot_cycle)

plot.show()

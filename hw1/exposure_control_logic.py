import numpy as np

exposure_threshold = 25 # a good value for the average intensity of the frame

# get the average intensity of a frame
frame_avg = np.average(linescan_frame)
 
if frame_avg > exposure_threshold:
	# do something to decrease the exposure

elif frame_avg < exposure_threshold:
	# do something to increase the exposure

else:
	continue



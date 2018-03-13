
# coding: utf-8

# # EECS192 Spring 2018 Track Finding from 1D line sensor data

# In[6]:

# changed to use 8 bit compressed line sensor values
# data format: 128 comma separated values, last value in line has space, not comma
# line samples are about 10 ms apart
#  csv file format time in ms, 128 byte array, velocity
# note AGC has already been applied to data, and camera has been calibrated for illumination effects


# In[8]:

import numpy as np
from scipy import stats
import matplotlib.pyplot as plt
# import scipy.ndimage as ndi  # useful for 1d filtering functions
plt.close("all")   # try to close all open figs

# In[9]:

# Graphing helper function, used in plot_frame function below
def setup_graph(title='', x_label='', y_label='', fig_size=None):
    fig = plt.figure()
    if fig_size != None:
        fig.set_size_inches(fig_size[0], fig_size[1])
    ax = fig.add_subplot(111)
    ax.set_title(title)
    ax.set_xlabel(x_label)
    ax.set_ylabel(y_label)


# Line scan plotting function.
# 

# In[10]:

def plot_frame(linearray):
    nframes = np.size(linearray)/128
    n = range(0,128)
    print 'number of frames', nframes
    print 'size of line', np.size(linearray[0,:])
    for i in range(0, nframes-1):
        setup_graph(title='$x[n]$', x_label='$n$', y_label='row'+str(i)+' $ xa[n]$', fig_size=(15,2))
        plt.subplot(1,3,1)
        _ = plt.plot(n,linearray[0,:])
        plt.subplot(1,3,2)
        _ = plt.plot(n,linearray[i,:])
    # plot simple difference between frame i and first frame
        plt.subplot(1,3,3)
        _ = plt.plot(n,linearray[i,:] - linearray[0,:])
        plt.ylabel('Frame n - Frame 0')


# ### grayscale plotting of line function:
# 

# In[11]:

CAMERA_LENGTH = 128
INTENSITY_MIN = 0
INTENSITY_MAX = 255
def plot_gray(fig, camera_data):
  # x fencepost positions of each data matrix element
  x_mesh = []
  for i in range(0, len(camera_data)+1):
    x_mesh.append([i-0.5] * (CAMERA_LENGTH + 1))
  x_mesh = np.array(x_mesh)
  
  # y fencepost positions of each data matrix element
  y_array = range(0, CAMERA_LENGTH + 1)
  y_array = list(map(lambda x: x - 0.5, y_array))
  y_mesh = np.array([y_array] * (len(camera_data)+1))
    
  data_mesh = np.array(camera_data)
  vmax1 = np.max(data_mesh)
  data_mesh = INTENSITY_MAX * data_mesh/vmax1  # normalize intensity
  
  fig.set_xlim([-0.5, len(camera_data) - 0.5])
  fig.set_ylim([-8.5, CAMERA_LENGTH - 0.5])

  fig.pcolorfast(x_mesh, y_mesh, data_mesh,
      cmap='gray', vmin=INTENSITY_MIN, vmax=INTENSITY_MAX,
      interpolation='None')


# In[12]:

### inputs:
# linescans - An array of length n where each element is an array of length 128. Represents n frames of linescan data.

### outputs:
# track_center_list - A length n array of integers from 0 to 127. Represents the predicted center of the line in each frame.
# track_found_list - A length n array of booleans. Represents whether or not each frame contains a detected line.
# cross_found_list - A length n array of booleans. Represents whether or not each frame contains a crossing.

def find_track(linescans):
    n = len(linescans)
    track_center_list = n * [64]
    track_found_list = n * [True]
    cross_found_list = n * [False]
    
    ### Code to be added here
    max_intensity = 0 # the max intensity value of all the frames
    prev_track = 64 # the previous iteration's track index
    
    for i in range(0, n): # iterate through the whole track
      track_detection_state = True

      frame = linescans[i] # a single frame
      frame_argmax = np.argmax(linescans[i]) # the argument of the maximum value of the frame
      frame_max = frame[frame_argmax] # the maximum value of the frame

      if frame_max > max_intensity:
        max_intensity = frame_max

      # we use the mode to gauge how different the detected spike is from it's surroundings (the mode)
      frame_mode = stats.mode(frame)[0][0] # (scipy returns some other stuff after the mode)
      
      # if the frame max wasn't at least twice as big as the mode of the frame then we've lost the track
      # or if the track location jumped more than 1/3 of the frame width, we've lost the track
      if (frame_max * 0.5) < frame_mode or (frame_argmax - prev_track) > 43: 
        track_detection_state = False
        track_found_list[i] = track_detection_state

      if (max_intensity * 0.3) < frame_mode:
        cross_found_list[i] = True

      if track_detection_state == True:
        track_center_list[i] = frame_argmax # the center of the track is simply the argmax of the frame
        prev_track = frame_argmax
      else: # if the track isn't detected just use the previous iteration's value for the track index
        track_center_list[i] = prev_track

    return track_center_list, track_found_list, cross_found_list

### IMPORT THE CSV FILE ###
import csv
filename = 'natcar2016_team1.csv'
#filename = 'natcar2016_team1_short.csv'
csvfile=open(filename, 'rb')
telemreader=csv.reader(csvfile, delimiter=',', quotechar='"')
# Actual Spring 2016 Natcar track recording by Team 1.
telemreader.next() # discard first line
telemdata = telemreader.next() # format time in ms, 128 byte array, velocity
linescans=[]  # linescan array
times=[]
velocities=[]
for row in telemreader:
    times.append(eval(row[0])) # sample time
    velocities.append(eval(row[2])) # measured velocity
    line = row[1] # get scan data
    arrayline=np.array(eval(line)) # convert line to an np array
    linescans.append(arrayline)
# print 'scan line0:', linescans[0]
# print 'scan line1:', linescans[1]

track_center_list, track_found_list, cross_found_list = find_track(linescans)
#for i, (track_center, track_found, cross_found) in enumerate(zip(track_center_list, track_found_list, cross_found_list)):
#    print 'scan # %d center at %d. Track_found = %s, Cross_found = %s' %(i,track_center,track_found, cross_found)


### PLOTS ###
#fig=plt.figure()
fig = plt.figure(figsize = (16, 3))
fig.set_size_inches(13, 4)
fig.suptitle("%s\n" % (filename))     
ax = plt.subplot(1, 1, 1)
#plot_gray(ax, linescans[0:1000])  # plot smaller range if hard too see
plot_gray(ax, linescans) 


# plot of velocities
fig = plt.figure(figsize = (8, 4))
# fig.set_size_inches(13, 4)
fig.suptitle("velocities %s\n" % (filename))   
plt.xlabel('time [ms]')
plt.ylabel('velocity (m/s)')  
plt.plot(times,velocities)

# plot of found track position 
fig = plt.figure(figsize = (8, 4))
# fig.set_size_inches(13, 4)
fig.suptitle("track center %s\n" % (filename))   
plt.xlabel('time [ms]')
plt.ylabel('track center')  
plt.plot(times,track_center_list)

# plot of track detection
fig = plt.figure(figsize = (8, 4))
# fig.set_size_inches(13, 4)
fig.suptitle("track detection %s\n" % (filename))   
plt.xlabel('time [ms]')
plt.ylabel('track detection')  
plt.plot(times,track_found_list)

# plot of crosses found
fig = plt.figure(figsize = (8, 4))
# fig.set_size_inches(13, 4)
fig.suptitle("crosses found %s\n" % (filename))   
plt.xlabel('time [ms]')
plt.ylabel('crosses')  
plt.plot(times,cross_found_list)

### EXPERIMENTING WITH DIFFERENCING ###
# plot of an individual frame (raw data)

# frame_number = 50

# # calculating the difference between the peak of the plot and the average
# frame_avg = np.mean(linescans[frame_number])
# frame_peak = np.max(linescans[frame_number])
# peak_avg_diff = frame_peak - frame_avg

# print 'raw peak - average = ', peak_avg_diff

# fig = plt.figure(figsize = (8, 4))
# fig.suptitle("track center %d\n" % (frame_number))   
# plt.xlabel('pixel')
# plt.ylabel('intensity')  
# plt.plot(linescans[frame_number])

# # plot of an individual frame with frame subtraction and peak detection

# frame_avg = np.mean(linescans[frame_number])
# result_frame = linescans[frame_number] - frame_avg

# # calculating the difference between the peak of the plot and the average
# result_peak = np.max(result_frame)
# result_avg = np.mean(result_frame) 
# peak_avg_diff = result_peak - result_avg

# print 'resultant peak - average = ', peak_avg_diff

# fig = plt.figure(figsize = (8, 4))
# fig.suptitle("track center %d\n" % (frame_number))   
# plt.xlabel('pixel')
# plt.ylabel('intensity')
# plt.plot(result_frame)

### GET THE TRACK ###
find_track(linescans)

plt.show() # show the plots


